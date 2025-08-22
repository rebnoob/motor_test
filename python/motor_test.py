import sys, time, math, threading, termios, tty
sys.path.append('../lib')
from unitree_actuator_sdk import *

# ===========================
# ===== User Tunables =======
# ===========================
MOTOR  = MotorType.GO_M8010_6
PORTS  = ['/dev/ttyUSB0', '/dev/ttyUSB1']     # [hip-pitch (thigh), knee (calf)]
IDS    = [0, 0]                                # motor IDs on each bus

# Link lengths (meters)  (updated by you)
L2, L3 = 0.11, 0.130

# Nominal stance target (meters) in the hip's y-z plane
Y0, Z0 = 0.14, 0.00

# IK knee configuration (+1 knee-forward, -1 knee-back)
KNEE_SIGN = +1

# Joint PD gains (rotor-side hybrid PD)
KP, KD  = 0.01, 0.01

# Control timing
CTRL_DT = 0.005   # 5 ms

# Output-side joint limits (±180° hard bound)
THETA_MIN = [-math.pi, -math.pi]
THETA_MAX = [ math.pi,  math.pi]

# Output-side joint motion limits (for slew)
THETA_VMAX = [2.0, 2.5]     # rad/s
THETA_AMAX = [10.0, 12.0]   # rad/s^2

# Rotor mapping (rotor = SIGN * GR * STAGE * theta + OFFSET)
# If a joint moves opposite, flip its SIGN_ROT entry.
SIGN_ROT  = [+1.0, +1.0]     # flip [1] if belt routing reverses knee direction
OFFS_ROT  = [ 0.0,  0.0]     # auto-set at boot

# --- Belt stage on the knee (L3 joint): 18T (motor) → 30T (joint) ---
BELT_NUM = 18.0
BELT_DEN = 30.0
STAGE    = [1.0, BELT_DEN / BELT_NUM]  # [thigh, knee] actuator-output-per-joint ratio

# Workspace (optional) soft bounds for y,z (meters)
Y_MIN, Y_MAX =  -0.35,  0.35
Z_MIN, Z_MAX = -0.35,  0.30

# ===========================
# ===== Helper Functions ====
# ===========================
def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def ik_planar(y, z, knee_sign=+1):
    """Closed-form IK for 2-link in y-z plane → (theta2, theta3) (link angles)."""
    r2 = y*y + z*z
    c  = (r2 - L2*L2 - L3*L3) / (2.0*L2*L3)
    if c < -1.0 or c > 1.0:
        raise ValueError("unreachable")
    s  = knee_sign * math.sqrt(max(0.0, 1.0 - c*c))
    th3 = math.atan2(s, c)
    phi = math.atan2(z, y)
    psi = math.atan2(L3*math.sin(th3), L2 + L3*math.cos(th3))
    th2 = phi - psi
    return th2, th3

def project_to_reachable(y, z):
    """If (y,z) is outside reachable annulus, project to nearest point on boundary."""
    r  = math.hypot(y, z)
    rmin, rmax = abs(L2 - L3), (L2 + L3)
    if r < 1e-6:
        return (rmin, 0.0)  # arbitrary direction
    if r < rmin:
        scale = rmin / r
    elif r > rmax:
        scale = rmax / r
    else:
        return y, z
    return y*scale, z*scale

class Slew:
    """Output-side (theta) slew with optional accel limit."""
    def __init__(self, vmax, amax=None):
        self.vmax, self.amax = vmax, amax
        self.v = 0.0
    def step(self, curr, target, dt):
        dv = target - curr
        v_des = clamp(dv / dt, -self.vmax, self.vmax)
        if self.amax is not None:
            dv_allowed = self.amax * dt
            v_des = clamp(v_des, self.v - dv_allowed, self.v + dv_allowed)
        self.v = v_des
        return curr + self.v * dt, self.v

# ===========================
# ===== Keyboard Handling ===
# ===========================
class KeyPoller:
    """Non-blocking single-char reader on POSIX."""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def getch(self):
        import select
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

# ===========================
# ===== Hardware Setup ======
# ===========================
serials = [SerialPort(p) for p in PORTS]
cmds    = [MotorCmd()  for _ in PORTS]
datas   = [MotorData() for _ in PORTS]
time.sleep(0.6)  # let drives boot

GR = queryGearRatio(MOTOR)  # ≈ 6.33 on GO-M8010-6

# Per-joint rotor-per-link gain, including belt stage (knee only)
JOINT2ROTOR_GAIN = [SIGN_ROT[0] * GR * STAGE[0],
                    SIGN_ROT[1] * GR * STAGE[1]]

for i in range(2):
    datas[i].motorType = MOTOR
    c = cmds[i]
    c.motorType = MOTOR
    c.mode = queryMotorMode(MOTOR, MotorMode.FOC)  # hybrid PD
    c.id   = IDS[i]
    c.kp, c.kd = KP, KD
    c.tau = 0.0
    c.q   = 0.0
    c.dq  = 0.0

# Initial target & IK
y_des, z_des = Y0, Z0
y_des, z_des = project_to_reachable(clamp(y_des, Y_MIN, Y_MAX),
                                    clamp(z_des, Z_MIN, Z_MAX))
th2_des, th3_des = ik_planar(y_des, z_des, KNEE_SIGN)
th2_des = clamp(th2_des, THETA_MIN[0], THETA_MAX[0])
th3_des = clamp(th3_des, THETA_MIN[1], THETA_MAX[1])

theta_cmd   = [th2_des, th3_des]     # output-side (link) commands
theta_dot_c = [0.0, 0.0]
slews       = [Slew(THETA_VMAX[0], THETA_AMAX[0]),
               Slew(THETA_VMAX[1], THETA_AMAX[1])]

# Auto-zero rotor offsets so initial q_des == q_meas (prevents jump/runaway)
for i in range(2):
    serials[i].sendRecv(cmds[i], datas[i])
OFFS_ROT = [ datas[i].q - JOINT2ROTOR_GAIN[i] * theta_cmd[i] for i in range(2) ]
print("Auto-zeroed rotor offsets (rad):", OFFS_ROT)
print(f"Knee belt stage = {STAGE[1]:.4f} (rotor/actuator includes GR={GR:.3f})")
print("JOINT2ROTOR_GAIN =", JOINT2ROTOR_GAIN)

# Live control state
step = 0.005    # meters per keypress
hold = False    # freeze target when True
motors_on = True  # kp>0 when True, kp=0 when False
running = True

print("""
Live IK control:
  w/s : z +/-
  a/d : y -/+      (y is forward/back in the leg plane)
  [   : halve step
  ]   : double step   (current step shown in telemetry)
  r   : reset target to nominal stance
  space: toggle HOLD target
  x   : toggle MOTORS ON/OFF (sets kp=0 when off)
  q   : quit
""")

##!/usr/bin/env python3
import time, math, sys, select, termios, tty
sys.path.append('../lib')
from unitree_actuator_sdk import *

# ===================== Geometry & Hardware =====================
L2, L3 = 0.11, 0.13

# Unitree GO-M8010-6 reductions (rotor -> actuator output)
G_HIP, G_KNEE = 6.33, 6.33

# Knee belt: Nm on actuator pulley, Nj on knee pulley (e.g., 19T -> 30T)
Nm, Nj = 19, 30

# Ports
HIP_PORT, KNEE_PORT = '/dev/ttyUSB0', '/dev/ttyUSB1'

# ===================== Control Gains & Limits =====================
KP_HIP, KD_HIP  = 0.01, 0.01
KP_KNEE, KD_KNEE = 0.01, 0.01

HIP_MIN, HIP_MAX   = math.radians(-120), math.radians(+120)
KNEE_MIN, KNEE_MAX = math.radians(+5),   math.radians(+155)  # knee > 0 bends

# Sign & zero offsets (rotor-side). Zeroing updates these live.
HIP_SIGN, KNEE_SIGN = +1.0, +1.0
HIP_ZERO, KNEE_ZERO = 0.0,  0.0

# Timing
CTRL_HZ = 1000.0
CTRL_DT = 1.0 / CTRL_HZ
PRINT_EVERY_SEC = 0.1

# Feed-forward cap (not used; we keep dq=0 for teleop stability)
DQ_DES_MAX = 12.0

# ===================== Workspace =====================
R_MIN = abs(L2 - L3) + 0.01
R_MAX = (L2 + L3) - 0.01

Y_MIN, Y_MAX = -0.20, +0.30
Z_MIN, Z_MAX = -0.22, +0.22

def clamp(v, vmin, vmax): return max(vmin, min(vmax, v))

def clamp_workspace(y, z):
    # box clamp
    y = clamp(y, Y_MIN, Y_MAX)
    z = clamp(z, Z_MIN, Z_MAX)
    # radial clamp
    r = math.hypot(y, z)
    if r < 1e-9:
        return Y_MIN, 0.0
    if r < R_MIN:
        s = R_MIN / r
        y, z = y * s, z * s
    elif r > R_MAX:
        s = R_MAX / r
        y, z = y * s, z * s
    return y, z

# ===================== Kinematics =====================
def ik_2link_yz(y, z, elbow='down', l2=L2, l3=L3):
    r2 = y*y + z*z
    r  = math.sqrt(r2)
    if r > l2 + l3 or r < abs(l2 - l3):
        raise ValueError("Target out of reach")
    c3 = (r2 - l2*l2 - l3*l3) / (2.0*l2*l3)
    c3 = max(-1.0, min(1.0, c3))
    s3m = math.sqrt(max(0.0, 1.0 - c3*c3))
    s3 = s3m if elbow == 'down' else -s3m
    th3 = math.atan2(s3, c3)
    th2 = math.atan2(z, y) - math.atan2(l3*s3, l2 + l3*c3)
    return th2, th3

def ik_best(y, z, last_th2=None, last_th3=None):
    c1 = ik_2link_yz(y, z, elbow='down')
    c2 = ik_2link_yz(y, z, elbow='up')
    if last_th2 is None or last_th3 is None:
        return c1
    return min((c1, c2), key=lambda th: (th[0]-last_th2)**2 + (th[1]-last_th3)**2)

def fk_yz(th2, th3, l2=L2, l3=L3):
    y = l2*math.cos(th2) + l3*math.cos(th2 + th3)
    z = l2*math.sin(th2) + l3*math.sin(th2 + th3)
    return y, z

def clamp_joint_limits(th2, th3):
    return (
        clamp(th2, HIP_MIN,  HIP_MAX),
        clamp(th3, KNEE_MIN, KNEE_MAX),
    )

# ===================== Joint <-> Rotor =====================
def hip_joint_to_rotor(theta2_joint):
    return HIP_SIGN * (theta2_joint * G_HIP) + HIP_ZERO

def knee_joint_to_rotor(theta3_joint):
    theta_act_out = theta3_joint * (Nj / Nm)   # joint -> actuator output
    return KNEE_SIGN * (theta_act_out * G_KNEE) + KNEE_ZERO

def hip_rotor_to_joint(q_rotor):
    return (q_rotor - HIP_ZERO) / (G_HIP * HIP_SIGN)

def knee_rotor_to_joint(q_rotor):
    act_out = (q_rotor - KNEE_ZERO) / (G_KNEE * KNEE_SIGN)
    return act_out * (Nm / Nj)                 # actuator -> joint

# ===================== Slew (rate limit) =====================
class Slew:
    def __init__(self, max_rate_rad_s):
        self.max_rate = float(max_rate_rad_s)
    def step(self, cur, tgt, dt):
        max_d = self.max_rate * dt
        d = clamp(tgt - cur, -max_d, +max_d)
        return cur + d

HIP_MAX_RATE  = 4.0   # rad/s hip
KNEE_MAX_RATE = 5.0   # rad/s knee
slew_hip  = Slew(HIP_MAX_RATE)
slew_knee = Slew(KNEE_MAX_RATE)

# ===================== Zeroing & Input =====================
def relax_and_read(ser, cmd, data):
    cmd.q = 0.0; cmd.dq = 0.0; cmd.kp = 0.0; cmd.kd = 0.0; cmd.tau = 0.0
    ser.sendRecv(cmd, data)
    time.sleep(0.002)
    ser.sendRecv(cmd, data)
    return data.q

def zero_current_pose(hip_ser, hip_cmd, hip_data, knee_ser, knee_cmd, knee_data, which="both"):
    global HIP_ZERO, KNEE_ZERO
    for cmd, ser, data in [(hip_cmd, hip_ser, hip_data), (knee_cmd, knee_ser, knee_data)]:
        cmd.q = 0.0; cmd.dq = 0.0; cmd.kp = 0.0; cmd.kd = 0.0; cmd.tau = 0.0
        ser.sendRecv(cmd, data)
    time.sleep(0.01)
    hip_q  = relax_and_read(hip_ser, hip_cmd, hip_data)
    knee_q = relax_and_read(knee_ser, knee_cmd, knee_data)
    if which in ("both", "hip"):  HIP_ZERO  = hip_q
    if which in ("both", "knee"): KNEE_ZERO = knee_q
    print(f"[ZERO] which={which} | HIP_ZERO={HIP_ZERO:+.4f} | KNEE_ZERO={KNEE_ZERO:+.4f}")

# --- Raw, non-blocking single-key poller (Linux/macOS terminals) ---
class KeyPoller:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)  # raw-ish; returns each key instantly
        return self
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def getch(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            ch = sys.stdin.read(1)
            return ch
        return None

# ===================== Main =====================
def main():
    # --- Serial
    hip_ser  = SerialPort(HIP_PORT)
    knee_ser = SerialPort(KNEE_PORT)
    hip_cmd,  hip_data  = MotorCmd(),  MotorData()
    knee_cmd, knee_data = MotorCmd(), MotorData()

    # --- Types & modes
    for c, d in [(hip_cmd, hip_data), (knee_cmd, knee_data)]:
        d.motorType = MotorType.GO_M8010_6
        c.motorType = MotorType.GO_M8010_6
        c.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        c.id = 0
    time.sleep(0.6)  # let boards boot

    # --- Initial sensor read
    hip_ser.sendRecv(hip_cmd, hip_data)
    knee_ser.sendRecv(knee_cmd, knee_data)

    # Initialize desired to current foot pose
    th2_now = hip_rotor_to_joint(hip_data.q)
    th3_now = knee_rotor_to_joint(knee_data.q)
    Y0, Z0 = fk_yz(th2_now, th3_now)
    y_des, z_des = clamp_workspace(Y0, Z0)

    # Joint command state
    th2_cmd, th3_cmd = th2_now, th3_now   # start from current
    motors_on = True
    hold = False
    step = 0.01  # m
    running = True
    t_last_print = 0.0
    last_th2, last_th3 = th2_now, th3_now

    print("[INFO] Teleop (no-Enter): w/s=z±, a/d=y±, [ ]=step±, space=hold, x=motors, r=reset, z/h/k=zero, q=quit")

    with KeyPoller() as kp:
        try:
            while running:
                # ---- Handle keyboard (non-blocking) ----
                ch = kp.getch()
                if ch:
                    c = ch.lower()
                    if c == 'q':
                        running = False
                    elif ch == ' ':
                        hold = not hold
                        print(f"[HOLD] {'ON' if hold else 'OFF'}")
                    elif c == 'x':
                        motors_on = not motors_on
                        print(f"[MOTORS] {'ON' if motors_on else 'OFF'}")
                    elif c == 'r':
                        y_des, z_des = Y0, Z0
                        print(f"[RESET] target -> (y,z)=({y_des:+.3f},{z_des:+.3f})")
                    elif ch == '[':
                        step = max(0.001, step * 0.5)
                        print(f"[STEP] {step*1000:.0f} mm")
                    elif ch == ']':
                        step = min(0.05, step * 2.0)
                        print(f"[STEP] {step*1000:.0f} mm")
                    elif c in ('z','h','k'):
                        which = 'both' if c=='z' else ('hip' if c=='h' else 'knee')
                        zero_current_pose(hip_ser, hip_cmd, hip_data, knee_ser, knee_cmd, knee_data, which=which)
                        # Recompute Y0,Z0 & target at current pose
                        th2_now = hip_rotor_to_joint(hip_data.q)
                        th3_now = knee_rotor_to_joint(knee_data.q)
                        Y0, Z0 = fk_yz(th2_now, th3_now)
                        y_des, z_des = clamp_workspace(Y0, Z0)
                        th2_cmd, th3_cmd = th2_now, th3_now
                        last_th2, last_th3 = th2_now, th3_now
                        print("[ZERO] Done. Targets reset to current foot position.")
                    elif not hold:
                        if c == 'w':  z_des += step
                        if c == 's':  z_des -= step
                        if c == 'a':  y_des -= step
                        if c == 'd':  y_des += step

                # Clamp & “project” target (use our workspace clamp as the projection)
                y_des, z_des = clamp_workspace(y_des, z_des)

                # Read sensors
                hip_ser.sendRecv(hip_cmd, hip_data)
                knee_ser.sendRecv(knee_cmd, knee_data)

                # IK to target (continuity via last_th*)
                try:
                    th2_des, th3_des = ik_best(y_des, z_des, last_th2, last_th3)
                except ValueError:
                    # keep last command if somehow out of reach
                    th2_des, th3_des = th2_cmd, th3_cmd
                th2_des, th3_des = clamp_joint_limits(th2_des, th3_des)

                # Slew on the link side for smoothness
                th2_cmd = slew_hip.step(th2_cmd, th2_des, CTRL_DT)
                th3_cmd = slew_knee.step(th3_cmd, th3_des, CTRL_DT)
                last_th2, last_th3 = th2_cmd, th3_cmd

                # Map link -> rotor (dq_des=0 for teleop stability)
                hip_cmd.q  = hip_joint_to_rotor(th2_cmd)
                knee_cmd.q = knee_joint_to_rotor(th3_cmd)
                hip_cmd.dq = 0.0
                knee_cmd.dq = 0.0

                # Gains / torque
                if motors_on:
                    hip_cmd.kp, hip_cmd.kd, hip_cmd.tau  = KP_HIP,  KD_HIP,  0.0
                    knee_cmd.kp, knee_cmd.kd, knee_cmd.tau = KP_KNEE, KD_KNEE, 0.0
                else:
                    # relax
                    for c in (hip_cmd, knee_cmd):
                        c.kp = 0.0; c.kd = 0.0; c.tau = 0.0

                # Send commands
                hip_ser.sendRecv(hip_cmd, hip_data)
                knee_ser.sendRecv(knee_cmd, knee_data)

                # Telemetry ~10 Hz
                now = time.time()
                if now - t_last_print > PRINT_EVERY_SEC:
                    t_last_print = now
                    # Measured FK
                    th2_meas = hip_rotor_to_joint(hip_data.q)
                    th3_meas = knee_rotor_to_joint(knee_data.q)
                    y_meas, z_meas = fk_yz(th2_meas, th3_meas)
                    print(f"\n[TELEOP] step={step*1000:.0f} mm | "
                          f"{'HOLD' if hold else 'LIVE'} | {'MOTORS ON' if motors_on else 'MOTORS OFF'}")
                    print(f"[FOOT ] target y={y_des:+.3f} z={z_des:+.3f} | meas y={y_meas:+.3f} z={z_meas:+.3f}")
                    print(f"[HIP  ] th_cmd={th2_cmd:+.3f} rad | q_rot={hip_data.q:+.3f} | dq={hip_data.dq:+.3f} | T={hip_data.temp:.1f} | err={hip_data.merror}")
                    print(f"[KNEE ] th_cmd={th3_cmd:+.3f} rad | q_rot={knee_data.q:+.3f} | dq={knee_data.dq:+.3f} | T={knee_data.temp:.1f} | err={knee_data.merror}")

                time.sleep(CTRL_DT)

        finally:
            # Safe fallback: relax motors
            try:
                for c, ser, data in [(hip_cmd, hip_ser, hip_data), (knee_cmd, knee_ser, knee_data)]:
                    c.kp = 0.0; c.kd = 0.0; c.tau = 0.0; c.dq = 0.0
                    ser.sendRecv(c, data)
            except Exception:
                pass
            print("Exited cleanly. Motors set to kp=0.")
            
if __name__ == "__main__":
    main()
