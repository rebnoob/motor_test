#!/usr/bin/env python3
import time, math, sys, select
sys.path.append('../lib')
from unitree_actuator_sdk import *

# ===================== Geometry & Hardware =====================
L2 = 0.115   # thigh [m]
L3 = 0.12   # shank [m]

# Unitree GO-M8010-6 reductions (rotor -> actuator output)
G_HIP  = 6.33
G_KNEE = 6.33

# Knee belt (Nm on actuator, Nj on knee). Default: 19T actuator, 30T knee.
Nm = 19
Nj = 30

# Ports
HIP_PORT  = '/dev/ttyUSB0'  # hip motor
KNEE_PORT = '/dev/ttyUSB1'  # knee motor

# ===================== Control Gains & Limits =====================
# Rotor-side PD gains (tune carefully)
KP_HIP,  KD_HIP  = 0.05, 0.01
KP_KNEE, KD_KNEE = 0.05, 0.01

# Joint limits (output-side, radians)
HIP_MIN,  HIP_MAX  = math.radians(-120), math.radians(+120)
# Knee: 0 = straight, positive = bending
KNEE_MIN, KNEE_MAX = math.radians(+5),  math.radians(+155)

# Sign & zero offsets (rotor-side) — updated at runtime by zeroing
HIP_SIGN,  KNEE_SIGN  = +1.0, +1.0
HIP_ZERO,  KNEE_ZERO  = 0.0,  0.0

# Timing
CTRL_HZ = 1000.0
CTRL_DT = 1.0 / CTRL_HZ
PRINT_EVERY = 50

# Desired rotor velocity cap (feed-forward clamp)
DQ_DES_MAX = 12.0  # rad/s

# ===================== Workspace Safety =====================
# Keep away from singular rings and within a safe box
R_MIN = abs(L2 - L3) + 0.01   # [m] min radius with margin
R_MAX = (L2 + L3) - 0.01      # [m] max radius with margin

Y_MIN = -0.2                  # [m] y bounds (adjust to your rig)
Y_MAX = +0.3
Z_MIN = -0.22
Z_MAX = +0.2

def clamp(v, vmin, vmax): return max(vmin, min(vmax, v))

def clamp_workspace(y, z):
    # box clamp
    y = clamp(y, Y_MIN, Y_MAX)
    z = clamp(z, Z_MIN, Z_MAX)
    # radial clamp (stay away from straight-leg / fully-folded singularities)
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
def ik_2link_yz(y, z, l2=L2, l3=L3, elbow='down'):
    r2 = y*y + z*z
    r  = math.sqrt(r2)
    if r > l2 + l3 or r < abs(l2 - l3):
        raise ValueError("Target out of reach")
    c3 = (r2 - l2*l2 - l3*l3) / (2.0*l2*l3)
    c3 = max(-1.0, min(1.0, c3))
    s3_mag = math.sqrt(max(0.0, 1.0 - c3*c3))
    s3 = s3_mag if elbow.lower() == 'down' else -s3_mag
    th3 = math.atan2(s3, c3)                                 # knee (relative)
    th2 = math.atan2(z, y) - math.atan2(l3*s3, l2 + l3*c3)   # hip (absolute from +y toward +z)
    return th2, th3

def ik_best(y, z, last_th2=None, last_th3=None):
    cands = [ik_2link_yz(y, z, elbow='down'), ik_2link_yz(y, z, elbow='up')]
    if last_th2 is None or last_th3 is None:
        return cands[0]
    return min(cands, key=lambda th: (th[0]-last_th2)**2 + (th[1]-last_th3)**2)

def clamp_joint_limits(th2, th3):
    """Clamp hip and knee joint angles to safe limits."""
    return (
        max(HIP_MIN, min(HIP_MAX, th2)),
        max(KNEE_MIN, min(KNEE_MAX, th3))
    )

def fk_yz(th2, th3, l2=L2, l3=L3):
    """Forward kinematics to foot in y–z plane."""
    y = l2*math.cos(th2) + l3*math.cos(th2 + th3)
    z = l2*math.sin(th2) + l3*math.sin(th2 + th3)
    return y, z

# ===================== Rotor<->Joint mapping =====================
def hip_joint_to_rotor(theta2_joint):
    return HIP_SIGN * (theta2_joint * G_HIP) + HIP_ZERO

def knee_joint_to_rotor(theta3_joint):
    # theta_joint = theta_act_out * (Nm/Nj)  =>  theta_act_out = theta_joint * (Nj/Nm)
    theta_act_out = theta3_joint * (Nj / Nm)
    return KNEE_SIGN * (theta_act_out * G_KNEE) + KNEE_ZERO

def hip_rotor_to_joint(q_rotor):
    """Inverse of hip_joint_to_rotor."""
    return (q_rotor - HIP_ZERO) / (G_HIP * HIP_SIGN)

def knee_rotor_to_joint(q_rotor):
    """Inverse of knee_joint_to_rotor."""
    act_out = (q_rotor - KNEE_ZERO) / (G_KNEE * KNEE_SIGN)
    return act_out * (Nm / Nj)  # actuator->joint

# ===================== Trajectory: Rectangle (anchor = bottom-right) =====================
# Box size and speed (BOTTOM-RIGHT corner chosen at runtime)
BOX_WY = 0.25   # width in y
BOX_HZ = 0.15   # height in z
BOX_RC = 0.005  # small fillet helps at corners (set 0.0 if you want sharp corners)
BOX_V  = 5.80   # path speed [m/s]

def rounded_square_points(br_y, br_z, wy=BOX_WY, hz=BOX_HZ, rc=BOX_RC, v=BOX_V, dt=CTRL_DT):
    """
    Generate a (possibly rounded) rectangular path whose
    BOTTOM-RIGHT corner is at (br_y, br_z). The returned list starts at (br_y, br_z).
    """
    # Limit fillet radius
    rc = min(rc, max(0.0, min(wy, hz)/2.0 - 0.005))

    # Convert bottom-right anchor -> center coords used for corner math
    # From previous definitions: BR = (yc + wy/2 - rc, zc - hz/2 + rc)
    yc = br_y - wy/2 + rc
    zc = br_z + hz/2 - rc

    # Corner centers (consistent with previous convention)
    top_left  = (yc - wy/2 + rc, zc + hz/2 - rc)
    top_right = (yc + wy/2 - rc, zc + hz/2 - rc)
    bot_right = (yc + wy/2 - rc, zc - hz/2 + rc)  # == (br_y, br_z)
    bot_left  = (yc - wy/2 + rc, zc - hz/2 + rc)

    points = []

    def sample_line(p0, p1):
        y0, z0 = p0; y1, z1 = p1
        dist = math.hypot(y1 - y0, z1 - z0)
        if dist < 1e-9:
            return
        steps = max(1, int(dist / (v*dt)))
        for i in range(1, steps+1):
            a = i / steps
            points.append((y0 + a*(y1 - y0), z0 + a*(z1 - z0)))

    def sample_arc(c, r, a0, a1):
        if r <= 1e-9:
            return
        da = a1 - a0
        arc_len = abs(da) * r
        if arc_len < 1e-9:
            return
        steps = max(1, int(arc_len / (v*dt)))
        for i in range(1, steps+1):
            a = a0 + da * (i / steps)
            points.append((c[0] + r*math.cos(a), c[1] + r*math.sin(a)))

    # Name corners
    TL, TR, BR, BL = top_left, top_right, bot_right, bot_left

    # Start exactly at the current point (bottom-right)
    points.append((BR[0], BR[1]))

    # Go around counter-clockwise starting from BR:
    # 1) Bottom straight: BR -> BL
    sample_line(BR, BL)
    # 2) Bottom-left arc: (-π/2) -> (-π)
    sample_arc(BL, rc, -math.pi/2, -math.pi)
    # 3) Left straight: BL -> TL
    sample_line(BL, TL)
    # 4) Top-left arc: (-π) -> (-π/2)
    sample_arc(TL, rc, -math.pi, -math.pi/2)
    # 5) Top straight: TL -> TR
    sample_line(TL, TR)
    # 6) Top-right arc: (π/2) -> 0
    sample_arc(TR, rc, math.pi/2, 0.0)
    # 7) Right straight: TR -> BR
    sample_line(TR, BR)
    # 8) Bottom-right arc: 0 -> (-π/2) to close curvature (optional)
    sample_arc(BR, rc, 0.0, -math.pi/2)

    # Safety clamp every waypoint
    points = [clamp_workspace(y, z) for (y, z) in points]
    return points

# -------- Arc-length path scheduler (constant speed along path) --------
def build_cumdist(pts):
    cd = [0.0]
    for i in range(1, len(pts)):
        dy = pts[i][0] - pts[i-1][0]
        dz = pts[i][1] - pts[i-1][1]
        cd.append(cd[-1] + math.hypot(dy, dz))
    return cd

def sample_path(pts, cd, s):
    L = cd[-1]
    if L <= 1e-9:
        return pts[0]
    s = s % L
    # linear scan (OK for a few hundred points)
    i = 1
    while i < len(cd) and cd[i] < s:
        i += 1
    i = min(i, len(cd)-1)
    s0, s1 = cd[i-1], cd[i]
    t = 0.0 if s1 == s0 else (s - s0)/(s1 - s0)
    y = pts[i-1][0] + t*(pts[i][0] - pts[i-1][0])
    z = pts[i-1][1] + t*(pts[i][1] - pts[i-1][1])
    return y, z

# Optional EMA smoothing of target (set ALPHA=0.0 to disable)
ALPHA = 0.15

# ===================== Zeroing & Input =====================
def relax_and_read(ser, cmd, data):
    """Relax motor briefly and read rotor angle."""
    cmd.q = 0.0; cmd.dq = 0.0; cmd.kp = 0.0; cmd.kd = 0.0; cmd.tau = 0.0
    ser.sendRecv(cmd, data)
    time.sleep(0.002)
    ser.sendRecv(cmd, data)
    return data.q

def zero_current_pose(hip_ser, hip_cmd, hip_data, knee_ser, knee_cmd, knee_data, which="both"):
    """Set current rotor readings as zeros so current pose becomes joint = 0."""
    global HIP_ZERO, KNEE_ZERO
    for cmd, ser, data in [(hip_cmd, hip_ser, hip_data), (knee_cmd, knee_ser, knee_data)]:
        cmd.q = 0.0; cmd.dq = 0.0; cmd.kp = 0.0; cmd.kd = 0.0; cmd.tau = 0.0
        ser.sendRecv(cmd, data)
    time.sleep(0.01)
    hip_q  = relax_and_read(hip_ser, hip_cmd, hip_data)
    knee_q = relax_and_read(knee_ser, knee_cmd, knee_data)
    if which in ("both", "hip"):  HIP_ZERO  = hip_q
    if which in ("both", "knee"): KNEE_ZERO = knee_q
    print(f"[ZERO] which={which} | HIP_ZERO={HIP_ZERO:+.4f} rad | KNEE_ZERO={KNEE_ZERO:+.4f} rad")

def poll_keyboard():
    """Non-blocking line input; returns a lowercased command or None."""
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline().strip().lower()
        return line if line else None
    return None

# ===================== Main Control Loop =====================
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

    print("[INFO] Controls: 'z' (zero both), 'h' (zero hip), 'k' (zero knee), 'b' (start box @ current foot as bottom-right), 's' (stop/hold) + ENTER")

    # --- State
    mode = "idle"          # 'idle' (hold), 'box' (rect path)
    hold_hip_q = None      # rotor hold setpoints in idle
    hold_knee_q = None

    # Path storage (regenerated when 'b' is pressed)
    path = []
    cumdist = None
    s_cursor = 0.0
    V_ALONG  = BOX_V  # m/s along the path

    # For continuity & feed-forward
    last_th2 = None
    last_th3 = None
    hip_q_des_prev  = None
    knee_q_des_prev = None

    # Timing instrumentation
    loop = 0
    t_prev = time.time()
    overruns = 0

    # EMA state
    y_tgt_s = None
    z_tgt_s = None

    while True:
        # --- Keyboard commands
        key = poll_keyboard()
        if key in ("z", "h", "k"):
            which = "both" if key == "z" else ("hip" if key == "h" else "knee")
            zero_current_pose(hip_ser, hip_cmd, hip_data, knee_ser, knee_cmd, knee_data, which=which)
            hip_q_des_prev = None
            knee_q_des_prev = None
            # Refresh hold setpoints to current measured rotors
            hold_hip_q  = hip_data.q
            hold_knee_q = knee_data.q
            mode = "idle"
            print("[MODE] idle (after zero)")

        elif key == "b":
            # Compute current foot position as the BOTTOM-RIGHT anchor of the box
            th2_now = hip_rotor_to_joint(hip_data.q)
            th3_now = knee_rotor_to_joint(knee_data.q)
            br_y, br_z = fk_yz(th2_now, th3_now)
            br_y, br_z = clamp_workspace(br_y, br_z)
            # Generate a fresh path anchored at current foot position (BR corner)
            path = rounded_square_points(br_y, br_z)
            if len(path) < 10:
                print("[WARN] Generated path too short; check BOX_* settings.")
                mode = "idle"
            else:
                cumdist = build_cumdist(path)
                s_cursor = 0.0
                hip_q_des_prev = None
                knee_q_des_prev = None
                last_th2, last_th3 = th2_now, th3_now  # continuity from current pose
                # initialize EMA to first target
                y0, z0 = path[0]
                y_tgt_s, z_tgt_s = y0, z0
                mode = "box"
                print(f"[MODE] box @ bottom-right y={br_y:+.3f}, z={br_z:+.3f} (points={len(path)}, length={cumdist[-1]:.3f} m)")

        elif key == "s":
            hold_hip_q  = hip_data.q
            hold_knee_q = knee_data.q
            hip_q_des_prev = None
            knee_q_des_prev = None
            mode = "idle"
            print("[MODE] idle (stop)")

        # --- Build desired rotor setpoints based on mode
        if mode == "idle":
            # capture current rotor as hold target on first pass
            if hold_hip_q is None or hold_knee_q is None:
                hold_hip_q  = hip_data.q
                hold_knee_q = knee_data.q
            hip_q_des, knee_q_des = hold_hip_q, hold_knee_q
            hip_dq_des, knee_dq_des = 0.0, 0.0

            # Desired foot (for print): convert desired rotors -> desired joints -> FK
            th2_des = hip_rotor_to_joint(hip_q_des)
            th3_des = knee_rotor_to_joint(knee_q_des)
            y_des, z_des = fk_yz(th2_des, th3_des)

        elif mode == "box":
            # Constant-speed progress along path (arc-length cursor)
            s_cursor += V_ALONG * CTRL_DT
            y_raw, z_raw = sample_path(path, cumdist, s_cursor)

            # Optional small EMA smoothing
            if y_tgt_s is None:
                y_tgt_s, z_tgt_s = y_raw, z_raw
            else:
                y_tgt_s = (1.0 - ALPHA) * y_tgt_s + ALPHA * y_raw
                z_tgt_s = (1.0 - ALPHA) * z_tgt_s + ALPHA * z_raw

            y_des, z_des = y_tgt_s, z_tgt_s

            # IK → joints
            try:
                th2, th3 = ik_best(y_des, z_des, last_th2, last_th3)
            except ValueError:
                y_des, z_des = clamp_workspace(y_des, z_des)
                th2, th3 = ik_best(y_des, z_des, last_th2, last_th3)

            th2, th3 = clamp_joint_limits(th2, th3)
            last_th2, last_th3 = th2, th3

            hip_q_des  = hip_joint_to_rotor(th2)
            knee_q_des = knee_joint_to_rotor(th3)

            # finite-difference desired velocity (feed-forward)
            if hip_q_des_prev is None:
                hip_dq_des = 0.0
                knee_dq_des = 0.0
            else:
                hip_dq_des  = clamp((hip_q_des  - hip_q_des_prev)  / CTRL_DT, -DQ_DES_MAX, DQ_DES_MAX)
                knee_dq_des = clamp((knee_q_des - knee_q_des_prev) / CTRL_DT, -DQ_DES_MAX, DQ_DES_MAX)

        else:
            # Fallback to idle
            hip_q_des, knee_q_des = hip_data.q, knee_data.q
            hip_dq_des, knee_dq_des = 0.0, 0.0
            th2_des = hip_rotor_to_joint(hip_q_des)
            th3_des = knee_rotor_to_joint(knee_q_des)
            y_des, z_des = fk_yz(th2_des, th3_des)

        # Remember last desired (for feed-forward next tick if in box)
        hip_q_des_prev, knee_q_des_prev = hip_q_des, knee_q_des

        # --- Send PD commands
        hip_cmd.q,  hip_cmd.dq  = hip_q_des,  hip_dq_des
        hip_cmd.kp, hip_cmd.kd  = KP_HIP,     KD_HIP
        hip_cmd.tau             = 0.0

        knee_cmd.q,  knee_cmd.dq  = knee_q_des, knee_dq_des
        knee_cmd.kp, knee_cmd.kd  = KP_KNEE,     KD_KNEE
        knee_cmd.tau              = 0.0

        hip_ser.sendRecv(hip_cmd, hip_data)
        knee_ser.sendRecv(knee_cmd, knee_data)

        # --- Compute measured foot position via FK
        th2_meas = hip_rotor_to_joint(hip_data.q)
        th3_meas = knee_rotor_to_joint(knee_data.q)
        y_meas, z_meas = fk_yz(th2_meas, th3_meas)

        # --- Status
        if (loop % PRINT_EVERY) == 0:
            print(f"\n[MODE] {mode}")
            print(f"[FOOT] desired: y={y_des:+.3f} z={z_des:+.3f} | measured: y={y_meas:+.3f} z={z_meas:+.3f}")
            print(f"[HIP ] q_des(rot): {hip_q_des:+.3f} | dq_des: {hip_dq_des:+.2f} | q_meas: {hip_data.q:+.3f} | dq_meas: {hip_data.dq:+.3f} | T:{hip_data.temp:.1f} | err:{hip_data.merror}")
            print(f"[KNEE] q_des(rot): {knee_q_des:+.3f} | dq_des: {knee_dq_des:+.2f} | q_meas: {knee_data.q:+.3f} | dq_meas: {knee_data.dq:+.3f} | T:{knee_data.temp:.1f} | err:{knee_data.merror}")

        loop += 1
        time.sleep(CTRL_DT)

        # ---- Timing instrumentation (detect overruns) ----
        t_now = time.time()
        dt_loop = t_now - t_prev
        t_prev = t_now
        if dt_loop > 1.5 * CTRL_DT:
            overruns += 1
            if overruns % 50 == 0:
                print(f"[TIMING] loop dt={dt_loop*1e3:.2f} ms (> {1.5*CTRL_DT*1e3:.2f} ms); overruns={overruns}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Leave relaxed
        try:
            hip_cmd = MotorCmd(); knee_cmd = MotorCmd()
            hip_cmd.motorType = knee_cmd.motorType = MotorType.GO_M8010_6
            hip_cmd.mode = knee_cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
            hip_cmd.id = knee_cmd.id = 0
            for c in (hip_cmd, knee_cmd):
                c.q = 0.0; c.dq = 0.0; c.kp = 0.0; c.kd = 0.0; c.tau = 0.0
            SerialPort(HIP_PORT).sendRecv(hip_cmd, MotorData())
            SerialPort(KNEE_PORT).sendRecv(knee_cmd, MotorData())
        except Exception:
            pass
        print("\n[INFO] Stopped by user.")
