import time, sys, math
sys.path.append('../lib')
from unitree_actuator_sdk import *

# ===== User Tunables =====
MOTOR        = MotorType.GO_M8010_6
MOTOR_ID     = 0

# Target OUTPUT positions [rad] (edit as needed; must be within SOFT_MIN/MAX)
TARGETS_OUT  = [-1.0, 1.5, 0.0, -1.5, 1.0]

# Soft travel limits on OUTPUT side [rad] — tighten to your mechanism
SOFT_MIN     = -1.5
SOFT_MAX     =  1.5

# PD gains (start gentle)
KP           = 0.6
KD           = 0.1

# Motion profile on OUTPUT side
V_MAX        = 10      # rad/s
A_MAX        = 10      # rad/s^2


# Settle criteria & loop timing
TOL_OUT      = 0.01     # rad
VEL_TOL      = 1    # rad/s
TIMEOUT_S    = 4.0      # per target
CTRL_DT      = 0.002    # 2 ms
PRINT_DT     = 0.05     # s

# =========================

def clamp(x,a,b): return a if x<a else b if x>b else x

def main():
    serial = SerialPort('/dev/ttyUSB0')
    cmd = MotorCmd(); data = MotorData()

    # Allow motor to finish internal init
    time.sleep(0.6)

    gear = queryGearRatio(MOTOR)
    print(f"[Init] Gear ratio = {gear:.3f}")

    # Static fields
    data.motorType = MOTOR
    cmd.motorType  = MOTOR
    cmd.id         = MOTOR_ID

    # Enter FOC and prime gains
    cmd.mode = queryMotorMode(MOTOR, MotorMode.FOC)
    cmd.kp = KP; cmd.kd = KD; cmd.tau = 0.0; cmd.dq = 0.0
    serial.sendRecv(cmd, data)

    # Read current pose and begin there
    serial.sendRecv(cmd, data)
    q_out = data.q / gear
    dq_out = data.dq / gear
    print(f"[Init] Start at q_out={q_out:.3f} rad")

    # Prepare test list with clamping to soft limits
    targets = [clamp(t, SOFT_MIN, SOFT_MAX) for t in TARGETS_OUT]
    if targets != TARGETS_OUT:
        print("[Warn] Some targets were clamped to soft limits.")

    # Test loop over 5 positions
    for i, q_target in enumerate(targets, 1):
        print(f"\n=== Test {i}/{len(targets)}: target {q_target:.3f} rad ===")

        # Reset profile state
        v = 0.0
        t0 = time.time()
        last = t0
        last_print = t0

        settled_count = 0

        # Initialize desired to current to avoid first-step jump
        q_des = q_out
        v_des = 0.0

        while True:
            now = time.time()
            dt = now - last
            if dt <= 0: dt = CTRL_DT
            last = now

            # Update measurement
            serial.sendRecv(cmd, data)
            q_out = data.q / gear
            dq_out = data.dq / gear

            # Trapezoidal profile on OUTPUT side
            dx = q_target - q_des
            # choose accel direction based on remaining distance and current vel
            stop = (v*v)/(2*A_MAX) * (1 if v>=0 else -1)
            if (dx - stop) * (1 if dx>=0 else -1) > 0:
                a = A_MAX if dx>0 else -A_MAX
            else:
                a = -A_MAX if v>0 else A_MAX
            v = clamp(v + a*dt, -V_MAX, V_MAX)
            q_des += v*dt
            v_des = v

            # Snap when close to target
            if abs(q_target - q_des) < TOL_OUT and abs(v_des) < 0.1:
                q_des = q_target
                v_des = 0.0

            # Command rotor-side setpoints
            cmd.mode = queryMotorMode(MOTOR, MotorMode.FOC)  # harmless if already FOC
            cmd.q  = q_des * gear
            cmd.dq = v_des * gear
            cmd.kp = KP; cmd.kd = KD; cmd.tau = 0.0
            serial.sendRecv(cmd, data)

            # Check settle (at measured state)
            pos_err = q_target - q_out
            if abs(pos_err) < TOL_OUT and abs(dq_out) < VEL_TOL:
                settled_count += 1
            else:
                settled_count = 0

            # Print status
            if now - last_print >= PRINT_DT:
                print(f"q={q_out:+.3f} rad | dq={dq_out:+.3f} rad/s | "
                      f"des={q_des:+.3f} | v_des={v_des:+.2f} | "
                      f"err={pos_err:+.3f} | temp={data.temp:.1f}C | merror={data.merror}")
                last_print = now

            # Exit conditions
            if settled_count >= 8:
                print(f"✔ Settled at {q_out:.3f} rad (|err|≈{abs(pos_err):.4f})")
                break

            if now - t0 > TIMEOUT_S:
                print(f"⚠ Timeout at q={q_out:.3f} rad (err={pos_err:+.3f}); continuing to next.")
                break

            time.sleep(CTRL_DT)

    # Hold last position gently
    cmd.mode = queryMotorMode(MOTOR, MotorMode.FOC)
    cmd.q = data.q; cmd.dq = 0.0; cmd.kp = 0.01; cmd.kd = 0.01; cmd.tau = 0.0
    serial.sendRecv(cmd, data)
    print("\n[Done] Finished 5-position test. Holding last position.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[Exit] Interrupted by user.")
