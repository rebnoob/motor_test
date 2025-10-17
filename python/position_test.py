#!/usr/bin/env python3
import time
import sys
import math
import threading
import tkinter as tk
from tkinter import ttk
import numpy as np
from collections import deque

# ---- Unitree SDK -------------------------------------------------------------
sys.path.append("../lib")
from unitree_actuator_sdk import *  # noqa
# -----------------------------------------------------------------------------

# ================== Robot / actuator config ==================
MOTOR_IDS = [0, 1, 2]  # 0: hip roll (θ1), 1: hip pitch (θ2), 2: knee (θ3)
MOTOR_TYPE = MotorType.GO_M8010_6
CONTROL_MODE = queryMotorMode(MOTOR_TYPE, MotorMode.FOC)

# Per-joint gear ratios: rotor -> joint (output). Knee has extra 18:30 pulley.
GEAR = {0: 6.33, 1: 6.33, 2: 6.33 * (30.0 / 18.0)}  # ≈ {0:6.33, 1:6.33, 2:10.55}

# Optional direction for mounting/belt inversion (+1 same, -1 inverted)
DIR  = {0: +1.0, 1: +1.0, 2: +1.0}  # set DIR[2] = -1.0 if your pulley flips the knee

# PD gains (gentle)
KP = 0.08
KD = 0.08
DT = 0.005  # 200 Hz control loop

# Joint soft-limits (output-side radians)
LIMIT_T1 = math.radians(60)       # hip roll
LIMIT_T2 = math.radians(140)      # hip pitch
LIMIT_T3 = math.radians(160)      # knee
VEL_LIMIT_OUTPUT = math.radians(720)  # output-side vel clamp (converted per joint to rotor)

def clamp(x, lo, hi): return max(lo, min(hi, x))

# ================== Leg geometry (mm), +x fwd, +y left, +z DOWN ==================
l1, l2, l3 = 80.0, 190.0, 360.0

def fk_leg(theta1, theta2, theta3, s_y=+1):
    """Forward kinematics (output-side radians) -> (x,y,z) mm, also returns planar z'."""
    z_p = l2*np.cos(theta2) + l3*np.cos(theta2 + theta3)
    x   = l2*np.sin(theta2) + l3*np.sin(theta2 + theta3)
    y   = s_y*l1*np.cos(theta1) - z_p*np.sin(theta1)
    z   = s_y*l1*np.sin(theta1) + z_p*np.cos(theta1)
    return np.array([x, y, z]), z_p

def ik_leg(x, y, z, s_y=+1):
    """Inverse kinematics (mm) -> list[(t1,t2,t3)] in output-side radians."""
    sols = []
    r_yz = np.hypot(y, z)
    if r_yz < l1 - 1e-9:
        return sols
    zprime_mag = np.sqrt(max(r_yz**2 - l1**2, 0.0))
    for z_p in (zprime_mag, -zprime_mag):
        theta1 = np.arctan2(z, y) - np.arctan2(z_p, s_y*l1)
        r = np.hypot(x, z_p)
        c3 = (r*r - l2*l2 - l3*l3) / (2.0*l2*l3)
        if c3 < -1.0 or c3 > 1.0:
            continue
        for knee_sign in (1.0, -1.0):  # elbow-down/up
            theta3 = knee_sign * np.arccos(np.clip(c3, -1.0, 1.0))
            theta2 = np.arctan2(x, z_p) - np.arctan2(l3*np.sin(theta3), l2 + l3*np.cos(theta3))
            sols.append((theta1, theta2, theta3))
    return sols

def pick_closest_solution(prev_angles, candidates):
    """Prefer elbow-down first time; else pick nearest in joint space."""
    if not candidates:
        return None
    if prev_angles is None:
        elbow_down = [c for c in candidates if c[2] >= 0]
        return elbow_down[0] if elbow_down else candidates[0]
    pa = np.array(prev_angles)
    best, best_dist = None, float("inf")
    for c in candidates:
        ca = np.array(c)
        d = (ca - pa + np.pi) % (2*np.pi) - np.pi  # wrap diffs into [-π,π]
        dist = np.linalg.norm(d)
        if dist < best_dist:
            best_dist, best = dist, c
    return best

def decode_merror(bits: int):
    labels = ["OK","Overheat","Overcurrent","Overvoltage","Encoder Fault","Undervoltage","Winding Overheat","Reserved"]
    if bits == 0: return "OK"
    out = []
    for b in range(min(8, len(labels))):
        if (bits >> b) & 1:
            if b == 0 and bits != 0:  # skip OK if faults exist
                continue
            out.append(labels[b])
    return ",".join(out) if out else "OK"

# ================== GUI ==================
class MotorGUI:
    def __init__(self, root):
        self.root = root
        root.title("Unitree GO-M8010-6 — Angle / IK Control (with live feedback)")

        # --- SDK objects
        self.serial = SerialPort("/dev/ttyUSB0")
        self.cmds  = {i: MotorCmd()  for i in MOTOR_IDS}
        self.datas = {i: MotorData() for i in MOTOR_IDS}
        for i in MOTOR_IDS:
            self.datas[i].motorType = MOTOR_TYPE
            c = self.cmds[i]
            c.motorType = MOTOR_TYPE
            c.mode = CONTROL_MODE
            c.id = i
            c.kp = 0.0; c.kd = 0.0
            c.q  = 0.0; c.dq = 0.0; c.tau = 0.0

        # Warmup reads
        for _ in range(50):
            for i in MOTOR_IDS:
                self.serial.sendRecv(self.cmds[i], self.datas[i])
            time.sleep(0.002)

        # Rotor-side zeros captured here
        self.q_home = {i: self.datas[i].q for i in MOTOR_IDS}

        # --- GUI state
        self.enabled = tk.BooleanVar(value=False)
        self.mode = tk.StringVar(value="IK")  # "Angles" or "IK"
        self.s_y = tk.IntVar(value=+1)        # +1 left, -1 right
        self.prev_solution = deque(maxlen=1)  # last chosen IK solution

        # Angle targets (output-side radians)
        self.t1 = tk.DoubleVar(value=0.0)
        self.t2 = tk.DoubleVar(value=0.0)
        self.t3 = tk.DoubleVar(value=0.0)

        # IK targets (mm)
        self.x = tk.DoubleVar(value=180.0)
        self.y = tk.DoubleVar(value=90.0)
        self.z = tk.DoubleVar(value=180.0)

        # Live feedback vars
        self.theta_live = {i: tk.StringVar(value="0.000") for i in MOTOR_IDS}  # output-side rad
        self.end_eff_xyz = {k: tk.StringVar(value="0.0") for k in ("x","y","z")}
        self.temp_live  = {i: tk.StringVar(value="0") for i in MOTOR_IDS}
        self.err_live   = {i: tk.StringVar(value="OK") for i in MOTOR_IDS}

        # --- Build layout
        frm = ttk.Frame(root, padding=12); frm.grid(sticky="nsew")
        root.columnconfigure(0, weight=1); root.rowconfigure(0, weight=1)

        ttk.Label(frm, text="GO-M8010-6 — Angle / IK Control", font=("Segoe UI", 14, "bold")).grid(row=0, column=0, columnspan=8, sticky="w")

        ttk.Checkbutton(frm, text="Enable", variable=self.enabled).grid(row=1, column=0, sticky="w", pady=(6,8))
        ttk.Button(frm, text="Set Current Pose as Zero", command=self.rezero).grid(row=1, column=1, sticky="w", padx=6)

        # Leg side
        ttk.Label(frm, text="Leg Side:").grid(row=1, column=2, sticky="e")
        self.side_choice = tk.StringVar(value="Left (+1)")
        side_box = ttk.Combobox(frm, values=["Left (+1)","Right (-1)"], width=12,
                                state="readonly", textvariable=self.side_choice)
        side_box.grid(row=1, column=3, sticky="w")
        def on_side_change(*_):
            self.s_y.set(+1 if "Left" in self.side_choice.get() else -1)
        self.side_choice.trace_add("write", on_side_change)

        # Mode select
        ttk.Label(frm, text="Mode:").grid(row=1, column=4, sticky="e")
        ttk.Combobox(frm, values=["IK","Angles"], textvariable=self.mode, width=8, state="readonly").grid(row=1, column=5, sticky="w")

        # Controls frame (switches contents by mode)
        self.ctrl_frame = ttk.Frame(frm); self.ctrl_frame.grid(row=2, column=0, columnspan=8, sticky="we", pady=(6,10))
        for c in range(8): frm.columnconfigure(c, weight=1)
        self.build_controls()

        # Feedback block
        ttk.Separator(frm).grid(row=3, column=0, columnspan=8, sticky="we", pady=6)
        ttk.Label(frm, text="Live Feedback", font=("Segoe UI", 11, "bold")).grid(row=4, column=0, sticky="w")

        hdr = ("Joint", "θ (rad, output)", "Temp (°C)", "Err", "End-effector (mm)")
        for c,h in enumerate(hdr):
            ttk.Label(frm, text=h).grid(row=5, column=c, sticky="w")
        for r, mid in enumerate(MOTOR_IDS, start=6):
            ttk.Label(frm, text=f"J{mid}").grid(row=r, column=0, sticky="w")
            ttk.Label(frm, textvariable=self.theta_live[mid], width=14).grid(row=r, column=1, sticky="w")
            ttk.Label(frm, textvariable=self.temp_live[mid], width=8).grid(row=r, column=2, sticky="w")
            ttk.Label(frm, textvariable=self.err_live[mid],  width=22).grid(row=r, column=3, sticky="w")
        # end-effector row (x,y,z)
        ee_row = 6
        ttk.Label(frm, text="x=").grid(row=ee_row,   column=4, sticky="e"); ttk.Label(frm, textvariable=self.end_eff_xyz["x"], width=9).grid(row=ee_row,   column=5, sticky="w")
        ttk.Label(frm, text="y=").grid(row=ee_row+1, column=4, sticky="e"); ttk.Label(frm, textvariable=self.end_eff_xyz["y"], width=9).grid(row=ee_row+1, column=5, sticky="w")
        ttk.Label(frm, text="z=").grid(row=ee_row+2, column=4, sticky="e"); ttk.Label(frm, textvariable=self.end_eff_xyz["z"], width=9).grid(row=ee_row+2, column=5, sticky="w")

        # Demo motion box (start/stop routine in IK)
        self._demo_running = False
        self._demo_thread  = None
        self._demo_status  = tk.StringVar(value="Idle")
        self._demo_type    = tk.StringVar(value="Circle (x–z)")
        self._demo_R       = tk.DoubleVar(value=60.0)     # mm
        self._demo_Hz      = tk.DoubleVar(value=0.25)     # Hz
        self._build_demo_box(frm, row=9)

        # Threads/timers
        self._running = True
        self._rezeroing = False
        self.ctrl_thread = threading.Thread(target=self.control_loop, daemon=True); self.ctrl_thread.start()
        self._ui_job = self.root.after(100, self.update_ui)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Rebuild controls when mode changes
        def on_mode_change(*_):
            for w in self.ctrl_frame.winfo_children(): w.destroy()
            self.build_controls()
        self.mode.trace_add("write", on_mode_change)

    # ---------- UI builders
    def build_controls(self):
        if self.mode.get() == "Angles":
            self._build_angle_controls()
        else:
            self._build_ik_controls()

    def _build_angle_controls(self):
        f = self.ctrl_frame
        ttk.Label(f, text="Angle Control (output-side radians)").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(f, text="θ1 (hip roll)").grid(row=1, column=0, sticky="w")
        ttk.Scale(f, from_=-LIMIT_T1, to=+LIMIT_T1, variable=self.t1, orient="horizontal", length=420).grid(row=1, column=1, columnspan=2, sticky="we")
        ttk.Label(f, text="θ2 (hip pitch)").grid(row=2, column=0, sticky="w")
        ttk.Scale(f, from_=-LIMIT_T2, to=+LIMIT_T2, variable=self.t2, orient="horizontal", length=420).grid(row=2, column=1, columnspan=2, sticky="we")
        ttk.Label(f, text="θ3 (knee)").grid(row=3, column=0, sticky="w")
        ttk.Scale(f, from_=-LIMIT_T3, to=+LIMIT_T3, variable=self.t3, orient="horizontal", length=420).grid(row=3, column=1, columnspan=2, sticky="we")

    def _build_ik_controls(self):
        f = self.ctrl_frame
        ttk.Label(f, text="IK Control (mm, hip frame; +x fwd, +y left, +z DOWN)").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(f, text="x").grid(row=1, column=0, sticky="w")
        ttk.Scale(f, from_=-50.0, to=520.0, variable=self.x, orient="horizontal", length=420).grid(row=1, column=1, columnspan=2, sticky="we")
        ttk.Label(f, text="y").grid(row=2, column=0, sticky="w")
        ttk.Scale(f, from_=-260.0, to=260.0, variable=self.y, orient="horizontal", length=420).grid(row=2, column=1, columnspan=2, sticky="we")
        ttk.Label(f, text="z").grid(row=3, column=0, sticky="w")
        ttk.Scale(f, from_=-30.0, to=520.0, variable=self.z, orient="horizontal", length=420).grid(row=3, column=1, columnspan=2, sticky="we")

    # ---------- Demo box
    def _build_demo_box(self, parent, row):
        box = ttk.Labelframe(parent, text="Demo Motion (IK)", padding=8)
        box.grid(row=row, column=0, columnspan=8, sticky="we", pady=(8,0))
        for c in range(8): parent.columnconfigure(c, weight=1)

        ttk.Label(box, text="Type:").grid(row=0, column=0, sticky="e")
        ttk.Combobox(
            box,
            values=["Circle (x–z)", "Bounce (z)", "Figure-8 (x–z)"],
            textvariable=self._demo_type,
            width=18, state="readonly"
        ).grid(row=0, column=1, sticky="w", padx=(4,12))

        ttk.Label(box, text="Status:").grid(row=0, column=2, sticky="e")
        ttk.Label(box, textvariable=self._demo_status).grid(row=0, column=3, columnspan=2, sticky="w")

        ttk.Label(box, text="Radius/Amplitude (mm):").grid(row=1, column=0, sticky="e")
        ttk.Scale(box, from_=5.0, to=160.0, variable=self._demo_R, orient="horizontal", length=260).grid(row=1, column=1, columnspan=2, sticky="we")

        ttk.Label(box, text="Frequency (Hz):").grid(row=1, column=3, sticky="e")
        ttk.Scale(box, from_=0.05, to=1.50, variable=self._demo_Hz, orient="horizontal", length=220).grid(row=1, column=4, sticky="we")

        ttk.Button(box, text="Start", command=self._start_demo).grid(row=2, column=0, sticky="we", pady=(6,0))
        ttk.Button(box, text="Stop",  command=self._stop_demo).grid(row=2, column=1, sticky="we", pady=(6,0))

    def _start_demo(self):
        if self._demo_running:
            return
        self.mode.set("IK")
        if not self.enabled.get():
            self.enabled.set(True)
        jt = self._measured_joint_angles()
        if jt is None:
            self._demo_status.set("No joint read")
            return
        cx, cy, cz = fk_leg(*jt, s_y=self.s_y.get())[0]
        self._demo_center = (float(cx), float(cy), float(cz))
        self._demo_running = True
        self._demo_status.set("Running")
        self._demo_thread = threading.Thread(target=self._demo_loop, daemon=True)
        self._demo_thread.start()

    def _stop_demo(self):
        self._demo_running = False
        self._demo_status.set("Idle")

    def _demo_loop(self):
        # Slider bounds (keep path inside)
        X_MIN, X_MAX = -50.0, 520.0
        Y_MIN, Y_MAX = -260.0, 260.0
        Z_MIN, Z_MAX = -30.0,  520.0

        cx, cy, cz = self._demo_center
        cx = clamp(cx, X_MIN, X_MAX)
        cy = clamp(cy, Y_MIN, Y_MAX)
        cz = clamp(cz, Z_MIN, Z_MAX)

        R  = float(self._demo_R.get())
        Hz = max(0.05, float(self._demo_Hz.get()))

        t0 = time.monotonic()
        while self._demo_running and self._running:
            t = time.monotonic() - t0
            w = 2.0 * math.pi * Hz
            typ = self._demo_type.get()

            if typ.startswith("Circle"):
                x = cx + R*math.cos(w*t); y = cy; z = cz + R*math.sin(w*t)
            elif typ.startswith("Bounce"):
                x = cx; y = cy; z = cz + R*math.sin(w*t)
            else:  # Figure-8
                x = cx + R*math.sin(w*t); y = cy; z = cz + R*math.sin(2.0*w*t)

            x = clamp(x, X_MIN, X_MAX)
            y = clamp(y, Y_MIN, Y_MAX)
            z = clamp(z, Z_MIN, Z_MAX)

            # Push into IK targets (simple Tk variable set is fine for scalars)
            self.x.set(x); self.y.set(y); self.z.set(z)
            time.sleep(0.01)  # ~100 Hz

        self._demo_status.set("Idle")

    # ---------- Actions
    def rezero(self):
        self._rezeroing = True
        try:
            # fresh reads + average a bit
            for _ in range(3):
                for i in MOTOR_IDS:
                    self.serial.sendRecv(self.cmds[i], self.datas[i])
                time.sleep(0.002)
            self.q_home = {i: self.datas[i].q for i in MOTOR_IDS}
            # zero UI targets
            self.t1.set(0.0); self.t2.set(0.0); self.t3.set(0.0)
            # set IK to current FK from measured pose
            jt = self._measured_joint_angles()
            if jt is not None:
                (t1,t2,t3) = jt
                ee,_ = fk_leg(t1,t2,t3, s_y=self.s_y.get())
                self.x.set(float(ee[0])); self.y.set(float(ee[1])); self.z.set(float(ee[2]))
                self.prev_solution.clear(); self.prev_solution.append((t1,t2,t3))
        finally:
            self._rezeroing = False
        print("Re-zeroed; targets synced to current pose.")

    # ---------- Helpers
    def _measured_joint_angles(self):
        """Read measured rotor q, convert to output-side angles relative to home per joint."""
        try:
            t = []
            for i in MOTOR_IDS:
                q_rotor = self.datas[i].q - self.q_home[i]
                # joint = DIR[i] * (rotor / GEAR[i])
                t.append(DIR[i] * (q_rotor / GEAR[i]))
            return tuple(t)  # (t1,t2,t3)
        except Exception:
            return None

    def _command_joint_targets(self, t1, t2, t3):
        """Send PD command in rotor space from desired output-side joint angles."""
        # Clamp (output-side)
        t1 = clamp(t1, -LIMIT_T1, +LIMIT_T1)
        t2 = clamp(t2, -LIMIT_T2, +LIMIT_T2)
        t3 = clamp(t3, -LIMIT_T3, +LIMIT_T3)
        t_des = {0: t1, 1: t2, 2: t3}

        # Desired absolute rotor angles per joint:
        # rotor_abs = q_home + (DIR[i] * joint_des * GEAR[i])
        q_des_abs_rotor = {
            i: self.q_home[i] + (DIR[i] * t_des[i] * GEAR[i])
            for i in MOTOR_IDS
        }

        # Per-joint rotor velocity limits
        vel_lim_rotor = {i: VEL_LIMIT_OUTPUT * abs(GEAR[i]) for i in MOTOR_IDS}

        for i in MOTOR_IDS:
            q_meas = self.datas[i].q
            err = q_des_abs_rotor[i] - q_meas
            dq_des = clamp(5.0 * err, -vel_lim_rotor[i], +vel_lim_rotor[i])

            c = self.cmds[i]
            c.mode = CONTROL_MODE
            c.kp = KP; c.kd = KD
            c.q  = q_des_abs_rotor[i]
            c.dq = dq_des
            c.tau = 0.0  # no DC bias to avoid creep
            self.serial.sendRecv(c, self.datas[i])

    # ---------- Loops
    def update_ui(self):
        # live joint angles from rotor feedback
        jt = self._measured_joint_angles()
        if jt is not None:
            (t1,t2,t3) = jt
            self.theta_live[0].set(f"{t1:.3f}")
            self.theta_live[1].set(f"{t2:.3f}")
            self.theta_live[2].set(f"{t3:.3f}")
            ee,_ = fk_leg(t1,t2,t3, s_y=self.s_y.get())
            self.end_eff_xyz["x"].set(f"{ee[0]:.1f}")
            self.end_eff_xyz["y"].set(f"{ee[1]:.1f}")
            self.end_eff_xyz["z"].set(f"{ee[2]:.1f}")

        # temps/errors
        for i in MOTOR_IDS:
            self.temp_live[i].set(f"{getattr(self.datas[i], 'temp', 0)}")
            self.err_live[i].set(decode_merror(getattr(self.datas[i], 'merror', 0)))

        if self._running:
            self._ui_job = self.root.after(100, self.update_ui)

    def control_loop(self):
        last = time.time()
        try:
            while self._running:
                now = time.time()
                if now - last < DT:
                    time.sleep(0.0005)
                    continue
                last = now

                # keep comms alive & datas fresh when disabled or rezeroing
                if not self.enabled.get() or self._rezeroing:
                    for i in MOTOR_IDS:
                        c = self.cmds[i]
                        c.mode = CONTROL_MODE
                        c.kp = 0.0; c.kd = 0.0
                        c.q  = self.datas[i].q
                        c.dq = 0.0; c.tau = 0.0
                        self.serial.sendRecv(c, self.datas[i])
                    continue

                if self.mode.get() == "Angles":
                    t1 = self.t1.get(); t2 = self.t2.get(); t3 = self.t3.get()
                    self._command_joint_targets(t1, t2, t3)
                else:
                    # IK: compute desired joint angles from (x,y,z)
                    x, y, z = self.x.get(), self.y.get(), self.z.get()
                    cands = ik_leg(x, y, z, s_y=self.s_y.get())
                    prev = self.prev_solution[0] if len(self.prev_solution) else None
                    sol = pick_closest_solution(prev, cands)
                    if sol is None:
                        # no solution; hold current softly
                        for i in MOTOR_IDS:
                            c = self.cmds[i]
                            c.mode = CONTROL_MODE
                            c.kp = 0.0; c.kd = 0.0
                            c.q  = self.datas[i].q
                            c.dq = 0.0; c.tau = 0.0
                            self.serial.sendRecv(c, self.datas[i])
                        continue
                    self.prev_solution.clear(); self.prev_solution.append(sol)
                    t1, t2, t3 = sol
                    self._command_joint_targets(t1, t2, t3)
        finally:
            # safe shutdown: zero torque
            for _ in range(10):
                for i in MOTOR_IDS:
                    c = self.cmds[i]
                    c.kp = 0.0; c.kd = 0.0
                    c.dq = 0.0; c.tau = 0.0
                    self.serial.sendRecv(c, self.datas[i])
                time.sleep(0.01)

    def on_close(self):
        # stop demo & control loops, close nicely
        self._demo_running = False
        self._running = False
        if self._ui_job is not None:
            try: self.root.after_cancel(self._ui_job)
            except Exception: pass
        self.root.after(200, self.root.destroy)

# ================== Main ==================
def main():
    root = tk.Tk()
    MotorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
