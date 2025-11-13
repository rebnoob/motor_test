# gui_leg.py
import os, sys, time, threading, tkinter as tk
from tkinter import ttk
import math
import numpy as np

THIS_DIR = os.path.dirname(__file__)
sys.path.append(THIS_DIR)
sys.path.append(os.path.join(THIS_DIR, "..", "lib"))

# --- project deps you already have ---
from config import (
    MOTOR_IDS, MOTOR_TYPE, CONTROL_MODE,
    GEAR, DIR, DT,
    LIMIT_T1, LIMIT_T2, LIMIT_T3,
    clamp,
)
from kinematics import (
    fk_leg, ik_leg, pick_closest_solution, clamp_to_workspace,
    l1, l2, l3,
)
from unitree_actuator_sdk import SerialPort, MotorCmd, MotorData

# =========================
# Tunables / Gains
# =========================
BASE_KP = 0.15
BASE_KD = 0.05

CROUCH_KP = 0.45
CROUCH_KD = 0.08

EXPLODE_KP = 2.5
EXPLODE_KD = 0.08

# feedforward torque used only if you want a bit more push
EXPLODE_TAU_FF = 0.0  # Nm (set small, hybrid PD already produces torque)

# absolute safety for torque-only sends (motor-side Nm)
MAX_TORQUE_NM = 23.0

# default scripted jump targets (radians)
SCRIPT_JOINT0 = 0.0
SCRIPT_J1_START, SCRIPT_J1_END = -1.557, -0.746
SCRIPT_J2_START, SCRIPT_J2_END =  2.262,  1.621

# =========================
# Helpers
# =========================
def decode_merror(bits: int):
    labels = ["OK","Overheat","Overcurrent","Overvoltage","Encoder Fault",
              "Undervoltage","Winding Overheat","Reserved"]
    if bits == 0:
        return "OK"
    out = []
    for b in range(min(8, len(labels))):
        if (bits >> b) & 1:
            if b == 0 and bits != 0:
                continue
            out.append(labels[b])
    return ",".join(out) if out else "OK"

def lerp(a, b, t):
    return a + t*(b - a)

# --- tiny Overleap-style force profile (200 ms, sin^2) ---
class ForceProfile:
    def __init__(self, duration_ms=200, impulse=2500.0, fx_max=0.0, fx_prop=0.0):
        self.duration_ms = duration_ms
        self.impulse = impulse
        self.fx_max = fx_max
        self.fx_prop = fx_prop
        self.b = 1.0 / max(1.0, duration_ms)  # ms^-1

    def sample(self, t_ms):
        if t_ms < 0 or t_ms > self.duration_ms:
            return 0.0, 0.0
        a = self.impulse / 100.0  # mirrors Overleap scaling
        s = math.sin(self.b * math.pi * t_ms)
        fy = a * (s * s)          # vertical (down +)
        fx = self.fx_prop * fy
        if abs(fx) > self.fx_max:
            fx = math.copysign(self.fx_max, fx)
        return fx, fy

# Jacobianᵀ torque map for the planar (pitch+knee) pair
def torques_from_foot_force(q_pitch, q_knee, fx, fy):
    # Using l2, l3 as the planar links (adjust if your naming differs)
    s1, c1 = math.sin(q_pitch), math.cos(q_pitch)
    s12, c12 = math.sin(q_pitch + q_knee), math.cos(q_pitch + q_knee)
    # τ = JᵀF
    tau_hip  = (l2 * s1 + l3 * s12) * fx + (-l2 * c1 - l3 * c12) * fy
    tau_knee = (l3 * s12) * fx + (-l3 * c12) * fy
    return tau_hip, tau_knee

# =========================
# GUI + Control
# =========================
class MotorGUI:
    def __init__(self, root):
        self.root = root
        root.title("Unitree Leg – Jump GUI (IK / Spring / Force)")

        # --- SDK setup ---
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

        # warmup IO
        for _ in range(50):
            for i in MOTOR_IDS:
                self.serial.sendRecv(self.cmds[i], self.datas[i])
            time.sleep(0.002)

        self.q_home = {i: self.datas[i].q for i in MOTOR_IDS}

        # --- GUI state ---
        self.enabled = tk.BooleanVar(value=False)
        self.mode    = tk.StringVar(value="IK")
        self.s_y     = tk.IntVar(value=+1)
        self.prev_solution = []

        # joint sliders (Angles mode)
        self.t1 = tk.DoubleVar(value=0.0)
        self.t2 = tk.DoubleVar(value=0.0)
        self.t3 = tk.DoubleVar(value=0.0)

        # IK targets (mm)
        self.x = tk.DoubleVar(value=180.0)
        self.y = tk.DoubleVar(value=90.0)
        self.z = tk.DoubleVar(value=180.0)

        # feedback strings
        self.theta_live = {i: tk.StringVar(value="0.000") for i in MOTOR_IDS}
        self.end_eff_xyz = {k: tk.StringVar(value="0.0") for k in ("x","y","z")}
        self.temp_live  = {i: tk.StringVar(value="0") for i in MOTOR_IDS}
        self.err_live   = {i: tk.StringVar(value="OK") for i in MOTOR_IDS}

        # --- layout ---
        frm = ttk.Frame(root, padding=12); frm.grid(sticky="nsew")
        root.columnconfigure(0, weight=1); root.rowconfigure(0, weight=1)
        for c in range(8): frm.columnconfigure(c, weight=1)

        ttk.Label(frm, text="Unitree Leg – Jump GUI", font=("Segoe UI", 14, "bold"))\
           .grid(row=0, column=0, columnspan=8, sticky="w")

        ttk.Checkbutton(frm, text="Enable", variable=self.enabled)\
            .grid(row=1, column=0, sticky="w", pady=(6,8))
        ttk.Button(frm, text="Set Current Pose as Zero", command=self.rezero)\
            .grid(row=1, column=1, sticky="w", padx=6)

        ttk.Label(frm, text="Leg Side:").grid(row=1, column=2, sticky="e")
        self.side_choice = tk.StringVar(value="Left (+1)")
        side_box = ttk.Combobox(frm, values=["Left (+1)","Right (-1)"], width=12,
                                state="readonly", textvariable=self.side_choice)
        side_box.grid(row=1, column=3, sticky="w")
        self.side_choice.trace_add("write", lambda *_: self.s_y.set(+1 if "Left" in self.side_choice.get() else -1))

        ttk.Label(frm, text="Mode:").grid(row=1, column=4, sticky="e")
        ttk.Combobox(frm, values=["IK","Angles"], textvariable=self.mode, width=8, state="readonly")\
            .grid(row=1, column=5, sticky="w")

        # control area
        self.ctrl_frame = ttk.Frame(frm); self.ctrl_frame.grid(row=2, column=0, columnspan=8, sticky="we", pady=(6,10))
        self.mode.trace_add("write", lambda *_: self._rebuild_ctrls())
        self._rebuild_ctrls()

        # feedback
        ttk.Separator(frm).grid(row=3, column=0, columnspan=8, sticky="we", pady=6)
        ttk.Label(frm, text="Live Feedback", font=("Segoe UI", 11, "bold")).grid(row=4, column=0, sticky="w")
        for r, mid in enumerate(MOTOR_IDS, start=6):
            ttk.Label(frm, text=f"J{mid}").grid(row=r, column=0, sticky="w")
            ttk.Label(frm, textvariable=self.theta_live[mid], width=14).grid(row=r, column=1, sticky="w")
            ttk.Label(frm, textvariable=self.temp_live[mid], width=8).grid(row=r, column=2, sticky="w")
            ttk.Label(frm, textvariable=self.err_live[mid],  width=22).grid(row=r, column=3, sticky="w")
        ee_row = 6
        ttk.Label(frm, text="x=").grid(row=ee_row,   column=4, sticky="e"); ttk.Label(frm, textvariable=self.end_eff_xyz["x"], width=9).grid(row=ee_row,   column=5, sticky="w")
        ttk.Label(frm, text="y=").grid(row=ee_row+1, column=4, sticky="e"); ttk.Label(frm, textvariable=self.end_eff_xyz["y"], width=9).grid(row=ee_row+1, column=5, sticky="w")
        ttk.Label(frm, text="z=").grid(row=ee_row+2, column=4, sticky="e"); ttk.Label(frm, textvariable=self.end_eff_xyz["z"], width=9).grid(row=ee_row+2, column=5, sticky="w")

        # jump controls
        self._jump_status = tk.StringVar(value="Idle")
        self._jump_phase  = None  # None / "crouch" / "explode"
        ttk.Separator(frm).grid(row=8, column=0, columnspan=8, sticky="we", pady=6)
        self._build_jump_box(frm, row=9)

        # control loop state
        self._last_cmd_joints = np.zeros(3, dtype=float)
        self.max_joint_step = 0.03

        # threads
        self._running = True
        self._rezeroing = False
        self.ctrl_thread = threading.Thread(target=self.control_loop, daemon=True); self.ctrl_thread.start()
        self._ui_job = self.root.after(100, self.update_ui)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # -------- UI builders --------
    def _rebuild_ctrls(self):
        for w in self.ctrl_frame.winfo_children(): w.destroy()
        if self.mode.get() == "Angles":
            self._build_angle_controls(self.ctrl_frame)
        else:
            self._build_ik_controls(self.ctrl_frame)

    def _build_angle_controls(self, f):
        ttk.Label(f, text="Angle Control (rad)").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(f, text="θ1").grid(row=1, column=0, sticky="w")
        ttk.Scale(f, from_=-LIMIT_T1, to=+LIMIT_T1, variable=self.t1, orient="horizontal", length=420)\
            .grid(row=1, column=1, columnspan=2, sticky="we")
        ttk.Label(f, text="θ2").grid(row=2, column=0, sticky="w")
        ttk.Scale(f, from_=-LIMIT_T2, to=+LIMIT_T2, variable=self.t2, orient="horizontal", length=420)\
            .grid(row=2, column=1, columnspan=2, sticky="we")
        ttk.Label(f, text="θ3").grid(row=3, column=0, sticky="w")
        ttk.Scale(f, from_=-LIMIT_T3, to=+LIMIT_T3, variable=self.t3, orient="horizontal", length=420)\
            .grid(row=3, column=1, columnspan=2, sticky="we")

    def _build_ik_controls(self, f):
        max_fwd = l2 + l3
        min_fwd = -0.25 * (l2 + l3)
        max_lat = l1 + l2
        max_down = l2 + l3
        max_up   = -30.0
        ttk.Label(f, text="IK Control (mm)").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(f, text="x").grid(row=1, column=0, sticky="w")
        ttk.Scale(f, from_=min_fwd, to=max_fwd, variable=self.x, orient="horizontal", length=420)\
            .grid(row=1, column=1, columnspan=2, sticky="we")
        ttk.Label(f, text="y").grid(row=2, column=0, sticky="w")
        ttk.Scale(f, from_=-max_lat, to=max_lat, variable=self.y, orient="horizontal", length=420)\
            .grid(row=2, column=1, columnspan=2, sticky="we")
        ttk.Label(f, text="z").grid(row=3, column=0, sticky="w")
        ttk.Scale(f, from_=max_up, to=max_down, variable=self.z, orient="horizontal", length=420)\
            .grid(row=3, column=1, columnspan=2, sticky="we")

    def _build_jump_box(self, parent, row):
        box = ttk.Labelframe(parent, text="Jump Controls", padding=8)
        box.grid(row=row, column=0, columnspan=8, sticky="we", pady=(8,0))
        for c in range(8): parent.columnconfigure(c, weight=1)

        ttk.Label(box, text="Status:").grid(row=0, column=0, sticky="e")
        ttk.Label(box, textvariable=self._jump_status).grid(row=0, column=1, sticky="w")
        ttk.Button(box, text="Scripted spring jump", command=self._start_spring_jump)\
            .grid(row=0, column=2, sticky="we")
        ttk.Button(box, text="Force-profile jump (Overleap)", command=self._start_force_jump)\
            .grid(row=0, column=3, sticky="we")

        # timing
        self.jump_hold_s    = tk.DoubleVar(value=0.25)
        self.jump_crouch_s  = tk.DoubleVar(value=0.25)
        self.jump_explode_s = tk.DoubleVar(value=0.18)

        ttk.Label(box, text="Hold (s):").grid(row=1, column=0, sticky="e")
        tk.Spinbox(box, from_=0.05, to=2.0, increment=0.05, textvariable=self.jump_hold_s, width=5)\
            .grid(row=1, column=1, sticky="w")
        ttk.Label(box, text="Crouch (s):").grid(row=1, column=2, sticky="e")
        tk.Spinbox(box, from_=0.05, to=2.0, increment=0.05, textvariable=self.jump_crouch_s, width=5)\
            .grid(row=1, column=3, sticky="w")
        ttk.Label(box, text="Explode (s):").grid(row=1, column=4, sticky="e")
        tk.Spinbox(box, from_=0.05, to=2.0, increment=0.05, textvariable=self.jump_explode_s, width=5)\
            .grid(row=1, column=5, sticky="w")

    # -------- jump handlers --------
    def _start_spring_jump(self):
        if not self.enabled.get():
            self.enabled.set(True)
        if getattr(self, "_spring_running", False):
            return
        self.mode.set("Angles")  # drive joints directly
        self._spring_running = True
        self._jump_status.set("Jump: hold")
        threading.Thread(target=self._run_spring_jump, daemon=True).start()

    def _run_spring_jump(self):
        try:
            j0 = SCRIPT_JOINT0
            j1s, j1e = SCRIPT_J1_START, SCRIPT_J1_END
            j2s, j2e = SCRIPT_J2_START, SCRIPT_J2_END

            hold_s    = float(self.jump_hold_s.get())
            crouch_s  = float(self.jump_crouch_s.get())
            explode_s = float(self.jump_explode_s.get())

            # 0) ensure current pose is target joint0
            self.t1.set(j0)

            # 1) hold to settle (stiff-ish)
            self._jump_phase = "crouch"
            t0 = time.time()
            while time.time() - t0 < max(0.0, hold_s):
                # keep crouch setpoints during hold (start pose)
                self.t2.set(j1s); self.t3.set(j2s)
                time.sleep(DT)

            # 2) go-to crouch smoothly
            self._jump_status.set("Jump: crouch")
            self._jump_phase = "crouch"
            steps = max(1, int(crouch_s / max(DT, 1e-3)))
            j1_init, j2_init = float(self.t2.get()), float(self.t3.get())
            for k in range(steps):
                a = (k + 1) / steps
                self.t1.set(j0)
                self.t2.set(lerp(j1_init, j1s, a))
                self.t3.set(lerp(j2_init, j2s, a))
                time.sleep(DT)

            time.sleep(0.03)  # tiny settle

            # 3) explode: ramp to extended pose; hybrid PD acts like a spring
            self._jump_status.set("Jump: explode")
            self._jump_phase = "explode"
            steps = max(1, int(explode_s / max(DT, 1e-3)))
            for k in range(steps):
                a = (k + 1) / steps
                self.t1.set(j0)
                self.t2.set(lerp(j1s, j1e, a))
                self.t3.set(lerp(j2s, j2e, a))
                time.sleep(DT)

            # 4) relax (drop to base gains)
            self._jump_phase = None
            self._jump_status.set("Jump: flight/relax")
            time.sleep(0.10)

            self._jump_status.set("Idle")
        finally:
            self._spring_running = False

    def _start_force_jump(self):
        """Overleap-style: push with a short force profile via Jacobianᵀ torques."""
        if not self.enabled.get():
            self.enabled.set(True)
        if getattr(self, "_force_running", False):
            return
        self._force_running = True
        self._jump_status.set("Jump: force")
        threading.Thread(target=self._run_force_jump, daemon=True).start()

    def _run_force_jump(self):
        try:
            # optional: ensure we're in a crouch-ish stance first (light PD via Angles mode)
            self.mode.set("Angles")
            self._jump_phase = "crouch"
            self.t1.set(SCRIPT_JOINT0)
            self.t2.set(SCRIPT_J1_START)
            self.t3.set(SCRIPT_J2_START)
            time.sleep(0.20)

            # switch to force push: foot assumed stationary during short burst
            prof = ForceProfile(duration_ms=200, impulse=2500.0, fx_max=0.0, fx_prop=0.0)
            t0 = time.time()
            while True:
                t_ms = (time.time() - t0) * 1000.0
                if t_ms > prof.duration_ms:
                    break

                jt = self._measured_joint_angles()
                if jt is None:
                    break
                # joints: 0=roll, 1=pitch, 2=knee
                q_roll, q_pitch, q_knee = jt

                fx, fy = prof.sample(t_ms)
                tau_pitch, tau_knee = torques_from_foot_force(q_pitch, q_knee, fx, fy)

                self.send_torque_only({
                    1: tau_pitch,
                    2: tau_knee,
                })

                time.sleep(max(0.002, DT))

            # zero torques
            self.send_torque_only({1: 0.0, 2: 0.0})
            self._jump_phase = None
            self._jump_status.set("Idle")
        finally:
            self._force_running = False

    # -------- core helpers --------
    def rezero(self):
        self._rezeroing = True
        try:
            for _ in range(3):
                for i in MOTOR_IDS:
                    self.serial.sendRecv(self.cmds[i], self.datas[i])
                time.sleep(0.002)
            self.q_home = {i: self.datas[i].q for i in MOTOR_IDS}
            self.t1.set(0.0); self.t2.set(0.0); self.t3.set(0.0)

            jt = self._measured_joint_angles()
            if jt is not None:
                ee,_ = fk_leg(*jt, s_y=self.s_y.get())
                self.x.set(float(ee[0])); self.y.set(float(ee[1])); self.z.set(float(ee[2]))
                self.prev_solution = [jt]
        finally:
            self._rezeroing = False

    def _measured_joint_angles(self):
        try:
            t = []
            for i in MOTOR_IDS:
                q_rotor = self.datas[i].q - self.q_home[i]
                t.append(DIR[i] * (q_rotor / GEAR[i]))
            return tuple(t)
        except Exception:
            return None

    def _foot_contact(self):
        # TODO: wire your FSR/force sensor here
        return True

    def send_torque_only(self, tau_dict):
        """Send torque commands while holding current rotor positions (low kp/kd)."""
        for i in MOTOR_IDS:
            c = self.cmds[i]
            c.mode = CONTROL_MODE
            c.q    = self.datas[i].q  # hold where you are
            c.dq   = 0.0
            c.kp   = 0.0
            c.kd   = 0.0
            tau = float(tau_dict.get(i, 0.0))
            # absolute clamp for safety
            tau = clamp(tau, -MAX_TORQUE_NM, +MAX_TORQUE_NM)
            c.tau  = tau
            self.serial.sendRecv(c, self.datas[i])

    def _command_joint_targets(self, t1, t2, t3, phase=None,
                               offset_pitch=0.0, offset_knee=0.0):
        # offsets apply to pitch (1) and knee (2)
        t1 = clamp(t1, -LIMIT_T1, +LIMIT_T1)
        t2 = clamp(t2 + offset_pitch, -LIMIT_T2, +LIMIT_T2)
        t3 = clamp(t3 + offset_knee,  -LIMIT_T3, +LIMIT_T3)
        t_des = {0: t1, 1: t2, 2: t3}

        q_des_abs_rotor = {
            i: self.q_home[i] + (DIR[i] * t_des[i] * GEAR[i])
            for i in MOTOR_IDS
        }

        on_ground = self._foot_contact()
        if phase == "explode" and not on_ground:
            phase = None

        for i in MOTOR_IDS:
            kp_use, kd_use = BASE_KP, BASE_KD
            tau_ff = 0.0
            if phase == "crouch":
                kp_use, kd_use = CROUCH_KP, CROUCH_KD
            elif phase == "explode":
                kp_use, kd_use = EXPLODE_KP, EXPLODE_KD
                if i in (1, 2):
                    tau_ff = EXPLODE_TAU_FF

            c = self.cmds[i]
            c.mode = CONTROL_MODE
            c.q    = q_des_abs_rotor[i]
            c.dq   = 0.0
            c.kp   = kp_use
            c.kd   = kd_use
            c.tau  = tau_ff
            self.serial.sendRecv(c, self.datas[i])

    # -------- UI updates + control loop --------
    def update_ui(self):
        jt = self._measured_joint_angles()
        if jt is not None:
            t1,t2,t3 = jt
            self.theta_live[0].set(f"{t1:.3f}")
            self.theta_live[1].set(f"{t2:.3f}")
            self.theta_live[2].set(f"{t3:.3f}")
            ee,_ = fk_leg(t1,t2,t3, s_y=self.s_y.get())
            self.end_eff_xyz["x"].set(f"{ee[0]:.1f}")
            self.end_eff_xyz["y"].set(f"{ee[1]:.1f}")
            self.end_eff_xyz["z"].set(f"{ee[2]:.1f}")

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

                fault = any(getattr(self.datas[i], "merror", 0) != 0 for i in MOTOR_IDS)
                if not self.enabled.get() or self._rezeroing or fault:
                    for i in MOTOR_IDS:
                        c = self.cmds[i]
                        c.mode = CONTROL_MODE
                        c.kp = 0.0; c.kd = 0.0
                        c.q  = self.datas[i].q
                        c.dq = 0.0; c.tau = 0.0
                        self.serial.sendRecv(c, self.datas[i])
                    continue

                # jump phase (affects gains inside _command_joint_targets)
                phase = self._jump_phase
                off_pitch = 0.0
                off_knee  = 0.0

                if self.mode.get() == "Angles":
                    self._command_joint_targets(self.t1.get(), self.t2.get(), self.t3.get(),
                                                phase=phase,
                                                offset_pitch=off_pitch,
                                                offset_knee=off_knee)
                    continue

                # IK mode
                x_req, y_req, z_req = self.x.get(), self.y.get(), self.z.get()
                x_c, y_c, z_c = clamp_to_workspace(x_req, y_req, z_req, s_y=self.s_y.get())
                if abs(x_c - x_req) > 1e-6 or abs(y_c - y_req) > 1e-6 or abs(z_c - z_req) > 1e-6:
                    self.x.set(x_c); self.y.set(y_c); self.z.set(z_c)

                cands = ik_leg(x_c, y_c, z_c, s_y=self.s_y.get())
                prev = self.prev_solution[0] if self.prev_solution else None
                sol = pick_closest_solution(prev, cands)
                if sol is None:
                    for i in MOTOR_IDS:
                        c = self.cmds[i]
                        c.mode = CONTROL_MODE
                        c.kp = 0.0; c.kd = 0.0
                        c.q  = self.datas[i].q
                        c.dq = 0.0; c.tau = 0.0
                        self.serial.sendRecv(c, self.datas[i])
                    continue

                self.prev_solution = [sol]
                desired = np.array(sol, dtype=float)

                # gentle slew in IK mode
                prev_cmd = self._last_cmd_joints
                max_step = self.max_joint_step
                if phase == "explode":
                    max_step = 0.5
                delta = desired - prev_cmd
                delta = np.clip(delta, -max_step, max_step)
                smoothed = prev_cmd + delta
                self._last_cmd_joints = smoothed

                t1, t2, t3 = smoothed.tolist()
                self._command_joint_targets(t1, t2, t3,
                                            phase=phase,
                                            offset_pitch=off_pitch,
                                            offset_knee=off_knee)
        finally:
            # safe stop
            for _ in range(10):
                for i in MOTOR_IDS:
                    c = self.cmds[i]
                    c.kp = 0.0; c.kd = 0.0
                    c.q  = self.datas[i].q
                    c.dq = 0.0; c.tau = 0.0
                    self.serial.sendRecv(c, self.datas[i])
                time.sleep(0.01)

    def on_close(self):
        self._running = False
        if self._ui_job is not None:
            try: self.root.after_cancel(self._ui_job)
            except Exception: pass
        self.root.after(200, self.root.destroy)

# -------- entrypoint --------
if __name__ == "__main__":
    root = tk.Tk()
    app = MotorGUI(root)
    root.mainloop()
