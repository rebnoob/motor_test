#!/usr/bin/env python3
import time
import sys
import math
import threading
import tkinter as tk
from tkinter import ttk

# ---- Unitree SDK -------------------------------------------------------------
sys.path.append("../lib")
from unitree_actuator_sdk import *  # noqa
# -----------------------------------------------------------------------------

MOTOR_IDS = [0, 1, 2]
MOTOR_TYPE = MotorType.GO_M8010_6
CONTROL_MODE = queryMotorMode(MOTOR_TYPE, MotorMode.FOC)  # PD in FOC
KP = 0.08
KD = 0.08
DT = 0.005  # 200 Hz
ANGLE_LIMIT = math.radians(180)
VEL_LIMIT = math.radians(720)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class MotorGUI:
    def __init__(self, root):
        self.root = root
        root.title("Unitree M8010-6 — θ Slider Control")

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
            c.kp = 0.0
            c.kd = 0.0
            c.q  = 0.0
            c.dq = 0.0
            c.tau = 0.0

        # Warmup reads
        for _ in range(50):
            for i in MOTOR_IDS:
                self.serial.sendRecv(self.cmds[i], self.datas[i])
            time.sleep(0.002)

        # Zero at startup
        self.q_home = {i: self.datas[i].q for i in MOTOR_IDS}

        # --- GUI state
        self.enabled = tk.BooleanVar(value=False)
        self.theta_targets = {i: tk.DoubleVar(value=0.0) for i in MOTOR_IDS}
        self.theta_disp    = {i: tk.StringVar(value="0.000") for i in MOTOR_IDS}
        self.dtheta_disp   = {i: tk.StringVar(value="0.000") for i in MOTOR_IDS}

        # --- Layout
        frm = ttk.Frame(root, padding=12)
        frm.grid(sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        title = ttk.Label(frm, text="Unitree GO_M8010_6 — Position Control (rad)", font=("Segoe UI", 14, "bold"))
        title.grid(row=0, column=0, columnspan=4, sticky="w")

        # Enable checkbox
        enable_chk = ttk.Checkbutton(frm, text="Enable control", variable=self.enabled)
        enable_chk.grid(row=1, column=0, sticky="w", pady=(6, 12))

        # Re-zero button
        zero_btn = ttk.Button(frm, text="Set Current Pose as Zero", command=self.rezero)
        zero_btn.grid(row=1, column=1, sticky="w", padx=10)

        # Quit button
        quit_btn = ttk.Button(frm, text="Quit", command=self.on_close)
        quit_btn.grid(row=1, column=3, sticky="e")

        # Sliders and readouts
        for idx, mid in enumerate(MOTOR_IDS, start=2):
            row = idx
            ttk.Label(frm, text=f"θ{mid} target (rad)").grid(row=row, column=0, sticky="w")
            s = ttk.Scale(
                frm,
                from_=-math.pi, to=math.pi,
                variable=self.theta_targets[mid],
                orient="horizontal",
                command=lambda _=None: None,  # no-op; we read vars in loop
                length=400
            )
            s.grid(row=row, column=1, columnspan=2, sticky="we", padx=10)
            frm.columnconfigure(2, weight=1)

            # Live readouts
            ttk.Label(frm, text="θ:").grid(row=row, column=3, sticky="e")
            ttk.Label(frm, textvariable=self.theta_disp[mid], width=8).grid(row=row, column=4, sticky="w")
            ttk.Label(frm, text=" dθ:").grid(row=row, column=5, sticky="e")
            ttk.Label(frm, textvariable=self.dtheta_disp[mid], width=8).grid(row=row, column=6, sticky="w")

        # Small hint
        hint = ttk.Label(frm, foreground="#555",
                         text="Tip: toggle Enable first, then move sliders. Use 'Set Current Pose as Zero' to redefine θ=0.")
        hint.grid(row=5, column=0, columnspan=7, sticky="w", pady=(10,0))

        # Control thread
        self._running = True
        self.ctrl_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.ctrl_thread.start()

        # UI update timer
        self._ui_job = self.root.after(100, self.update_ui)

        # Clean close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def rezero(self):
        # read fresh and use as new zeros
        for i in MOTOR_IDS:
            self.serial.sendRecv(self.cmds[i], self.datas[i])
        self.q_home = {i: self.datas[i].q for i in MOTOR_IDS}
        # keep targets where they are (relative)
        print("Re-zeroed at current pose.")

    def update_ui(self):
        # Show zeroed angles and dq
        for i in MOTOR_IDS:
            q_rel = (self.datas[i].q - self.q_home[i])
            self.theta_disp[i].set(f"{q_rel:.3f}")
            self.dtheta_disp[i].set(f"{self.datas[i].dq:.3f}")
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

                if not self.enabled.get():
                    # Send harmless frames (kp=kd=0) to keep comms alive
                    for i in MOTOR_IDS:
                        c = self.cmds[i]
                        c.mode = CONTROL_MODE
                        c.kp = 0.0
                        c.kd = 0.0
                        c.q  = self.datas[i].q  # hold current
                        c.dq = 0.0
                        c.tau = 0.0
                        self.serial.sendRecv(c, self.datas[i])
                    continue

                # Enabled: PD position control to slider targets
                for i in MOTOR_IDS:
                    t = clamp(self.theta_targets[i].get(), -ANGLE_LIMIT, ANGLE_LIMIT)
                    q_des_abs = self.q_home[i] + t
                    q_meas = self.datas[i].q
                    dq_meas = self.datas[i].dq
                    err = q_des_abs - q_meas
                    dq_des = clamp(5.0 * err, -VEL_LIMIT, VEL_LIMIT)

                    c = self.cmds[i]
                    c.mode = CONTROL_MODE
                    c.kp = KP
                    c.kd = KD
                    c.q  = q_des_abs
                    c.dq = dq_des
                    c.tau = 0.1
                    self.serial.sendRecv(c, self.datas[i])
        finally:
            # Zero torque on shutdown
            for _ in range(10):
                for i in MOTOR_IDS:
                    c = self.cmds[i]
                    c.kp = 0.0
                    c.kd = 0.0
                    c.dq = 0.0
                    c.tau = 0.0
                    self.serial.sendRecv(c, self.datas[i])
                time.sleep(0.01)

    def on_close(self):
        # stop threads and close cleanly
        self._running = False
        if self._ui_job is not None:
            try:
                self.root.after_cancel(self._ui_job)
            except Exception:
                pass
        self.root.after(200, self.root.destroy)

def main():
    root = tk.Tk()
    MotorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
