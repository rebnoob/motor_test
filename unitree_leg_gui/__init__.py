"""
unitree_leg_gui package
-----------------------
This package provides a Tkinter-based GUI and kinematics control system
for a 3-DoF Unitree GO-M8010-6 leg.

Modules:
    config.py       — configuration constants (gear ratios, limits, etc.)
    kinematics.py   — forward/inverse kinematics utilities
    gui_leg.py      — main GUI interface and control loop
    main.py         — entry point (run with `python3 -m unitree_leg_gui.main`)

Make sure you have `../lib` (the Unitree actuator SDK) accessible so that
`from unitree_actuator_sdk import *` works.
"""

import os
import sys

# ensure ../lib (Unitree SDK) and current folder are importable
THIS_DIR = os.path.dirname(__file__)
SDK_PATH = os.path.join(THIS_DIR, "..", "lib")

if SDK_PATH not in sys.path:
    sys.path.append(SDK_PATH)
if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)

# (optional) re-export key classes for convenience
from .gui_leg import MotorGUI
from . import config
from . import kinematics
