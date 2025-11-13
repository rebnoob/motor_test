# main.py
import tkinter as tk
from gui_leg import MotorGUI

def main():
    root = tk.Tk()
    MotorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
