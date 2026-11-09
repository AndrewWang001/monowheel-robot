"""Pick which firmware mode to build, then build / flash / monitor it.

    python tools/pick_mode.py

Writes main/mode_select.h (gitignored), which main/main.c reads. Without that file the
firmware builds in Drive mode. VS Code's own Build / Flash buttons also use the mode you
picked last, so you can use this window or not.

Only uses the Python standard library (tkinter).
"""
import glob
import os
import re
import subprocess
import sys
import tkinter as tk
from tkinter import messagebox, ttk

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODE_FILE = os.path.join(REPO, "main", "mode_select.h")

# (label, APP_MODE, SERVO_CAL_STEP or None, description). Must match main/modes/modes.h.
# The servo ID tool is deliberately NOT listed (see main/modes/tool_servo_set_id.c).
MODES = [
    ("Drive (default)", "MODE_DRIVE", None,
     "IMU + controller. CH5 kill switch, CH2 right stick = drive motors (up fwd, down rev), CH7 = leg poses, CH6 = stand-up (press again to pause)."),
    ("Test: IMU", "MODE_TEST_IMU", None,
     "BNO085 only. Prints roll / pitch / yaw / gyro."),
    ("Test: controller", "MODE_TEST_RC", None,
     "ELRS receiver only. Prints all 16 channels."),
    ("Test: leg sweep", "MODE_TEST_LEG_SWEEP", None,
     "Leg servos IDs 1-6 swing +/-45 deg around their zero. Missing servos are skipped."),
    ("Test: motor sweep", "MODE_TEST_MOTOR_SWEEP", None,
     "Drive motor scripted, no controller: slow fwd, slow rev, up to 80%, stop. Clamp the motor!"),
    ("Test: motor + controller", "MODE_TEST_MOTOR_RC", None,
     "Drive motors from the controller only. CH5 kill switch, CH2 right stick: up forward, down reverse (95% cap)."),
    ("Servo calibration: center", "MODE_TOOL_SERVO_CAL", "CAL_CENTER",
     "Servos go to mid travel (500) and hold. Fit horns in the zero pose. Links OFF."),
    ("Servo calibration: read", "MODE_TOOL_SERVO_CAL", "CAL_READ",
     "Torque off. Prints raw positions + angles to copy into components/legs/legs.c."),
    ("Servo calibration: check", "MODE_TOOL_SERVO_CAL", "CAL_CHECK",
     "Torque on. Each joint goes to +20 deg and back to confirm directions."),
    ("Servo calibration: slider", "MODE_TOOL_SERVO_CAL", "CAL_SLIDER",
     "Torque on. Close the monitor, then run  python sim/servo_sliders.py"),
]


def read_current():
    """Index into MODES of the mode in mode_select.h, -1 if hand-written / unknown, 0 if none."""
    if not os.path.exists(MODE_FILE):
        return 0
    text = open(MODE_FILE, encoding="utf-8").read()
    mode = re.search(r"#define\s+APP_MODE\s+(\w+)", text)
    step = re.search(r"#define\s+SERVO_CAL_STEP\s+(\w+)", text)
    for i, (_, m, s, _) in enumerate(MODES):
        if mode and mode.group(1) == m and (s is None or (step and step.group(1) == s)):
            return i
    return -1


def write_mode(i):
    label, mode, step, _ = MODES[i]
    lines = [
        "// Written by tools/pick_mode.py. Gitignored: local to your machine.",
        f"// Mode: {label}",
        f"#define APP_MODE {mode}",
    ]
    if step:
        lines.append(f"#define SERVO_CAL_STEP {step}")
    with open(MODE_FILE, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


def find_idf_profile():
    """The ESP-IDF PowerShell setup script (Windows EIM install), newest version first."""
    found = sorted(glob.glob(r"C:\Espressif\tools\Microsoft.v*.PowerShell_profile.ps1"), reverse=True)
    return found[0] if found else None


def run_idf(args):
    """Run idf.py <args> in a new terminal window that stays open."""
    cmd = "idf.py " + " ".join(args)
    if sys.platform == "win32":
        profile = find_idf_profile()
        setup = f". '{profile}'; " if profile else ""
        ps = f"{setup}Set-Location '{REPO}'; {cmd}"
        subprocess.Popen(["powershell", "-NoExit", "-Command", ps],
                         creationflags=subprocess.CREATE_NEW_CONSOLE)
    else:
        # Mac / Linux: needs ESP-IDF's export.sh already sourced in the shell that started this
        subprocess.Popen(cmd, shell=True, cwd=REPO)


def main():
    root = tk.Tk()
    root.title("Monowheel: pick firmware mode")
    root.resizable(False, False)
    pad = {"padx": 10, "pady": 3}

    current = read_current()
    choice = tk.IntVar(value=max(current, 0))

    status = tk.StringVar()

    def refresh_status():
        cur = read_current()
        if cur == -1:
            status.set("Current: hand-written main/mode_select.h (not one of these). Saving will replace it.")
        else:
            status.set(f"Current: {MODES[cur][0]}")

    ttk.Label(root, text="Firmware mode", font=("Segoe UI", 12, "bold")).grid(row=0, column=0, columnspan=2,
                                                                           sticky="w", **pad)
    for i, (label, _, _, desc) in enumerate(MODES):
        ttk.Radiobutton(root, text=label, variable=choice, value=i).grid(row=i + 1, column=0, sticky="w", **pad)
        ttk.Label(root, text=desc, foreground="#555").grid(row=i + 1, column=1, sticky="w", **pad)

    row = len(MODES) + 1
    port_frame = ttk.Frame(root)
    port_frame.grid(row=row, column=0, columnspan=2, sticky="w", **pad)
    ttk.Label(port_frame, text="Port (blank = auto):").pack(side="left")
    port = tk.StringVar(value="")
    ttk.Entry(port_frame, textvariable=port, width=10).pack(side="left", padx=6)

    def port_args():
        p = port.get().strip()
        return ["-p", p] if p else []

    def save():
        write_mode(choice.get())
        refresh_status()

    def build():
        save()
        run_idf(["build"])

    def flash_monitor():
        save()
        run_idf(port_args() + ["build", "flash", "monitor"])

    def monitor():
        run_idf(port_args() + ["monitor"])

    buttons = ttk.Frame(root)
    buttons.grid(row=row + 1, column=0, columnspan=2, sticky="w", **pad)
    ttk.Button(buttons, text="Save mode", command=save).pack(side="left", padx=4)
    ttk.Button(buttons, text="Build", command=build).pack(side="left", padx=4)
    ttk.Button(buttons, text="Build + Flash + Monitor", command=flash_monitor).pack(side="left", padx=4)
    ttk.Button(buttons, text="Monitor only", command=monitor).pack(side="left", padx=4)

    ttk.Label(root, textvariable=status, foreground="#0a5").grid(row=row + 2, column=0, columnspan=2,
                                                                sticky="w", **pad)
    refresh_status()

    if sys.platform == "win32" and not find_idf_profile() and "IDF_PATH" not in os.environ:
        messagebox.showwarning("ESP-IDF not found",
                               "Couldn't find C:\\Espressif\\tools\\Microsoft.v*.PowerShell_profile.ps1.\n"
                               "Build buttons will only work if idf.py is already on your PATH.")
    root.mainloop()


if __name__ == "__main__":
    main()
