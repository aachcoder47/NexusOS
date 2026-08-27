"""
NexusOS Native Desktop GUI (Tkinter)
Runs a standalone desktop application window locally without requiring a web browser.
"""

import sys
import time
import math
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hal.plugins.diff_drive import DiffDriveBasePlugin
from hal.base import MotorCommandData, TwistData, Vector3D, RobotMode


class NexusDesktopApp:
    """Native Desktop GUI App for NexusOS."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("NexusOS Desktop Control Center (Native GUI)")
        self.root.geometry("800x600")
        self.root.configure(bg="#0f172a")

        # Initialize HAL Plugin
        self.plugin = DiffDriveBasePlugin(robot_id="desktop-rover-01")
        self.sim_angle = 0.0

        self.setup_ui()
        self.root.after(100, self.update_telemetry_loop)

    def setup_ui(self):
        # Header Label
        header = tk.Label(
            self.root,
            text="NEXUSOS NATIVE DESKTOP CONTROL CENTER",
            font=("Helvetica", 16, "bold"),
            fg="#38bdf8",
            bg="#0f172a",
            pady=10
        )
        header.pack(fill=tk.X)

        # Main Container
        main_frame = tk.Frame(self.root, bg="#0f172a", padx=20, pady=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Left Panel: Live Trajectory Canvas
        left_frame = tk.LabelFrame(main_frame, text=" Live 2D Robot Trajectory ", fg="#94a3b8", bg="#1e293b", padx=10, pady=10)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        self.canvas = tk.Canvas(left_frame, bg="#090d16", highlightthickness=1, highlightbackground="#334155")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.draw_grid()

        # Right Panel: Telemetry Gauges & Controls
        right_frame = tk.LabelFrame(main_frame, text=" Robot Telemetry & Controls ", fg="#94a3b8", bg="#1e293b", padx=10, pady=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5)

        # Telemetry Labels
        self.lbl_status = tk.Label(right_frame, text="Status: INITIALIZING", font=("Consolas", 11), fg="#f59e0b", bg="#1e293b", anchor="w")
        self.lbl_status.pack(fill=tk.X, pady=5)

        self.lbl_mode = tk.Label(right_frame, text="Mode: MANUAL", font=("Consolas", 11), fg="#38bdf8", bg="#1e293b", anchor="w")
        self.lbl_mode.pack(fill=tk.X, pady=5)

        self.lbl_pose = tk.Label(right_frame, text="Pose (X, Y): 0.00 m, 0.00 m", font=("Consolas", 11), fg="#cbd5e1", bg="#1e293b", anchor="w")
        self.lbl_pose.pack(fill=tk.X, pady=5)

        self.lbl_speed = tk.Label(right_frame, text="Linear Speed: 0.00 m/s", font=("Consolas", 11), fg="#cbd5e1", bg="#1e293b", anchor="w")
        self.lbl_speed.pack(fill=tk.X, pady=5)

        self.lbl_battery = tk.Label(right_frame, text="Battery: 100.0%", font=("Consolas", 11), fg="#10b981", bg="#1e293b", anchor="w")
        self.lbl_battery.pack(fill=tk.X, pady=5)

        ttk.Separator(right_frame, orient='horizontal').pack(fill=tk.X, pady=15)

        # Control Buttons
        btn_fwd = tk.Button(right_frame, text="▲ DRIVE FORWARD", font=("Helvetica", 10, "bold"), bg="#0284c7", fg="white", command=self.cmd_forward)
        btn_fwd.pack(fill=tk.X, pady=3)

        btn_left = tk.Button(right_frame, text="◄ TURN LEFT", font=("Helvetica", 10, "bold"), bg="#0284c7", fg="white", command=self.cmd_left)
        btn_left.pack(fill=tk.X, pady=3)

        btn_right = tk.Button(right_frame, text="► TURN RIGHT", font=("Helvetica", 10, "bold"), bg="#0284c7", fg="white", command=self.cmd_right)
        btn_right.pack(fill=tk.X, pady=3)

        btn_stop = tk.Button(right_frame, text="■ STOP", font=("Helvetica", 10, "bold"), bg="#64748b", fg="white", command=self.cmd_stop)
        btn_stop.pack(fill=tk.X, pady=3)

        btn_estop = tk.Button(right_frame, text="🚨 EMERGENCY STOP", font=("Helvetica", 11, "bold"), bg="#dc2626", fg="white", command=self.cmd_estop)
        btn_estop.pack(fill=tk.X, pady=15)

    def draw_grid(self):
        self.canvas.delete("all")
        width = self.canvas.winfo_width() or 400
        height = self.canvas.winfo_height() or 400

        # Draw grid lines
        for x in range(0, width, 40):
            self.canvas.create_line(x, 0, x, height, fill="#1e293b", dash=(2, 4))
        for y in range(0, height, 40):
            self.canvas.create_line(0, y, width, y, fill="#1e293b", dash=(2, 4))

        # Center origin crosshair
        cx, cy = width // 2, height // 2
        self.canvas.create_line(cx - 10, cy, cx + 10, cy, fill="#475569")
        self.canvas.create_line(cx, cy - 10, cx, cy + 10, fill="#475569")

    def update_telemetry_loop(self):
        import asyncio
        loop = asyncio.get_event_loop()
        telemetry = loop.run_until_complete(self.plugin.read_telemetry())

        # Update labels
        self.lbl_status.config(text=f"Status: {telemetry.status_message}")
        self.lbl_mode.config(text=f"Mode: {telemetry.mode.value}")
        self.lbl_pose.config(text=f"Pose (X, Y): {telemetry.pose.position.x:.2f} m, {telemetry.pose.position.y:.2f} m")
        self.lbl_speed.config(text=f"Linear Speed: {telemetry.twist.linear.x:.2f} m/s")
        self.lbl_battery.config(text=f"Battery: {telemetry.battery.percentage:.1f}%")

        # Draw robot position on canvas
        width = self.canvas.winfo_width() or 400
        height = self.canvas.winfo_height() or 400
        cx, cy = width // 2, height // 2

        rx = cx + int(telemetry.pose.position.x * 20)
        ry = cy - int(telemetry.pose.position.y * 20)

        self.draw_grid()
        # Draw robot marker
        self.canvas.create_oval(rx - 8, ry - 8, rx + 8, ry + 8, fill="#38bdf8", outline="#f0f9ff", width=2)
        self.canvas.create_text(rx, ry - 14, text=self.plugin.robot_id, fill="#94a3b8", font=("Helvetica", 8))

        self.root.after(100, self.update_telemetry_loop)

    def cmd_forward(self):
        import asyncio
        cmd = MotorCommandData(target_twist=TwistData(linear=Vector3D(x=0.5)))
        asyncio.get_event_loop().run_until_complete(self.plugin.send_command(cmd))

    def cmd_left(self):
        import asyncio
        cmd = MotorCommandData(target_twist=TwistData(angular=Vector3D(z=0.5)))
        asyncio.get_event_loop().run_until_complete(self.plugin.send_command(cmd))

    def cmd_right(self):
        import asyncio
        cmd = MotorCommandData(target_twist=TwistData(angular=Vector3D(z=-0.5)))
        asyncio.get_event_loop().run_until_complete(self.plugin.send_command(cmd))

    def cmd_stop(self):
        import asyncio
        cmd = MotorCommandData(target_twist=TwistData(linear=Vector3D(x=0.0), angular=Vector3D(z=0.0)))
        asyncio.get_event_loop().run_until_complete(self.plugin.send_command(cmd))

    def cmd_estop(self):
        import asyncio
        asyncio.get_event_loop().run_until_complete(self.plugin.emergency_stop())


if __name__ == "__main__":
    root = tk.Tk()
    app = NexusDesktopApp(root)
    root.mainloop()
