"""
NexusOS Native Desktop 3D GUI (Tkinter + 3D Perspective Projection Engine)
Runs a standalone 3D digital twin window locally without requiring an external browser.
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


class NexusDesktop3DApp:
    """Native Desktop 3D GUI App for NexusOS."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("NexusOS 3D Desktop Control Center (Digital Twin)")
        self.root.geometry("900x650")
        self.root.configure(bg="#0a0f1d")

        # Event loop for async HAL plugin
        import asyncio
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        # Initialize HAL Plugin
        self.plugin = DiffDriveBasePlugin(robot_id="desktop-rover-01")
        self.loop.run_until_complete(self.plugin.initialize())

        # 3D Camera & View Parameters
        self.cam_azimuth = math.radians(45)   # Horizontal rotation (yaw)
        self.cam_elevation = math.radians(30) # Vertical pitch
        self.cam_distance = 8.0               # Zoom distance
        self.fov = 350.0                      # Projection focal length
        self.last_mouse_x = 0
        self.last_mouse_y = 0

        # Trajectory History: list of (x, y, z)
        self.trajectory = []

        self.setup_ui()
        self.setup_mouse_controls()

        # Telemetry timer
        self.root.after(100, self.update_telemetry_loop)

    def setup_ui(self):
        # Header Label
        header = tk.Label(
            self.root,
            text="NEXUSOS 3D DIGITAL TWIN CONTROL CENTER",
            font=("Helvetica", 15, "bold"),
            fg="#38bdf8",
            bg="#0a0f1d",
            pady=8
        )
        header.pack(fill=tk.X)

        # Main Container
        main_frame = tk.Frame(self.root, bg="#0a0f1d", padx=16, pady=6)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Left Panel: 3D Viewport
        left_frame = tk.LabelFrame(main_frame, text=" 3D Perspective Digital Twin (Left-Click Drag to Orbit, Scroll to Zoom) ", fg="#94a3b8", bg="#131d31", padx=8, pady=8)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        self.canvas = tk.Canvas(left_frame, bg="#050811", highlightthickness=1, highlightbackground="#1e293b")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Right Panel: Telemetry Gauges & Controls
        right_frame = tk.LabelFrame(main_frame, text=" Telemetry & Teleop Controls ", fg="#94a3b8", bg="#131d31", padx=14, pady=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5)

        # Telemetry Labels
        self.lbl_status = tk.Label(right_frame, text="Status: INITIALIZING", font=("Consolas", 11), fg="#f59e0b", bg="#131d31", anchor="w")
        self.lbl_status.pack(fill=tk.X, pady=4)

        self.lbl_mode = tk.Label(right_frame, text="Mode: MANUAL", font=("Consolas", 11), fg="#38bdf8", bg="#131d31", anchor="w")
        self.lbl_mode.pack(fill=tk.X, pady=4)

        self.lbl_pose = tk.Label(right_frame, text="Pos (X,Y): 0.00, 0.00 m", font=("Consolas", 11), fg="#cbd5e1", bg="#131d31", anchor="w")
        self.lbl_pose.pack(fill=tk.X, pady=4)

        self.lbl_speed = tk.Label(right_frame, text="Linear: 0.00 m/s", font=("Consolas", 11), fg="#cbd5e1", bg="#131d31", anchor="w")
        self.lbl_speed.pack(fill=tk.X, pady=4)

        self.lbl_battery = tk.Label(right_frame, text="Battery: 100.0%", font=("Consolas", 11), fg="#10b981", bg="#131d31", anchor="w")
        self.lbl_battery.pack(fill=tk.X, pady=4)

        ttk.Separator(right_frame, orient='horizontal').pack(fill=tk.X, pady=12)

        # 3D View Presets
        lbl_view = tk.Label(right_frame, text="3D View Presets:", font=("Helvetica", 9, "bold"), fg="#94a3b8", bg="#131d31", anchor="w")
        lbl_view.pack(fill=tk.X, pady=2)

        btn_view_iso = tk.Button(right_frame, text="Isometric View", font=("Helvetica", 9), bg="#1e293b", fg="white", command=lambda: self.set_view(45, 30))
        btn_view_iso.pack(fill=tk.X, pady=2)

        btn_view_top = tk.Button(right_frame, text="Top-Down View", font=("Helvetica", 9), bg="#1e293b", fg="white", command=lambda: self.set_view(0, 85))
        btn_view_top.pack(fill=tk.X, pady=2)

        ttk.Separator(right_frame, orient='horizontal').pack(fill=tk.X, pady=12)

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
        btn_estop.pack(fill=tk.X, pady=12)

    def setup_mouse_controls(self):
        self.canvas.bind("<ButtonPress-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)

    def set_view(self, az_deg, el_deg):
        self.cam_azimuth = math.radians(az_deg)
        self.cam_elevation = math.radians(el_deg)
        self.render_3d_scene()

    def on_mouse_down(self, event):
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y

    def on_mouse_drag(self, event):
        dx = event.x - self.last_mouse_x
        dy = event.y - self.last_mouse_y
        self.cam_azimuth += dx * 0.01
        self.cam_elevation = max(0.05, min(math.pi / 2 - 0.05, self.cam_elevation - dy * 0.01))
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y
        self.render_3d_scene()

    def on_mouse_wheel(self, event):
        if event.delta > 0:
            self.cam_distance = max(3.0, self.cam_distance - 0.5)
        else:
            self.cam_distance = min(20.0, self.cam_distance + 0.5)
        self.render_3d_scene()

    # -------------------------------------------------------------
    # 3D PERSPECTIVE MATH ENGINE
    # -------------------------------------------------------------
    def project_3d(self, x, y, z, cx, cy):
        """Projects a 3D world coordinate (x, y, z) into 2D canvas coordinates (sx, sy)."""
        # 1. Translate relative to camera orbit position
        cos_az = math.cos(self.cam_azimuth)
        sin_az = math.sin(self.cam_azimuth)
        cos_el = math.cos(self.cam_elevation)
        sin_el = math.sin(self.cam_elevation)

        # Orbit transformation
        x1 = x * cos_az - y * sin_az
        y1 = x * sin_az + y * cos_az
        z1 = z

        # Pitch elevation transformation
        x_cam = x1
        y_cam = y1 * sin_el - z1 * cos_el
        z_cam = y1 * cos_el + z1 * sin_el + self.cam_distance

        if z_cam <= 0.1:
            z_cam = 0.1

        # Perspective projection
        factor = self.fov / z_cam
        sx = cx + x_cam * factor
        sy = cy - y_cam * factor  # Invert Y for screen coordinates
        return sx, sy

    def render_3d_scene(self):
        self.canvas.delete("all")
        w = self.canvas.winfo_width() or 500
        h = self.canvas.winfo_height() or 400
        cx = w / 2
        cy = h / 2

        # 1. Render 3D Grid Floor
        grid_size = 6
        step = 1.0
        grid_lines = []
        for i in range(-grid_size, grid_size + 1):
            p1 = self.project_3d(i * step, -grid_size * step, 0, cx, cy)
            p2 = self.project_3d(i * step, grid_size * step, 0, cx, cy)
            color = "#1e293b" if i != 0 else "#38bdf8"
            self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=color, width=1 if i != 0 else 2)

            p3 = self.project_3d(-grid_size * step, i * step, 0, cx, cy)
            p4 = self.project_3d(grid_size * step, i * step, 0, cx, cy)
            self.canvas.create_line(p3[0], p3[1], p4[0], p4[1], fill=color, width=1 if i != 0 else 2)

        # 2. Render Trajectory Ribbon
        if len(self.trajectory) > 1:
            for i in range(len(self.trajectory) - 1):
                pt1 = self.project_3d(self.trajectory[i][0], self.trajectory[i][1], 0.05, cx, cy)
                pt2 = self.project_3d(self.trajectory[i+1][0], self.trajectory[i+1][1], 0.05, cx, cy)
                self.canvas.create_line(pt1[0], pt1[1], pt2[0], pt2[1], fill="#a855f7", width=2)

        # 3. Render 3D Robot Model (Chassis Box + Wheels)
        rx = float(self.plugin._x)
        ry = float(self.plugin._y)
        ryaw = float(self.plugin._theta)

        # Robot local corners (dx, dy, dz)
        bw, bl, bh = 0.35, 0.5, 0.25
        corners = [
            (-bw, -bl, 0.05), (bw, -bl, 0.05), (bw, bl, 0.05), (-bw, bl, 0.05), # Bottom
            (-bw, -bl, bh),   (bw, -bl, bh),   (bw, bl, bh),   (-bw, bl, bh)     # Top
        ]

        # Rotate robot corners by heading yaw
        rot_corners = []
        for (dx, dy, dz) in corners:
            wx = rx + dx * math.cos(ryaw) - dy * math.sin(ryaw)
            wy = ry + dx * math.sin(ryaw) + dy * math.cos(ryaw)
            rot_corners.append(self.project_3d(wx, wy, dz, cx, cy))

        # Draw Chassis Faces
        faces = [
            (0, 1, 2, 3), # Bottom
            (4, 5, 6, 7), # Top
            (0, 1, 5, 4), # Front
            (2, 3, 7, 6), # Back
            (0, 3, 7, 4), # Left
            (1, 2, 6, 5)  # Right
        ]

        # Top Face with Cyber Glow
        top_pts = [rot_corners[i] for i in (4, 5, 6, 7)]
        flat_top = [coord for pt in top_pts for coord in pt]
        self.canvas.create_polygon(flat_top, fill="#1e293b", outline="#38bdf8", width=2)

        # Heading Arrow
        fwd_x = rx + 0.6 * math.cos(ryaw)
        fwd_y = ry + 0.6 * math.sin(ryaw)
        p_arrow_tip = self.project_3d(fwd_x, fwd_y, bh / 2, cx, cy)
        p_center = self.project_3d(rx, ry, bh / 2, cx, cy)
        self.canvas.create_line(p_center[0], p_center[1], p_arrow_tip[0], p_arrow_tip[1], fill="#38bdf8", width=3, arrow=tk.LAST)

    def update_telemetry_loop(self):
        # Read plugin telemetry
        try:
            telem = self.loop.run_until_complete(self.plugin.read_telemetry())
        except Exception:
            return

        pos_x = telem.pose.position.x
        pos_y = telem.pose.position.y
        self.trajectory.append((pos_x, pos_y, 0))
        if len(self.trajectory) > 200:
            self.trajectory.pop(0)

        # Update GUI labels
        self.lbl_status.config(text=f"Status: {telem.status_message}")
        self.lbl_mode.config(text=f"Mode: {telem.mode.value}")
        self.lbl_pose.config(text=f"Pos (X,Y): {pos_x:.2f}, {pos_y:.2f} m")
        self.lbl_speed.config(text=f"Linear: {telem.twist.linear.x:.2f} m/s")
        self.lbl_battery.config(text=f"Battery: {telem.battery.percentage:.1f}%")

        # Re-render 3D scene
        self.render_3d_scene()
        self.root.after(50, self.update_telemetry_loop)

    def cmd_forward(self):
        cmd = MotorCommandData(target_twist=TwistData(linear=Vector3D(x=0.5)), requested_mode=RobotMode.MANUAL)
        self.loop.run_until_complete(self.plugin.send_command(cmd))

    def cmd_left(self):
        cmd = MotorCommandData(target_twist=TwistData(angular=Vector3D(z=0.6)), requested_mode=RobotMode.MANUAL)
        self.loop.run_until_complete(self.plugin.send_command(cmd))

    def cmd_right(self):
        cmd = MotorCommandData(target_twist=TwistData(angular=Vector3D(z=-0.6)), requested_mode=RobotMode.MANUAL)
        self.loop.run_until_complete(self.plugin.send_command(cmd))

    def cmd_stop(self):
        cmd = MotorCommandData(target_twist=TwistData(linear=Vector3D(x=0.0), angular=Vector3D(z=0.0)), requested_mode=RobotMode.MANUAL)
        self.loop.run_until_complete(self.plugin.send_command(cmd))

    def cmd_estop(self):
        self.loop.run_until_complete(self.plugin.emergency_stop())


def main():
    root = tk.Tk()
    app = NexusDesktop3DApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
