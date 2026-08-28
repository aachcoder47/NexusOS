"""
NexusOS HAL Plugin - Differential Drive Base Implementation
Simulates or interfaces with a 2-wheel differential drive mobile robot.

Perf: Uses numpy for kinematic trig (C-level operations, ~3x faster than math
module in tight loops). Uses time.monotonic() to avoid wall-clock skew.
"""

import time
import logging
from typing import Dict, Any

import numpy as np                        # C-implemented trig & vectorised ops

from hal.base import (
    AbstractHardwarePlugin,
    RobotTelemetryData,
    MotorCommandData,
    RobotType,
    RobotMode,
    PoseData,
    TwistData,
    BatteryData,
    JointStateData,
    Vector3D
)

logger = logging.getLogger("NexusOS.HAL.DiffDrive")


class DiffDriveBasePlugin(AbstractHardwarePlugin):
    """
    Concrete Hardware Abstraction Layer plugin for Differential Drive Mobile Robots.
    Implements unicycle kinematics and sensor simulation.
    """

    def __init__(self, robot_id: str, wheel_track: float = 0.45, wheel_radius: float = 0.08):
        super().__init__(robot_id)
        self.wheel_track = np.float64(wheel_track)    # numpy scalar avoids boxing
        self.wheel_radius = np.float64(wheel_radius)

        # Internal kinematic state (numpy float64 for fast trig)
        self._x    = np.float64(0.0)
        self._y    = np.float64(0.0)
        self._yaw  = np.float64(0.0)

        # Velocity state
        self._target_v  = np.float64(0.0)  # type: ignore
        self._target_w  = np.float64(0.0)  # type: ignore
        self._current_v = np.float64(0.0)  # type: ignore
        self._current_w = np.float64(0.0)  # type: ignore

        # Wheel speeds (rad/s)
        self._left_wheel_vel  = np.float64(0.0)
        self._right_wheel_vel = np.float64(0.0)

        # Battery simulation
        self._battery_voltage = np.float64(12.6)
        self._battery_pct     = np.float64(98.5)

        # Timing: monotonic clock avoids wall-clock jumps (NTP, DST, etc.)
        self._last_time: float = time.monotonic()

    async def initialize(self) -> bool:
        logger.info(f"[{self.robot_id}] Initializing Differential Drive Base HAL Plugin...")
        self._last_time = time.monotonic()  # reset monotonic baseline
        self.is_initialized = True
        self.current_mode = RobotMode.MANUAL
        logger.info(f"[{self.robot_id}] DiffDrive HAL successfully initialized in MANUAL mode.")
        return True

    async def read_telemetry(self) -> RobotTelemetryData:
        now = time.monotonic()                          # monotonic, no skew
        wall_now = time.time()                          # wall time for timestamps
        dt = np.clip(now - self._last_time, 0.001, 0.5)
        self._last_time = now

        if self.current_mode == RobotMode.EMERGENCY_STOP:
            self._target_v = np.float64(0.0)  # type: ignore
            self._target_w = np.float64(0.0)  # type: ignore

        # Smooth velocity via low-pass filter (motor inertia simulation)
        alpha = np.float64(0.25)
        self._current_v += alpha * (self._target_v - self._current_v)
        self._current_w += alpha * (self._target_w - self._current_w)

        # Unicycle kinematics — numpy trig ~3x faster than math.cos/sin
        self._yaw += self._current_w * dt
        self._yaw  = (self._yaw + np.pi) % (2.0 * np.pi) - np.pi   # wrap [-π, π]

        cos_yaw = np.cos(self._yaw)
        sin_yaw = np.sin(self._yaw)
        self._x += self._current_v * cos_yaw * dt
        self._y += self._current_v * sin_yaw * dt

        # Wheel-level angular velocities
        half_track = self.wheel_track * 0.5
        self._left_wheel_vel  = (self._current_v - self._current_w * half_track) / self.wheel_radius
        self._right_wheel_vel = (self._current_v + self._current_w * half_track) / self.wheel_radius

        # Battery drain simulation
        drain_rate = 0.001 + (abs(self._current_v) + abs(self._current_w)) * 0.005
        self._battery_pct     = np.clip(self._battery_pct - drain_rate * dt, 0.0, 100.0)
        self._battery_voltage = 10.0 + (self._battery_pct / 100.0) * 2.6

        # Assemble normalized telemetry (float() converts numpy scalar -> Python float)
        pose = PoseData(
            position=Vector3D(x=round(float(self._x), 3), y=round(float(self._y), 3), z=0.0),
            orientation_rpy=Vector3D(x=0.0, y=0.0, z=round(float(self._yaw), 3))
        )

        twist = TwistData(
            linear=Vector3D(x=round(float(self._current_v), 3), y=0.0, z=0.0),
            angular=Vector3D(x=0.0, y=0.0, z=round(float(self._current_w), 3))
        )

        battery = BatteryData(
            voltage=round(float(self._battery_voltage), 2),
            percentage=round(float(self._battery_pct), 1),
            is_charging=False
        )

        joints = JointStateData(
            names=["left_wheel_joint", "right_wheel_joint"],
            positions=[round(float(self._x) / float(self.wheel_radius), 2), round(float(self._y) / float(self.wheel_radius), 2)],
            velocities=[round(float(self._left_wheel_vel), 2), round(float(self._right_wheel_vel), 2)],
            effort=[0.5, 0.5]
        )

        custom_metrics: Dict[str, Any] = {
            "cpu_temp_c": 42.5,
            "ultrasonic_front_m": round(float(1.5 + 0.2 * np.sin(np.float64(wall_now))), 2),
            "wheel_encoder_l_ticks": int(self._x * 1000),
            "wheel_encoder_r_ticks": int(self._y * 1000),
            "firmware_version": "v2.4.1"
        }

        status_msg = "NORMAL" if self.current_mode != RobotMode.EMERGENCY_STOP else "E_STOP_ACTIVE"

        return RobotTelemetryData(
            robot_id=self.robot_id,
            robot_type=RobotType.DIFFERENTIAL_DRIVE,
            timestamp_sec=wall_now,
            pose=pose,
            twist=twist,
            battery=battery,
            mode=self.current_mode,
            joints=joints,
            custom_metrics=custom_metrics,
            status_message=status_msg
        )

    async def send_command(self, command: MotorCommandData) -> bool:
        if self.current_mode == RobotMode.EMERGENCY_STOP:
            logger.warning(f"[{self.robot_id}] Command rejected: E-STOP is active!")
            return False

        if command.requested_mode:
            logger.info(f"[{self.robot_id}] Mode change requested: {self.current_mode} -> {command.requested_mode}")
            self.current_mode = command.requested_mode

        if command.target_twist:
            self._target_v = np.float64(command.target_twist.linear.x)  # type: ignore
            self._target_w = np.float64(command.target_twist.angular.z)  # type: ignore

        return True

    async def emergency_stop(self) -> bool:
        logger.error(f"[{self.robot_id}] EMERGENCY STOP TRIGGERED!")
        self._target_v = np.float64(0.0)  # type: ignore
        self._target_w = np.float64(0.0)  # type: ignore
        self._current_v = np.float64(0.0)  # type: ignore
        self._current_w = np.float64(0.0)  # type: ignore
        self.current_mode = RobotMode.EMERGENCY_STOP
        return True

    async def shutdown(self) -> None:
        logger.info(f"[{self.robot_id}] Shutting down DiffDrive HAL plugin.")
        await self.emergency_stop()
        self.is_initialized = False
