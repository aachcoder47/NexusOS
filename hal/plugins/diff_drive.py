"""
NexusOS HAL Plugin - Differential Drive Base Implementation
Simulates or interfaces with a 2-wheel differential drive mobile robot.
"""

import math
import time
import logging
from typing import Dict, Any

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
        self.wheel_track = wheel_track  # meters between wheels
        self.wheel_radius = wheel_radius  # meters

        # Internal kinematic state
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0

        # Current target velocity
        self._target_v: float = 0.0
        self._target_w: float = 0.0
        self._current_v: float = 0.0
        self._current_w: float = 0.0

        # Wheel speeds (rad/s)
        self._left_wheel_vel: float = 0.0
        self._right_wheel_vel: float = 0.0

        # Battery simulation
        self._battery_voltage: float = 12.6
        self._battery_pct: float = 98.5

        # Timing tracking
        self._last_time: float = time.time()

    async def initialize(self) -> bool:
        logger.info(f"[{self.robot_id}] Initializing Differential Drive Base HAL Plugin...")
        # Simulate hardware connection check
        self._last_time = time.time()
        self.is_initialized = True
        self.current_mode = RobotMode.MANUAL
        logger.info(f"[{self.robot_id}] DiffDrive HAL successfully initialized in MANUAL mode.")
        return True

    async def read_telemetry(self) -> RobotTelemetryData:
        now = time.time()
        dt = max(0.001, min(0.5, now - self._last_time))
        self._last_time = now

        if self.current_mode == RobotMode.EMERGENCY_STOP:
            self._target_v = 0.0
            self._target_w = 0.0

        # Smooth velocity acceleration toward target (simulate motor inertia)
        alpha = 0.25
        self._current_v += alpha * (self._target_v - self._current_v)
        self._current_w += alpha * (self._target_w - self._current_w)

        # Kinematic state update (Unicycle Model)
        self._yaw += self._current_w * dt
        # Normalize yaw to [-pi, pi]
        self._yaw = (self._yaw + math.pi) % (2 * math.pi) - math.pi

        self._x += self._current_v * math.cos(self._yaw) * dt
        self._y += self._current_v * math.sin(self._yaw) * dt

        # Compute individual wheel speeds: v_l = v - (w*d)/2, v_r = v + (w*d)/2
        v_left = self._current_v - (self._current_w * self.wheel_track / 2.0)
        v_right = self._current_v + (self._current_w * self.wheel_track / 2.0)

        self._left_wheel_vel = v_left / self.wheel_radius
        self._right_wheel_vel = v_right / self.wheel_radius

        # Battery slow drain simulation based on speed
        drain_rate = 0.001 + (abs(self._current_v) + abs(self._current_w)) * 0.005
        self._battery_pct = max(0.0, self._battery_pct - (drain_rate * dt))
        self._battery_voltage = 10.0 + (self._battery_pct / 100.0) * 2.6

        # Assemble normalized telemetry structure
        pose = PoseData(
            position=Vector3D(x=round(self._x, 3), y=round(self._y, 3), z=0.0),
            orientation_rpy=Vector3D(x=0.0, y=0.0, z=round(self._yaw, 3))
        )

        twist = TwistData(
            linear=Vector3D(x=round(self._current_v, 3), y=0.0, z=0.0),
            angular=Vector3D(x=0.0, y=0.0, z=round(self._current_w, 3))
        )

        battery = BatteryData(
            voltage=round(self._battery_voltage, 2),
            percentage=round(self._battery_pct, 1),
            is_charging=False
        )

        joints = JointStateData(
            names=["left_wheel_joint", "right_wheel_joint"],
            positions=[round(self._x / self.wheel_radius, 2), round(self._y / self.wheel_radius, 2)],
            velocities=[round(self._left_wheel_vel, 2), round(self._right_wheel_vel, 2)],
            effort=[0.5, 0.5]
        )

        custom_metrics: Dict[str, Any] = {
            "cpu_temp_c": 42.5,
            "ultrasonic_front_m": round(1.5 + 0.2 * math.sin(now), 2),
            "wheel_encoder_l_ticks": int(self._x * 1000),
            "wheel_encoder_r_ticks": int(self._y * 1000),
            "firmware_version": "v2.4.1"
        }

        status_msg = "NORMAL" if self.current_mode != RobotMode.EMERGENCY_STOP else "E_STOP_ACTIVE"

        return RobotTelemetryData(
            robot_id=self.robot_id,
            robot_type=RobotType.DIFFERENTIAL_DRIVE,
            timestamp_sec=now,
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
            self._target_v = command.target_twist.linear.x
            self._target_w = command.target_twist.angular.z

        return True

    async def emergency_stop(self) -> bool:
        logger.error(f"[{self.robot_id}] EMERGENCY STOP TRIGGERED!")
        self._target_v = 0.0
        self._target_w = 0.0
        self._current_v = 0.0
        self._current_w = 0.0
        self.current_mode = RobotMode.EMERGENCY_STOP
        return True

    async def shutdown(self) -> None:
        logger.info(f"[{self.robot_id}] Shutting down DiffDrive HAL plugin.")
        await self.emergency_stop()
        self.is_initialized = False
