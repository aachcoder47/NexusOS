"""
Demo: Custom Hardware Plugin Implementation for NexusOS Fleet.

This example demonstrates how developers can extend `AbstractHardwarePlugin`
to integrate custom robotics hardware (e.g. an autonomous quadcopter drone or custom AMR)
seamlessly into the NexusOS ecosystem.
"""

import time
import math
import sys
from pathlib import Path
from typing import Dict, Any, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

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


class QuadcopterDronePlugin(AbstractHardwarePlugin):
    """
    Simulated 4-Rotor Autonomous UAV Hardware Plugin for NexusOS.
    Demonstrates custom 3D kinematics, altitude control, and battery telemetry.
    """

    def __init__(self, robot_id: str):
        super().__init__(robot_id=robot_id)
        self.robot_type = RobotType.DRONE

        # Position (X, Y, Z in meters)
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0

        # Orientation angles (roll, pitch, yaw in radians)
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0

        # Velocity state
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.vz: float = 0.0
        self.yaw_rate: float = 0.0

        # Battery simulation
        self.battery_voltage: float = 14.8  # 4S LiPo battery
        self.battery_soc_pct: float = 100.0

        # State tracking
        self.armed: bool = False
        self._last_update: float = time.monotonic()

    async def initialize(self) -> bool:
        """Initialize drone avionics, sensor calibration, and telemetry."""
        self._last_update = time.monotonic()
        self.current_mode = RobotMode.MANUAL
        self.is_initialized = True
        return True

    async def read_telemetry(self) -> RobotTelemetryData:
        """Calculate flight dynamics and return real-time telemetry snapshot."""
        now = time.monotonic()
        dt = max(1e-4, now - self._last_update)
        self._last_update = now

        # Update position based on velocities
        if self.current_mode in (RobotMode.AUTONOMOUS, RobotMode.MANUAL) and self.armed:
            self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
            self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
            self.z = max(0.0, self.z + self.vz * dt)
            self.yaw = (self.yaw + self.yaw_rate * dt) % (2.0 * math.pi)

            drain_rate = 0.05 * (1.0 + abs(self.vz) + math.hypot(self.vx, self.vy))
            self.battery_soc_pct = max(0.0, self.battery_soc_pct - drain_rate * dt)
            self.battery_voltage = 13.0 + (self.battery_soc_pct / 100.0) * 1.8
        else:
            if self.z > 0.0:
                self.z = max(0.0, self.z - 2.0 * dt)

        # Build telemetry frame matching NexusOS HAL schema
        return RobotTelemetryData(
            robot_id=self.robot_id,
            robot_type=self.robot_type,
            timestamp_sec=time.time(),
            mode=self.current_mode,
            pose=PoseData(
                position=Vector3D(x=float(self.x), y=float(self.y), z=float(self.z)),
                orientation_rpy=Vector3D(x=float(self.roll), y=float(self.pitch), z=float(self.yaw))
            ),
            twist=TwistData(
                linear=Vector3D(x=float(self.vx), y=float(self.vy), z=float(self.vz)),
                angular=Vector3D(x=float(self.roll), y=float(self.pitch), z=float(self.yaw_rate))
            ),
            battery=BatteryData(
                voltage=float(self.battery_voltage),
                percentage=float(self.battery_soc_pct),
                is_charging=False
            ),
            joints=JointStateData(
                names=["rotor_1", "rotor_2", "rotor_3", "rotor_4"],
                positions=[0.0, 0.0, 0.0, 0.0],
                velocities=[800.0, 800.0, 800.0, 800.0] if self.armed else [0.0, 0.0, 0.0, 0.0]
            ),
            custom_metrics={
                "altitude_agl_meters": round(self.z, 2),
                "flight_mode": "ARMED_AIRBORNE" if self.armed and self.z > 0.1 else ("ARMED_GROUND" if self.armed else "DISARMED"),
                "estimated_flight_time_min": round((self.battery_soc_pct / 100.0) * 22.0, 1)
            },
            status_message="AIRBORNE" if self.armed and self.z > 0.1 else ("ARMED" if self.armed else "DISARMED")
        )

    async def send_command(self, command: MotorCommandData) -> bool:
        """Process incoming 3D flight control velocity vectors and arming state."""
        if self.current_mode == RobotMode.EMERGENCY_STOP:
            return False

        if command.requested_mode:
            self.current_mode = command.requested_mode

        if command.parameters and "arm" in command.parameters:
            self.armed = bool(command.parameters["arm"])

        if command.target_twist and self.armed:
            self.vx = command.target_twist.linear.x
            self.vy = command.target_twist.linear.y
            self.vz = command.target_twist.linear.z
            self.yaw_rate = command.target_twist.angular.z

        return True

    async def emergency_stop(self) -> bool:
        """Immediate rotor shutdown and motor disarm."""
        self.armed = False
        self.vx = 0.0
        self.vy = 0.0
        self.vz = -1.5
        self.yaw_rate = 0.0
        self.current_mode = RobotMode.EMERGENCY_STOP
        return True

    async def shutdown(self) -> None:
        """Gracefully land and shut down plugin."""
        await self.emergency_stop()
        self.is_initialized = False
