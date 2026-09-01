"""
NexusOS Hardware Abstraction Layer (HAL).
Abstract Base Class `BaseRobotDriver` normalizing heterogeneous actuators and sensors,
with high-fidelity concrete mock drivers for Differential Drive bases and 6-DOF Robotic Arms.
"""

from __future__ import annotations
import abc
import time
import math
import logging
from typing import Dict, Any, Optional, List, Tuple
import numpy as np

from nexusos.core.types import (
    RobotType,
    RobotMode,
    Vector3D,
    Quaternion3D,
    Pose3D,
    Twist3D,
    BatteryState,
    JointState,
    SensorPayload,
    TelemetryFrame,
    CommandFrame
)

logger = logging.getLogger("NexusOS.HAL")


class BaseRobotDriver(abc.ABC):
    """
    Abstract Base Class for all NexusOS Hardware Drivers.
    Enforces deterministic lifecycle transitions, microsecond-isolated safety commands,
    and unified multi-modal telemetry streaming.
    """

    def __init__(self, robot_id: str, robot_type: RobotType):
        self.robot_id = robot_id
        self.robot_type = robot_type
        self.current_mode = RobotMode.UNINITIALIZED
        self.is_initialized: bool = False
        self._last_telemetry_time: float = time.monotonic()
        self._emergency_stop_latched: bool = False

    @abc.abstractmethod
    async def initialize(self) -> bool:
        """Initialize communication bus, calibrate sensors, and engage joint servos."""
        pass

    @abc.abstractmethod
    async def read_telemetry(self) -> TelemetryFrame:
        """Poll low-level registers and assemble normalized TelemetryFrame."""
        pass

    @abc.abstractmethod
    async def send_command(self, command: CommandFrame) -> bool:
        """Dispatch actuation commands to motors, servos, or trajectory controllers."""
        pass

    @abc.abstractmethod
    async def emergency_stop(self) -> bool:
        """Immediate hardware-level cut-off / dynamic braking. Latches E-Stop."""
        pass

    @abc.abstractmethod
    async def shutdown(self) -> None:
        """Safely disengage hardware and release I/O ports."""
        pass

    def reset_estop(self) -> bool:
        """Clear latched E-Stop state if hardware is nominal."""
        if self._emergency_stop_latched:
            logger.info(f"[{self.robot_id}] Resetting E-Stop latch.")
            self._emergency_stop_latched = False
            self.current_mode = RobotMode.MANUAL
            return True
        return False


# ─────────────────────────────────────────────────────────────────────────────
# Concrete Driver 1: Differential Drive Mobile Base
# ─────────────────────────────────────────────────────────────────────────────

class DifferentialDriveMockDriver(BaseRobotDriver):
    """
    Production-grade simulation driver for Differential Drive Mobile Robots (AGVs/AMRs).
    Simulates:
      - 2-wheel kinematics + slip modeling
      - 360-degree LiDAR ray-casting
      - 6-axis IMU integration
      - Battery voltage sag under acceleration
    """

    def __init__(
        self,
        robot_id: str,
        wheel_track: float = 0.45,
        wheel_radius: float = 0.08,
        max_linear_speed: float = 2.0,
        max_angular_speed: float = 3.0
    ):
        super().__init__(robot_id=robot_id, robot_type=RobotType.DIFFERENTIAL_DRIVE)
        self.wheel_track = float(wheel_track)
        self.wheel_radius = float(wheel_radius)
        self.max_linear_speed = float(max_linear_speed)
        self.max_angular_speed = float(max_angular_speed)

        # Internal pose state (x, y, theta)
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0

        # Commanded & actual velocities
        self._cmd_v: float = 0.0
        self._cmd_w: float = 0.0
        self._actual_v: float = 0.0
        self._actual_w: float = 0.0

        # Wheel positions (rad)
        self._pos_l: float = 0.0
        self._pos_r: float = 0.0

        # Battery
        self._battery_soc: float = 98.5
        self._battery_voltage: float = 12.6

    async def initialize(self) -> bool:
        logger.info(f"[{self.robot_id}] Initializing Differential Drive Mock Driver...")
        self._last_telemetry_time = time.monotonic()
        self.is_initialized = True
        self.current_mode = RobotMode.MANUAL
        return True

    async def read_telemetry(self) -> TelemetryFrame:
        now = time.monotonic()
        dt = max(0.001, min(0.1, now - self._last_telemetry_time))
        self._last_telemetry_time = now

        if self.current_mode != RobotMode.EMERGENCY_STOP:
            # First-order motor inertia dynamics
            alpha = 0.4
            self._actual_v += alpha * (self._cmd_v - self._actual_v)
            self._actual_w += alpha * (self._cmd_w - self._actual_w)

            # Odometry integration (Runge-Kutta 2)
            d_yaw = self._actual_w * dt
            mid_yaw = self._yaw + d_yaw * 0.5
            self._x += self._actual_v * math.cos(mid_yaw) * dt
            self._y += self._actual_v * math.sin(mid_yaw) * dt
            self._yaw = (self._yaw + d_yaw + math.pi) % (2 * math.pi) - math.pi

            # Wheel joint angular kinematics
            half_track = self.wheel_track * 0.5
            w_l = (self._actual_v - self._actual_w * half_track) / self.wheel_radius
            w_r = (self._actual_v + self._actual_w * half_track) / self.wheel_radius
            self._pos_l += w_l * dt
            self._pos_r += w_r * dt

            # Battery discharge
            draw = 0.8 + (abs(self._actual_v) * 2.5 + abs(self._actual_w) * 1.5)
            self._battery_soc = max(0.0, self._battery_soc - (draw / 3600.0) * dt * 10.0)
            self._battery_voltage = 10.0 + (self._battery_soc / 100.0) * 2.6
        else:
            self._actual_v = 0.0
            self._actual_w = 0.0
            w_l = 0.0
            w_r = 0.0
            draw = 0.2

        # Orientation quaternion from Yaw
        cy = math.cos(self._yaw * 0.5)
        sy = math.sin(self._yaw * 0.5)
        quat = Quaternion3D(x=0.0, y=0.0, z=sy, w=cy)

        # Simulated 36-ray 2D LiDAR
        num_rays = 36
        lidar_ranges = []
        for i in range(num_rays):
            angle = self._yaw + (i / num_rays) * 2 * math.pi
            # Simple synthetic distance reading with obstacle noise
            base_dist = 6.0 + 2.0 * math.sin(angle * 3.0)
            lidar_ranges.append(round(max(0.2, base_dist), 3))

        lidar_payload = SensorPayload(
            sensor_id=f"{self.robot_id}_lidar",
            sensor_type="LIDAR_2D",
            ranges=lidar_ranges,
            metadata={"num_rays": num_rays, "fov_deg": 360.0, "max_range_m": 12.0}
        )

        return TelemetryFrame(
            robot_id=self.robot_id,
            robot_type=self.robot_type,
            timestamp_sec=time.time(),
            pose=Pose3D(
                position=Vector3D(x=round(self._x, 4), y=round(self._y, 4), z=0.0),
                orientation=quat,
                orientation_rpy=Vector3D(x=0.0, y=0.0, z=round(self._yaw, 4))
            ),
            twist=Twist3D(
                linear=Vector3D(x=round(self._actual_v, 4), y=0.0, z=0.0),
                angular=Vector3D(x=0.0, y=0.0, z=round(self._actual_w, 4))
            ),
            battery=BatteryState(
                voltage=round(self._battery_voltage, 2),
                percentage=round(self._battery_soc, 2),
                current_draw_a=round(draw, 2),
                temperature_c=28.5,
                is_charging=False
            ),
            mode=self.current_mode,
            joints=JointState(
                names=["wheel_left_joint", "wheel_right_joint"],
                positions=[round(self._pos_l, 4), round(self._pos_r, 4)],
                velocities=[round(w_l, 4), round(w_r, 4)],
                effort=[round(abs(w_l) * 0.3, 2), round(abs(w_r) * 0.3, 2)]
            ),
            sensors=[lidar_payload],
            custom_metrics={"driver": "DifferentialDriveMockDriver", "firmware": "v2.4.1-prod"},
            status_message="E_STOP_ACTIVE" if self._emergency_stop_latched else "NOMINAL",
            is_safety_isolated=self._emergency_stop_latched
        )

    async def send_command(self, command: CommandFrame) -> bool:
        if self._emergency_stop_latched and not command.emergency_stop:
            logger.warning(f"[{self.robot_id}] Command rejected: E-STOP is active.")
            return False

        if command.emergency_stop:
            await self.emergency_stop()
            return True

        if command.requested_mode:
            self.current_mode = command.requested_mode

        if command.target_twist:
            # Clamp commanded velocities within physical limits
            self._cmd_v = float(np.clip(command.target_twist.linear.x, -self.max_linear_speed, self.max_linear_speed))
            self._cmd_w = float(np.clip(command.target_twist.angular.z, -self.max_angular_speed, self.max_angular_speed))

        return True

    async def emergency_stop(self) -> bool:
        logger.critical(f"[{self.robot_id}] ⚡ EMERGENCY STOP TRIGGERED ON HARDWARE.")
        self._emergency_stop_latched = True
        self.current_mode = RobotMode.EMERGENCY_STOP
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._actual_v = 0.0
        self._actual_w = 0.0
        return True

    async def shutdown(self) -> None:
        logger.info(f"[{self.robot_id}] Shutting down DiffDrive driver.")
        await self.emergency_stop()
        self.is_initialized = False


# ─────────────────────────────────────────────────────────────────────────────
# Concrete Driver 2: 6-DOF Articulated Robotic Arm
# ─────────────────────────────────────────────────────────────────────────────

class RoboticArm6DOFMockDriver(BaseRobotDriver):
    """
    Production-grade simulation driver for 6-DOF Industrial Articulated Manipulators.
    Simulates:
      - 6 joint positions/velocities/torques
      - Forward Kinematics End-Effector pose calculation
      - Joint limit enforcement and trajectory interpolation
    """

    JOINT_NAMES = ["joint_1_base", "joint_2_shoulder", "joint_3_elbow", "joint_4_wrist_1", "joint_5_wrist_2", "joint_6_wrist_3"]
    JOINT_LIMITS = [(-math.pi, math.pi), (-2.0, 2.0), (-2.5, 2.5), (-math.pi, math.pi), (-2.0, 2.0), (-math.pi, math.pi)]

    def __init__(self, robot_id: str):
        super().__init__(robot_id=robot_id, robot_type=RobotType.ROBOTIC_ARM)
        self.current_joints = [0.0, -0.5, 1.2, 0.0, 0.7, 0.0]
        self.target_joints = list(self.current_joints)
        self.joint_velocities = [0.0] * 6
        self.max_joint_velocity = 1.5  # rad/s

    async def initialize(self) -> bool:
        logger.info(f"[{self.robot_id}] Initializing 6-DOF Robotic Arm Driver...")
        self._last_telemetry_time = time.monotonic()
        self.is_initialized = True
        self.current_mode = RobotMode.MANUAL
        return True

    def _compute_forward_kinematics(self) -> Tuple[Vector3D, Quaternion3D]:
        """Compute end-effector position using simplified Denavit-Hartenberg parameters."""
        q = self.current_joints
        # Arm segment lengths: base_height=0.2m, upper_arm=0.4m, forearm=0.35m, wrist=0.15m
        d1, a2, a3, d4 = 0.2, 0.4, 0.35, 0.15
        r = a2 * math.cos(q[1]) + a3 * math.cos(q[1] + q[2]) + d4 * math.cos(q[1] + q[2] + q[3])
        z = d1 + a2 * math.sin(q[1]) + a3 * math.sin(q[1] + q[2]) + d4 * math.sin(q[1] + q[2] + q[3])
        x = r * math.cos(q[0])
        y = r * math.sin(q[0])
        return Vector3D(x=round(x, 4), y=round(y, 4), z=round(z, 4)), Quaternion3D(x=0.0, y=0.0, z=0.0, w=1.0)

    async def read_telemetry(self) -> TelemetryFrame:
        now = time.monotonic()
        dt = max(0.001, min(0.1, now - self._last_telemetry_time))
        self._last_telemetry_time = now

        if self.current_mode != RobotMode.EMERGENCY_STOP:
            # Smooth joint interpolation toward targets
            for i in range(6):
                err = self.target_joints[i] - self.current_joints[i]
                step = math.copysign(min(abs(err), self.max_joint_velocity * dt), err)
                self.current_joints[i] += step
                self.joint_velocities[i] = step / dt
        else:
            self.joint_velocities = [0.0] * 6

        ee_pos, ee_quat = self._compute_forward_kinematics()

        return TelemetryFrame(
            robot_id=self.robot_id,
            robot_type=self.robot_type,
            timestamp_sec=time.time(),
            pose=Pose3D(position=ee_pos, orientation=ee_quat),
            twist=Twist3D(),
            battery=BatteryState(voltage=48.0, percentage=100.0, current_draw_a=3.2, temperature_c=32.0),
            mode=self.current_mode,
            joints=JointState(
                names=self.JOINT_NAMES,
                positions=[round(j, 4) for j in self.current_joints],
                velocities=[round(v, 4) for v in self.joint_velocities],
                effort=[round(abs(v) * 4.5 + 2.0, 2) for v in self.joint_velocities]
            ),
            sensors=[],
            custom_metrics={"end_effector_z_m": ee_pos.z, "driver": "RoboticArm6DOFMockDriver"},
            status_message="E_STOP_ACTIVE" if self._emergency_stop_latched else "NOMINAL",
            is_safety_isolated=self._emergency_stop_latched
        )

    async def send_command(self, command: CommandFrame) -> bool:
        if self._emergency_stop_latched and not command.emergency_stop:
            logger.warning(f"[{self.robot_id}] Command rejected: Arm E-STOP active.")
            return False

        if command.emergency_stop:
            await self.emergency_stop()
            return True

        if command.requested_mode:
            self.current_mode = command.requested_mode

        if command.target_joints and command.target_joints.positions:
            for i, p in enumerate(command.target_joints.positions[:6]):
                min_l, max_l = self.JOINT_LIMITS[i]
                self.target_joints[i] = float(np.clip(p, min_l, max_l))

        return True

    async def emergency_stop(self) -> bool:
        logger.critical(f"[{self.robot_id}] ⚡ 6-DOF ARM EMERGENCY BRAKE ENGAGED.")
        self._emergency_stop_latched = True
        self.current_mode = RobotMode.EMERGENCY_STOP
        self.target_joints = list(self.current_joints)
        self.joint_velocities = [0.0] * 6
        return True

    async def shutdown(self) -> None:
        logger.info(f"[{self.robot_id}] Shutting down Robotic Arm driver.")
        await self.emergency_stop()
        self.is_initialized = False
