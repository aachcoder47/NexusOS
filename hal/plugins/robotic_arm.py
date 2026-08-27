"""
NexusOS HAL Plugin - Robotic Arm Manipulator Implementation
Simulates or interfaces with a 6-DOF industrial/collaborative robotic arm.
"""

import math
import time
import logging
from typing import Dict, Any, List

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

logger = logging.getLogger("NexusOS.HAL.RoboticArm")


class RoboticArmPlugin(AbstractHardwarePlugin):
    """
    Concrete Hardware Abstraction Layer plugin for 6-DOF Robotic Manipulators.
    Handles joint trajectory tracking and end-effector kinematics.
    """

    JOINT_NAMES = ["joint_1_base", "joint_2_shoulder", "joint_3_elbow", "joint_4_wrist1", "joint_5_wrist2", "joint_6_wrist3"]

    def __init__(self, robot_id: str, num_joints: int = 6):
        super().__init__(robot_id)
        self.num_joints = num_joints

        # Current joint positions (radians)
        self._current_joints: List[float] = [0.0] * num_joints
        self._target_joints: List[float] = [0.0] * num_joints
        self._joint_velocities: List[float] = [0.0] * num_joints
        self._joint_torques: List[float] = [0.1] * num_joints

        # Simulated end-effector pose
        self._ee_x: float = 0.5
        self._ee_y: float = 0.0
        self._ee_z: float = 0.4

        self._battery_voltage: float = 24.0
        self._battery_pct: float = 100.0
        self._last_time: float = time.time()

    async def initialize(self) -> bool:
        logger.info(f"[{self.robot_id}] Initializing 6-DOF Robotic Arm HAL Plugin...")
        self._last_time = time.time()
        self.is_initialized = True
        self.current_mode = RobotMode.MANUAL
        logger.info(f"[{self.robot_id}] Robotic Arm HAL successfully initialized in MANUAL mode.")
        return True

    async def read_telemetry(self) -> RobotTelemetryData:
        now = time.time()
        dt = max(0.001, min(0.5, now - self._last_time))
        self._last_time = now

        if self.current_mode == RobotMode.EMERGENCY_STOP:
            self._joint_velocities = [0.0] * self.num_joints
        else:
            # Interpolate joints toward targets
            max_vel = 1.0  # rad/s max velocity
            for i in range(self.num_joints):
                error = self._target_joints[i] - self._current_joints[i]
                desired_v = max(-max_vel, min(max_vel, error * 2.0))
                self._joint_velocities[i] = desired_v
                self._current_joints[i] += desired_v * dt
                self._joint_torques[i] = round(0.1 + abs(desired_v) * 0.5, 2)

        # Approximate Forward Kinematics for End-Effector position
        q1 = self._current_joints[0]
        q2 = self._current_joints[1]
        q3 = self._current_joints[2]
        r = 0.4 * math.cos(q2) + 0.3 * math.cos(q2 + q3)
        self._ee_x = r * math.cos(q1)
        self._ee_y = r * math.sin(q1)
        self._ee_z = 0.2 + 0.4 * math.sin(q2) + 0.3 * math.sin(q2 + q3)

        pose = PoseData(
            position=Vector3D(x=round(self._ee_x, 3), y=round(self._ee_y, 3), z=round(self._ee_z, 3)),
            orientation_rpy=Vector3D(x=round(self._current_joints[3], 2), y=round(self._current_joints[4], 2), z=round(self._current_joints[5], 2))
        )

        twist = TwistData(
            linear=Vector3D(x=0.0, y=0.0, z=0.0),
            angular=Vector3D(x=0.0, y=0.0, z=0.0)
        )

        battery = BatteryData(
            voltage=self._battery_voltage,
            percentage=self._battery_pct,
            is_charging=True  # Usually tethered to AC power
        )

        joints = JointStateData(
            names=self.JOINT_NAMES[:self.num_joints],
            positions=[round(p, 3) for p in self._current_joints],
            velocities=[round(v, 3) for v in self._joint_velocities],
            effort=self._joint_torques
        )

        custom_metrics: Dict[str, Any] = {
            "bus_temperature_c": 38.2,
            "gripper_state": "OPEN",
            "payload_kg": 1.25,
            "safety_circuit_ok": self.current_mode != RobotMode.EMERGENCY_STOP
        }

        return RobotTelemetryData(
            robot_id=self.robot_id,
            robot_type=RobotType.ROBOTIC_ARM,
            timestamp_sec=now,
            pose=pose,
            twist=twist,
            battery=battery,
            mode=self.current_mode,
            joints=joints,
            custom_metrics=custom_metrics,
            status_message="NORMAL" if self.current_mode != RobotMode.EMERGENCY_STOP else "E_STOP_ACTIVE"
        )

    async def send_command(self, command: MotorCommandData) -> bool:
        if self.current_mode == RobotMode.EMERGENCY_STOP:
            logger.warning(f"[{self.robot_id}] Command rejected: E-STOP is active!")
            return False

        if command.requested_mode:
            self.current_mode = command.requested_mode

        if command.target_joints and command.target_joints.positions:
            count = min(len(command.target_joints.positions), self.num_joints)
            for i in range(count):
                self._target_joints[i] = command.target_joints.positions[i]
            logger.info(f"[{self.robot_id}] New joint targets set: {self._target_joints}")

        return True

    async def emergency_stop(self) -> bool:
        logger.error(f"[{self.robot_id}] EMERGENCY STOP TRIGGERED ON ROBOTIC ARM!")
        self.current_mode = RobotMode.EMERGENCY_STOP
        self._target_joints = list(self._current_joints)
        self._joint_velocities = [0.0] * self.num_joints
        return True

    async def shutdown(self) -> None:
        logger.info(f"[{self.robot_id}] Shutting down Robotic Arm HAL plugin.")
        await self.emergency_stop()
        self.is_initialized = False
