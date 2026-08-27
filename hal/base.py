"""
NexusOS - Hardware Abstraction Layer (HAL) Base Classes
Defines the normalized interfaces and data schemas for heterogeneous robot hardware.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Any
import time


class RobotType(str, Enum):
    DIFFERENTIAL_DRIVE = "DIFFERENTIAL_DRIVE"
    ROBOTIC_ARM = "ROBOTIC_ARM"
    QUADRUPED = "QUADRUPED"
    DRONE = "DRONE"
    GENERIC = "GENERIC"


class RobotMode(str, Enum):
    UNINITIALIZED = "UNINITIALIZED"
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    FAULT = "FAULT"


@dataclass
class Vector3D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class PoseData:
    position: Vector3D = field(default_factory=Vector3D)
    orientation_rpy: Vector3D = field(default_factory=Vector3D)  # Roll, Pitch, Yaw in radians


@dataclass
class TwistData:
    linear: Vector3D = field(default_factory=Vector3D)   # Velocity m/s
    angular: Vector3D = field(default_factory=Vector3D)  # Angular velocity rad/s


@dataclass
class BatteryData:
    voltage: float = 12.6
    percentage: float = 100.0
    is_charging: bool = False


@dataclass
class JointStateData:
    names: List[str] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)   # Radians or Meters
    velocities: List[float] = field(default_factory=list)  # Rad/s or M/s
    effort: List[float] = field(default_factory=list)      # Nm or N


@dataclass
class RobotTelemetryData:
    robot_id: str
    robot_type: RobotType
    timestamp_sec: float = field(default_factory=time.time)
    pose: PoseData = field(default_factory=PoseData)
    twist: TwistData = field(default_factory=TwistData)
    battery: BatteryData = field(default_factory=BatteryData)
    mode: RobotMode = RobotMode.UNINITIALIZED
    joints: JointStateData = field(default_factory=JointStateData)
    custom_metrics: Dict[str, Any] = field(default_factory=dict)
    status_message: str = "OK"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "robot_type": self.robot_type.value,
            "timestamp_sec": self.timestamp_sec,
            "pose": {
                "position": {"x": self.pose.position.x, "y": self.pose.position.y, "z": self.pose.position.z},
                "orientation_rpy": {"roll": self.pose.orientation_rpy.x, "pitch": self.pose.orientation_rpy.y, "yaw": self.pose.orientation_rpy.z}
            },
            "twist": {
                "linear": {"x": self.twist.linear.x, "y": self.twist.linear.y, "z": self.twist.linear.z},
                "angular": {"x": self.twist.angular.x, "y": self.twist.angular.y, "z": self.twist.angular.z}
            },
            "battery": {
                "voltage": self.battery.voltage,
                "percentage": self.battery.percentage,
                "is_charging": self.battery.is_charging
            },
            "mode": self.mode.value,
            "joints": {
                "names": self.joints.names,
                "positions": self.joints.positions,
                "velocities": self.joints.velocities,
                "effort": self.joints.effort
            },
            "custom_metrics": self.custom_metrics,
            "status_message": self.status_message
        }


@dataclass
class MotorCommandData:
    target_twist: Optional[TwistData] = None
    target_joints: Optional[JointStateData] = None
    requested_mode: Optional[RobotMode] = None
    parameters: Dict[str, Any] = field(default_factory=dict)


class AbstractHardwarePlugin(ABC):
    """
    Abstract Base Class for Hardware Plugins.
    Normalizes low-level driver inputs/outputs to standard NexusOS data structures.
    """

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.is_initialized: bool = False
        self.current_mode: RobotMode = RobotMode.UNINITIALIZED

    @abstractmethod
    async def initialize(self) -> bool:
        """Initialize connection to physical driver / micro-controller / hardware."""
        pass

    @abstractmethod
    async def read_telemetry(self) -> RobotTelemetryData:
        """Read and return normalized telemetry from hardware sensors."""
        pass

    @abstractmethod
    async def send_command(self, command: MotorCommandData) -> bool:
        """Execute motor velocity or position target commands on hardware."""
        pass

    @abstractmethod
    async def emergency_stop(self) -> bool:
        """Trigger immediate hard stop / e-stop sequence."""
        pass

    @abstractmethod
    async def shutdown(self) -> None:
        """Safe shutdown of hardware outputs and close driver interfaces."""
        pass
