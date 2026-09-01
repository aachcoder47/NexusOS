"""
NexusOS Core Module - Hardware Abstraction, Registry, and Types.
"""

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
    CommandFrame,
    CommandAck,
    DeviceHandshake,
    HandshakeResponse,
    SpatialEntity,
    EnvironmentFact,
    AgentContextBlock,
    LicenseTier
)
from nexusos.core.hal import BaseRobotDriver, DifferentialDriveMockDriver, RoboticArm6DOFMockDriver
from nexusos.core.registry import DriverRegistry

__all__ = [
    "RobotType",
    "RobotMode",
    "Vector3D",
    "Quaternion3D",
    "Pose3D",
    "Twist3D",
    "BatteryState",
    "JointState",
    "SensorPayload",
    "TelemetryFrame",
    "CommandFrame",
    "CommandAck",
    "DeviceHandshake",
    "HandshakeResponse",
    "SpatialEntity",
    "EnvironmentFact",
    "AgentContextBlock",
    "LicenseTier",
    "BaseRobotDriver",
    "DifferentialDriveMockDriver",
    "RoboticArm6DOFMockDriver",
    "DriverRegistry",
]
