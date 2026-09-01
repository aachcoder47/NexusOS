"""
NexusOS: Universal Robotics Operating System and Agent Context Middleware.
Hardware-agnostic robot control, high-frequency HAL, real-time spatial memory, and fleet orchestration.
"""

__version__ = "1.0.0"
__author__ = "NexusOS Open Core & Robotics Maintainers"
__license__ = "NexusOS Community & Commercial License (NCL-1.0)"

from nexusos.core.types import (
    RobotType,
    RobotMode,
    Vector3D,
    Quaternion3D,
    Pose3D,
    Twist3D,
    BatteryState,
    JointState,
    TelemetryFrame,
    CommandFrame,
    DeviceHandshake,
    AgentContextBlock
)
from nexusos.core.hal import BaseRobotDriver, DifferentialDriveMockDriver, RoboticArm6DOFMockDriver
from nexusos.core.registry import DriverRegistry
from nexusos.memory.engine import AgentContextEngine
from nexusos.agent.daemon import EdgeDaemon
from nexusos.licensing.verifier import LicenseVerifier, LicenseTier

__all__ = [
    "RobotType",
    "RobotMode",
    "Vector3D",
    "Quaternion3D",
    "Pose3D",
    "Twist3D",
    "BatteryState",
    "JointState",
    "TelemetryFrame",
    "CommandFrame",
    "DeviceHandshake",
    "AgentContextBlock",
    "BaseRobotDriver",
    "DifferentialDriveMockDriver",
    "RoboticArm6DOFMockDriver",
    "DriverRegistry",
    "AgentContextEngine",
    "EdgeDaemon",
    "LicenseVerifier",
    "LicenseTier",
]
