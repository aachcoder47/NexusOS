"""
NexusOS Core Data Types & Pydantic v2 Schemas.
Hardware-agnostic primitives for kinematics, telemetry, control frames, and spatial entities.
"""

from __future__ import annotations
import time
from enum import Enum
from typing import Dict, List, Optional, Any, Tuple
from pydantic import BaseModel, Field, ConfigDict


class RobotType(str, Enum):
    UNSPECIFIED = "UNSPECIFIED"
    DIFFERENTIAL_DRIVE = "DIFFERENTIAL_DRIVE"
    ROBOTIC_ARM = "ROBOTIC_ARM"
    QUADRUPED = "QUADRUPED"
    DRONE = "DRONE"
    HUMANOID = "HUMANOID"


class RobotMode(str, Enum):
    UNINITIALIZED = "UNINITIALIZED"
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    GRACE_FALLBACK = "GRACE_FALLBACK"
    FAULT = "FAULT"


class LicenseTier(str, Enum):
    UNSPECIFIED = "UNSPECIFIED"
    COMMUNITY = "COMMUNITY"
    COMMERCIAL = "COMMERCIAL"
    ENTERPRISE = "ENTERPRISE"


# ─────────────────────────────────────────────────────────────────────────────
# 3D Math Primitives
# ─────────────────────────────────────────────────────────────────────────────

class Vector3D(BaseModel):
    model_config = ConfigDict(frozen=True)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)


class Quaternion3D(BaseModel):
    model_config = ConfigDict(frozen=True)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


class Pose3D(BaseModel):
    position: Vector3D = Field(default_factory=Vector3D)
    orientation: Quaternion3D = Field(default_factory=Quaternion3D)
    orientation_rpy: Optional[Vector3D] = None  # Roll, Pitch, Yaw in radians


class Twist3D(BaseModel):
    linear: Vector3D = Field(default_factory=Vector3D)
    angular: Vector3D = Field(default_factory=Vector3D)


class BatteryState(BaseModel):
    voltage: float = 12.0
    percentage: float = 100.0
    current_draw_a: float = 0.0
    temperature_c: float = 25.0
    is_charging: bool = False


class JointState(BaseModel):
    names: List[str] = Field(default_factory=list)
    positions: List[float] = Field(default_factory=list)
    velocities: List[float] = Field(default_factory=list)
    effort: List[float] = Field(default_factory=list)


class SensorPayload(BaseModel):
    sensor_id: str
    sensor_type: str  # "LIDAR_2D", "LIDAR_3D", "IMU_6AXIS", "RGBD_CAMERA", etc.
    ranges: List[float] = Field(default_factory=list)
    point_cloud: List[List[float]] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)


# ─────────────────────────────────────────────────────────────────────────────
# Telemetry & Control Frames
# ─────────────────────────────────────────────────────────────────────────────

class DeviceHandshake(BaseModel):
    robot_id: str
    robot_type: RobotType = RobotType.DIFFERENTIAL_DRIVE
    hardware_signature: str
    firmware_version: str = "1.0.0"
    license_token: str = ""
    capabilities: List[str] = Field(default_factory=list)
    timestamp_utc: int = Field(default_factory=lambda: int(time.time()))


class HandshakeResponse(BaseModel):
    accepted: bool
    session_id: str
    active_tier: LicenseTier = LicenseTier.COMMUNITY
    grace_mode_active: bool = False
    error_reason: str = ""
    server_time_utc: int = Field(default_factory=lambda: int(time.time()))


class TelemetryFrame(BaseModel):
    robot_id: str
    robot_type: RobotType
    timestamp_sec: float = Field(default_factory=time.time)
    pose: Pose3D = Field(default_factory=Pose3D)
    twist: Twist3D = Field(default_factory=Twist3D)
    battery: BatteryState = Field(default_factory=BatteryState)
    mode: RobotMode = RobotMode.MANUAL
    joints: JointState = Field(default_factory=JointState)
    sensors: List[SensorPayload] = Field(default_factory=list)
    custom_metrics: Dict[str, Any] = Field(default_factory=dict)
    status_message: str = "OK"
    is_safety_isolated: bool = False


class CommandFrame(BaseModel):
    command_id: str
    robot_id: str
    timestamp_sec: float = Field(default_factory=time.time)
    requested_mode: Optional[RobotMode] = None
    target_twist: Optional[Twist3D] = None
    target_joints: Optional[JointState] = None
    target_waypoint: Optional[Pose3D] = None
    emergency_stop: bool = False
    parameters: Dict[str, Any] = Field(default_factory=dict)


class CommandAck(BaseModel):
    command_id: str
    robot_id: str
    executed: bool
    message: str = "ACK"
    execution_latency_ms: float = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Agent Context & Spatial Memory Types
# ─────────────────────────────────────────────────────────────────────────────

class SpatialEntity(BaseModel):
    entity_id: str
    label: str
    position: Vector3D
    bounding_box: Vector3D = Field(default_factory=Vector3D)
    confidence: float = 1.0
    first_seen_utc: int = Field(default_factory=lambda: int(time.time()))
    last_seen_utc: int = Field(default_factory=lambda: int(time.time()))
    attributes: Dict[str, Any] = Field(default_factory=dict)


class EnvironmentFact(BaseModel):
    fact_id: str
    subject: str
    predicate: str
    object: str
    temporal_validity_sec: float = 3600.0
    timestamp: float = Field(default_factory=time.time)


class AgentContextBlock(BaseModel):
    context_id: str
    robot_id: str
    timestamp_sec: float = Field(default_factory=time.time)
    current_pose: Pose3D = Field(default_factory=Pose3D)
    detected_entities: List[SpatialEntity] = Field(default_factory=list)
    facts: List[EnvironmentFact] = Field(default_factory=list)
    embedding_vector: List[float] = Field(default_factory=list)
    conversational_state: str = ""
    active_intent: str = ""
