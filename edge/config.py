"""
NexusOS Edge Agent Configuration Schema
"""

from dataclasses import dataclass
from typing import Optional
from hal.base import RobotType


@dataclass
class EdgeConfig:
    robot_id: str = "robot-01"
    robot_type: RobotType = RobotType.DIFFERENTIAL_DRIVE
    server_url: str = "ws://localhost:8000/ws/telemetry"
    telemetry_rate_hz: float = 20.0  # Telemetry stream frequency
    watchdog_timeout_sec: float = 2.0  # E-Stop if no cloud command received for 2 seconds (in autonomous mode)
    reconnect_interval_sec: float = 3.0
    use_protobuf: bool = True
