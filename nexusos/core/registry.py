"""
NexusOS Driver & Plugin Registry.
Thread-safe factory for discovering, registering, and dynamically instantiating hardware drivers.
"""

from __future__ import annotations
import logging
from typing import Dict, Type, Optional, List
from nexusos.core.types import RobotType
from nexusos.core.hal import BaseRobotDriver, DifferentialDriveMockDriver, RoboticArm6DOFMockDriver

logger = logging.getLogger("NexusOS.Registry")


class DriverRegistry:
    """Thread-safe registry for robot drivers."""

    _registry: Dict[str, Type[BaseRobotDriver]] = {}

    @classmethod
    def register(cls, name: str, driver_cls: Type[BaseRobotDriver]) -> None:
        """Register a driver class under a unique type key."""
        cls._registry[name.upper()] = driver_cls
        logger.info(f"Registered driver '{name.upper()}' -> {driver_cls.__name__}")

    @classmethod
    def get_driver_class(cls, name: str) -> Optional[Type[BaseRobotDriver]]:
        """Retrieve registered driver class by name."""
        return cls._registry.get(name.upper())

    @classmethod
    def create_driver(cls, name: str, robot_id: str, **kwargs) -> BaseRobotDriver:
        """Instantiate a registered driver."""
        driver_cls = cls.get_driver_class(name)
        if driver_cls is None:
            raise KeyError(f"Driver type '{name}' is not registered. Available: {list(cls._registry.keys())}")
        return driver_cls(robot_id=robot_id, **kwargs)

    @classmethod
    def list_registered_drivers(cls) -> List[str]:
        """Return list of all registered driver names."""
        return list(cls._registry.keys())


# Register default standard drivers
DriverRegistry.register("DIFFERENTIAL_DRIVE", DifferentialDriveMockDriver)
DriverRegistry.register("DIFF_DRIVE", DifferentialDriveMockDriver)
DriverRegistry.register("ROBOTIC_ARM", RoboticArm6DOFMockDriver)
DriverRegistry.register("ARM_6DOF", RoboticArm6DOFMockDriver)
