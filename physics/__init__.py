"""
NexusOS Physics Engine Module.
Provides 3D rigid body dynamics, gravity simulation, SAT collision detection,
impulse-based contact solvers, LiDAR / IMU sensor simulators, and vehicle dynamics models.
"""

from physics.vector import Vec3, Quaternion
from physics.shapes import (
    ShapeType,
    Shape,
    BoxShape,
    SphereShape,
    CylinderShape,
    PlaneShape,
    Ray,
    RaycastHit
)
from physics.body import BodyType, RigidBody3D
from physics.collision import CollisionDetector, ImpulseSolver, ContactManifold
from physics.world import PhysicsWorld
from physics.sensors import LiDARSensor, IMUSensor
from physics.materials import (
    MaterialProperties,
    TireSurface,
    MATERIALS,
    TIRE_SURFACES
)
from physics.vehicle import (
    DCMotorModel,
    WheelModel,
    SuspensionModel,
    AerodynamicsModel,
    BatteryModel
)

__all__ = [
    "Vec3",
    "Quaternion",
    "ShapeType",
    "Shape",
    "BoxShape",
    "SphereShape",
    "CylinderShape",
    "PlaneShape",
    "Ray",
    "RaycastHit",
    "BodyType",
    "RigidBody3D",
    "CollisionDetector",
    "ImpulseSolver",
    "ContactManifold",
    "PhysicsWorld",
    "LiDARSensor",
    "IMUSensor",
    "MaterialProperties",
    "TireSurface",
    "MATERIALS",
    "TIRE_SURFACES",
    "DCMotorModel",
    "WheelModel",
    "SuspensionModel",
    "AerodynamicsModel",
    "BatteryModel"
]
