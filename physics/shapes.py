"""
NexusOS Physics Engine - Collision Shapes & Raycasting Primitives.
Provides geometric bounding volumes, ray intersection queries, and support functions.
"""

from __future__ import annotations
import math
from typing import Optional, List, Tuple
from enum import Enum
import numpy as np

from physics.vector import Vec3, Quaternion


class ShapeType(str, Enum):
    BOX = "BOX"
    SPHERE = "SPHERE"
    CYLINDER = "CYLINDER"
    PLANE = "PLANE"


class Shape:
    """Base abstract shape class."""

    def __init__(self, shape_type: ShapeType):
        self.shape_type = shape_type

    def compute_aabb(self, position: Vec3, orientation: Quaternion) -> Tuple[Vec3, Vec3]:
        """Compute Axis-Aligned Bounding Box (min_corner, max_corner)."""
        raise NotImplementedError


class BoxShape(Shape):
    """
    3D Box (Oriented Bounding Box) defined by half extents (hx, hy, hz).
    Full dimensions: (2*hx, 2*hy, 2*hz).
    """

    def __init__(self, half_extents: Vec3):
        super().__init__(ShapeType.BOX)
        self.half_extents = half_extents.copy()

    @classmethod
    def from_dimensions(cls, width: float, length: float, height: float) -> BoxShape:
        return cls(Vec3(width * 0.5, length * 0.5, height * 0.5))

    def get_vertices(self, position: Vec3, orientation: Quaternion) -> List[Vec3]:
        """Compute the 8 world-space vertices of the box."""
        hx, hy, hz = self.half_extents.x, self.half_extents.y, self.half_extents.z
        local_verts = [
            Vec3(-hx, -hy, -hz), Vec3(hx, -hy, -hz),
            Vec3(hx,  hy, -hz), Vec3(-hx,  hy, -hz),
            Vec3(-hx, -hy,  hz), Vec3(hx, -hy,  hz),
            Vec3(hx,  hy,  hz), Vec3(-hx,  hy,  hz)
        ]
        return [position + orientation.rotate_vector(v) for v in local_verts]

    def get_axes(self, orientation: Quaternion) -> List[Vec3]:
        """Return the 3 principal local axes rotated to world orientation."""
        return [
            orientation.rotate_vector(Vec3(1.0, 0.0, 0.0)).normalized(),
            orientation.rotate_vector(Vec3(0.0, 1.0, 0.0)).normalized(),
            orientation.rotate_vector(Vec3(0.0, 0.0, 1.0)).normalized()
        ]

    def compute_aabb(self, position: Vec3, orientation: Quaternion) -> Tuple[Vec3, Vec3]:
        verts = self.get_vertices(position, orientation)
        min_v = Vec3(min(v.x for v in verts), min(v.y for v in verts), min(v.z for v in verts))
        max_v = Vec3(max(v.x for v in verts), max(v.y for v in verts), max(v.z for v in verts))
        return (min_v, max_v)


class SphereShape(Shape):
    """3D Sphere defined by radius r."""

    def __init__(self, radius: float):
        super().__init__(ShapeType.SPHERE)
        self.radius = float(radius)

    def compute_aabb(self, position: Vec3, orientation: Quaternion) -> Tuple[Vec3, Vec3]:
        r = self.radius
        min_v = position - Vec3(r, r, r)
        max_v = position + Vec3(r, r, r)
        return (min_v, max_v)


class CylinderShape(Shape):
    """3D Cylinder aligned with local z-axis with radius r and half height hz."""

    def __init__(self, radius: float, height: float):
        super().__init__(ShapeType.CYLINDER)
        self.radius = float(radius)
        self.half_height = float(height * 0.5)

    def compute_aabb(self, position: Vec3, orientation: Quaternion) -> Tuple[Vec3, Vec3]:
        r = self.radius
        hz = self.half_height
        extent = math.sqrt(r * r + hz * hz)
        return (position - Vec3(extent, extent, extent), position + Vec3(extent, extent, extent))


class PlaneShape(Shape):
    """Infinite or bounded 3D Plane defined by normal n and distance d (n . p = d)."""

    def __init__(self, normal: Vec3 = Vec3(0.0, 0.0, 1.0), distance: float = 0.0):
        super().__init__(ShapeType.PLANE)
        self.normal = normal.normalized()
        self.distance = float(distance)

    def compute_aabb(self, position: Vec3, orientation: Quaternion) -> Tuple[Vec3, Vec3]:
        inf = 1e6
        return (Vec3(-inf, -inf, -inf), Vec3(inf, inf, inf))


class Ray:
    """Ray used for sensor simulation (LiDAR, ultrasonic, depth camera) and raycasting."""

    __slots__ = ("origin", "direction", "max_distance")

    def __init__(self, origin: Vec3, direction: Vec3, max_distance: float = 100.0):
        self.origin = origin.copy()
        self.direction = direction.normalized()
        self.max_distance = float(max_distance)

    def point_at(self, t: float) -> Vec3:
        return self.origin + self.direction * t


class RaycastHit:
    """Result of a raycast query against physical geometry."""

    __slots__ = ("hit", "distance", "point", "normal", "collider_id")

    def __init__(
        self,
        hit: bool = False,
        distance: float = float("inf"),
        point: Optional[Vec3] = None,
        normal: Optional[Vec3] = None,
        collider_id: str = ""
    ):
        self.hit = hit
        self.distance = distance
        self.point = point if point else Vec3()
        self.normal = normal if normal else Vec3(0.0, 0.0, 1.0)
        self.collider_id = collider_id

    def to_dict(self) -> dict:
        return {
            "hit": self.hit,
            "distance": round(self.distance, 3),
            "point": self.point.to_dict(),
            "normal": self.normal.to_dict(),
            "collider_id": self.collider_id
        }
