"""
NexusOS Physics Engine - High-Performance 3D Vector & Transform Math.
Pure Python + NumPy for zero external C++ engine dependencies.
"""

from __future__ import annotations
import math
from typing import Tuple, Union, List
import numpy as np


class Vec3:
    """3D Vector class optimized with numpy backing and high-throughput vector math."""

    __slots__ = ("_v",)

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self._v = np.array([x, y, z], dtype=np.float64)

    @classmethod
    def from_array(cls, arr: Union[np.ndarray, List[float], Tuple[float, ...]]) -> Vec3:
        v = cls()
        v._v = np.asarray(arr, dtype=np.float64)[:3]
        return v

    @property
    def x(self) -> float:
        return float(self._v[0])

    @x.setter
    def x(self, val: float):
        self._v[0] = val

    @property
    def y(self) -> float:
        return float(self._v[1])

    @y.setter
    def y(self, val: float):
        self._v[1] = val

    @property
    def z(self) -> float:
        return float(self._v[2])

    @z.setter
    def z(self, val: float):
        self._v[2] = val

    @property
    def array(self) -> np.ndarray:
        return self._v

    def copy(self) -> Vec3:
        v = Vec3()
        v._v = np.copy(self._v)
        return v

    def __add__(self, other: Union[Vec3, float]) -> Vec3:
        v = Vec3()
        if isinstance(other, Vec3):
            v._v = self._v + other._v
        else:
            v._v = self._v + other
        return v

    def __radd__(self, other: Union[Vec3, float]) -> Vec3:
        return self.__add__(other)

    def __sub__(self, other: Union[Vec3, float]) -> Vec3:
        v = Vec3()
        if isinstance(other, Vec3):
            v._v = self._v - other._v
        else:
            v._v = self._v - other
        return v

    def __mul__(self, scalar: float) -> Vec3:
        v = Vec3()
        v._v = self._v * scalar
        return v

    def __rmul__(self, scalar: float) -> Vec3:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> Vec3:
        if abs(scalar) < 1e-12:
            raise ZeroDivisionError("Division by zero in Vec3")
        v = Vec3()
        v._v = self._v / scalar
        return v

    def __neg__(self) -> Vec3:
        v = Vec3()
        v._v = -self._v
        return v

    def dot(self, other: Vec3) -> float:
        return float(np.dot(self._v, other._v))

    def cross(self, other: Vec3) -> Vec3:
        v = Vec3()
        v._v = np.cross(self._v, other._v)
        return v

    def length_sq(self) -> float:
        return float(np.dot(self._v, self._v))

    def length(self) -> float:
        return float(np.linalg.norm(self._v))

    def normalized(self) -> Vec3:
        l = self.length()
        if l < 1e-12:
            return Vec3(0.0, 0.0, 0.0)
        return self / l

    def distance_to(self, other: Vec3) -> float:
        return float(np.linalg.norm(self._v - other._v))

    def to_tuple(self) -> Tuple[float, float, float]:
        return (float(self._v[0]), float(self._v[1]), float(self._v[2]))

    def to_dict(self) -> dict:
        return {"x": round(float(self._v[0]), 4), "y": round(float(self._v[1]), 4), "z": round(float(self._v[2]), 4)}

    def __repr__(self) -> str:
        return f"Vec3({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"


class Quaternion:
    """Quaternion representation for 3D rigid body orientation and rotation operations."""

    __slots__ = ("w", "x", "y", "z")

    def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Create Quaternion from Euler angles (Roll, Pitch, Yaw in radians)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return cls(w, x, y, z).normalized()

    def to_euler(self) -> Tuple[float, float, float]:
        """Convert quaternion to Euler angles (Roll, Pitch, Yaw in radians)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (self.w * self.y - self.z * self.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    def copy(self) -> Quaternion:
        return Quaternion(self.w, self.x, self.y, self.z)

    def normalized(self) -> Quaternion:
        norm = math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        if norm < 1e-12:
            return Quaternion(1.0, 0.0, 0.0, 0.0)
        return Quaternion(self.w / norm, self.x / norm, self.y / norm, self.z / norm)

    def rotate_vector(self, v: Vec3) -> Vec3:
        """Rotate a 3D vector by this quaternion."""
        u = Vec3(self.x, self.y, self.z)
        s = self.w
        return 2.0 * u.dot(v) * u + (s * s - u.dot(u)) * v + 2.0 * s * u.cross(v)

    def to_rotation_matrix(self) -> np.ndarray:
        """Return 3x3 rotation matrix from quaternion."""
        w, x, y, z = self.w, self.x, self.y, self.z
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)]
        ], dtype=np.float64)

    def __mul__(self, other: Quaternion) -> Quaternion:
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        return Quaternion(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ).normalized()

    def integrate_angular_velocity(self, omega: Vec3, dt: float) -> Quaternion:
        """Integrate angular velocity over time step dt."""
        half_dt = 0.5 * dt
        dq = Quaternion(0.0, omega.x * half_dt, omega.y * half_dt, omega.z * half_dt)
        q_new = Quaternion(
            self.w + dq.w * self.w - dq.x * self.x - dq.y * self.y - dq.z * self.z,
            self.x + dq.w * self.x + dq.x * self.w + dq.y * self.z - dq.z * self.y,
            self.y + dq.w * self.y - dq.x * self.z + dq.y * self.w + dq.z * self.x,
            self.z + dq.w * self.z + dq.x * self.y - dq.y * self.x + dq.z * self.w
        )
        return q_new.normalized()
