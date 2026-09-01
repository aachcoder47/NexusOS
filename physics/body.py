"""
NexusOS Physics Engine - 3D Rigid Body Representation.
Encapsulates linear & angular physical dynamics, inertia tensors, force accumulators,
gyroscopic precession, velocity-dependent drag, sleeping, and material friction.
"""

from __future__ import annotations
import math
from enum import Enum
from typing import Optional, Dict, Any, List
import numpy as np

from physics.vector import Vec3, Quaternion
from physics.shapes import Shape, BoxShape, SphereShape, CylinderShape


class BodyType(str, Enum):
    DYNAMIC = "DYNAMIC"      # Affected by gravity, forces, impulses, and collisions
    STATIC = "STATIC"        # Immovable obstacles (walls, terrain, fixed fixtures)
    KINEMATIC = "KINEMATIC"  # Velocity driven directly without force responses (moving lifts, conveyors)


class RigidBody3D:
    """
    3D Rigid Body representation with accurate Newtonian mechanics,
    quaternion orientation integration, and friction/restitution material properties.
    """

    def __init__(
        self,
        body_id: str,
        shape: Shape,
        body_type: BodyType = BodyType.DYNAMIC,
        mass: float = 1.0,
        position: Optional[Vec3] = None,
        orientation: Optional[Quaternion] = None,
        restitution: float = 0.1,        # Bounciness [0.0 = inelastic, 1.0 = elastic]
        friction_static: float = 0.6,    # Static friction coefficient
        friction_dynamic: float = 0.4,   # Dynamic/kinetic friction coefficient
        linear_damping: float = 0.05,    # Viscous drag coefficient (velocity linear term)
        angular_damping: float = 0.1,    # Rotational viscous drag
        drag_coeff_sq: float = 0.0,      # Quadratic (velocity²) drag coefficient (kg/m)
        com_offset: Optional[Vec3] = None # Centre-of-mass offset from geometry centre (m)
    ):
        self.body_id = body_id
        self.shape = shape
        self.body_type = body_type

        # Mass & Inertia Properties
        if body_type == BodyType.STATIC:
            self.mass = 0.0
            self.inv_mass = 0.0
            self.inertia_tensor = np.zeros((3, 3), dtype=np.float64)
            self.inv_inertia_tensor = np.zeros((3, 3), dtype=np.float64)
        else:
            self.mass = max(1e-4, float(mass))
            self.inv_mass = 1.0 / self.mass
            self.inertia_tensor = self._compute_inertia_tensor(shape, self.mass)
            self.inv_inertia_tensor = np.linalg.inv(self.inertia_tensor)

        # Pose
        self.position: Vec3 = position.copy() if position else Vec3()
        self.orientation: Quaternion = orientation.copy() if orientation else Quaternion()

        # Velocities
        self.linear_velocity: Vec3 = Vec3()
        self.angular_velocity: Vec3 = Vec3()

        # Accelerations & Force Accumulators
        self.force_accumulator: Vec3 = Vec3()
        self.torque_accumulator: Vec3 = Vec3()

        # Material Properties
        self.restitution = float(restitution)
        self.friction_static = float(friction_static)
        self.friction_dynamic = float(friction_dynamic)
        self.linear_damping = float(linear_damping)
        self.angular_damping = float(angular_damping)

        # Quadratic aerodynamic drag coefficient (½ρCdA packed into a scalar, kg/m)
        self.drag_coeff_sq = float(drag_coeff_sq)

        # Centre-of-mass offset from geometric centre (body-frame)
        self.com_offset: Vec3 = com_offset.copy() if com_offset else Vec3()

        # Sleep / activity tracking
        self._sleep_timer: float = 0.0
        self._sleep_threshold_linear: float = 0.005   # m/s
        self._sleep_threshold_angular: float = 0.01   # rad/s
        self._sleep_delay_s: float = 0.4              # seconds below threshold before sleeping

        # State flags
        self.is_grounded: bool = False
        self.is_sleeping: bool = False
        self.custom_data: Dict[str, Any] = {}

    def _compute_inertia_tensor(self, shape: Shape, mass: float) -> np.ndarray:
        """Compute standard inertia tensor for basic shapes."""
        if isinstance(shape, BoxShape):
            w = shape.half_extents.x * 2.0
            l = shape.half_extents.y * 2.0
            h = shape.half_extents.z * 2.0
            ixx = (1.0 / 12.0) * mass * (l * l + h * h)
            iyy = (1.0 / 12.0) * mass * (w * w + h * h)
            izz = (1.0 / 12.0) * mass * (w * w + l * l)
            return np.diag([ixx, iyy, izz])
        elif isinstance(shape, SphereShape):
            r = shape.radius
            i = (2.0 / 5.0) * mass * r * r
            return np.diag([i, i, i])
        elif isinstance(shape, CylinderShape):
            r = shape.radius
            h = shape.half_height * 2.0
            ixx = (1.0 / 12.0) * mass * (3 * r * r + h * h)
            iyy = ixx
            izz = 0.5 * mass * r * r
            return np.diag([ixx, iyy, izz])
        # Default cube fallback
        i = (1.0 / 6.0) * mass
        return np.diag([i, i, i])

    def get_world_inv_inertia(self) -> np.ndarray:
        """Return the inverse inertia tensor rotated into world coordinate frame."""
        if self.body_type == BodyType.STATIC or self.inv_mass == 0.0:
            return np.zeros((3, 3), dtype=np.float64)
        r = self.orientation.to_rotation_matrix()
        return r @ self.inv_inertia_tensor @ r.T

    def apply_force(self, force: Vec3):
        """Apply a central force at the center of mass."""
        if self.body_type == BodyType.DYNAMIC:
            self.force_accumulator = self.force_accumulator + force

    def apply_force_at_point(self, force: Vec3, point: Vec3):
        """Apply a force at a specific world point, generating both force and torque."""
        if self.body_type == BodyType.DYNAMIC:
            self.force_accumulator = self.force_accumulator + force
            r = point - self.position
            torque = r.cross(force)
            self.torque_accumulator = self.torque_accumulator + torque

    def apply_torque(self, torque: Vec3):
        """Apply a rotational torque."""
        if self.body_type == BodyType.DYNAMIC:
            self.torque_accumulator = self.torque_accumulator + torque

    def apply_impulse(self, impulse: Vec3, contact_point: Optional[Vec3] = None):
        """Apply an instantaneous velocity impulse (from collision or contact force)."""
        if self.body_type != BodyType.DYNAMIC:
            return
        self.linear_velocity = self.linear_velocity + impulse * self.inv_mass
        if contact_point is not None:
            r = contact_point - self.position
            rot_impulse = r.cross(impulse)
            inv_i = self.get_world_inv_inertia()
            d_omega_arr = inv_i @ rot_impulse.array
            self.angular_velocity = self.angular_velocity + Vec3.from_array(d_omega_arr)

    def integrate(self, dt: float, gravity: Vec3):
        """
        Integrate Newtonian dynamics over timestep dt using semi-implicit Euler integration.
        Includes:
          - Gyroscopic (Euler) term: τ_gyro = -ω × (I·ω)  (prevents torque-free precession divergence)
          - Quadratic aerodynamic drag: F_drag = -k_sq * |v| * v
          - Clamped linear & angular viscous damping
          - Body sleep / wake hysteresis to save CPU on stationary objects
        """
        if self.body_type != BodyType.DYNAMIC:
            self.force_accumulator = Vec3()
            self.torque_accumulator = Vec3()
            return

        if self.is_sleeping:
            # Wake on external force or torque
            if self.force_accumulator.length() > 1e-4 or self.torque_accumulator.length() > 1e-4:
                self.is_sleeping = False
                self._sleep_timer = 0.0
            else:
                self.force_accumulator = Vec3()
                self.torque_accumulator = Vec3()
                return

        # ─── 1. Linear Dynamics ───────────────────────────────────────────────
        gravity_force = gravity * self.mass

        # Quadratic aerodynamic / viscous drag: F_drag = -k_sq * |v|² * v̂
        speed = self.linear_velocity.length()
        if speed > 0.001 and self.drag_coeff_sq > 0.0:
            drag_force = self.linear_velocity.normalized() * (-self.drag_coeff_sq * speed * speed)
        else:
            drag_force = Vec3()

        total_force = self.force_accumulator + gravity_force + drag_force
        linear_acc = total_force * self.inv_mass

        # Semi-implicit Euler: update velocity then position
        self.linear_velocity = self.linear_velocity + linear_acc * dt

        # Viscous linear damping (stable exponential form): v *= exp(-k*dt) ≈ 1/(1+k*dt)
        inv_damp = 1.0 / (1.0 + self.linear_damping * dt)
        self.linear_velocity = self.linear_velocity * inv_damp

        # Update centre-of-mass position
        self.position = self.position + self.linear_velocity * dt

        # ─── 2. Angular Dynamics ──────────────────────────────────────────────
        rot_matrix = self.orientation.to_rotation_matrix()
        # World-space inertia tensor
        I_world = rot_matrix @ self.inertia_tensor @ rot_matrix.T
        inv_I_world = rot_matrix @ self.inv_inertia_tensor @ rot_matrix.T

        omega_arr = self.angular_velocity.array

        # Gyroscopic (Euler) term: τ_gyro = -ω × (I_world · ω)
        I_omega = I_world @ omega_arr
        gyro_torque = -np.cross(omega_arr, I_omega)

        net_torque_arr = self.torque_accumulator.array + gyro_torque
        angular_acc_arr = inv_I_world @ net_torque_arr
        self.angular_velocity = self.angular_velocity + Vec3.from_array(angular_acc_arr) * dt

        # Viscous angular damping
        inv_ang_damp = 1.0 / (1.0 + self.angular_damping * dt)
        self.angular_velocity = self.angular_velocity * inv_ang_damp

        # Integrate orientation quaternion
        self.orientation = self.orientation.integrate_angular_velocity(self.angular_velocity, dt)

        # ─── 3. Sleep Hysteresis ──────────────────────────────────────────────
        if (speed < self._sleep_threshold_linear and
                self.angular_velocity.length() < self._sleep_threshold_angular):
            self._sleep_timer += dt
            if self._sleep_timer >= self._sleep_delay_s:
                self.is_sleeping = True
        else:
            self._sleep_timer = 0.0

        # ─── 4. Reset Accumulators ────────────────────────────────────────────
        self.force_accumulator = Vec3()
        self.torque_accumulator = Vec3()

    def get_velocity_at_point(self, world_point: Vec3) -> Vec3:
        """Calculate linear velocity at a specific world coordinate point."""
        r = world_point - self.position
        return self.linear_velocity + self.angular_velocity.cross(r)

    def to_dict(self) -> Dict[str, Any]:
        roll, pitch, yaw = self.orientation.to_euler()
        speed = self.linear_velocity.length()
        return {
            "body_id": self.body_id,
            "body_type": self.body_type.value,
            "mass": round(self.mass, 4),
            "position": self.position.to_dict(),
            "orientation_rpy": {"roll": round(roll, 4), "pitch": round(pitch, 4), "yaw": round(yaw, 4)},
            "linear_velocity": self.linear_velocity.to_dict(),
            "angular_velocity": self.angular_velocity.to_dict(),
            "speed_ms": round(speed, 4),
            "kinetic_energy_j": round(0.5 * self.mass * speed * speed, 4) if self.mass > 0 else 0.0,
            "is_grounded": self.is_grounded,
            "is_sleeping": self.is_sleeping,
            "custom_data": self.custom_data
        }
