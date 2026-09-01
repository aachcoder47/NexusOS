"""
NexusOS Physics Engine - Physics World Simulator.
Manages numerical sub-stepping, gravity, rigid body collections, multi-contact resolution, and scene raycasts.
"""

from __future__ import annotations
import time
import math
import logging
from typing import Dict, List, Optional, Tuple, Any

from physics.vector import Vec3, Quaternion
from physics.shapes import Shape, BoxShape, SphereShape, PlaneShape, Ray, RaycastHit
from physics.body import RigidBody3D, BodyType
from physics.collision import CollisionDetector, ImpulseSolver, ContactManifold

logger = logging.getLogger("NexusOS.PhysicsWorld")


class PhysicsWorld:
    """
    Complete 3D Physics Simulator with gravity, rigid body mechanics,
    continuous contact resolution, and fast sensor raycasting.
    """

    def __init__(
        self,
        gravity: Vec3 = Vec3(0.0, 0.0, -9.81),
        time_step: float = 0.01,         # 100 Hz simulation rate
        sub_steps: int = 4,              # 4 sub-steps per frame for high stability
        enable_ground_plane: bool = True
    ):
        self.gravity = gravity
        self.time_step = float(time_step)
        self.sub_steps = max(1, int(sub_steps))

        self.bodies: Dict[str, RigidBody3D] = {}
        self.last_sim_time: float = time.monotonic()
        self.simulation_time: float = 0.0
        self.total_collisions: int = 0
        self.active_manifolds: List[ContactManifold] = []

        if enable_ground_plane:
            self._init_ground_plane()

    def _init_ground_plane(self):
        """Create static infinite ground plane at z = 0."""
        ground_shape = PlaneShape(normal=Vec3(0.0, 0.0, 1.0), distance=0.0)
        ground_body = RigidBody3D(
            body_id="world_ground",
            shape=ground_shape,
            body_type=BodyType.STATIC,
            mass=0.0,
            position=Vec3(0.0, 0.0, 0.0),
            restitution=0.05,
            friction_static=0.8,
            friction_dynamic=0.6
        )
        self.add_body(ground_body)

    def add_body(self, body: RigidBody3D) -> RigidBody3D:
        """Add a rigid body into the physical world."""
        self.bodies[body.body_id] = body
        return body

    def remove_body(self, body_id: str) -> Optional[RigidBody3D]:
        """Remove a body by its identifier."""
        return self.bodies.pop(body_id, None)

    def get_body(self, body_id: str) -> Optional[RigidBody3D]:
        return self.bodies.get(body_id)

    def add_box_obstacle(
        self,
        obstacle_id: str,
        position: Vec3,
        width: float = 1.0,
        length: float = 1.0,
        height: float = 1.0,
        is_dynamic: bool = False,
        mass: float = 20.0,
        yaw: float = 0.0
    ) -> RigidBody3D:
        """Helper to spawn rectangular crate / pillar / wall obstacles."""
        box_shape = BoxShape.from_dimensions(width, length, height)
        b_type = BodyType.DYNAMIC if is_dynamic else BodyType.STATIC
        q = Quaternion.from_euler(0.0, 0.0, yaw)
        body = RigidBody3D(
            body_id=obstacle_id,
            shape=box_shape,
            body_type=b_type,
            mass=mass if is_dynamic else 0.0,
            position=position,
            orientation=q,
            restitution=0.1,
            friction_static=0.7,
            friction_dynamic=0.5
        )
        return self.add_body(body)

    def add_sphere_obstacle(
        self,
        obstacle_id: str,
        position: Vec3,
        radius: float = 0.5,
        is_dynamic: bool = False,
        mass: float = 10.0
    ) -> RigidBody3D:
        """Helper to spawn spherical obstacles / ball objects."""
        sphere_shape = SphereShape(radius=radius)
        b_type = BodyType.DYNAMIC if is_dynamic else BodyType.STATIC
        body = RigidBody3D(
            body_id=obstacle_id,
            shape=sphere_shape,
            body_type=b_type,
            mass=mass if is_dynamic else 0.0,
            position=position,
            restitution=0.3,
            friction_static=0.5,
            friction_dynamic=0.3
        )
        return self.add_body(body)

    def step(self, delta_time: Optional[float] = None) -> List[ContactManifold]:
        """
        Advance the physical simulation by delta_time using multiple sub-steps for stability.
        Returns the list of contact manifolds generated during this step.
        """
        now = time.monotonic()
        dt = delta_time if delta_time is not None else (now - self.last_sim_time)
        dt = min(max(dt, 0.001), 0.1)  # Clamp to prevent spiral of death
        self.last_sim_time = now

        sub_dt = dt / self.sub_steps
        all_manifolds: List[ContactManifold] = []

        for _ in range(self.sub_steps):
            # 1. Reset grounded flags on dynamic bodies
            for b in self.bodies.values():
                if b.body_type == BodyType.DYNAMIC:
                    b.is_grounded = False

            # 2. Integrate forces & update positions
            for b in self.bodies.values():
                b.integrate(sub_dt, self.gravity)

            # 3. Collision Detection (Broadphase + Narrowphase SAT)
            step_manifolds = self._detect_collisions()

            # 4. Resolve Contacts via Impulse Solver
            for manifold in step_manifolds:
                ImpulseSolver.resolve_contact(manifold, sub_dt)

            all_manifolds.extend(step_manifolds)
            self.simulation_time += sub_dt

        self.active_manifolds = all_manifolds
        self.total_collisions += len(all_manifolds)
        return all_manifolds

    def _detect_collisions(self) -> List[ContactManifold]:
        """Broadphase & Narrowphase collision pipeline."""
        manifolds: List[ContactManifold] = []
        body_list = list(self.bodies.values())
        n = len(body_list)

        for i in range(n):
            body_a = body_list[i]
            for j in range(i + 1, n):
                body_b = body_list[j]

                # Skip static vs static
                if body_a.body_type == BodyType.STATIC and body_b.body_type == BodyType.STATIC:
                    continue

                # Broadphase AABB quick check
                if not self._aabb_overlap(body_a, body_b):
                    continue

                # Narrowphase
                manifold = CollisionDetector.check_collision(body_a, body_b)
                if manifold and manifold.depth > 0.0:
                    manifolds.append(manifold)

        return manifolds

    def _aabb_overlap(self, body_a: RigidBody3D, body_b: RigidBody3D) -> bool:
        """Fast AABB overlap test."""
        min_a, max_a = body_a.shape.compute_aabb(body_a.position, body_a.orientation)
        min_b, max_b = body_b.shape.compute_aabb(body_b.position, body_b.orientation)

        return (
            min_a.x <= max_b.x and max_a.x >= min_b.x and
            min_a.y <= max_b.y and max_a.y >= min_b.y and
            min_a.z <= max_b.z and max_a.z >= min_b.z
        )

    def raycast(self, ray: Ray, ignore_body_ids: Optional[List[str]] = None) -> RaycastHit:
        """
        Cast a ray through the physics scene and return the closest hit point.
        Used by LiDAR, sonar, and proximity sensors.
        """
        ignore = set(ignore_body_ids or [])
        closest_hit = RaycastHit(hit=False, distance=ray.max_distance)

        for body_id, body in self.bodies.items():
            if body_id in ignore:
                continue

            hit = CollisionDetector.raycast_body(ray, body)
            if hit.hit and hit.distance < closest_hit.distance:
                closest_hit = hit

        return closest_hit

    def to_dict(self) -> Dict[str, Any]:
        """Export world state for WebSocket broadcasting and 3D UI Digital Twin rendering."""
        return {
            "simulation_time": round(self.simulation_time, 3),
            "gravity": self.gravity.to_dict(),
            "total_bodies": len(self.bodies),
            "active_collisions_count": len(self.active_manifolds),
            "bodies": [b.to_dict() for b in self.bodies.values() if b.body_id != "world_ground"]
        }
