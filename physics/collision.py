"""
NexusOS Physics Engine - Collision Detection & Impulse Contact Resolution.
Implements Separating Axis Theorem (SAT) for 3D OBBs, Sphere-Sphere, Sphere-Box,
Raycasting intersection algorithms, and Coulomb Friction Impulse Solvers.
"""

from __future__ import annotations
import math
from typing import Optional, List, Tuple
import numpy as np

from physics.vector import Vec3, Quaternion
from physics.shapes import ShapeType, BoxShape, SphereShape, CylinderShape, PlaneShape, Ray, RaycastHit
from physics.body import RigidBody3D, BodyType


class ContactManifold:
    """Represents a collision contact between two rigid bodies."""

    __slots__ = ("body_a", "body_b", "normal", "depth", "contact_points", "restitution", "friction")

    def __init__(
        self,
        body_a: RigidBody3D,
        body_b: RigidBody3D,
        normal: Vec3,               # Normal pointing from A to B
        depth: float,               # Penetration depth
        contact_points: List[Vec3], # Contact points in world space
        restitution: float = 0.2,
        friction: float = 0.4
    ):
        self.body_a = body_a
        self.body_b = body_b
        self.normal = normal.normalized()
        self.depth = max(0.0, float(depth))
        self.contact_points = contact_points
        self.restitution = float(restitution)
        self.friction = float(friction)


class CollisionDetector:
    """High-performance collision detector supporting SAT, Sphere, Plane, and Ray queries."""

    @staticmethod
    def check_collision(body_a: RigidBody3D, body_b: RigidBody3D) -> Optional[ContactManifold]:
        """Dispatch collision test between two rigid bodies based on shape types."""
        # Both static bodies don't need collision resolution
        if body_a.body_type == BodyType.STATIC and body_b.body_type == BodyType.STATIC:
            return None

        shape_a = body_a.shape
        shape_b = body_b.shape

        # Sphere vs Sphere
        if isinstance(shape_a, SphereShape) and isinstance(shape_b, SphereShape):
            return CollisionDetector._sphere_vs_sphere(body_a, shape_a, body_b, shape_b)

        # Box vs Box (SAT)
        elif isinstance(shape_a, BoxShape) and isinstance(shape_b, BoxShape):
            return CollisionDetector._box_vs_box_sat(body_a, shape_a, body_b, shape_b)

        # Box vs Sphere
        elif isinstance(shape_a, BoxShape) and isinstance(shape_b, SphereShape):
            return CollisionDetector._box_vs_sphere(body_a, shape_a, body_b, shape_b)
        elif isinstance(shape_a, SphereShape) and isinstance(shape_b, BoxShape):
            manifold = CollisionDetector._box_vs_sphere(body_b, shape_b, body_a, shape_a)
            if manifold:
                manifold.body_a, manifold.body_b = body_a, body_b
                manifold.normal = -manifold.normal
            return manifold

        # Box vs Plane
        elif isinstance(shape_a, BoxShape) and isinstance(shape_b, PlaneShape):
            return CollisionDetector._box_vs_plane(body_a, shape_a, body_b, shape_b)
        elif isinstance(shape_a, PlaneShape) and isinstance(shape_b, BoxShape):
            manifold = CollisionDetector._box_vs_plane(body_b, shape_b, body_a, shape_a)
            if manifold:
                manifold.body_a, manifold.body_b = body_a, body_b
                manifold.normal = -manifold.normal
            return manifold

        # Sphere vs Plane
        elif isinstance(shape_a, SphereShape) and isinstance(shape_b, PlaneShape):
            return CollisionDetector._sphere_vs_plane(body_a, shape_a, body_b, shape_b)
        elif isinstance(shape_a, PlaneShape) and isinstance(shape_b, SphereShape):
            manifold = CollisionDetector._sphere_vs_plane(body_b, shape_b, body_a, shape_a)
            if manifold:
                manifold.body_a, manifold.body_b = body_a, body_b
                manifold.normal = -manifold.normal
            return manifold

        return None

    @staticmethod
    def _sphere_vs_sphere(
        body_a: RigidBody3D, shape_a: SphereShape,
        body_b: RigidBody3D, shape_b: SphereShape
    ) -> Optional[ContactManifold]:
        delta = body_b.position - body_a.position
        dist = delta.length()
        radius_sum = shape_a.radius + shape_b.radius

        if dist >= radius_sum:
            return None

        if dist < 1e-6:
            normal = Vec3(0.0, 0.0, 1.0)
            depth = radius_sum
        else:
            normal = delta / dist
            depth = radius_sum - dist

        contact_pt = body_a.position + normal * (shape_a.radius - depth * 0.5)
        restitution = min(body_a.restitution, body_b.restitution)
        friction = math.sqrt(body_a.friction_dynamic * body_b.friction_dynamic)

        return ContactManifold(body_a, body_b, normal, depth, [contact_pt], restitution, friction)

    @staticmethod
    def _box_vs_box_sat(
        body_a: RigidBody3D, shape_a: BoxShape,
        body_b: RigidBody3D, shape_b: BoxShape
    ) -> Optional[ContactManifold]:
        """
        3D Separating Axis Theorem (SAT) for Oriented Bounding Boxes (OBB).
        Tests 15 potential separating axes:
          - 3 axes of Box A
          - 3 axes of Box B
          - 9 cross-product axes
        """
        axes_a = shape_a.get_axes(body_a.orientation)
        axes_b = shape_b.get_axes(body_b.orientation)

        test_axes: List[Vec3] = []
        test_axes.extend(axes_a)
        test_axes.extend(axes_b)

        for a in axes_a:
            for b in axes_b:
                cross_axis = a.cross(b)
                if cross_axis.length_sq() > 1e-4:
                    test_axes.append(cross_axis.normalized())

        min_overlap = float("inf")
        best_axis = Vec3()

        verts_a = shape_a.get_vertices(body_a.position, body_a.orientation)
        verts_b = shape_b.get_vertices(body_b.position, body_b.orientation)

        delta_center = body_b.position - body_a.position

        for axis in test_axes:
            proj_a = [v.dot(axis) for v in verts_a]
            min_a, max_a = min(proj_a), max(proj_a)

            proj_b = [v.dot(axis) for v in verts_b]
            min_b, max_b = min(proj_b), max(proj_b)

            overlap = min(max_a, max_b) - max(min_a, min_b)
            if overlap <= 0.0:
                # Found separating axis -> No collision
                return None

            if overlap < min_overlap:
                min_overlap = overlap
                # Ensure normal points from body A to body B
                if delta_center.dot(axis) < 0.0:
                    best_axis = -axis
                else:
                    best_axis = axis

        # Find contact points (deepest vertices of B inside A)
        contact_points = []
        for vb in verts_b:
            d = (vb - body_a.position).dot(best_axis)
            if d < 0:
                contact_points.append(vb)
        if not contact_points:
            contact_points = [(body_a.position + body_b.position) * 0.5]

        restitution = min(body_a.restitution, body_b.restitution)
        friction = math.sqrt(body_a.friction_dynamic * body_b.friction_dynamic)

        return ContactManifold(
            body_a, body_b, best_axis, min_overlap, contact_points, restitution, friction
        )

    @staticmethod
    def _box_vs_sphere(
        body_box: RigidBody3D, shape_box: BoxShape,
        body_sphere: RigidBody3D, shape_sphere: SphereShape
    ) -> Optional[ContactManifold]:
        # Transform sphere center into local coordinates of the box
        local_sphere_pos = body_box.position - body_sphere.position
        # Use inverse rotation
        r = body_box.orientation.to_rotation_matrix()
        rel_pos = body_sphere.position - body_box.position
        local_p = Vec3.from_array(r.T @ rel_pos.array)

        hx, hy, hz = shape_box.half_extents.x, shape_box.half_extents.y, shape_box.half_extents.z
        # Clamp point to box half-extents
        clamped_x = max(-hx, min(hx, local_p.x))
        clamped_y = max(-hy, min(hy, local_p.y))
        clamped_z = max(-hz, min(hz, local_p.z))
        closest_local = Vec3(clamped_x, clamped_y, clamped_z)

        # World-space closest point on box
        closest_world = body_box.position + Vec3.from_array(r @ closest_local.array)
        delta = body_sphere.position - closest_world
        dist = delta.length()

        if dist >= shape_sphere.radius:
            return None

        if dist < 1e-6:
            normal = Vec3(0.0, 0.0, 1.0)
            depth = shape_sphere.radius
        else:
            normal = delta / dist
            depth = shape_sphere.radius - dist

        restitution = min(body_box.restitution, body_sphere.restitution)
        friction = math.sqrt(body_box.friction_dynamic * body_sphere.friction_dynamic)

        return ContactManifold(
            body_box, body_sphere, normal, depth, [closest_world], restitution, friction
        )

    @staticmethod
    def _box_vs_plane(
        body_box: RigidBody3D, shape_box: BoxShape,
        body_plane: RigidBody3D, shape_plane: PlaneShape
    ) -> Optional[ContactManifold]:
        n = shape_plane.normal
        d_plane = shape_plane.distance

        verts = shape_box.get_vertices(body_box.position, body_box.orientation)
        contact_points = []
        max_penetration = 0.0

        for v in verts:
            dist = v.dot(n) - d_plane
            if dist < 0.0:  # Penetrating below plane
                penetration = -dist
                if penetration > max_penetration:
                    max_penetration = penetration
                contact_points.append(v)

        if not contact_points:
            return None

        restitution = min(body_box.restitution, body_plane.restitution)
        friction = math.sqrt(body_box.friction_dynamic * body_plane.friction_dynamic)

        # Normal points from box to plane (inward to the plane)
        return ContactManifold(
            body_box, body_plane, -n, max_penetration, contact_points, restitution, friction
        )

    @staticmethod
    def _sphere_vs_plane(
        body_sphere: RigidBody3D, shape_sphere: SphereShape,
        body_plane: RigidBody3D, shape_plane: PlaneShape
    ) -> Optional[ContactManifold]:
        n = shape_plane.normal
        d_plane = shape_plane.distance
        dist_to_center = body_sphere.position.dot(n) - d_plane

        if dist_to_center >= shape_sphere.radius:
            return None

        depth = shape_sphere.radius - dist_to_center
        contact_pt = body_sphere.position - n * (shape_sphere.radius - depth * 0.5)

        restitution = min(body_sphere.restitution, body_plane.restitution)
        friction = math.sqrt(body_sphere.friction_dynamic * body_plane.friction_dynamic)

        return ContactManifold(
            body_sphere, body_plane, -n, depth, [contact_pt], restitution, friction
        )

    # ------------------------------------------------------------------
    # Raycasting Queries (Used for LiDAR, Sonar & Depth Cameras)
    # ------------------------------------------------------------------

    @staticmethod
    def raycast_body(ray: Ray, body: RigidBody3D) -> RaycastHit:
        """Raycast against a rigid body's shape."""
        shape = body.shape
        if isinstance(shape, SphereShape):
            return CollisionDetector._ray_vs_sphere(ray, body.position, shape.radius, body.body_id)
        elif isinstance(shape, BoxShape):
            return CollisionDetector._ray_vs_obb(ray, body.position, body.orientation, shape.half_extents, body.body_id)
        elif isinstance(shape, PlaneShape):
            return CollisionDetector._ray_vs_plane(ray, shape.normal, shape.distance, body.body_id)
        return RaycastHit(hit=False)

    @staticmethod
    def _ray_vs_sphere(ray: Ray, sphere_center: Vec3, radius: float, body_id: str) -> RaycastHit:
        oc = ray.origin - sphere_center
        b = oc.dot(ray.direction)
        c = oc.dot(oc) - radius * radius
        discriminant = b * b - c

        if discriminant < 0.0:
            return RaycastHit(hit=False)

        sqrt_disc = math.sqrt(discriminant)
        t = -b - sqrt_disc
        if t < 0.0:
            t = -b + sqrt_disc

        if 0.0 <= t <= ray.max_distance:
            hit_pt = ray.point_at(t)
            normal = (hit_pt - sphere_center).normalized()
            return RaycastHit(hit=True, distance=t, point=hit_pt, normal=normal, collider_id=body_id)

        return RaycastHit(hit=False)

    @staticmethod
    def _ray_vs_obb(ray: Ray, box_pos: Vec3, box_rot: Quaternion, half_extents: Vec3, body_id: str) -> RaycastHit:
        # Transform ray into local OBB space
        r = box_rot.to_rotation_matrix()
        rel_origin = ray.origin - box_pos
        local_origin = Vec3.from_array(r.T @ rel_origin.array)
        local_dir = Vec3.from_array(r.T @ ray.direction.array)

        hx, hy, hz = half_extents.x, half_extents.y, half_extents.z
        min_b = Vec3(-hx, -hy, -hz)
        max_b = Vec3(hx, hy, hz)

        t_min = 0.0
        t_max = ray.max_distance
        hit_normal_local = Vec3()

        for i in range(3):
            orig_i = local_origin.array[i]
            dir_i = local_dir.array[i]
            min_i = min_b.array[i]
            max_i = max_b.array[i]

            if abs(dir_i) < 1e-8:
                if orig_i < min_i or orig_i > max_i:
                    return RaycastHit(hit=False)
            else:
                inv_d = 1.0 / dir_i
                t1 = (min_i - orig_i) * inv_d
                t2 = (max_i - orig_i) * inv_d
                normal_sign = -1.0 if t1 > t2 else 1.0
                if t1 > t2:
                    t1, t2 = t2, t1

                if t1 > t_min:
                    t_min = t1
                    hit_normal_local = Vec3()
                    hit_normal_local.array[i] = -normal_sign if dir_i > 0 else normal_sign
                t_max = min(t_max, t2)

                if t_min > t_max:
                    return RaycastHit(hit=False)

        if 0.0 <= t_min <= ray.max_distance:
            hit_pt = ray.point_at(t_min)
            world_normal = Vec3.from_array(r @ hit_normal_local.array).normalized()
            return RaycastHit(hit=True, distance=t_min, point=hit_pt, normal=world_normal, collider_id=body_id)

        return RaycastHit(hit=False)

    @staticmethod
    def _ray_vs_plane(ray: Ray, plane_normal: Vec3, plane_dist: float, body_id: str) -> RaycastHit:
        denom = plane_normal.dot(ray.direction)
        if abs(denom) < 1e-6:
            return RaycastHit(hit=False)

        t = (plane_dist - plane_normal.dot(ray.origin)) / denom
        if 0.0 <= t <= ray.max_distance:
            hit_pt = ray.point_at(t)
            return RaycastHit(hit=True, distance=t, point=hit_pt, normal=plane_normal, collider_id=body_id)

        return RaycastHit(hit=False)


class ImpulseSolver:
    """
    Impulse-Based Velocity and Coulomb Friction Solver.
    Resolves contact manifolds between rigid bodies with restitution and positional Baumgarte stabilization.
    """

    @staticmethod
    def resolve_contact(manifold: ContactManifold, dt: float, baumgarte_beta: float = 0.2, slop: float = 0.005):
        bodyA = manifold.body_a
        bodyB = manifold.body_b
        normal = manifold.normal

        inv_mass_sum = bodyA.inv_mass + bodyB.inv_mass
        if inv_mass_sum == 0.0:
            return

        inv_ia = bodyA.get_world_inv_inertia()
        inv_ib = bodyB.get_world_inv_inertia()

        num_contacts = len(manifold.contact_points)
        if num_contacts == 0:
            return

        for cp in manifold.contact_points:
            ra = cp - bodyA.position
            rb = cp - bodyB.position

            # Relative velocity at contact point: v_rel = (v_B + w_B x r_B) - (v_A + w_A x r_A)
            va = bodyA.get_velocity_at_point(cp)
            vb = bodyB.get_velocity_at_point(cp)
            v_rel = vb - va

            rel_normal_vel = v_rel.dot(normal)

            # Do not resolve if separating
            if rel_normal_vel > 0.0:
                continue

            # Rotational inertia terms
            ra_cross_n = ra.cross(normal)
            rb_cross_n = rb.cross(normal)
            term_a = normal.dot(Vec3.from_array(inv_ia @ ra_cross_n.cross(ra).array))
            term_b = normal.dot(Vec3.from_array(inv_ib @ rb_cross_n.cross(rb).array))

            effective_mass_normal = inv_mass_sum + term_a + term_b
            if effective_mass_normal < 1e-12:
                continue

            # Baumgarte positional stabilization to eliminate sinking
            bias = (baumgarte_beta / dt) * max(0.0, manifold.depth - slop)

            # Normal impulse magnitude: j = -(1 + e) * v_rel_n / effective_mass
            jn = (-(1.0 + manifold.restitution) * rel_normal_vel + bias) / (effective_mass_normal * num_contacts)
            jn = max(0.0, jn)  # Clamping to prevent negative attractive forces

            normal_impulse = normal * jn

            # Apply Normal Impulse
            bodyA.apply_impulse(-normal_impulse, cp)
            bodyB.apply_impulse(normal_impulse, cp)

            # -------------------------------------------------------------
            # Friction Impulse (Coulomb Friction Cone)
            # -------------------------------------------------------------
            # Recompute relative velocity after normal impulse
            va = bodyA.get_velocity_at_point(cp)
            vb = bodyB.get_velocity_at_point(cp)
            v_rel = vb - va

            tangent = (v_rel - normal * v_rel.dot(normal))
            tangent_speed = tangent.length()

            if tangent_speed > 1e-6:
                tangent = tangent / tangent_speed

                ra_cross_t = ra.cross(tangent)
                rb_cross_t = rb.cross(tangent)
                term_ta = tangent.dot(Vec3.from_array(inv_ia @ ra_cross_t.cross(ra).array))
                term_tb = tangent.dot(Vec3.from_array(inv_ib @ rb_cross_t.cross(rb).array))

                effective_mass_tangent = inv_mass_sum + term_ta + term_tb
                if effective_mass_tangent > 1e-12:
                    jt = -v_rel.dot(tangent) / (effective_mass_tangent * num_contacts)

                    # Clamp by Coulomb friction cone: |jt| <= mu * jn
                    max_friction = manifold.friction * jn
                    jt = max(-max_friction, min(max_friction, jt))

                    friction_impulse = tangent * jt
                    bodyA.apply_impulse(-friction_impulse, cp)
                    bodyB.apply_impulse(friction_impulse, cp)

        # Flag grounded state if normal is pointing upward
        if normal.z > 0.7:
            bodyB.is_grounded = True
        elif normal.z < -0.7:
            bodyA.is_grounded = True
