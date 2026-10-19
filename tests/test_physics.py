"""
NexusOS Unit Tests - 3D Physics Engine Subsystem.
Tests Vector/Quaternion math, Rigid Body dynamics, Gravity, SAT Collisions, Impulse Resolution, and LiDAR raycasting.
"""

import unittest
import math
import numpy as np

from physics.vector import Vec3, Quaternion
from physics.shapes import BoxShape, SphereShape, PlaneShape, Ray, RaycastHit
from physics.body import RigidBody3D, BodyType
from physics.collision import CollisionDetector, ImpulseSolver, ContactManifold
from physics.world import PhysicsWorld
from physics.sensors import LiDARSensor, IMUSensor


class TestPhysicsVectorMath(unittest.TestCase):
    def test_vec3_operations(self):
        v1 = Vec3(1.0, 2.0, 3.0)
        v2 = Vec3(4.0, 5.0, 6.0)

        # Addition & Subtraction
        v_add = v1 + v2
        self.assertAlmostEqual(v_add.x, 5.0)
        self.assertAlmostEqual(v_add.y, 7.0)
        self.assertAlmostEqual(v_add.z, 9.0)

        v_sub = v2 - v1
        self.assertAlmostEqual(v_sub.x, 3.0)
        self.assertAlmostEqual(v_sub.y, 3.0)
        self.assertAlmostEqual(v_sub.z, 3.0)

        # Dot & Cross Product
        dot_val = v1.dot(v2)
        self.assertAlmostEqual(dot_val, 32.0)  # 1*4 + 2*5 + 3*6 = 32

        v_cross = v1.cross(v2)
        # (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
        self.assertAlmostEqual(v_cross.x, -3.0)
        self.assertAlmostEqual(v_cross.y, 6.0)
        self.assertAlmostEqual(v_cross.z, -3.0)

        # Normalization
        v_norm = Vec3(0.0, 3.0, 4.0).normalized()
        self.assertAlmostEqual(v_norm.length(), 1.0)
        self.assertAlmostEqual(v_norm.y, 0.6)
        self.assertAlmostEqual(v_norm.z, 0.8)

    def test_quaternion_rotations(self):
        # 90 degrees yaw rotation around Z-axis
        q = Quaternion.from_euler(0.0, 0.0, math.pi * 0.5)
        v = Vec3(1.0, 0.0, 0.0)
        v_rot = q.rotate_vector(v)

        # (1, 0, 0) rotated 90 deg around Z becomes (0, 1, 0)
        self.assertAlmostEqual(v_rot.x, 0.0, places=4)
        self.assertAlmostEqual(v_rot.y, 1.0, places=4)
        self.assertAlmostEqual(v_rot.z, 0.0, places=4)

        # Euler round-trip
        roll, pitch, yaw = q.to_euler()
        self.assertAlmostEqual(roll, 0.0, places=4)
        self.assertAlmostEqual(pitch, 0.0, places=4)
        self.assertAlmostEqual(yaw, math.pi * 0.5, places=4)


class TestRigidBodyDynamics(unittest.TestCase):
    def test_gravity_free_fall(self):
        # Create a body falling from z = 10m under g = -9.81 m/s^2
        shape = SphereShape(radius=0.5)
        body = RigidBody3D(
            body_id="ball",
            shape=shape,
            body_type=BodyType.DYNAMIC,
            mass=2.0,
            position=Vec3(0.0, 0.0, 10.0),
            linear_damping=0.0
        )
        gravity = Vec3(0.0, 0.0, -9.81)

        dt = 0.1
        for _ in range(10):  # 1.0 second of free fall
            body.integrate(dt, gravity)

        # Velocity after 1s: v = -9.81 m/s
        self.assertAlmostEqual(body.linear_velocity.z, -9.81, places=1)
        # Position after 1s: p = 10 - 0.5 * 9.81 * 1.0^2 = 5.095m (semi-implicit Euler)
        self.assertTrue(4.5 < body.position.z < 5.5)

    def test_applied_torque_and_angular_velocity(self):
        shape = BoxShape.from_dimensions(1.0, 1.0, 1.0)
        body = RigidBody3D(body_id="box", shape=shape, body_type=BodyType.DYNAMIC, mass=1.0)
        body.apply_torque(Vec3(0.0, 0.0, 10.0))
        body.integrate(0.1, Vec3(0.0, 0.0, 0.0))

        # Angular velocity along Z should be positive
        self.assertGreater(body.angular_velocity.z, 0.0)


class TestCollisionDetectionAndResolution(unittest.TestCase):
    def test_sphere_sphere_collision(self):
        s1 = SphereShape(radius=1.0)
        s2 = SphereShape(radius=1.0)
        b1 = RigidBody3D("s1", s1, position=Vec3(0.0, 0.0, 0.0))
        b2 = RigidBody3D("s2", s2, position=Vec3(1.5, 0.0, 0.0))

        manifold = CollisionDetector.check_collision(b1, b2)
        self.assertIsNotNone(manifold)
        assert manifold is not None
        self.assertAlmostEqual(manifold.depth, 0.5, places=4)
        self.assertAlmostEqual(manifold.normal.x, 1.0, places=4)

    def test_box_box_sat_collision(self):
        b1 = BoxShape.from_dimensions(1.0, 1.0, 1.0)
        b2 = BoxShape.from_dimensions(1.0, 1.0, 1.0)
        body1 = RigidBody3D("box1", b1, position=Vec3(0.0, 0.0, 0.0))
        body2 = RigidBody3D("box2", b2, position=Vec3(0.8, 0.0, 0.0))

        manifold = CollisionDetector.check_collision(body1, body2)
        self.assertIsNotNone(manifold)
        assert manifold is not None
        self.assertAlmostEqual(manifold.depth, 0.2, places=3)

    def test_raycast_obb(self):
        box_shape = BoxShape.from_dimensions(2.0, 2.0, 2.0)
        box_body = RigidBody3D("target_box", box_shape, position=Vec3(5.0, 0.0, 0.0))

        ray = Ray(origin=Vec3(0.0, 0.0, 0.0), direction=Vec3(1.0, 0.0, 0.0), max_distance=20.0)
        hit = CollisionDetector.raycast_body(ray, box_body)

        self.assertTrue(hit.hit)
        # Front face is at x = 5.0 - 1.0 = 4.0m
        self.assertAlmostEqual(hit.distance, 4.0, places=3)
        self.assertAlmostEqual(hit.normal.x, -1.0, places=3)


class TestPhysicsWorldAndSensors(unittest.TestCase):
    def test_world_ground_contact(self):
        world = PhysicsWorld(gravity=Vec3(0.0, 0.0, -9.81), enable_ground_plane=True)
        box = BoxShape.from_dimensions(1.0, 1.0, 1.0)
        body = RigidBody3D("falling_crate", box, mass=10.0, position=Vec3(0.0, 0.0, 1.0))
        world.add_body(body)

        # Run 100 simulation steps
        for _ in range(50):
            world.step(0.02)

        # The crate should come to rest on the ground plane (z ≈ 0.5m)
        self.assertAlmostEqual(body.position.z, 0.5, delta=0.08)
        self.assertTrue(body.is_grounded)

    def test_lidar_sensor_scan(self):
        world = PhysicsWorld(enable_ground_plane=True)
        # Add obstacle at (4.0, 0.0)
        world.add_box_obstacle("wall", Vec3(4.0, 0.0, 0.5), width=0.5, length=4.0, height=1.0)

        robot_box = BoxShape.from_dimensions(0.6, 0.6, 0.3)
        robot = RigidBody3D("rover", robot_box, position=Vec3(0.0, 0.0, 0.15))
        world.add_body(robot)

        lidar = LiDARSensor("lidar_test", num_rays=36, max_range=10.0)
        scan = lidar.scan(world, robot)

        self.assertEqual(len(scan["ranges"]), 36)
        self.assertTrue(len(scan["point_cloud_world"]) > 0)


if __name__ == "__main__":
    unittest.main()
