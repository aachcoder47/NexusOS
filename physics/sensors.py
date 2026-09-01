"""
NexusOS Physics Engine - Sensor Simulation Models.
Provides high-fidelity simulated LiDAR Raycasters, 6-Axis IMUs, and Odometry Encoders.
"""

from __future__ import annotations
import math
import random
from typing import List, Dict, Any, Optional, Tuple
import numpy as np

from physics.vector import Vec3, Quaternion
from physics.shapes import Ray, RaycastHit
from physics.world import PhysicsWorld
from physics.body import RigidBody3D


class LiDARSensor:
    """
    2D / 3D Raycasting LiDAR Sensor Simulator.
    Generates laser range scans, polar scans, and Cartesian point clouds directly from the physical environment.
    """

    def __init__(
        self,
        sensor_id: str,
        num_rays: int = 36,              # 36 rays = 10-degree resolution (fast & high fidelity)
        min_angle_rad: float = -math.pi, # -180 deg
        max_angle_rad: float = math.pi,  # +180 deg (full 360 scan)
        min_range: float = 0.1,          # 10 cm minimum detection
        max_range: float = 12.0,         # 12 meters maximum range
        noise_stddev: float = 0.015,     # 1.5 cm realistic Gaussian noise
        mount_offset: Optional[Vec3] = None
    ):
        self.sensor_id = sensor_id
        self.num_rays = num_rays
        self.min_angle_rad = min_angle_rad
        self.max_angle_rad = max_angle_rad
        self.min_range = min_range
        self.max_range = max_range
        self.noise_stddev = noise_stddev
        self.mount_offset = mount_offset if mount_offset else Vec3(0.0, 0.0, 0.2)

        # Precompute ray angle offsets
        self.angles = np.linspace(min_angle_rad, max_angle_rad, num_rays, endpoint=False)

    def scan(self, world: PhysicsWorld, robot_body: RigidBody3D) -> Dict[str, Any]:
        """
        Cast simulated laser rays from robot position against all physical obstacles in world.
        Returns ranges array and point cloud coordinates in robot and world frames.
        """
        # Sensor world position
        origin = robot_body.position + robot_body.orientation.rotate_vector(self.mount_offset)
        robot_yaw = robot_body.orientation.to_euler()[2]

        ranges: List[float] = []
        point_cloud_world: List[Dict[str, float]] = []
        point_cloud_local: List[Dict[str, float]] = []

        ignore_ids = [robot_body.body_id, "world_ground"]

        for angle in self.angles:
            beam_yaw = robot_yaw + angle
            dir_x = math.cos(beam_yaw)
            dir_y = math.sin(beam_yaw)
            ray_dir = Vec3(dir_x, dir_y, 0.0).normalized()

            ray = Ray(origin=origin, direction=ray_dir, max_distance=self.max_range)
            hit: RaycastHit = world.raycast(ray, ignore_body_ids=ignore_ids)

            dist = self.max_range
            if hit.hit and hit.distance >= self.min_range:
                # Add Gaussian noise
                noisy_dist = hit.distance + random.gauss(0.0, self.noise_stddev)
                dist = max(self.min_range, min(self.max_range, noisy_dist))

                # World point
                pt_w = origin + ray_dir * dist
                point_cloud_world.append({"x": round(pt_w.x, 3), "y": round(pt_w.y, 3), "z": round(pt_w.z, 3)})

                # Local robot point
                local_x = dist * math.cos(angle)
                local_y = dist * math.sin(angle)
                point_cloud_local.append({"x": round(local_x, 3), "y": round(local_y, 3)})

            ranges.append(round(dist, 3))

        return {
            "sensor_id": self.sensor_id,
            "ranges": ranges,
            "min_angle": self.min_angle_rad,
            "max_angle": self.max_angle_rad,
            "min_range": self.min_range,
            "max_range": self.max_range,
            "num_beams": len(ranges),
            "point_cloud_world": point_cloud_world,
            "point_cloud_local": point_cloud_local
        }


class IMUSensor:
    """
    6-Axis Inertial Measurement Unit (IMU) Simulator.
    Measures linear accelerations (including gravity vector decomposition) and angular rates.
    """

    def __init__(self, sensor_id: str, noise_accel: float = 0.02, noise_gyro: float = 0.005):
        self.sensor_id = sensor_id
        self.noise_accel = noise_accel
        self.noise_gyro = noise_gyro
        self._prev_velocity = Vec3()
        self._prev_time = 0.0

    def read(self, robot_body: RigidBody3D, gravity: Vec3, dt: float = 0.01) -> Dict[str, Any]:
        # Linear acceleration from dv/dt
        accel_world = (robot_body.linear_velocity - self._prev_velocity) / max(dt, 1e-4)
        self._prev_velocity = robot_body.linear_velocity.copy()

        # Specific force measured by accelerometer: a_measured = a_world - g
        specific_force_world = accel_world - gravity

        # Rotate into body coordinates
        r = robot_body.orientation.to_rotation_matrix()
        accel_body = Vec3.from_array(r.T @ specific_force_world.array)
        gyro_body = Vec3.from_array(r.T @ robot_body.angular_velocity.array)

        # Add sensor noise
        ax = accel_body.x + random.gauss(0.0, self.noise_accel)
        ay = accel_body.y + random.gauss(0.0, self.noise_accel)
        az = accel_body.z + random.gauss(0.0, self.noise_accel)

        gx = gyro_body.x + random.gauss(0.0, self.noise_gyro)
        gy = gyro_body.y + random.gauss(0.0, self.noise_gyro)
        gz = gyro_body.z + random.gauss(0.0, self.noise_gyro)

        return {
            "sensor_id": self.sensor_id,
            "accelerometer": {"x": round(ax, 3), "y": round(ay, 3), "z": round(az, 3)},
            "gyroscope": {"x": round(gx, 4), "y": round(gy, 4), "z": round(gz, 4)}
        }
