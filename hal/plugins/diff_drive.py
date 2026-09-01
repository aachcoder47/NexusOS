"""
NexusOS HAL Plugin - Differential Drive Base Implementation.
Full real-world physics integration:
  • DC Motor torque curves with back-EMF (DCMotorModel)
  • Pacejka Magic Formula wheel slip (longitudinal + lateral forces)
  • Suspension spring-damper with dynamic normal force variation
  • Aerodynamic drag (Cd·A), gyroscopic precession from spinning wheels
  • Li-Ion battery electrochemical model (SoC, Peukert, Joule heating)
  • 6-DOF Rigid body with gyroscopic Euler term and body sleep
  • Autonomous Navigation (A*/DWA) with LiDAR obstacle fusion
"""

import time
import math
import logging
from typing import Dict, Any, Optional, Tuple, List

import numpy as np

from hal.base import (
    AbstractHardwarePlugin,
    RobotTelemetryData,
    MotorCommandData,
    RobotType,
    RobotMode,
    PoseData,
    TwistData,
    BatteryData,
    JointStateData,
    Vector3D
)
from physics.vector import Vec3, Quaternion
from physics.shapes import BoxShape
from physics.body import RigidBody3D, BodyType
from physics.world import PhysicsWorld
from physics.sensors import LiDARSensor, IMUSensor
from physics.vehicle import DCMotorModel, WheelModel, SuspensionModel, AerodynamicsModel, BatteryModel
from navigation.costmap import Costmap2D
from navigation.navigator import AutonomousNavigator, NavigationState

logger = logging.getLogger("NexusOS.HAL.DiffDrive")


class DiffDriveBasePlugin(AbstractHardwarePlugin):
    """
    Hardware Abstraction Layer plugin for Differential Drive Mobile Robots.
    Fully integrated with realistic vehicle physics:
      - DC Motor torque curves (back-EMF, gearbox, winding resistance)
      - Pacejka Magic Formula tire model (longitudinal + lateral slip forces)
      - Spring-damper suspension per wheel
      - Aerodynamic drag (quadratic velocity)
      - Gyroscopic effects from spinning wheel masses
      - Electrochemical Li-Ion battery with Peukert & Joule heating
      - Autonomous navigation with sensor fusion
    """

    def __init__(
        self,
        robot_id: str,
        wheel_track: float = 0.45,         # Track width (distance between wheel centres) [m]
        wheel_radius: float = 0.08,        # Wheel outer radius [m]
        robot_mass: float = 15.0,          # Total robot mass [kg]
        tire_surface: str = "dry_concrete", # Surface type for Pacejka model
        physics_world: Optional[PhysicsWorld] = None
    ):
        super().__init__(robot_id)
        self.wheel_track = float(wheel_track)
        self.wheel_radius = float(wheel_radius)
        self.robot_mass = float(robot_mass)
        self.tire_surface_name = tire_surface

        # ── Physics World ─────────────────────────────────────────────────────
        self.world: PhysicsWorld = physics_world if physics_world is not None else PhysicsWorld()

        # ── Rigid Body (chassis geometry: width × length × height) ────────────
        self.body_shape = BoxShape.from_dimensions(float(wheel_track), 0.6, 0.25)
        self.body = RigidBody3D(
            body_id=self.robot_id,
            shape=self.body_shape,
            body_type=BodyType.DYNAMIC,
            mass=self.robot_mass,
            position=Vec3(0.0, 0.0, 0.15),
            orientation=Quaternion.from_euler(0.0, 0.0, 0.0),
            restitution=0.05,
            friction_static=0.8,
            friction_dynamic=0.6,
            linear_damping=0.10,      # Viscous chassis drag (low: aerodynamics handles quadratic)
            angular_damping=0.25,     # Rotational drag
            drag_coeff_sq=0.08,       # ½ρCdA for boxy AGV shape (≈ 0.5*1.225*0.6*0.12)
            com_offset=Vec3(0.0, 0.02, -0.03)  # CoM slightly forward and low for stability
        )
        self.world.add_body(self.body)

        # ── Motor Models (one per drive wheel) ────────────────────────────────
        # Spec: typical 12V 150W AGV drive motor with 30:1 planetary gearbox
        self.motor_left = DCMotorModel(
            stall_torque_nm=3.5,
            no_load_rpm=5000.0,
            gear_ratio=30.0,
            motor_efficiency=0.82,
            armature_resistance_ohm=0.8,
            rated_voltage=12.0,
            motor_inertia_kgm2=2.5e-5
        )
        self.motor_right = DCMotorModel(
            stall_torque_nm=3.5,
            no_load_rpm=5000.0,
            gear_ratio=30.0,
            motor_efficiency=0.82,
            armature_resistance_ohm=0.8,
            rated_voltage=12.0,
            motor_inertia_kgm2=2.5e-5
        )

        # ── Wheel Models (Pacejka tire physics) ───────────────────────────────
        static_normal_force = (self.robot_mass * 9.81) / 2.0  # per drive wheel
        self.wheel_left = WheelModel(
            wheel_id=f"{robot_id}_wheel_L",
            radius_m=self.wheel_radius,
            width_m=0.04,
            mass_kg=0.28,
            normal_force_n=static_normal_force,
            tire_surface=tire_surface,
            rolling_resistance=0.010
        )
        self.wheel_right = WheelModel(
            wheel_id=f"{robot_id}_wheel_R",
            radius_m=self.wheel_radius,
            width_m=0.04,
            mass_kg=0.28,
            normal_force_n=static_normal_force,
            tire_surface=tire_surface,
            rolling_resistance=0.010
        )

        # ── Suspension (one per wheel) ────────────────────────────────────────
        self.susp_left = SuspensionModel(
            spring_constant_nm=4500.0,
            damping_coeff_ns_m=300.0,
            rest_length_m=self.wheel_radius + 0.04,
            max_travel_m=0.04
        )
        self.susp_right = SuspensionModel(
            spring_constant_nm=4500.0,
            damping_coeff_ns_m=300.0,
            rest_length_m=self.wheel_radius + 0.04,
            max_travel_m=0.04
        )

        # ── Aerodynamics ──────────────────────────────────────────────────────
        self.aerodynamics = AerodynamicsModel(
            drag_coefficient=0.60,
            frontal_area_m2=0.6 * 0.25,  # length × height of chassis
            air_density_kg_m3=1.225
        )

        # ── Battery ───────────────────────────────────────────────────────────
        self.battery = BatteryModel(
            nominal_voltage_v=12.0,
            capacity_ah=10.0,
            internal_resistance_ohm=0.08,
            initial_soc=0.98,
            peukert_exponent=1.05
        )

        # ── Sensors ───────────────────────────────────────────────────────────
        self.lidar = LiDARSensor(sensor_id=f"{robot_id}_lidar_360", num_rays=36, max_range=12.0)
        self.imu = IMUSensor(sensor_id=f"{robot_id}_imu_6axis")

        # ── Autonomous Navigator ───────────────────────────────────────────────
        self.costmap = Costmap2D(width_m=30.0, height_m=30.0, resolution=0.1)
        self.navigator = AutonomousNavigator(
            costmap=self.costmap,
            planner_type="A_STAR",
            local_planner_type="DWA"
        )

        # ── Controller State ──────────────────────────────────────────────────
        self._throttle_left: float = 0.0   # Motor throttle [-1..+1]
        self._throttle_right: float = 0.0
        self._target_v: float = 0.0        # Target linear velocity (m/s)
        self._target_w: float = 0.0        # Target angular velocity (rad/s)
        self._current_v: float = 0.0       # Actual forward speed from physics (m/s)
        self._current_w: float = 0.0       # Actual yaw rate from physics (rad/s)

        # Diagnostics
        self._total_current_draw_a: float = 0.0
        self._step_count: int = 0
        self._last_time: float = time.monotonic()

    # ──────────────────────────────────────────────────────────────────────────
    # HAL Lifecycle
    # ──────────────────────────────────────────────────────────────────────────

    async def initialize(self) -> bool:
        logger.info(f"[{self.robot_id}] Initializing Full-Physics DiffDrive HAL Plugin...")
        self._last_time = time.monotonic()
        self.is_initialized = True
        self.current_mode = RobotMode.MANUAL
        logger.info(f"[{self.robot_id}] DiffDrive HAL initialized. Motor Ke={self.motor_left.Ke:.4f} V·s/rad, "
                    f"Output stall torque={self.motor_left.output_stall_torque:.2f} N·m")
        return True

    # ──────────────────────────────────────────────────────────────────────────
    # Navigation Interface
    # ──────────────────────────────────────────────────────────────────────────

    def set_navigation_goal(self, gx: float, gy: float, gyaw: float = 0.0) -> bool:
        self.current_mode = RobotMode.AUTONOMOUS
        rx, ry = self.body.position.x, self.body.position.y
        _, _, ryaw = self.body.orientation.to_euler()
        return self.navigator.set_goal((rx, ry, ryaw), (gx, gy, gyaw))

    def cancel_navigation(self):
        self.navigator.cancel_goal()
        self._target_v = 0.0
        self._target_w = 0.0

    def set_tire_surface(self, surface_name: str):
        """Dynamically change tire-ground surface conditions (wet, gravel, ice, etc.)"""
        from physics.materials import TIRE_SURFACES
        ts = TIRE_SURFACES.get(surface_name)
        if ts:
            self.wheel_left.tire_surface = ts
            self.wheel_right.tire_surface = ts
            self.tire_surface_name = surface_name
            logger.info(f"[{self.robot_id}] Tire surface changed to: {surface_name}")

    # ──────────────────────────────────────────────────────────────────────────
    # Main Telemetry Loop  (called at ~50 Hz from edge agent)
    # ──────────────────────────────────────────────────────────────────────────

    async def read_telemetry(self) -> RobotTelemetryData:
        now = time.monotonic()
        wall_now = time.time()
        dt = float(np.clip(now - self._last_time, 0.002, 0.1))
        self._last_time = now
        self._step_count += 1

        # ── A. Pose and velocity from physics body ────────────────────────────
        roll, pitch, yaw = self.body.orientation.to_euler()
        rx, ry = self.body.position.x, self.body.position.y

        # Decompose world velocity into body-frame forward/lateral
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        v_world_x = self.body.linear_velocity.x
        v_world_y = self.body.linear_velocity.y
        self._current_v = v_world_x * cos_yaw + v_world_y * sin_yaw  # body-frame forward
        v_lateral = -v_world_x * sin_yaw + v_world_y * cos_yaw        # body-frame lateral
        self._current_w = self.body.angular_velocity.z

        # ── B. LiDAR Sensor Scan ──────────────────────────────────────────────
        lidar_data = self.lidar.scan(self.world, self.body)

        # ── C. Autonomous Navigator update ───────────────────────────────────
        if self.current_mode == RobotMode.AUTONOMOUS:
            nav_v, nav_w = self.navigator.update(
                (rx, ry, yaw),
                (self._current_v, self._current_w),
                lidar_data.get("point_cloud_world", [])
            )
            self._target_v = float(nav_v)
            self._target_w = float(nav_w)
        elif self.current_mode == RobotMode.EMERGENCY_STOP:
            self._target_v = 0.0
            self._target_w = 0.0

        # ── D. Differential Drive Velocity → Per-Wheel Speed Setpoints ────────
        # Desired wheel angular velocities (rad/s)
        half_track = self.wheel_track * 0.5
        omega_L_desired = (self._target_v - self._target_w * half_track) / self.wheel_radius
        omega_R_desired = (self._target_v + self._target_w * half_track) / self.wheel_radius

        # Throttle = desired / no_load_speed (normalised to [-1,+1])
        no_load_output = self.motor_left.output_no_load_rads
        self._throttle_left = float(np.clip(omega_L_desired / max(no_load_output, 0.1), -1.0, 1.0))
        self._throttle_right = float(np.clip(omega_R_desired / max(no_load_output, 0.1), -1.0, 1.0))

        # ── E. Motor Torque Computation ───────────────────────────────────────
        torque_L, current_L = self.motor_left.compute_output_torque(
            self._throttle_left, self.wheel_left.angular_velocity_rads
        )
        torque_R, current_R = self.motor_right.compute_output_torque(
            self._throttle_right, self.wheel_right.angular_velocity_rads
        )
        self._total_current_draw_a = current_L + current_R

        # ── F. Suspension: Dynamic Normal Force (spring-damper) ───────────────
        # Wheel contact points are at ±half_track from robot centre in body frame
        ground_z = 0.0  # flat floor (extend with heightmap if available)
        wheel_contact_z_L = max(0.0, self.body.position.z - (self.wheel_track * 0.5) * math.sin(roll))
        wheel_contact_z_R = max(0.0, self.body.position.z + (self.wheel_track * 0.5) * math.sin(roll))
        Fn_L = self.susp_left.compute_normal_force(ground_z, wheel_contact_z_L, dt)
        Fn_R = self.susp_right.compute_normal_force(ground_z, wheel_contact_z_R, dt)

        # Fallback to static normal force if suspension unloaded
        if Fn_L < 1.0:
            Fn_L = (self.robot_mass * 9.81) * 0.5
        if Fn_R < 1.0:
            Fn_R = (self.robot_mass * 9.81) * 0.5

        # ── G. Pacejka Wheel Forces ───────────────────────────────────────────
        Fx_L, Fy_L, Frr_L = self.wheel_left.compute_forces(
            torque_L, self._current_v, v_lateral, Fn_L, dt
        )
        Fx_R, Fy_R, Frr_R = self.wheel_right.compute_forces(
            torque_R, self._current_v, v_lateral, Fn_R, dt
        )

        # Combined traction forces (symmetric L/R drive)
        Fx_total = Fx_L + Fx_R + Frr_L + Frr_R  # Net longitudinal force
        Fy_total = (Fy_L + Fy_R) * 0.5           # Lateral restoring force (centering)

        # ── H. Apply Traction Forces to Rigid Body ────────────────────────────
        # Body-frame forces → world frame
        force_world = Vec3(
            Fx_total * cos_yaw - Fy_total * sin_yaw,
            Fx_total * sin_yaw + Fy_total * cos_yaw,
            0.0
        )
        self.body.apply_force(force_world)

        # Differential steering torque: τ_z = (F_R - F_L) * half_track
        diff_torque = (Fx_R - Fx_L) * half_track
        self.body.apply_torque(Vec3(0.0, 0.0, diff_torque))

        # ── I. Aerodynamic Drag ───────────────────────────────────────────────
        drag_force = self.aerodynamics.compute_drag_vector(self.body.linear_velocity)
        self.body.apply_force(drag_force)

        # ── J. Gyroscopic Precession from Wheel Spin ──────────────────────────
        gyro_L = self.wheel_left.compute_gyroscopic_torque(self._current_w)
        gyro_R = self.wheel_right.compute_gyroscopic_torque(self._current_w)
        self.body.apply_torque(gyro_L + gyro_R)

        # ── K. Step Physics World (sub-stepped internally) ────────────────────
        self.world.step(dt)

        # ── L. IMU Reading ────────────────────────────────────────────────────
        imu_data = self.imu.read(self.body, self.world.gravity, dt)

        # ── M. Battery Electrochemical Update ─────────────────────────────────
        battery_state = self.battery.update(self._total_current_draw_a, dt)

        # ── N. Assemble Telemetry ─────────────────────────────────────────────
        roll, pitch, yaw = self.body.orientation.to_euler()

        pose = PoseData(
            position=Vector3D(
                x=round(self.body.position.x, 3),
                y=round(self.body.position.y, 3),
                z=round(self.body.position.z, 3)
            ),
            orientation_rpy=Vector3D(x=round(roll, 4), y=round(pitch, 4), z=round(yaw, 4))
        )

        twist = TwistData(
            linear=Vector3D(
                x=round(self.body.linear_velocity.x, 4),
                y=round(self.body.linear_velocity.y, 4),
                z=round(self.body.linear_velocity.z, 4)
            ),
            angular=Vector3D(
                x=round(self.body.angular_velocity.x, 4),
                y=round(self.body.angular_velocity.y, 4),
                z=round(self.body.angular_velocity.z, 4)
            )
        )

        battery = BatteryData(
            voltage=battery_state["voltage"],
            percentage=battery_state["percentage"],
            is_charging=False
        )

        joints = JointStateData(
            names=["left_wheel_joint", "right_wheel_joint"],
            positions=[
                round(self.wheel_left.angular_velocity_rads * dt, 4),
                round(self.wheel_right.angular_velocity_rads * dt, 4)
            ],
            velocities=[
                round(self.wheel_left.angular_velocity_rads, 4),
                round(self.wheel_right.angular_velocity_rads, 4)
            ],
            effort=[
                round(torque_L, 3),
                round(torque_R, 3)
            ]
        )

        custom_metrics: Dict[str, Any] = {
            "lidar": lidar_data,
            "imu": imu_data,
            "navigation": self.navigator.to_dict(),
            "physics": {
                "mass_kg": self.robot_mass,
                "speed_ms": round(self._current_v, 4),
                "yaw_rate_rads": round(self._current_w, 4),
                "is_grounded": self.body.is_grounded,
                "is_sleeping": self.body.is_sleeping,
                "kinetic_energy_j": round(0.5 * self.robot_mass * self._current_v ** 2, 3),
                "active_collisions": len(self.world.active_manifolds),
                "total_bodies": len(self.world.bodies)
            },
            "motors": {
                "throttle_L": round(self._throttle_left, 3),
                "throttle_R": round(self._throttle_right, 3),
                "torque_L_nm": round(torque_L, 3),
                "torque_R_nm": round(torque_R, 3),
                "current_L_a": round(current_L, 3),
                "current_R_a": round(current_R, 3),
                "total_current_a": round(self._total_current_draw_a, 3),
                "power_w": round(battery_state["power_w"], 1)
            },
            "wheels": {
                "slip_ratio_L": round(self.wheel_left.slip_ratio, 4),
                "slip_ratio_R": round(self.wheel_right.slip_ratio, 4),
                "slip_angle_L_deg": round(math.degrees(self.wheel_left.slip_angle_rad), 2),
                "slip_angle_R_deg": round(math.degrees(self.wheel_right.slip_angle_rad), 2),
                "Fx_L_n": round(Fx_L, 2), "Fx_R_n": round(Fx_R, 2),
                "Fy_L_n": round(Fy_L, 2), "Fy_R_n": round(Fy_R, 2),
                "Frr_L_n": round(Frr_L, 2), "Frr_R_n": round(Frr_R, 2),
                "Fn_L_n": round(Fn_L, 2), "Fn_R_n": round(Fn_R, 2),
                "tire_surface": self.tire_surface_name
            },
            "suspension": {
                "compression_L_mm": round(self.susp_left.compression_m * 1000.0, 2),
                "compression_R_mm": round(self.susp_right.compression_m * 1000.0, 2),
            },
            "aerodynamics": {
                "drag_n": round(self.aerodynamics.compute_drag_force(abs(self._current_v)), 3),
                "Cd": self.aerodynamics.Cd,
                "frontal_area_m2": self.aerodynamics.A
            },
            "battery_detail": {
                "soc_percent": battery_state["percentage"],
                "voltage_v": battery_state["voltage"],
                "temperature_c": battery_state["temperature_c"],
                "total_energy_wh": battery_state["total_energy_wh"]
            },
            "firmware_version": "v4.0.0-RealWorldPhysics"
        }

        # Status message
        status_msg = "NORMAL"
        if self.current_mode == RobotMode.EMERGENCY_STOP:
            status_msg = "E_STOP_ACTIVE"
        elif self.navigator.state == NavigationState.NAVIGATING:
            status_msg = "AUTONOMOUS_NAVIGATING"
        elif self.navigator.state == NavigationState.OBSTACLE_AVOIDANCE:
            status_msg = "OBSTACLE_AVOIDANCE_ACTIVE"
        elif self.navigator.state == NavigationState.GOAL_REACHED:
            status_msg = "GOAL_REACHED"

        return RobotTelemetryData(
            robot_id=self.robot_id,
            robot_type=RobotType.DIFFERENTIAL_DRIVE,
            timestamp_sec=wall_now,
            pose=pose,
            twist=twist,
            battery=battery,
            mode=self.current_mode,
            joints=joints,
            custom_metrics=custom_metrics,
            status_message=status_msg
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Command Interface
    # ──────────────────────────────────────────────────────────────────────────

    async def send_command(self, command: MotorCommandData) -> bool:
        if self.current_mode == RobotMode.EMERGENCY_STOP and command.requested_mode != RobotMode.MANUAL:
            logger.warning(f"[{self.robot_id}] Command rejected: E-STOP is active!")
            return False

        if command.requested_mode:
            logger.info(f"[{self.robot_id}] Mode: {self.current_mode} → {command.requested_mode}")
            self.current_mode = command.requested_mode

        params = command.parameters or {}

        # Navigation goal
        if "nav_goal" in params or "goal" in params:
            g = params.get("nav_goal") or params.get("goal")
            self.set_navigation_goal(
                float(g.get("x", 0.0)),
                float(g.get("y", 0.0)),
                float(g.get("yaw", 0.0))
            )
        elif params.get("cancel_nav"):
            self.cancel_navigation()

        # Surface change (e.g. from cloud dashboard)
        if "tire_surface" in params:
            self.set_tire_surface(params["tire_surface"])

        # Manual twist override
        if command.target_twist:
            self._target_v = float(command.target_twist.linear.x)
            self._target_w = float(command.target_twist.angular.z)

        return True

    # ──────────────────────────────────────────────────────────────────────────
    # Safety
    # ──────────────────────────────────────────────────────────────────────────

    async def emergency_stop(self) -> bool:
        logger.error(f"[{self.robot_id}] ⚠ EMERGENCY STOP TRIGGERED!")
        self.cancel_navigation()
        self._target_v = 0.0
        self._target_w = 0.0
        self._throttle_left = 0.0
        self._throttle_right = 0.0
        self.body.linear_velocity = Vec3()
        self.body.angular_velocity = Vec3()
        self.current_mode = RobotMode.EMERGENCY_STOP
        return True

    async def shutdown(self) -> None:
        logger.info(f"[{self.robot_id}] Shutting down DiffDrive HAL plugin.")
        await self.emergency_stop()
        self.world.remove_body(self.robot_id)
        self.is_initialized = False
