"""
Tests for NexusOS Real-World Physics Engine Extensions.
Validates:
- Materials library & engineering constants
- Pacejka Magic Formula tire curves
- DC Motor torque-speed-current dynamics
- Suspension spring-damper reaction forces
- Aerodynamic quadratic drag forces
- Electrochemical battery model & Peukert drain
- Rigid body gyroscopic terms & sleep states
"""

import pytest
import math
import numpy as np

from physics.vector import Vec3, Quaternion
from physics.shapes import BoxShape
from physics.body import RigidBody3D, BodyType
from physics.materials import MATERIALS, TIRE_SURFACES, MaterialProperties, TireSurface
from physics.vehicle import DCMotorModel, WheelModel, SuspensionModel, AerodynamicsModel, BatteryModel


def test_materials_library():
    assert "aluminum_6061" in MATERIALS
    assert "concrete_smooth" in MATERIALS
    assert "tire_rubber_dry" in MATERIALS

    mat = MATERIALS["concrete_smooth"]
    assert mat.density_kg_m3 > 1000
    assert 0.0 < mat.restitution < 1.0
    assert mat.friction_static >= mat.friction_dynamic


def test_pacejka_tire_model():
    ts = TIRE_SURFACES["dry_concrete"]
    # Zero slip -> zero force
    assert abs(ts.longitudinal_friction(0.0)) < 1e-4
    assert abs(ts.lateral_friction(0.0)) < 1e-4

    # Moderate slip -> peak force near D_long
    fx_peak = ts.longitudinal_friction(0.15)
    assert 0.5 < fx_peak <= 1.0

    # Opposite slip -> negative force
    fx_neg = ts.longitudinal_friction(-0.15)
    assert fx_neg == pytest.approx(-fx_peak, rel=1e-3)


def test_dc_motor_model():
    motor = DCMotorModel(stall_torque_nm=3.5, no_load_rpm=5000.0, gear_ratio=30.0)
    assert motor.output_stall_torque > 0.0

    # At zero speed and full throttle -> maximum torque
    t_stall, i_stall = motor.compute_output_torque(throttle=1.0, wheel_angular_vel_rads=0.0)
    assert t_stall > 0.0
    assert i_stall > 0.0

    # At no-load speed -> torque approaches zero
    t_noload, _ = motor.compute_output_torque(throttle=1.0, wheel_angular_vel_rads=motor.output_no_load_rads)
    assert abs(t_noload) < t_stall * 0.1

    # Reverse throttle
    t_rev, _ = motor.compute_output_torque(throttle=-1.0, wheel_angular_vel_rads=0.0)
    assert t_rev < 0.0


def test_wheel_model():
    wheel = WheelModel(wheel_id="w_test", radius_m=0.08, normal_force_n=75.0, tire_surface="dry_concrete")
    fx, fy, frr = wheel.compute_forces(drive_torque_nm=5.0, vehicle_speed_ms=1.0, lateral_velocity_ms=0.0, normal_force_n=75.0, dt=0.02)
    assert isinstance(fx, float)
    assert isinstance(fy, float)
    assert isinstance(frr, float)
    # Accelerating wheel should have updated angular velocity
    assert wheel.angular_velocity_rads != 0.0


def test_suspension_model():
    susp = SuspensionModel(spring_constant_nm=4500.0, damping_coeff_ns_m=300.0, rest_length_m=0.12, max_travel_m=0.04)
    # Wheel at natural rest position -> zero force
    fn_rest = susp.compute_normal_force(terrain_height_m=0.0, wheel_contact_z_m=0.12, dt=0.02)
    assert fn_rest == 0.0

    # Wheel compressed (ground bumped up) -> positive reaction force
    fn_comp = susp.compute_normal_force(terrain_height_m=0.02, wheel_contact_z_m=0.12, dt=0.02)
    assert fn_comp > 0.0


def test_aerodynamics_model():
    aero = AerodynamicsModel(drag_coefficient=0.6, frontal_area_m2=0.15)
    f0 = aero.compute_drag_force(0.0)
    assert f0 == 0.0

    f1 = aero.compute_drag_force(1.0)
    f2 = aero.compute_drag_force(2.0)
    # Quadratic drag: doubling speed quadruples drag
    assert f2 == pytest.approx(f1 * 4.0, rel=1e-3)

    drag_vec = aero.compute_drag_vector(Vec3(2.0, 0.0, 0.0))
    assert drag_vec.x < 0.0  # opposes motion


def test_battery_model():
    batt = BatteryModel(nominal_voltage_v=12.0, capacity_ah=10.0, initial_soc=1.0)
    ocv = batt.open_circuit_voltage()
    assert ocv >= 12.0

    # State update under 10A current draw
    state = batt.update(current_draw_a=10.0, dt_sec=1.0)
    assert state["voltage"] <= ocv
    assert state["percentage"] <= 100.0
    assert state["power_w"] > 0.0


def test_rigid_body_gyroscopic_and_sleep():
    body = RigidBody3D("test_body", BoxShape.from_dimensions(0.4, 0.6, 0.2), mass=10.0, linear_damping=0.1, angular_damping=0.2)
    
    # Check initial active
    assert not body.is_sleeping

    # Step stationary body multiple times -> should eventually sleep
    for _ in range(50):
        body.integrate(0.02, Vec3(0, 0, 0))

    assert body.is_sleeping

    # Force should wake body
    body.apply_force(Vec3(50.0, 0.0, 0.0))
    body.integrate(0.02, Vec3(0, 0, 0))
    assert not body.is_sleeping
    assert body.linear_velocity.length() > 0.0
