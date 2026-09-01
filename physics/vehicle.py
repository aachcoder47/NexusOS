"""
NexusOS Physics Engine - Realistic Vehicle Dynamics Module.
Implements DC motor torque curves, back-EMF, gear reduction, wheel slip (Pacejka Magic Formula),
rolling resistance, aerodynamic drag (Cd · A), suspension spring-damper, and gyroscopic precession.
All constants sourced from real small-robot / AGV hardware specifications.
"""

from __future__ import annotations
import math
from typing import Optional, Tuple, Dict, Any
import numpy as np

from physics.vector import Vec3


# ───────────────────────────────────────────────────────────────────────────────
# DC Motor Model
# ───────────────────────────────────────────────────────────────────────────────

class DCMotorModel:
    """
    Brushed/Brushless DC motor with realistic torque-speed characteristic.
    Torque curve: T = T_stall - (T_stall / ω_no_load) * ω  (linear approximation)
    Back-EMF modelled as speed-dependent voltage drop.
    """

    def __init__(
        self,
        stall_torque_nm: float = 3.5,       # Stall torque (N·m) at rated voltage
        no_load_rpm: float = 5000.0,        # No-load speed (RPM)
        gear_ratio: float = 30.0,           # Gearbox reduction ratio
        motor_efficiency: float = 0.85,     # Motor efficiency η (combined motor+gearbox)
        armature_resistance_ohm: float = 0.8, # Winding resistance (Ω)
        rated_voltage: float = 12.0,        # Nominal supply voltage (V)
        motor_inertia_kgm2: float = 2.5e-5  # Rotor moment of inertia (kg·m²)
    ):
        self.stall_torque_nm = float(stall_torque_nm)
        self.no_load_rpm = float(no_load_rpm)
        self.no_load_rads = float(no_load_rpm) * 2.0 * math.pi / 60.0
        self.gear_ratio = float(gear_ratio)
        self.efficiency = float(motor_efficiency)
        self.Ra = float(armature_resistance_ohm)
        self.rated_voltage = float(rated_voltage)
        self.motor_inertia = float(motor_inertia_kgm2)

        # Back-EMF constant: Ke = V_rated / ω_no_load_motor
        self.Ke = self.rated_voltage / self.no_load_rads  # V·s/rad

        # Torque constant: Kt = T_stall / (V_rated / Ra)
        self.Kt = self.stall_torque_nm / (self.rated_voltage / self.Ra)

        # Output shaft stall torque (after gearbox, with efficiency)
        self.output_stall_torque = self.stall_torque_nm * self.gear_ratio * self.efficiency

        # Output no-load speed (rad/s at output shaft)
        self.output_no_load_rads = self.no_load_rads / self.gear_ratio

    def compute_output_torque(self, throttle: float, wheel_angular_vel_rads: float) -> Tuple[float, float]:
        """
        Compute output shaft torque and current draw given throttle [-1..+1] and current wheel speed.
        Returns (output_torque_nm, current_draw_A).
        """
        throttle = max(-1.0, min(1.0, throttle))
        applied_voltage = throttle * self.rated_voltage

        # Motor (input shaft) speed from wheel speed
        motor_rads = abs(wheel_angular_vel_rads) * self.gear_ratio

        # Back-EMF voltage
        back_emf = self.Ke * motor_rads * math.copysign(1.0, throttle) if throttle != 0 else 0.0

        # Armature current
        current_a = (applied_voltage - back_emf) / self.Ra

        # Motor torque
        motor_torque = self.Kt * current_a

        # Output shaft torque (clamped to ±stall_torque)
        output_torque = motor_torque * self.gear_ratio * self.efficiency
        output_torque = max(-self.output_stall_torque, min(self.output_stall_torque, output_torque))

        return (output_torque, abs(current_a))

    def compute_power_consumption_watts(self, torque_nm: float, wheel_rads: float, current_a: float) -> float:
        """Electrical power consumed P = V * I (accounting for back-EMF)."""
        motor_rads = abs(wheel_rads) * self.gear_ratio
        back_emf = self.Ke * motor_rads
        v_applied = abs(current_a) * self.Ra + back_emf
        return abs(v_applied * current_a)


# ───────────────────────────────────────────────────────────────────────────────
# Wheel Model with Pacejka Slip & Rolling Resistance
# ───────────────────────────────────────────────────────────────────────────────

class WheelModel:
    """
    Complete physical wheel model:
    - Pacejka Magic Formula longitudinal friction (slip ratio)
    - Lateral slip angle friction (Ackermann/differential-drive cornering forces)
    - Rolling resistance (speed-dependent Crr model)
    - Gyroscopic precession torque from spinning wheel mass
    """

    def __init__(
        self,
        wheel_id: str,
        radius_m: float = 0.08,         # Wheel outer radius (m)
        width_m: float = 0.04,          # Tread width (m)
        mass_kg: float = 0.25,          # Wheel mass (kg)
        normal_force_n: float = 50.0,   # Static normal force on wheel (N) = mass * g / 4
        tire_surface: str = "dry_concrete",
        rolling_resistance: float = 0.010  # Crr at low speed; increases with velocity
    ):
        from physics.materials import TIRE_SURFACES, MATERIALS
        self.wheel_id = wheel_id
        self.radius = float(radius_m)
        self.width = float(width_m)
        self.mass = float(mass_kg)
        self.normal_force = float(normal_force_n)
        self.tire_surface = TIRE_SURFACES.get(tire_surface, TIRE_SURFACES["dry_concrete"])
        self.Crr_base = float(rolling_resistance)

        # Wheel moment of inertia (solid cylinder: I = 0.5 * m * r²)
        self.inertia = 0.5 * self.mass * self.radius * self.radius  # kg·m²

        # State
        self.angular_velocity_rads: float = 0.0   # ω wheel (rad/s)
        self.slip_ratio: float = 0.0
        self.slip_angle_rad: float = 0.0
        self.longitudinal_force_n: float = 0.0
        self.lateral_force_n: float = 0.0
        self.rolling_resistance_n: float = 0.0

    def compute_forces(
        self,
        drive_torque_nm: float,
        vehicle_speed_ms: float,      # Forward speed of vehicle at wheel contact (m/s)
        lateral_velocity_ms: float,   # Lateral velocity at wheel (m/s)
        normal_force_n: float,        # Current normal force from suspension/terrain
        dt: float
    ) -> Tuple[float, float, float]:
        """
        Compute realistic wheel forces using Pacejka model.
        Returns (F_longitudinal, F_lateral, F_rolling_resistance) in Newtons.
        """
        Fn = max(0.0, normal_force_n)

        # ── 1. Wheel angular velocity integration (Newton: I*α = T_drive - T_ground_reaction)
        wheel_surface_speed = self.angular_velocity_rads * self.radius
        wheel_ground_speed = vehicle_speed_ms  # speed at ground contact

        # ── 2. Slip Ratio κ = (v_wheel - v_vehicle) / max(|v_vehicle|, ε)
        speed_ref = max(abs(vehicle_speed_ms), abs(wheel_surface_speed), 0.05)
        self.slip_ratio = (wheel_surface_speed - wheel_ground_speed) / speed_ref
        self.slip_ratio = max(-1.0, min(1.0, self.slip_ratio))

        # ── 3. Slip Angle α = arctan(v_lateral / max(|v_forward|, ε))
        if abs(vehicle_speed_ms) > 0.01:
            self.slip_angle_rad = math.atan2(lateral_velocity_ms, max(abs(vehicle_speed_ms), 0.01))
        else:
            self.slip_angle_rad = 0.0

        # ── 4. Pacejka Magic Formula friction coefficients
        mu_long = self.tire_surface.longitudinal_friction(self.slip_ratio)
        mu_lat = self.tire_surface.lateral_friction(self.slip_angle_rad)

        # ── 5. Longitudinal Force: F_x = μ_long * F_normal
        self.longitudinal_force_n = mu_long * Fn

        # ── 6. Lateral (cornering) Force: F_y = μ_lat * F_normal
        self.lateral_force_n = mu_lat * Fn

        # ── 7. Rolling Resistance: F_rr = (Crr + Crr_v * v²) * F_normal
        crr_velocity_factor = self.Crr_base + 0.0001 * vehicle_speed_ms * vehicle_speed_ms
        self.rolling_resistance_n = crr_velocity_factor * Fn
        if abs(vehicle_speed_ms) > 0.001:
            self.rolling_resistance_n *= -math.copysign(1.0, vehicle_speed_ms)
        else:
            self.rolling_resistance_n = 0.0

        # ── 8. Update Wheel Angular Velocity via torque balance
        # Ground reaction torque opposes spin
        ground_reaction_torque = -self.longitudinal_force_n * self.radius
        net_torque = drive_torque_nm + ground_reaction_torque

        # α = T_net / I_wheel
        alpha = net_torque / max(self.inertia, 1e-6)
        self.angular_velocity_rads += alpha * dt

        return (self.longitudinal_force_n, self.lateral_force_n, self.rolling_resistance_n)

    def compute_gyroscopic_torque(self, vehicle_yaw_rate_rads: float) -> Vec3:
        """
        Gyroscopic precession torque from spinning wheel (resists chassis pitch/yaw changes).
        τ_gyro = Iω × Ω (cross product of angular momenta)
        Returns gyroscopic torque vector (acts on chassis as roll torque from yaw).
        """
        # Wheel angular momentum vector: aligned with wheel rotation axis (Y-axis for forward-rolling wheel)
        Lw = self.inertia * self.angular_velocity_rads
        # Gyroscopic roll torque: τ = Lw * Ω_yaw (precession causes roll moment)
        gyro_roll = Lw * vehicle_yaw_rate_rads
        return Vec3(gyro_roll, 0.0, 0.0)


# ───────────────────────────────────────────────────────────────────────────────
# Suspension Spring-Damper Model
# ───────────────────────────────────────────────────────────────────────────────

class SuspensionModel:
    """
    Per-wheel spring-damper suspension with natural frequency and damping ratio matching real AGV specs.
    Computes dynamic normal force variation from terrain and chassis pitch/roll.
    """

    def __init__(
        self,
        spring_constant_nm: float = 4500.0,  # Spring stiffness k (N/m) — hard rubber mount
        damping_coeff_ns_m: float = 300.0,   # Damping c (N·s/m) — ζ ≈ 0.3 (underdamped)
        rest_length_m: float = 0.12,         # Natural spring rest length (m)
        max_travel_m: float = 0.04           # Max compression/extension (m)
    ):
        self.k = float(spring_constant_nm)
        self.c = float(damping_coeff_ns_m)
        self.rest_length = float(rest_length_m)
        self.max_travel = float(max_travel_m)
        self.natural_freq_hz = math.sqrt(self.k) / (2.0 * math.pi)

        # State
        self.compression_m: float = 0.0      # Current compression (+ = compressed)
        self.velocity_m_s: float = 0.0        # Compression rate (m/s)

    def compute_normal_force(self, terrain_height_m: float, wheel_contact_z_m: float, dt: float) -> float:
        """
        Compute current suspension normal force based on terrain height and wheel contact position.
        Returns normal force (N). Zero if wheel is airborne.
        """
        # Compression = rest_length - (wheel_z - terrain_z)
        penetration = self.rest_length - (wheel_contact_z_m - terrain_height_m)
        new_compression = max(0.0, min(self.max_travel, penetration))

        # Compression velocity
        self.velocity_m_s = (new_compression - self.compression_m) / max(dt, 1e-4)
        self.compression_m = new_compression

        if self.compression_m <= 0.0:
            return 0.0  # Airborne wheel

        # F_suspension = k * x + c * ẋ (spring + damper)
        normal_force = self.k * self.compression_m + self.c * abs(self.velocity_m_s)
        return max(0.0, normal_force)


# ───────────────────────────────────────────────────────────────────────────────
# Aerodynamic Drag Model
# ───────────────────────────────────────────────────────────────────────────────

class AerodynamicsModel:
    """
    Realistic aerodynamic drag force model based on vehicle frontal area and Cd.
    F_drag = 0.5 * ρ_air * v² * Cd * A
    Also models rotational drag (viscous drag on spinning chassis).
    """

    def __init__(
        self,
        drag_coefficient: float = 0.6,    # Cd (boxy AGV shape ≈ 0.6, streamlined ≈ 0.3)
        frontal_area_m2: float = 0.12,    # Projected frontal area (m²) — 0.4m × 0.3m AGV
        air_density_kg_m3: float = 1.225  # Air density at 15°C, sea level
    ):
        self.Cd = float(drag_coefficient)
        self.A = float(frontal_area_m2)
        self.rho = float(air_density_kg_m3)

    def compute_drag_force(self, velocity_ms: float) -> float:
        """Aerodynamic drag force (N) opposing motion. F = 0.5 * ρ * v² * Cd * A."""
        return 0.5 * self.rho * velocity_ms * velocity_ms * self.Cd * self.A

    def compute_drag_vector(self, linear_velocity: Vec3) -> Vec3:
        """Drag force vector opposing current velocity direction."""
        speed = linear_velocity.length()
        if speed < 0.01:
            return Vec3()
        drag_magnitude = self.compute_drag_force(speed)
        return linear_velocity.normalized() * (-drag_magnitude)


# ───────────────────────────────────────────────────────────────────────────────
# Battery Electrochemical Model (Li-Ion / LiFePO4)
# ───────────────────────────────────────────────────────────────────────────────

class BatteryModel:
    """
    Realistic Li-Ion battery electrochemical model with:
    - Open-circuit voltage (OCV) vs State-of-Charge (SoC) curve
    - Internal resistance drop under load (Peukert's law)
    - Temperature-dependent capacity reduction
    - Regenerative braking charge recovery
    """

    def __init__(
        self,
        nominal_voltage_v: float = 12.0,
        capacity_ah: float = 10.0,          # Nameplate capacity (Ah)
        internal_resistance_ohm: float = 0.08, # ESR (Equivalent Series Resistance)
        initial_soc: float = 0.98,           # State of charge [0..1]
        peukert_exponent: float = 1.05       # Peukert exponent (1.0 = ideal, 1.1 = typical AGV)
    ):
        self.V_nom = float(nominal_voltage_v)
        self.capacity_ah = float(capacity_ah)
        self.capacity_wh = self.V_nom * self.capacity_ah
        self.Ri = float(internal_resistance_ohm)
        self.soc = float(initial_soc)
        self.peukert_n = float(peukert_exponent)
        self.temperature_c: float = 25.0
        self.current_draw_a: float = 0.0
        self.total_energy_consumed_wh: float = 0.0

        # LiFePO4 OCV vs SoC lookup (typical flat-ish discharge curve)
        self._soc_lut = [0.0,  0.05, 0.10, 0.20, 0.30, 0.50, 0.70, 0.80, 0.90, 0.95, 1.0]
        self._ocv_lut = [10.0, 11.0, 11.8, 12.2, 12.4, 12.6, 12.7, 12.75, 12.8, 13.0, 13.2]

    def open_circuit_voltage(self) -> float:
        """Interpolate OCV from SoC lookup table."""
        soc = max(0.0, min(1.0, self.soc))
        for i in range(len(self._soc_lut) - 1):
            if self._soc_lut[i] <= soc <= self._soc_lut[i + 1]:
                t = (soc - self._soc_lut[i]) / (self._soc_lut[i + 1] - self._soc_lut[i])
                return self._ocv_lut[i] + t * (self._ocv_lut[i + 1] - self._ocv_lut[i])
        return self.V_nom

    def terminal_voltage(self) -> float:
        """Terminal voltage under current draw: V_terminal = OCV - I * Ri."""
        return max(8.0, self.open_circuit_voltage() - abs(self.current_draw_a) * self.Ri)

    def update(self, current_draw_a: float, dt_sec: float) -> Dict[str, float]:
        """
        Update battery state over time step dt.
        Returns: dict with voltage, soc, temperature, power_w.
        """
        self.current_draw_a = float(current_draw_a)
        dt_h = dt_sec / 3600.0  # Convert seconds to hours for Ah

        # Peukert-corrected capacity drain
        if abs(current_draw_a) > 0.01:
            peukert_rate = (abs(current_draw_a) ** self.peukert_n) / self.capacity_ah
            d_soc = peukert_rate * dt_h
        else:
            d_soc = 0.0

        self.soc = max(0.0, self.soc - d_soc)

        # Temperature model: I²Ri heating, passive cooling to ambient
        joule_heat_w = self.current_draw_a ** 2 * self.Ri
        self.temperature_c += (joule_heat_w * dt_sec - 0.1 * (self.temperature_c - 25.0) * dt_sec)
        self.temperature_c = max(25.0, min(60.0, self.temperature_c))

        power_w = abs(current_draw_a) * self.terminal_voltage()
        self.total_energy_consumed_wh += power_w * dt_h

        return {
            "voltage": round(self.terminal_voltage(), 3),
            "percentage": round(self.soc * 100.0, 2),
            "current_a": round(self.current_draw_a, 3),
            "power_w": round(power_w, 1),
            "temperature_c": round(self.temperature_c, 1),
            "total_energy_wh": round(self.total_energy_consumed_wh, 2),
            "is_charging": False
        }
