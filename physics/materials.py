"""
NexusOS Physics Engine - Real-World Material & Surface Properties Database.
Physically accurate material constants for friction, restitution, density, and thermal properties.
All values sourced from engineering handbooks (Shigley's, Machinery's Handbook, ISO 8295).
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class MaterialProperties:
    """Physical properties for a real-world material."""
    name: str
    density_kg_m3: float         # Material density (kg/m³)
    youngs_modulus_pa: float     # Young's modulus for compliance (Pa)
    restitution: float           # Coefficient of restitution [0..1]
    friction_static: float       # Static friction coefficient (steel on surface)
    friction_dynamic: float      # Kinetic/dynamic friction coefficient
    friction_rolling: float      # Rolling resistance coefficient (Crr)
    thermal_conductivity: float  # W/m·K


# ───────────────────────────────────────────────────────────────────────────────
# Standard Engineering Materials Library
# ───────────────────────────────────────────────────────────────────────────────

MATERIALS: Dict[str, MaterialProperties] = {
    # Robot chassis materials
    "aluminum_6061":  MaterialProperties("Aluminum 6061",        2700,  69e9,  0.35, 0.61, 0.47, 0.002, 167.0),
    "steel_1020":     MaterialProperties("Steel 1020",            7870, 200e9, 0.20, 0.74, 0.57, 0.002, 51.9),
    "polycarbonate":  MaterialProperties("Polycarbonate",         1200,   2e9, 0.60, 0.47, 0.36, 0.004, 0.21),
    "carbon_fiber":   MaterialProperties("Carbon Fiber Composite", 1550,  70e9, 0.25, 0.50, 0.40, 0.002, 7.0),

    # Ground / floor surfaces
    "concrete_smooth":   MaterialProperties("Smooth Concrete",   2300, 30e9, 0.15, 0.80, 0.70, 0.010, 1.4),
    "concrete_rough":    MaterialProperties("Rough Concrete",     2300, 30e9, 0.10, 0.90, 0.80, 0.015, 1.4),
    "asphalt":           MaterialProperties("Asphalt",            2240, 10e9, 0.12, 0.75, 0.65, 0.012, 0.75),
    "polished_tile":     MaterialProperties("Polished Tile",      2500, 70e9, 0.30, 0.40, 0.25, 0.004, 1.0),
    "wood_floor":        MaterialProperties("Wood Floor",          700,  10e9, 0.45, 0.50, 0.40, 0.008, 0.15),
    "carpet_low_pile":   MaterialProperties("Carpet Low Pile",     200,   0.5e6, 0.05, 0.70, 0.60, 0.035, 0.05),
    "carpet_high_pile":  MaterialProperties("Carpet High Pile",    150,   0.2e6, 0.02, 0.80, 0.70, 0.060, 0.04),
    "packed_dirt":       MaterialProperties("Packed Dirt",        1600,   5e6, 0.10, 0.70, 0.55, 0.025, 0.50),
    "gravel":            MaterialProperties("Gravel",             1500,   1e6, 0.05, 0.65, 0.50, 0.050, 0.40),
    "sand_dry":          MaterialProperties("Dry Sand",           1600,   0.5e6, 0.02, 0.55, 0.45, 0.100, 0.30),
    "sand_wet":          MaterialProperties("Wet Sand",           1900,   1e6, 0.03, 0.70, 0.60, 0.080, 0.50),
    "grass_short":       MaterialProperties("Short Grass",         200,   0.3e6, 0.04, 0.60, 0.50, 0.055, 0.08),

    # Obstacle/object materials
    "rubber_hard":    MaterialProperties("Hard Rubber",   1200,  0.01e9, 0.75, 0.80, 0.65, 0.010, 0.25),
    "rubber_soft":    MaterialProperties("Soft Rubber",    900,   0.001e9, 0.85, 0.90, 0.80, 0.015, 0.15),
    "wood_oak":       MaterialProperties("Oak Wood",        760,  12e9, 0.50, 0.60, 0.45, 0.006, 0.17),
    "foam_rigid":     MaterialProperties("Rigid Foam",       30,   0.2e6, 0.05, 0.45, 0.35, 0.030, 0.04),
    "glass":          MaterialProperties("Glass",           2500,  70e9, 0.55, 0.40, 0.30, 0.002, 1.00),

    # Wheel tire materials (rubber compounds)
    "tire_rubber_dry": MaterialProperties("Tire Rubber Dry Road",   1150, 0.005e9, 0.05, 0.85, 0.75, 0.008, 0.20),
    "tire_rubber_wet": MaterialProperties("Tire Rubber Wet Road",   1150, 0.005e9, 0.05, 0.50, 0.40, 0.012, 0.20),
}


@dataclass
class TireSurface:
    """
    Pacejka Magic Formula simplified coefficients for realistic tire-ground friction curves.
    Models the nonlinear Slip Ratio → Longitudinal Force and Slip Angle → Lateral Force relationships.
    B: Stiffness factor, C: Shape factor, D: Peak factor, E: Curvature factor.
    """
    name: str
    # Longitudinal (drive/braking) Magic Formula coefficients
    B_long: float = 11.0
    C_long: float = 1.9
    D_long: float = 1.0   # Peak friction coefficient
    E_long: float = 0.97

    # Lateral (cornering) Magic Formula coefficients
    B_lat: float = 7.5
    C_lat: float = 1.4
    D_lat: float = 0.9
    E_lat: float = -0.5

    def longitudinal_friction(self, slip_ratio: float) -> float:
        """
        Pacejka Magic Formula (simplified): F_x = D * sin(C * arctan(B*k - E*(B*k - arctan(B*k))))
        slip_ratio ∈ [-1, +1]: +1 = full wheel spin, -1 = full lock up
        """
        k = max(-1.0, min(1.0, slip_ratio))
        bk = self.B_long * k
        return self.D_long * math.sin(self.C_long * math.atan(bk - self.E_long * (bk - math.atan(bk))))

    def lateral_friction(self, slip_angle_rad: float) -> float:
        """
        Lateral force coefficient from slip angle α (radians).
        """
        a = max(-1.0, min(1.0, math.tan(slip_angle_rad)))
        ba = self.B_lat * a
        return self.D_lat * math.sin(self.C_lat * math.atan(ba - self.E_lat * (ba - math.atan(ba))))


import math  # noqa: E402 – needed for TireSurface methods

# Prebuilt tire-surface combinations
TIRE_SURFACES: Dict[str, TireSurface] = {
    "dry_concrete":     TireSurface("Dry Concrete",   B_long=11.0, C_long=1.9, D_long=0.90, E_long=0.97, B_lat=7.5, C_lat=1.4, D_lat=0.85),
    "wet_concrete":     TireSurface("Wet Concrete",   B_long=9.0,  C_long=1.8, D_long=0.60, E_long=0.95, B_lat=6.0, C_lat=1.3, D_lat=0.55),
    "dry_asphalt":      TireSurface("Dry Asphalt",    B_long=10.5, C_long=1.9, D_long=0.88, E_long=0.97, B_lat=7.0, C_lat=1.4, D_lat=0.82),
    "wet_asphalt":      TireSurface("Wet Asphalt",    B_long=8.5,  C_long=1.8, D_long=0.55, E_long=0.94, B_lat=5.5, C_lat=1.3, D_lat=0.50),
    "packed_dirt":      TireSurface("Packed Dirt",    B_long=7.0,  C_long=1.7, D_long=0.65, E_long=0.90, B_lat=4.5, C_lat=1.3, D_lat=0.60),
    "gravel":           TireSurface("Gravel",         B_long=5.5,  C_long=1.6, D_long=0.55, E_long=0.88, B_lat=4.0, C_lat=1.2, D_lat=0.50),
    "grass":            TireSurface("Grass",          B_long=5.0,  C_long=1.5, D_long=0.50, E_long=0.85, B_lat=3.5, C_lat=1.2, D_lat=0.45),
    "sand":             TireSurface("Sand",           B_long=3.0,  C_long=1.4, D_long=0.35, E_long=0.80, B_lat=2.5, C_lat=1.1, D_lat=0.30),
    "polished_tile":    TireSurface("Polished Tile",  B_long=9.0,  C_long=1.9, D_long=0.40, E_long=0.92, B_lat=6.0, C_lat=1.3, D_lat=0.35),
    "ice":              TireSurface("Ice",            B_long=4.0,  C_long=2.0, D_long=0.12, E_long=0.99, B_lat=3.0, C_lat=1.5, D_lat=0.10),
}
