"""Aerodynamic configuration and coefficient sets."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class AeroCoefficients:
    """Reference coefficients at design ride height / zero yaw."""

    Cd: float = 0.34          # drag
    Cl_front: float = -0.45   # negative = downforce
    Cl_rear: float = -0.70
    Cy_beta: float = -0.8     # side force per rad yaw
    Cm_pitch: float = 0.05    # pitch moment about CG (non-dim by chord)
    Cn_yaw: float = -0.15     # yaw moment per rad
    Cl_roll: float = 0.0      # roll moment per rad roll (unused Phase 9.0)


@dataclass
class AeroConfig:
    """
    Vehicle aero configuration.

    enabled=False → all loads zero (regression contract).
    """

    enabled: bool = True
    rho: float = 1.225          # kg/m³
    frontal_area: float = 1.90  # m²
    wheelbase: float = 2.70     # m
    track: float = 1.55         # m
    cg_height: float = 0.45     # m
    ref_chord: float = 2.70     # m (use L for moment non-dim)

    # Design ride heights (m)
    h_front_ref: float = 0.080
    h_rear_ref: float = 0.100

    coeffs: AeroCoefficients = field(default_factory=AeroCoefficients)

    # Component breakdown fractions of total |Cl| / Cd (for reporting)
    front_wing_share: float = 0.25
    rear_wing_share: float = 0.35
    underfloor_share: float = 0.30
    body_share: float = 0.10
    cooling_drag_fraction: float = 0.08  # of Cd
