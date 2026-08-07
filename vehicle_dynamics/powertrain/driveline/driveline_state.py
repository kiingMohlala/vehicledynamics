"""Unified advanced driveline state (extends Phase 10.2 interface fields)."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class AdvancedDrivelineState:
    # --- Phase 10.2 compatible outputs ---
    torque_left: float = 0.0
    torque_right: float = 0.0
    torque_input: float = 0.0
    axle_speed: float = 0.0
    delta_omega: float = 0.0
    efficiency: float = 1.0

    # --- Phase 10.3 torsional diagnostics ---
    engine_speed: float = 0.0
    gearbox_speed: float = 0.0
    propshaft_speed: float = 0.0
    wheel_speed_left: float = 0.0
    wheel_speed_right: float = 0.0

    shaft_twist: float = 0.0           # rad (propshaft)
    halfshaft_twist_L: float = 0.0
    halfshaft_twist_R: float = 0.0
    mesh_theta: float = 0.0

    torsional_energy: float = 0.0      # J
    backlash_engaged: bool = False
    backlash_side: int = 0

    peak_torque: float = 0.0
    oscillation_freq_hz: float = 0.0   # estimate from k, J

    enabled: bool = True
