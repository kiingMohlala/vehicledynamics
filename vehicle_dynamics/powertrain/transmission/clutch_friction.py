"""Clutch friction capacity and heat generation."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class ClutchFrictionParams:
    mu: float = 0.35              # friction coefficient
    mean_radius: float = 0.12     # m
    n_surfaces: int = 2
    # Capacity must cover engine peak (~590–650 N·m for 280 kW class).
    # T_max = μ F r n → F=8500 → T_max≈714 N·m (was 420 N·m — undersized vs engine).
    max_clamp_force: float = 8500.0  # N
    fade_temp_C: float = 250.0
    fade_factor_min: float = 0.55


def clutch_capacity(
    engagement: float,
    params: ClutchFrictionParams,
    temp_C: float = 80.0,
) -> float:
    """
    T_max = μ F r_m n
    engagement 0..1 scales clamp force.
    """
    e = float(np.clip(engagement, 0.0, 1.0))
    F = e * params.max_clamp_force
    # Simple thermal fade
    fade = 1.0
    if temp_C > params.fade_temp_C:
        over = (temp_C - params.fade_temp_C) / 100.0
        fade = max(params.fade_factor_min, 1.0 - 0.3 * over)
    return params.mu * F * params.mean_radius * params.n_surfaces * fade


def clutch_heat_power(torque: float, omega_slip: float) -> float:
    """Heat generation W = |T * ω_slip|."""
    return abs(torque * omega_slip)
