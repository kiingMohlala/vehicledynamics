"""Locked differential: equal speeds; torque by traction capacity."""

from __future__ import annotations

import numpy as np


def locked_split(
    T_in: float,
    mu_left: float = 1.0,
    mu_right: float = 1.0,
    Fz_left: float = 4000.0,
    Fz_right: float = 4000.0,
    radius: float = 0.32,
) -> tuple[float, float]:
    """
    Distribute torque proportional to available traction while summing to T_in.
    T_max,i = μ_i Fz_i r
    """
    cap_L = max(mu_left * Fz_left * radius, 1e-6)
    cap_R = max(mu_right * Fz_right * radius, 1e-6)
    total_cap = cap_L + cap_R
    # Ideal proportional split
    T_L = T_in * (cap_L / total_cap)
    T_R = T_in * (cap_R / total_cap)
    # Clamp to capacity (excess discarded / absorbed as slip loss proxy)
    T_L = float(np.clip(T_L, -cap_L, cap_L))
    T_R = float(np.clip(T_R, -cap_R, cap_R))
    return T_L, T_R
