"""Active torque vectoring bias."""

from __future__ import annotations

import numpy as np

from .open_diff import open_split


def torque_vector_split(
    T_in: float,
    delta_T: float = 0.0,
    *,
    max_delta: float = 500.0,
) -> tuple[float, float, float]:
    """
    ΔT commanded by controller (positive → more torque to right).
    T_L = T_in/2 - ΔT/2
    T_R = T_in/2 + ΔT/2
    """
    T_L0, T_R0 = open_split(T_in)
    d = float(np.clip(delta_T, -max_delta, max_delta))
    # Limit so neither side goes far beyond input magnitude
    T_L = T_L0 - 0.5 * d
    T_R = T_R0 + 0.5 * d
    return float(T_L), float(T_R), float(d)
