"""Clutch-type limited-slip differential."""

from __future__ import annotations

import numpy as np

from .open_diff import open_split


def clutch_lsd_split(
    T_in: float,
    omega_L: float,
    omega_R: float,
    *,
    preload: float = 80.0,
    k_lock: float = 15.0,
    max_bias: float = 400.0,
) -> tuple[float, float, float]:
    """
    T_bias = preload + k |Δω|
    Bias transfers from fast to slow wheel (helps traction).
    Returns (T_L, T_R, bias_magnitude).
    """
    T_L0, T_R0 = open_split(T_in)
    dw = omega_L - omega_R
    bias = float(np.clip(preload + k_lock * abs(dw), 0.0, max_bias))
    # Transfer bias toward slower wheel
    if abs(dw) < 1e-9:
        return T_L0, T_R0, 0.0
    # Positive dw => left faster => transfer torque to right
    sign = np.sign(dw)
    T_L = T_L0 - sign * bias * 0.5
    T_R = T_R0 + sign * bias * 0.5
    return float(T_L), float(T_R), bias
