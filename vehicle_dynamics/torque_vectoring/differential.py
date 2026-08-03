"""
Axle torque split models.

Open differential: equal torque left/right (ignores speed difference for force).
Fixed bias: constant left fraction.
Active: controller-specified left fraction or delta-T.
"""

from __future__ import annotations

import numpy as np
from .parameters import TVParameters


def split_axle_torque(
    T_axle: float,
    mode: str,
    left_fraction: float = 0.5,
    delta_T: float = 0.0,
    max_delta_T: float = 1200.0,
) -> tuple[float, float]:
    """
    Split axle torque into (T_left, T_right).

    Positive torque = drive (accelerating).
    """
    T_axle = max(float(T_axle), 0.0)

    if mode == "open":
        return 0.5 * T_axle, 0.5 * T_axle

    if mode == "fixed_bias":
        fl = float(np.clip(left_fraction, 0.0, 1.0))
        return fl * T_axle, (1.0 - fl) * T_axle

    if mode == "active_rear":
        dT = float(np.clip(delta_T, -max_delta_T, max_delta_T))
        # T_L = T/2 + dT/2, T_R = T/2 - dT/2
        T_l = 0.5 * T_axle + 0.5 * dT
        T_r = 0.5 * T_axle - 0.5 * dT
        # Keep both non-negative when possible
        if T_l < 0.0:
            T_r += T_l
            T_l = 0.0
        if T_r < 0.0:
            T_l += T_r
            T_r = 0.0
        return max(T_l, 0.0), max(T_r, 0.0)

    # default open
    return 0.5 * T_axle, 0.5 * T_axle


def distribute_drive(
    throttle: float,
    params: TVParameters,
    rear_delta_T: float = 0.0,
) -> np.ndarray:
    """
    Return drive torque array [T_FL, T_FR, T_RL, T_RR].
    """
    throttle = float(np.clip(throttle, 0.0, 1.0))
    T_total = params.max_total_drive_torque * throttle
    T_front = T_total * params.front_drive_fraction
    T_rear = T_total * (1.0 - params.front_drive_fraction)

    # Front always open (or fixed if desired later)
    T_fl, T_fr = split_axle_torque(T_front, "open")

    mode = params.mode
    if mode == "fixed_bias":
        T_rl, T_rr = split_axle_torque(
            T_rear, "fixed_bias", left_fraction=params.fixed_left_fraction
        )
    elif mode == "active_rear":
        T_rl, T_rr = split_axle_torque(
            T_rear, "active_rear",
            delta_T=rear_delta_T,
            max_delta_T=params.max_delta_T,
        )
    else:
        T_rl, T_rr = split_axle_torque(T_rear, "open")

    return np.array([T_fl, T_fr, T_rl, T_rr], dtype=float)
