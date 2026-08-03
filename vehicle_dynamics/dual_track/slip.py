"""
Phase 5.2 – Per-wheel longitudinal slip helpers.

Thin wrappers around kinematics.slip_ratio for clarity at the brake layer.
"""

from __future__ import annotations

import numpy as np
from .kinematics import (
    wheel_body_velocity,
    wheel_frame_velocity,
    slip_ratio as _slip_ratio,
)


def wheel_longitudinal_slip(
    vx: float,
    vy: float,
    r: float,
    omega: float,
    x_i: float,
    y_i: float,
    delta_i: float,
    R: float,
    v_eps: float,
) -> float:
    """κ for one wheel given body state and wheel steer."""
    Vx_b, Vy_b = wheel_body_velocity(vx, vy, r, x_i, y_i)
    Vx_w, _ = wheel_frame_velocity(Vx_b, Vy_b, delta_i)
    return _slip_ratio(Vx_w, omega, R, v_eps)


def all_wheel_slips(
    vx: float,
    vy: float,
    r: float,
    omegas: list | np.ndarray,
    x_w: np.ndarray,
    y_w: np.ndarray,
    deltas: list | np.ndarray,
    R: float,
    v_eps: float,
) -> np.ndarray:
    kappas = np.zeros(4)
    for i in range(4):
        kappas[i] = wheel_longitudinal_slip(
            vx, vy, r, omegas[i], x_w[i], y_w[i], deltas[i], R, v_eps
        )
    return kappas
