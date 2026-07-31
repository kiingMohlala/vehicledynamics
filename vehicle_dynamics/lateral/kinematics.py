"""Slip-angle and path kinematics for the bicycle model."""

import numpy as np
from .parameters import BicycleParameters

def front_slip_angle(vy: float, r: float, vx: float, delta: float, params: BicycleParameters) -> float:
    """
    α_f = δ - atan2(vy + a·r, max(|Vx|, v_eps))
    """
    vx_safe = max(abs(vx), params.v_eps)
    return delta - np.arctan2(vy + params.a * r, vx_safe)

def rear_slip_angle(vy: float, r: float, vx: float, params: BicycleParameters) -> float:
    """
    α_r = -atan2(vy - b·r, max(|Vx|, v_eps))
    """
    vx_safe = max(abs(vx), params.v_eps)
    return -np.arctan2(vy - params.b * r, vx_safe)

def inertial_rates(vx: float, vy: float, psi: float):
    """Body to inertial velocity components."""
    X_dot = vx * np.cos(psi) - vy * np.sin(psi)
    Y_dot = vx * np.sin(psi) + vy * np.cos(psi)
    return X_dot, Y_dot
