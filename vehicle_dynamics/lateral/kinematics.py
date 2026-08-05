
import numpy as np
from .parameters import BicycleParameters

def front_slip_angle(vy, r, vx, delta, params):
    vx_safe = max(abs(vx), params.v_eps)
    return delta - np.arctan2(vy + params.a * r, vx_safe)

def rear_slip_angle(vy, r, vx, params):
    vx_safe = max(abs(vx), params.v_eps)
    return -np.arctan2(vy - params.b * r, vx_safe)

def inertial_rates(vx, vy, psi):
    X_dot = vx * np.cos(psi) - vy * np.sin(psi)
    Y_dot = vx * np.sin(psi) + vy * np.cos(psi)
    return X_dot, Y_dot
