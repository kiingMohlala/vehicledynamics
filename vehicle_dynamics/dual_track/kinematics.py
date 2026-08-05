"""Per-wheel velocities and slip angles for the dual-track model."""

import numpy as np

# Wheel index order: FL, FR, RL, RR
FL, FR, RL, RR = 0, 1, 2, 3

def wheel_positions(a, b, track_f, track_r):
    """Return x, y arrays for FL, FR, RL, RR (y positive left)."""
    x = np.array([a, a, -b, -b], dtype=float)
    y = np.array([track_f / 2, -track_f / 2, track_r / 2, -track_r / 2], dtype=float)
    return x, y

def wheel_body_velocity(vx, vy, r, x_i, y_i):
    """Velocity of wheel centre in body frame."""
    return vx - r * y_i, vy + r * x_i

def wheel_frame_velocity(Vx_b, Vy_b, delta):
    """Rotate body-frame wheel velocity into wheel frame (delta=0 for rear)."""
    c, s = np.cos(delta), np.sin(delta)
    Vx_w = c * Vx_b + s * Vy_b
    Vy_w = -s * Vx_b + c * Vy_b
    return Vx_w, Vy_w

def slip_ratio(Vx_w, omega, R, v_eps):
    Vx_safe = max(abs(Vx_w), v_eps)
    return (Vx_safe - omega * R) / Vx_safe

def slip_angle(Vx_w, Vy_w, v_eps):
    """
    Slip angle in the same convention as Phase 4 bicycle:
    positive alpha produces positive Fy (to the left).

    At rest with positive steer delta, wheel-frame Vy_w is negative
    (see wheel_frame_velocity), so we use:
        alpha = -atan2(Vy_w, |Vx_w|)
    which yields alpha ≈ +delta at zero sideslip.
    """
    return -np.arctan2(Vy_w, max(abs(Vx_w), v_eps))

def body_forces_from_wheel(Fx_w, Fy_w, delta):
    """Transform wheel-frame forces to body frame."""
    c, s = np.cos(delta), np.sin(delta)
    Fx_b = c * Fx_w - s * Fy_w
    Fy_b = s * Fx_w + c * Fy_w
    return Fx_b, Fy_b
