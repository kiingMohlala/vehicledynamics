"""Steady-state handling extraction from time histories."""

from __future__ import annotations

import numpy as np
from .metrics import SteadyStateMetrics, understeer_gradient, G


def extract_steady_state(
    time: np.ndarray,
    vx: np.ndarray,
    vy: np.ndarray,
    r: np.ndarray,
    delta: np.ndarray,
    wheelbase: float,
    window: float = 1.0,
) -> SteadyStateMetrics:
    """
    Use the last `window` seconds (or last 20% of samples) as steady state.
    """
    t = np.asarray(time, dtype=float)
    if t.size < 2:
        return SteadyStateMetrics(
            0, 0, 0, 0, 0, float("inf"), None, None, 0, 0, 0, 0
        )

    t1 = t[-1]
    mask = t >= (t1 - window)
    if np.count_nonzero(mask) < 3:
        n = max(3, t.size // 5)
        mask = np.zeros(t.size, dtype=bool)
        mask[-n:] = True

    vx_ss = float(np.mean(vx[mask]))
    r_ss = float(np.mean(r[mask]))
    delta_ss = float(np.mean(delta[mask]))
    # ay ≈ vx * r in steady circular motion
    ay_ss = float(np.mean(vx[mask] * r[mask]))
    max_ay = float(np.max(np.abs(vx * r)))

    R = abs(vx_ss / r_ss) if abs(r_ss) > 1e-4 else float("inf")
    K = understeer_gradient(delta_ss, wheelbase, R if np.isfinite(R) else 1e6, ay_ss)

    yaw_gain = abs(r_ss / ay_ss) if abs(ay_ss) > 0.5 else 0.0
    steer_gain = abs(ay_ss / delta_ss) if abs(delta_ss) > 1e-4 else 0.0

    # Characteristic / critical speed from linear bicycle approx:
    # V_char = sqrt(L g / K_rad) for understeer; V_crit for oversteer
    K_rad = np.radians(K)
    char_speed = None
    crit_speed = None
    if K > 0.05 and K_rad > 0:
        char_speed = float(np.sqrt(wheelbase * G / K_rad))
    elif K < -0.05 and K_rad < 0:
        crit_speed = float(np.sqrt(wheelbase * G / abs(K_rad)))

    return SteadyStateMetrics(
        understeer_gradient_deg_per_g=K,
        yaw_rate_gain=yaw_gain,
        steering_gain=steer_gain,
        max_ay=max_ay,
        max_ay_g=max_ay / G,
        turning_radius=float(R) if np.isfinite(R) else float("inf"),
        characteristic_speed=char_speed,
        critical_speed=crit_speed,
        yaw_rate_ss=r_ss,
        ay_ss=ay_ss,
        delta_ss=delta_ss,
        vx_ss=vx_ss,
    )
