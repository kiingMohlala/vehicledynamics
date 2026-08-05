"""Relative wind vector and dynamic pressure."""

from __future__ import annotations

import numpy as np


def relative_velocity(
    v_vehicle: np.ndarray,
    v_wind: np.ndarray,
) -> np.ndarray:
    """V_rel = V_vehicle - V_wind (vehicle frame inertial components)."""
    return np.asarray(v_vehicle, dtype=float) - np.asarray(v_wind, dtype=float)


def dynamic_pressure_rel(rho: float, v_rel: np.ndarray) -> float:
    speed = float(np.linalg.norm(v_rel))
    return 0.5 * rho * speed * speed


def air_speed_and_sideslip(v_rel: np.ndarray) -> tuple[float, float]:
    """
    Air-relative speed and aerodynamic sideslip.
    v_rel = [Vx, Vy, Vz] in vehicle-aligned inertial axes (x forward, y left).
    """
    vx, vy = float(v_rel[0]), float(v_rel[1])
    speed = float(np.hypot(vx, vy))
    beta = float(np.arctan2(vy, max(abs(vx), 1e-6)))
    return speed, beta
