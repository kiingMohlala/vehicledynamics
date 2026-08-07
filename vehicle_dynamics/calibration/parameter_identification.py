"""
Simple physics models used for parameter identification demos.

Coastdown / aero-drag model:
  m * ax = -0.5 * rho * Cd * A * v^2 - m * g * rr
"""
from __future__ import annotations

from typing import Callable
import numpy as np

from .parameter_sets import ParameterSet, CalibParameter


def coastdown_vx(
    t: np.ndarray,
    v0: float,
    mass: float,
    Cd: float,
    area: float = 2.2,
    rr: float = 0.015,
    rho: float = 1.225,
    g: float = 9.81,
) -> np.ndarray:
    """Integrate longitudinal coastdown ODE."""
    v = float(v0)
    out = np.zeros_like(t, dtype=float)
    out[0] = v
    for i in range(1, len(t)):
        dt = float(t[i] - t[i - 1])
        drag = 0.5 * rho * Cd * area * v * v
        ax = -(drag / max(mass, 1.0)) - g * rr
        v = max(0.0, v + ax * dt)
        out[i] = v
    return out


def make_coastdown_model(t: np.ndarray, v0: float, area: float = 2.2) -> Callable[[dict[str, float]], np.ndarray]:
    def model(params: dict[str, float]) -> np.ndarray:
        return coastdown_vx(
            t, v0,
            mass=float(params.get("mass", 1400)),
            Cd=float(params.get("Cd", 0.34)),
            area=area,
            rr=float(params.get("rolling_resistance", 0.015)),
        )
    return model


def tire_force_curve(slip: np.ndarray, mu: float, Cx: float, Fz: float = 4000.0) -> np.ndarray:
    """Simple linear + saturation tire curve for identification demos."""
    Fx = Cx * slip
    lim = mu * Fz
    return np.clip(Fx, -lim, lim)


def suspension_step_response(
    t: np.ndarray,
    k: float,
    c: float,
    m: float = 300.0,
    z0: float = 0.05,
) -> np.ndarray:
    """Underdamped/overdamped free response of 1-DOF mass-spring-damper."""
    wn = np.sqrt(k / m)
    zeta = c / (2 * np.sqrt(k * m))
    if zeta < 1:
        wd = wn * np.sqrt(1 - zeta ** 2)
        return z0 * np.exp(-zeta * wn * t) * (np.cos(wd * t) + (zeta * wn / wd) * np.sin(wd * t))
    else:
        return z0 * np.exp(-wn * t)
