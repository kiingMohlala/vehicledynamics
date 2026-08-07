"""Curvature and reference-speed helpers along a path."""
from __future__ import annotations

import numpy as np


def curvature_from_xy(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Discrete curvature κ ≈ |x'y'' - y'x''| / (x'²+y'²)^{3/2}."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(x)
    if n < 3:
        return np.zeros(n)
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    num = np.abs(dx * ddy - dy * ddx)
    den = (dx * dx + dy * dy) ** 1.5
    den = np.maximum(den, 1e-12)
    return num / den


def reference_speed(curvature: np.ndarray, mu: float = 1.0, g: float = 9.81, v_max: float = 80.0) -> np.ndarray:
    """Simple friction-limited speed from curvature: v = sqrt(μ g / κ)."""
    kappa = np.asarray(curvature, dtype=float)
    v = np.full_like(kappa, v_max, dtype=float)
    mask = kappa > 1e-6
    v[mask] = np.minimum(v_max, np.sqrt(mu * g / kappa[mask]))
    return v
