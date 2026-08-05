"""Inverse-distance and local linear interpolation over aero samples."""

from __future__ import annotations

import numpy as np
from .cfd_map import AeroSample, AeroMapND


# Relative scales for nondimensional state distance
_SCALES = np.array([40.0, 0.05, 0.05, 0.05, 0.1, 0.05, 0.15, 1.0])


def _normalize(x: np.ndarray) -> np.ndarray:
    return x / _SCALES


def interpolate_sample(
    query: AeroSample,
    amap: AeroMapND,
    *,
    k: int = 4,
    power: float = 2.0,
    max_distance: float = 3.0,
) -> tuple[AeroSample, float, bool]:
    """
    IDW interpolation of coefficients.

    Returns (interpolated_sample, min_distance, in_bounds).
    If no samples or too far, returns query coeffs unchanged / zeros with in_bounds=False.
    """
    if len(amap) == 0:
        return query, np.inf, False

    X = amap.state_matrix()
    Y = amap.coeff_matrix()
    q = query.state_vector()
    qn = _normalize(q)
    Xn = _normalize(X)
    d = np.linalg.norm(Xn - qn, axis=1)
    order = np.argsort(d)
    k = min(k, len(order))
    idx = order[:k]
    d_k = d[idx]
    d_min = float(d_k[0])

    bounds = amap.bounds()
    in_bounds = True
    for i, lab in enumerate(amap.axes_labels):
        lo, hi = bounds[lab]
        if q[i] < lo - 1e-9 or q[i] > hi + 1e-9:
            in_bounds = False

    if d_min < 1e-12:
        y = Y[idx[0]]
    else:
        # Protect against far extrapolation
        if d_min > max_distance:
            return query, d_min, False
        w = 1.0 / np.power(np.maximum(d_k, 1e-12), power)
        w /= w.sum()
        y = w @ Y[idx]

    out = AeroSample(
        speed=query.speed,
        h_front=query.h_front,
        h_rear=query.h_rear,
        pitch=query.pitch,
        yaw=query.yaw,
        roll=query.roll,
        wing_angle=query.wing_angle,
        drs=query.drs,
        Cd=float(y[0]),
        Cl_front=float(y[1]),
        Cl_rear=float(y[2]),
        Cy=float(y[3]),
        Cm_pitch=float(y[4]),
        Cn_yaw=float(y[5]),
        x_cop=float(y[6]),
        source="interpolated",
    )
    return out, d_min, in_bounds
