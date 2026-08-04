"""Transient / stability metrics."""

from __future__ import annotations

import numpy as np
from .metrics import StabilityMetrics, DriverMetrics, sideslip_beta, rms


def extract_stability(
    time: np.ndarray,
    vx: np.ndarray,
    vy: np.ndarray,
    r: np.ndarray,
    load_transfer: np.ndarray | None = None,
    jacking: np.ndarray | None = None,
    rc_migration: np.ndarray | None = None,
) -> StabilityMetrics:
    beta = sideslip_beta(vx, vy)
    return StabilityMetrics(
        peak_yaw_rate=float(np.max(np.abs(r))),
        rms_yaw_rate=rms(r),
        peak_beta_deg=float(np.degrees(np.max(np.abs(beta)))),
        rms_beta_deg=float(np.degrees(rms(beta))),
        peak_load_transfer=(
            float(np.max(np.abs(load_transfer))) if load_transfer is not None else None
        ),
        peak_jacking=(
            float(np.max(np.abs(jacking))) if jacking is not None else None
        ),
        peak_rc_migration=(
            float(np.max(np.abs(rc_migration))) if rc_migration is not None else None
        ),
    )


def extract_driver(
    time: np.ndarray,
    vx: np.ndarray,
    delta: np.ndarray,
    X: np.ndarray | None = None,
    Y: np.ndarray | None = None,
) -> DriverMetrics:
    t = np.asarray(time, dtype=float)
    stop_dist = None
    stop_100 = None
    if vx[0] > 1.0 and vx[-1] < 0.5:
        # approximate stopping distance along path
        if X is not None and Y is not None:
            dX = np.diff(X)
            dY = np.diff(Y)
            stop_dist = float(np.sum(np.hypot(dX, dY)))
        else:
            stop_dist = float(np.trapz(vx, t))
        # scale 100–0 km/h equivalent from actual entry speed
        v0 = float(vx[0])
        if v0 > 1.0 and stop_dist is not None:
            # s ∝ v² → scale to 100 km/h = 27.78 m/s
            stop_100 = stop_dist * (27.778 / v0) ** 2

    return DriverMetrics(
        max_steer_rad=float(np.max(np.abs(delta))),
        max_steer_deg=float(np.degrees(np.max(np.abs(delta)))),
        entry_speed=float(vx[0]),
        exit_speed=float(vx[-1]),
        average_speed=float(np.mean(vx)),
        corner_time=float(t[-1] - t[0]) if t.size > 1 else 0.0,
        stopping_distance=stop_dist,
        stop_100_0_kmh=stop_100,
    )
