"""Error and fit quality metrics."""
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike


def rmse(a: ArrayLike, b: ArrayLike) -> float:
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    n = min(len(a), len(b))
    if n == 0:
        return 0.0
    return float(np.sqrt(np.mean((a[:n] - b[:n]) ** 2)))


def mae(a: ArrayLike, b: ArrayLike) -> float:
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    n = min(len(a), len(b))
    if n == 0:
        return 0.0
    return float(np.mean(np.abs(a[:n] - b[:n])))


def peak_error(a: ArrayLike, b: ArrayLike) -> float:
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    n = min(len(a), len(b))
    if n == 0:
        return 0.0
    return float(np.max(np.abs(a[:n] - b[:n])))


def correlation(a: ArrayLike, b: ArrayLike) -> float:
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    n = min(len(a), len(b))
    if n < 2:
        return 0.0
    c = np.corrcoef(a[:n], b[:n])[0, 1]
    return float(c) if np.isfinite(c) else 0.0


def r2_score(a: ArrayLike, b: ArrayLike) -> float:
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    n = min(len(a), len(b))
    if n < 2:
        return 0.0
    ss_res = np.sum((a[:n] - b[:n]) ** 2)
    ss_tot = np.sum((a[:n] - np.mean(a[:n])) ** 2)
    if ss_tot < 1e-15:
        return 1.0
    return float(1.0 - ss_res / ss_tot)


def phase_lag_seconds(a: ArrayLike, b: ArrayLike, dt: float, max_lag: int = 50) -> float:
    from .synchronization import lag_by_correlation
    lag = lag_by_correlation(a, b, max_lag=max_lag)
    return float(lag * dt)


def summary_metrics(measured: ArrayLike, simulated: ArrayLike, dt: float = 0.01) -> dict[str, float]:
    return {
        "rmse": rmse(measured, simulated),
        "mae": mae(measured, simulated),
        "peak_error": peak_error(measured, simulated),
        "correlation": correlation(measured, simulated),
        "r2": r2_score(measured, simulated),
        "phase_lag_s": phase_lag_seconds(measured, simulated, dt),
    }
