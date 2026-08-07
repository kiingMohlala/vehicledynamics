"""Time alignment between measured and simulated signals."""
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike


def lag_by_correlation(a: ArrayLike, b: ArrayLike, max_lag: int = 50) -> int:
    """Return lag (samples) to shift b to align with a (positive => b is late)."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    n = min(len(a), len(b))
    a, b = a[:n] - np.mean(a[:n]), b[:n] - np.mean(b[:n])
    max_lag = min(max_lag, n - 1)
    best_lag, best_corr = 0, -1e18
    for lag in range(-max_lag, max_lag + 1):
        if lag < 0:
            c = np.corrcoef(a[-lag:], b[: n + lag])[0, 1]
        elif lag > 0:
            c = np.corrcoef(a[: n - lag], b[lag:])[0, 1]
        else:
            c = np.corrcoef(a, b)[0, 1]
        if np.isfinite(c) and c > best_corr:
            best_corr, best_lag = c, lag
    return int(best_lag)


def apply_lag(y: ArrayLike, lag: int) -> np.ndarray:
    y = np.asarray(y, dtype=float)
    if lag == 0:
        return y.copy()
    out = np.empty_like(y)
    if lag > 0:
        out[lag:] = y[:-lag]
        out[:lag] = y[0]
    else:
        out[:lag] = y[-lag:]
        out[lag:] = y[-1]
    return out


def align_signals(
    t_meas: np.ndarray,
    y_meas: np.ndarray,
    t_sim: np.ndarray,
    y_sim: np.ndarray,
    max_lag: int = 50,
) -> dict:
    # resample sim onto meas time
    y_sim_i = np.interp(t_meas, t_sim, y_sim, left=y_sim[0], right=y_sim[-1])
    lag = lag_by_correlation(y_meas, y_sim_i, max_lag=max_lag)
    y_sim_a = apply_lag(y_sim_i, lag)
    return {
        "t": t_meas,
        "measured": y_meas,
        "simulated": y_sim_a,
        "lag_samples": lag,
        "lag_seconds": float(lag * (t_meas[1] - t_meas[0])) if len(t_meas) > 1 else 0.0,
    }
