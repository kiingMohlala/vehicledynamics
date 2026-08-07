"""Resampling, filtering, unit conversion, missing-data handling."""
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from .telemetry_loader import TelemetryData


def resample(t: np.ndarray, y: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(t) < 2:
        return t.copy(), y.copy()
    t_new = np.arange(t[0], t[-1] + 0.5 * dt, dt)
    return t_new, np.interp(t_new, t, y)


def moving_average(y: ArrayLike, window: int = 5) -> np.ndarray:
    y = np.asarray(y, dtype=float)
    window = max(1, int(window))
    if window == 1 or len(y) < window:
        return y.copy()
    kernel = np.ones(window) / window
    pad = window // 2
    yp = np.pad(y, (pad, window - 1 - pad), mode="edge")
    return np.convolve(yp, kernel, mode="valid")


def butterworth_lowpass_simple(y: ArrayLike, alpha: float = 0.2) -> np.ndarray:
    """First-order low-pass (EWMA) as a robust filter without SciPy dependency."""
    y = np.asarray(y, dtype=float)
    alpha = float(np.clip(alpha, 1e-6, 1.0))
    out = np.empty_like(y)
    out[0] = y[0]
    for i in range(1, len(y)):
        out[i] = alpha * y[i] + (1 - alpha) * out[i - 1]
    return out


def fill_nan(y: ArrayLike) -> np.ndarray:
    y = np.asarray(y, dtype=float).copy()
    nans = ~np.isfinite(y)
    if not np.any(nans):
        return y
    idx = np.arange(len(y))
    y[nans] = np.interp(idx[nans], idx[~nans], y[~nans]) if np.any(~nans) else 0.0
    return y


def estimate_noise_std(y: ArrayLike) -> float:
    y = np.asarray(y, dtype=float)
    if len(y) < 3:
        return 0.0
    # high-frequency residual via difference
    return float(np.std(np.diff(y)) / np.sqrt(2))


def process_telemetry(data: TelemetryData, dt: float = 0.01, filter_alpha: float = 0.25) -> TelemetryData:
    channels = {}
    t_ref = data.time
    for name, y in data.channels.items():
        y = fill_nan(y)
        t2, y2 = resample(t_ref, y, dt)
        y2 = butterworth_lowpass_simple(y2, alpha=filter_alpha)
        channels[name] = y2
        t_out = t2
    if not channels:
        t_out = np.arange(0.0, max(data.duration, dt), dt)
    return TelemetryData(time=t_out, channels=channels, source=data.source + "+processed", meta=dict(data.meta))
