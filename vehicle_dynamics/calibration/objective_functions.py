"""Calibration cost functions comparing sim vs measured."""
from __future__ import annotations

from typing import Callable
import numpy as np

from .validation_metrics import rmse, mae


def signal_cost(
    measured: np.ndarray,
    simulated: np.ndarray,
    metric: str = "rmse",
) -> float:
    if metric == "mae":
        return mae(measured, simulated)
    return rmse(measured, simulated)


def weighted_multi_signal_cost(
    pairs: list[tuple[np.ndarray, np.ndarray, float]],
    metric: str = "rmse",
) -> float:
    total_w = 0.0
    acc = 0.0
    for meas, sim, w in pairs:
        acc += w * signal_cost(meas, sim, metric=metric)
        total_w += w
    return acc / total_w if total_w > 0 else 0.0


def make_coastdown_cost(
    t: np.ndarray,
    vx_meas: np.ndarray,
    model_fn: Callable[[dict[str, float]], np.ndarray],
) -> Callable[[dict[str, float]], float]:
    """model_fn(params) -> vx_sim on same time grid."""
    def cost(params: dict[str, float]) -> float:
        vx_sim = model_fn(params)
        n = min(len(vx_meas), len(vx_sim))
        return rmse(vx_meas[:n], vx_sim[:n])
    return cost
