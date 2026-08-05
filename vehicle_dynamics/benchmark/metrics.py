"""Extract benchmark metrics from DualTrackResult."""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, asdict


@dataclass
class ScenarioMetrics:
    name: str
    passed: bool
    final_vx: float
    peak_ay: float          # approx vx * r
    peak_yaw_rate: float
    rms_yaw: float
    max_utilization: float
    stopping_distance: float | None
    finite: bool
    notes: str = ""


def compute_metrics(name: str, res) -> ScenarioMetrics:
    finite = bool(
        np.all(np.isfinite(res.vx))
        and np.all(np.isfinite(res.r))
        and np.all(np.isfinite(res.utilization))
    )
    peak_r = float(np.max(np.abs(res.r))) if len(res.r) else 0.0
    peak_ay = float(np.max(np.abs(res.vx * res.r))) if len(res.vx) else 0.0
    rms_r = float(np.sqrt(np.mean(res.r ** 2))) if len(res.r) else 0.0
    max_u = float(np.max(res.utilization)) if len(res.utilization) else 0.0
    final_vx = float(res.vx[-1]) if len(res.vx) else 0.0

    # Stopping distance if vehicle nearly stops
    stop_dist = None
    if final_vx < 1.0 and hasattr(res, "X") and len(res.X) > 1:
        stop_dist = float(np.hypot(res.X[-1] - res.X[0], res.Y[-1] - res.Y[0]))

    passed = (
        finite
        and max_u <= 1.05
        and peak_r < 5.0          # soft physical bound
        and peak_ay < 20.0
    )

    return ScenarioMetrics(
        name=name,
        passed=passed,
        final_vx=final_vx,
        peak_ay=peak_ay,
        peak_yaw_rate=peak_r,
        rms_yaw=rms_r,
        max_utilization=max_u,
        stopping_distance=stop_dist,
        finite=finite,
    )


def metrics_to_dict(m: ScenarioMetrics) -> dict:
    return asdict(m)
