"""Summary performance metrics."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .telemetry_recorder import TelemetryRecorder


@dataclass
class SimulationStatistics:
    duration: float = 0.0
    distance: float = 0.0
    max_speed: float = 0.0
    max_ax: float = 0.0
    max_ay: float = 0.0
    time_0_100: float | None = None   # s (if achieved)
    time_100_0: float | None = None
    fuel_used_g: float = 0.0
    avg_downforce: float = 0.0
    avg_drag: float = 0.0
    peak_rpm: float = 0.0
    n_samples: int = 0


def compute_statistics(log: TelemetryRecorder) -> SimulationStatistics:
    if not log.samples:
        return SimulationStatistics()
    d = log.to_numpy()
    t = d["time"]
    vx = d["vx"]
    ax = d["ax"]
    ay = d["ay"]
    # Distance via trapezoid on speed
    if len(t) > 1:
        dist = float(np.trapezoid(np.abs(vx), t))
    else:
        dist = 0.0
    stats = SimulationStatistics(
        duration=float(t[-1] - t[0]) if len(t) else 0.0,
        distance=dist,
        max_speed=float(np.max(np.abs(vx))),
        max_ax=float(np.max(np.abs(ax))),
        max_ay=float(np.max(np.abs(ay))),
        fuel_used_g=float(d["fuel_g"][-1] - d["fuel_g"][0]),
        avg_downforce=float(np.mean(d["downforce"])),
        avg_drag=float(np.mean(d["drag"])),
        peak_rpm=float(np.max(d["engine_rpm"])),
        n_samples=len(log.samples),
    )
    # 0–100 km/h ≈ 0–27.78 m/s
    v100 = 100.0 / 3.6
    above = np.where(vx >= v100)[0]
    if len(above):
        stats.time_0_100 = float(t[above[0]] - t[0])
    # 100–0 if starts above and ends near 0
    if vx[0] >= v100 * 0.9:
        below = np.where(vx <= 1.0)[0]
        if len(below):
            stats.time_100_0 = float(t[below[0]] - t[0])
    return stats
