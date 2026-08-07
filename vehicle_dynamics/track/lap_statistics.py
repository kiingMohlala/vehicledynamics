"""Lap and session performance metrics."""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class LapMetrics:
    lap_time: float
    average_speed: float
    top_speed: float
    min_corner_speed: float
    max_ax: float
    max_ay: float
    distance: float
    fuel_used: float = 0.0
    battery_used: float = 0.0
    peak_downforce: float = 0.0
    peak_drag: float = 0.0
    peak_tire_load: float = 0.0
    sector_dt: list[float] = field(default_factory=list)


@dataclass
class SessionStatistics:
    n_laps: int
    best_lap: float
    best_lap_index: int
    average_lap: float
    total_distance: float
    total_time: float
    lap_metrics: list[LapMetrics] = field(default_factory=list)
    best_sectors: list[float] = field(default_factory=list)

    def summary(self) -> str:
        lines = [
            f"Laps completed : {self.n_laps}",
            f"Best lap       : {self.best_lap:.3f} s (#{self.best_lap_index + 1})",
            f"Average lap    : {self.average_lap:.3f} s",
            f"Total distance : {self.total_distance:.1f} m",
            f"Total time     : {self.total_time:.2f} s",
        ]
        if self.best_sectors:
            lines.append("Best sectors   : " + ", ".join(f"{t:.3f}s" for t in self.best_sectors))
        return "\n".join(lines)


def compute_lap_metrics(
    t: np.ndarray,
    s: np.ndarray,
    vx: np.ndarray,
    ax: np.ndarray | None = None,
    ay: np.ndarray | None = None,
    fuel: np.ndarray | None = None,
    soc: np.ndarray | None = None,
    downforce: np.ndarray | None = None,
    drag: np.ndarray | None = None,
    tire_load: np.ndarray | None = None,
    sector_dt: list[float] | None = None,
) -> LapMetrics:
    t = np.asarray(t, dtype=float)
    s = np.asarray(s, dtype=float)
    vx = np.asarray(vx, dtype=float)
    lap_time = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    dist = float(s[-1] - s[0]) if len(s) > 1 else 0.0
    avg = dist / lap_time if lap_time > 1e-9 else 0.0
    return LapMetrics(
        lap_time=lap_time,
        average_speed=avg,
        top_speed=float(np.max(vx)) if len(vx) else 0.0,
        min_corner_speed=float(np.min(vx)) if len(vx) else 0.0,
        max_ax=float(np.max(np.abs(ax))) if ax is not None and len(ax) else 0.0,
        max_ay=float(np.max(np.abs(ay))) if ay is not None and len(ay) else 0.0,
        distance=dist,
        fuel_used=float(fuel[-1] - fuel[0]) if fuel is not None and len(fuel) > 1 else 0.0,
        battery_used=float(soc[0] - soc[-1]) if soc is not None and len(soc) > 1 else 0.0,
        peak_downforce=float(np.max(downforce)) if downforce is not None and len(downforce) else 0.0,
        peak_drag=float(np.max(drag)) if drag is not None and len(drag) else 0.0,
        peak_tire_load=float(np.max(tire_load)) if tire_load is not None and len(tire_load) else 0.0,
        sector_dt=list(sector_dt or []),
    )
