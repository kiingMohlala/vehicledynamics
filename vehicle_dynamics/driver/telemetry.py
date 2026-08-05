"""Telemetry logging and CSV export."""

from __future__ import annotations

from dataclasses import dataclass, fields, asdict
from pathlib import Path
import csv
import numpy as np


@dataclass
class TelemetrySample:
    time: float = 0.0
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    vx: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    yaw_rate: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    cross_track: float = 0.0
    heading_error: float = 0.0
    speed_error: float = 0.0
    s_path: float = 0.0
    target_speed: float = 0.0


class TelemetryLogger:
    def __init__(self) -> None:
        self.samples: list[TelemetrySample] = []

    def clear(self) -> None:
        self.samples.clear()

    def log(self, sample: TelemetrySample) -> None:
        self.samples.append(sample)

    def to_arrays(self) -> dict[str, np.ndarray]:
        if not self.samples:
            return {}
        names = [f.name for f in fields(TelemetrySample)]
        data = {n: np.array([getattr(s, n) for s in self.samples]) for n in names}
        return data

    def export_csv(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        names = [f.name for f in fields(TelemetrySample)]
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=names)
            w.writeheader()
            for s in self.samples:
                w.writerow(asdict(s))
        return path

    @property
    def rms_cross_track(self) -> float:
        if not self.samples:
            return 0.0
        e = np.array([s.cross_track for s in self.samples])
        return float(np.sqrt(np.mean(e * e)))
