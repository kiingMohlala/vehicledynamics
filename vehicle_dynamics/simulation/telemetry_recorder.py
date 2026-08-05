"""High-rate simulation telemetry."""

from __future__ import annotations

from dataclasses import dataclass, fields, asdict
from pathlib import Path
import csv
import numpy as np


@dataclass
class SimSample:
    time: float = 0.0
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    yaw_rate: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    engine_rpm: float = 0.0
    engine_torque: float = 0.0
    gear: int = 0
    torque_L: float = 0.0
    torque_R: float = 0.0
    downforce: float = 0.0
    drag: float = 0.0
    fuel_g: float = 0.0
    mu_scale: float = 1.0
    slip_max: float = 0.0


class TelemetryRecorder:
    def __init__(self) -> None:
        self.samples: list[SimSample] = []

    def clear(self) -> None:
        self.samples.clear()

    def record(self, s: SimSample) -> None:
        self.samples.append(s)

    def to_numpy(self) -> dict[str, np.ndarray]:
        if not self.samples:
            return {}
        names = [f.name for f in fields(SimSample)]
        return {n: np.array([getattr(s, n) for s in self.samples]) for n in names}

    def export_csv(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        names = [f.name for f in fields(SimSample)]
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=names)
            w.writeheader()
            for s in self.samples:
                w.writerow(asdict(s))
        return path
