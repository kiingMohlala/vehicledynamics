"""Load measured telemetry from CSV / generic logger formats."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import numpy as np


@dataclass
class TelemetryData:
    time: np.ndarray
    channels: dict[str, np.ndarray] = field(default_factory=dict)
    source: str = ""
    units: dict[str, str] = field(default_factory=dict)
    meta: dict[str, Any] = field(default_factory=dict)

    @property
    def duration(self) -> float:
        return float(self.time[-1] - self.time[0]) if len(self.time) > 1 else 0.0

    @property
    def n(self) -> int:
        return len(self.time)

    def get(self, name: str, default: float = 0.0) -> np.ndarray:
        if name in self.channels:
            return self.channels[name]
        return np.full_like(self.time, default, dtype=float)


def load_telemetry(path: str | Path, time_key: str = "time") -> TelemetryData:
    path = Path(path)
    with open(path) as f:
        header = f.readline().strip().split(",")
    data = np.genfromtxt(path, delimiter=",", skip_header=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    cols = {h.strip(): data[:, i] for i, h in enumerate(header) if i < data.shape[1]}
    # resolve time column
    tkey = time_key
    if tkey not in cols:
        for alt in ("Time", "t", "timestamp", "TIME"):
            if alt in cols:
                tkey = alt
                break
    if tkey not in cols:
        # synthesize time from sample index at 100 Hz
        t = np.arange(data.shape[0], dtype=float) * 0.01
    else:
        t = cols.pop(tkey)
    return TelemetryData(time=np.asarray(t, dtype=float), channels=cols, source=str(path))


def synthesize_telemetry(
    duration: float = 5.0,
    dt: float = 0.01,
    v0: float = 20.0,
    ax: float = -1.5,
    noise: float = 0.05,
    seed: int = 0,
) -> TelemetryData:
    """Synthetic coastdown-like log for calibration tests."""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, duration, dt)
    vx = np.maximum(v0 + ax * t, 0.0) + rng.normal(0, noise, size=t.shape)
    ax_s = np.gradient(vx, t) + rng.normal(0, noise * 0.5, size=t.shape)
    return TelemetryData(
        time=t,
        channels={
            "vx": vx,
            "ax": ax_s,
            "throttle": np.zeros_like(t),
            "brake": np.clip(-ax_s / 10.0, 0, 1),
            "steer": np.zeros_like(t),
            "rpm": 1500 + 50 * vx + rng.normal(0, 5, size=t.shape),
        },
        source="synthetic",
        meta={"v0": v0, "ax": ax},
    )
