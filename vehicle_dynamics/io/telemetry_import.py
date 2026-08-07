"""Import measured telemetry (CSV / simple MoTeC-style) for sim comparison."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import numpy as np

from .csv_import import load_csv_columns


@dataclass
class TelemetryLog:
    time: np.ndarray
    channels: dict[str, np.ndarray]

    def align_to(self, t_query: np.ndarray) -> dict[str, np.ndarray]:
        out = {}
        for k, v in self.channels.items():
            out[k] = np.interp(t_query, self.time, v, left=v[0], right=v[-1])
        return out


def load_telemetry_csv(path: str | Path, time_key: str = "time") -> TelemetryLog:
    cols = load_csv_columns(path)
    if time_key not in cols:
        # try common aliases
        for alt in ("Time", "t", "timestamp"):
            if alt in cols:
                time_key = alt
                break
    if time_key not in cols:
        raise KeyError(f"No time column in {path}")
    t = cols.pop(time_key)
    return TelemetryLog(time=t, channels=cols)


def compare_traces(sim: np.ndarray, measured: np.ndarray) -> dict[str, float]:
    sim = np.asarray(sim, dtype=float)
    measured = np.asarray(measured, dtype=float)
    n = min(len(sim), len(measured))
    err = sim[:n] - measured[:n]
    return {
        "rmse": float(np.sqrt(np.mean(err ** 2))),
        "max_abs": float(np.max(np.abs(err))),
        "bias": float(np.mean(err)),
        "n": float(n),
    }
