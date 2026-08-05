"""Store and compare regression baselines."""

from __future__ import annotations

from dataclasses import dataclass, asdict, field
from pathlib import Path
import json
import hashlib
import numpy as np

from vehicle_dynamics.simulation.telemetry_recorder import TelemetryRecorder
from vehicle_dynamics.simulation.statistics import SimulationStatistics


@dataclass
class BaselineRecord:
    name: str
    n_samples: int
    duration: float
    max_speed: float
    max_ax: float
    max_ay: float
    peak_rpm: float
    fuel_used_g: float
    distance: float
    final_vx: float
    final_x: float
    final_y: float
    checksum: str = ""
    meta: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "BaselineRecord":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


def _checksum(log: TelemetryRecorder) -> str:
    d = log.to_numpy()
    if not d:
        return "empty"
    parts = []
    for k in sorted(d.keys()):
        arr = np.asarray(d[k], dtype=float).ravel()
        parts.append(f"{k}:{arr.mean():.6g}:{arr.std():.6g}:{arr[-1] if len(arr) else 0:.6g}")
    return hashlib.sha256("|".join(parts).encode()).hexdigest()[:16]


class RegressionDatabase:
    def __init__(self, root: str | Path | None = None):
        self.root = Path(root) if root else Path(__file__).parent / "baselines"
        self.root.mkdir(parents=True, exist_ok=True)

    def path_for(self, name: str) -> Path:
        safe = name.replace(" ", "_").replace("/", "_")
        return self.root / f"{safe}.json"

    def capture(
        self,
        name: str,
        log: TelemetryRecorder,
        stats: SimulationStatistics,
        final_vx: float = 0.0,
        final_x: float = 0.0,
        final_y: float = 0.0,
        meta: dict | None = None,
    ) -> BaselineRecord:
        rec = BaselineRecord(
            name=name,
            n_samples=stats.n_samples,
            duration=stats.duration,
            max_speed=stats.max_speed,
            max_ax=stats.max_ax,
            max_ay=stats.max_ay,
            peak_rpm=stats.peak_rpm,
            fuel_used_g=stats.fuel_used_g,
            distance=stats.distance,
            final_vx=final_vx,
            final_x=final_x,
            final_y=final_y,
            checksum=_checksum(log),
            meta=meta or {},
        )
        path = self.path_for(name)
        path.write_text(json.dumps(rec.to_dict(), indent=2))
        return rec

    def load(self, name: str) -> BaselineRecord | None:
        path = self.path_for(name)
        if not path.exists():
            return None
        return BaselineRecord.from_dict(json.loads(path.read_text()))

    def compare(
        self,
        name: str,
        log: TelemetryRecorder,
        stats: SimulationStatistics,
        final_vx: float = 0.0,
        tol_rel: float = 1e-6,
        tol_abs: float = 1e-6,
    ) -> tuple[bool, dict]:
        ref = self.load(name)
        if ref is None:
            return False, {"error": "no baseline", "name": name}
        cur_cs = _checksum(log)
        checks = {
            "n_samples": abs(stats.n_samples - ref.n_samples) == 0,
            "max_speed": abs(stats.max_speed - ref.max_speed) <= tol_abs + tol_rel * abs(ref.max_speed),
            "duration": abs(stats.duration - ref.duration) <= tol_abs + tol_rel * abs(ref.duration),
            "checksum": cur_cs == ref.checksum,
            "final_vx": abs(final_vx - ref.final_vx) <= tol_abs + tol_rel * max(abs(ref.final_vx), 1.0),
        }
        ok = all(checks.values())
        return ok, {
            "checks": checks,
            "ref_checksum": ref.checksum,
            "cur_checksum": cur_cs,
            "ref_max_speed": ref.max_speed,
            "cur_max_speed": stats.max_speed,
        }

    def list_baselines(self) -> list[str]:
        return sorted(p.stem for p in self.root.glob("*.json"))
