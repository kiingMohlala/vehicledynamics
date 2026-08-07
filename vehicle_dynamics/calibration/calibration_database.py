"""Store calibration runs, parameters, and metrics."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any
import json
import time


@dataclass
class CalibrationRecord:
    record_id: str
    parameters: dict[str, float]
    metrics: dict[str, float]
    method: str = ""
    nfev: int = 0
    timestamp: float = field(default_factory=time.time)
    meta: dict[str, Any] = field(default_factory=dict)


@dataclass
class CalibrationDatabase:
    records: list[CalibrationRecord] = field(default_factory=list)
    path: Path | None = None

    def add(
        self,
        parameters: dict[str, float],
        metrics: dict[str, float],
        method: str = "",
        nfev: int = 0,
        meta: dict | None = None,
    ) -> CalibrationRecord:
        rid = f"cal_{len(self.records):04d}"
        rec = CalibrationRecord(
            record_id=rid,
            parameters=dict(parameters),
            metrics=dict(metrics),
            method=method,
            nfev=nfev,
            meta=meta or {},
        )
        self.records.append(rec)
        return rec

    def save(self, path: str | Path | None = None) -> Path:
        path = Path(path or self.path or "calibration_db.json")
        path.write_text(json.dumps([asdict(r) for r in self.records], indent=2, default=float))
        self.path = path
        return path

    def load(self, path: str | Path) -> "CalibrationDatabase":
        path = Path(path)
        raw = json.loads(path.read_text())
        self.records = [CalibrationRecord(**r) for r in raw]
        self.path = path
        return self

    def __len__(self) -> int:
        return len(self.records)
