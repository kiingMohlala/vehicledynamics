"""Store and reload DOE experiment records."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any
import json
import time


@dataclass
class ExperimentRecord:
    experiment_id: str
    design: dict[str, float]
    output: dict[str, Any]
    feasible: bool = True
    timestamp: float = field(default_factory=time.time)
    meta: dict[str, Any] = field(default_factory=dict)


@dataclass
class ExperimentDatabase:
    path: Path | None = None
    records: list[ExperimentRecord] = field(default_factory=list)

    def add(self, design: dict[str, float], output: dict[str, Any], feasible: bool = True, meta: dict | None = None) -> ExperimentRecord:
        rid = f"exp_{len(self.records):05d}"
        rec = ExperimentRecord(
            experiment_id=rid,
            design=dict(design),
            output=dict(output),
            feasible=feasible,
            meta=meta or {},
        )
        self.records.append(rec)
        return rec

    def add_batch(self, designs: list[dict], outputs: list[dict], feasible: list[bool] | None = None) -> None:
        feasible = feasible or [True] * len(designs)
        for d, o, f in zip(designs, outputs, feasible):
            self.add(d, o, f)

    def save(self, path: str | Path | None = None) -> Path:
        path = Path(path or self.path or "experiment_db.json")
        data = [asdict(r) for r in self.records]
        path.write_text(json.dumps(data, indent=2, default=float))
        self.path = path
        return path

    def load(self, path: str | Path) -> "ExperimentDatabase":
        path = Path(path)
        raw = json.loads(path.read_text())
        self.records = [ExperimentRecord(**r) for r in raw]
        self.path = path
        return self

    def __len__(self) -> int:
        return len(self.records)
