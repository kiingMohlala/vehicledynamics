"""Link evidence artifacts to requirements and simulations."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import json
import time


@dataclass
class Evidence:
    evidence_id: str
    kind: str                      # telemetry | report | plot | calibration | optimization | validation
    path: str = ""
    req_ids: list[str] = field(default_factory=list)
    simulation_id: str = ""
    meta: dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class EvidenceDatabase:
    items: list[Evidence] = field(default_factory=list)

    def add(self, evidence: Evidence) -> None:
        self.items.append(evidence)

    def for_requirement(self, req_id: str) -> list[Evidence]:
        return [e for e in self.items if req_id in e.req_ids]

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.write_text(json.dumps([e.__dict__ for e in self.items], indent=2, default=float))
        return path

    def load(self, path: str | Path) -> "EvidenceDatabase":
        raw = json.loads(Path(path).read_text())
        self.items = [Evidence(**d) for d in raw]
        return self

    def __len__(self) -> int:
        return len(self.items)
