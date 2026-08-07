"""Vehicle design revisions with parameter deltas and notes."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import time
import copy


@dataclass
class VehicleRevision:
    revision_id: str
    label: str
    parameters: dict[str, Any]
    notes: str = ""
    parent_id: str | None = None
    timestamp: float = field(default_factory=time.time)
    author: str = ""

    def delta_from(self, other: "VehicleRevision") -> dict[str, tuple[Any, Any]]:
        keys = set(self.parameters) | set(other.parameters)
        out = {}
        for k in keys:
            a, b = other.parameters.get(k), self.parameters.get(k)
            if a != b:
                out[k] = (a, b)
        return out


@dataclass
class RevisionHistory:
    revisions: list[VehicleRevision] = field(default_factory=list)

    def add(self, rev: VehicleRevision) -> None:
        self.revisions.append(rev)

    def latest(self) -> VehicleRevision | None:
        return self.revisions[-1] if self.revisions else None

    def get(self, revision_id: str) -> VehicleRevision:
        for r in self.revisions:
            if r.revision_id == revision_id:
                return r
        raise KeyError(revision_id)

    def __len__(self) -> int:
        return len(self.revisions)
