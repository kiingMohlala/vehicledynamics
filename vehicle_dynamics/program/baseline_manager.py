"""Freeze and restore engineering baselines."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import time
import copy


@dataclass
class Baseline:
    baseline_id: str
    label: str
    vehicle_definition: dict[str, Any]
    requirements_snapshot: list[dict[str, Any]] = field(default_factory=list)
    software_version: str = ""
    git_tag: str = ""
    timestamp: float = field(default_factory=time.time)
    meta: dict[str, Any] = field(default_factory=dict)


@dataclass
class BaselineManager:
    baselines: list[Baseline] = field(default_factory=list)

    def freeze(
        self,
        baseline_id: str,
        label: str,
        vehicle_definition: dict[str, Any],
        requirements: list[dict[str, Any]] | None = None,
        software_version: str = "",
        git_tag: str = "",
        meta: dict | None = None,
    ) -> Baseline:
        b = Baseline(
            baseline_id=baseline_id,
            label=label,
            vehicle_definition=copy.deepcopy(vehicle_definition),
            requirements_snapshot=copy.deepcopy(requirements or []),
            software_version=software_version,
            git_tag=git_tag,
            meta=meta or {},
        )
        self.baselines.append(b)
        return b

    def get(self, baseline_id: str) -> Baseline:
        for b in self.baselines:
            if b.baseline_id == baseline_id:
                return b
        raise KeyError(baseline_id)

    def list_ids(self) -> list[str]:
        return [b.baseline_id for b in self.baselines]
