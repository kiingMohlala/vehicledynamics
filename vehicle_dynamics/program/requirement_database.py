"""Store and query requirements."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import json

from .requirements import Requirement


@dataclass
class RequirementDatabase:
    requirements: list[Requirement] = field(default_factory=list)

    def add(self, req: Requirement) -> None:
        self.requirements = [r for r in self.requirements if r.req_id != req.req_id]
        self.requirements.append(req)

    def get(self, req_id: str) -> Requirement:
        for r in self.requirements:
            if r.req_id == req_id:
                return r
        raise KeyError(req_id)

    def by_category(self, category: str) -> list[Requirement]:
        return [r for r in self.requirements if r.category == category]

    def active(self) -> list[Requirement]:
        return [r for r in self.requirements if r.active]

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.write_text(json.dumps([r.to_dict() for r in self.requirements], indent=2))
        return path

    def load(self, path: str | Path) -> "RequirementDatabase":
        data = json.loads(Path(path).read_text())
        self.requirements = [Requirement.from_dict(d) for d in data]
        return self

    def __len__(self) -> int:
        return len(self.requirements)
