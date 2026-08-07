"""Track engineering changes between revisions / baselines."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import time


@dataclass
class ChangeRecord:
    change_id: str
    description: str
    before: dict[str, Any]
    after: dict[str, Any]
    author: str = ""
    timestamp: float = field(default_factory=time.time)
    related_req: list[str] = field(default_factory=list)


@dataclass
class ChangeLog:
    changes: list[ChangeRecord] = field(default_factory=list)

    def record(
        self,
        change_id: str,
        description: str,
        before: dict[str, Any],
        after: dict[str, Any],
        author: str = "",
        related_req: list[str] | None = None,
    ) -> ChangeRecord:
        c = ChangeRecord(
            change_id=change_id,
            description=description,
            before=dict(before),
            after=dict(after),
            author=author,
            related_req=list(related_req or []),
        )
        self.changes.append(c)
        return c

    def __len__(self) -> int:
        return len(self.changes)
