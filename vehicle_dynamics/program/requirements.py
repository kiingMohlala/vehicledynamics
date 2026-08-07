"""Engineering requirement definitions."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable
import operator as op_mod


OPS = {
    "<=": op_mod.le,
    ">=": op_mod.ge,
    "<": op_mod.lt,
    ">": op_mod.gt,
    "==": op_mod.eq,
    "!=": op_mod.ne,
}


@dataclass
class Requirement:
    req_id: str
    target: float
    operator: str = "<="
    metric: str = ""
    unit: str = ""
    description: str = ""
    category: str = "performance"  # performance | safety | comfort | durability | regulatory
    priority: str = "must"         # must | should | may
    active: bool = True
    tags: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.operator not in OPS:
            raise ValueError(f"Unsupported operator: {self.operator}")
        if not self.metric:
            self.metric = self.req_id

    def evaluate(self, metrics: dict[str, Any]) -> "RequirementResult":
        if self.metric not in metrics:
            return RequirementResult(self.req_id, "MISSING", None, self.target, self.operator, "metric not found")
        value = float(metrics[self.metric])
        passed = bool(OPS[self.operator](value, self.target))
        status = "PASS" if passed else "FAIL"
        return RequirementResult(self.req_id, status, value, self.target, self.operator, self.description)

    def to_dict(self) -> dict[str, Any]:
        return {
            "req_id": self.req_id,
            "target": self.target,
            "operator": self.operator,
            "metric": self.metric,
            "unit": self.unit,
            "description": self.description,
            "category": self.category,
            "priority": self.priority,
            "active": self.active,
            "tags": list(self.tags),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Requirement":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class RequirementResult:
    req_id: str
    status: str
    value: float | None
    target: float
    operator: str
    note: str = ""

    @property
    def passed(self) -> bool:
        return self.status == "PASS"
