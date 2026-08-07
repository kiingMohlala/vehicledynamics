"""Evaluate a set of requirements against simulation metrics."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .requirements import Requirement, RequirementResult


@dataclass
class CheckReport:
    results: list[RequirementResult] = field(default_factory=list)

    @property
    def n_pass(self) -> int:
        return sum(1 for r in self.results if r.status == "PASS")

    @property
    def n_fail(self) -> int:
        return sum(1 for r in self.results if r.status == "FAIL")

    @property
    def n_missing(self) -> int:
        return sum(1 for r in self.results if r.status == "MISSING")

    @property
    def all_passed(self) -> bool:
        return self.n_fail == 0 and self.n_missing == 0 and len(self.results) > 0

    def as_table(self) -> str:
        lines = [f"{'Requirement':28s} {'Status':8s} {'Value':>12s} {'Target':>12s}"]
        lines.append("-" * 64)
        for r in self.results:
            val = f"{r.value:.4g}" if r.value is not None else "—"
            tgt = f"{r.operator} {r.target:.4g}"
            lines.append(f"{r.req_id:28s} {r.status:8s} {val:>12s} {tgt:>12s}")
        lines.append("-" * 64)
        lines.append(f"PASS {self.n_pass}  FAIL {self.n_fail}  MISSING {self.n_missing}")
        return "\n".join(lines)


def check_requirements(requirements: list[Requirement], metrics: dict[str, Any]) -> CheckReport:
    return CheckReport(results=[r.evaluate(metrics) for r in requirements if r.active])
