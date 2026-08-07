"""Map requirements → scenarios → evidence → PASS/FAIL."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class VerificationLink:
    req_id: str
    scenario: str
    simulation_id: str
    evidence_id: str
    status: str
    metrics: dict[str, Any] = field(default_factory=dict)


@dataclass
class VerificationMatrix:
    links: list[VerificationLink] = field(default_factory=list)

    def add(self, link: VerificationLink) -> None:
        self.links.append(link)

    def for_requirement(self, req_id: str) -> list[VerificationLink]:
        return [L for L in self.links if L.req_id == req_id]

    def summary(self) -> dict[str, int]:
        s = {"PASS": 0, "FAIL": 0, "MISSING": 0, "PENDING": 0}
        for L in self.links:
            s[L.status] = s.get(L.status, 0) + 1
        return s

    def as_table(self) -> str:
        lines = [f"{'Req':16s} {'Scenario':20s} {'Sim':12s} {'Evidence':12s} {'Status':8s}"]
        lines.append("-" * 72)
        for L in self.links:
            lines.append(f"{L.req_id:16s} {L.scenario:20s} {L.simulation_id:12s} {L.evidence_id:12s} {L.status:8s}")
        return "\n".join(lines)
