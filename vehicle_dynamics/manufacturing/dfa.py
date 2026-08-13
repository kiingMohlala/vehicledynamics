"""Design for Assembly analysis."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class DFAReport:
    part_count: int
    fastener_count: int
    estimated_time_hours: float
    complexity: float
    score: float
    notes: list[str] = field(default_factory=list)


def evaluate_dfa(parts: list[dict[str, Any]], fasteners_per_part: float = 4.0) -> DFAReport:
    n = len(parts)
    fasteners = int(round(n * fasteners_per_part * 0.5))  # not every part fully fastened
    # Boothroyd-ish: ~3s handle + 6s insert per part baseline
    time_s = n * 12.0 + fasteners * 8.0
    hours = time_s / 3600.0
    complexity = n * 0.1 + fasteners * 0.05
    score = max(0.0, 100.0 - complexity * 5 - max(0, n - 40) * 0.5)
    notes = []
    if n > 80:
        notes.append("High part count — consider consolidation")
    if fasteners > 200:
        notes.append("High fastener count — consider clips/adhesives")
    return DFAReport(n, fasteners, hours, complexity, score, notes)
