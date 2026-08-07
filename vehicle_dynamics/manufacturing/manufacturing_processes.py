"""Process catalog and selection heuristics."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class Process:
    name: str
    category: str  # cnc, weld, sheet, cast, forge, composite, am, tube
    setup_hours: float
    rate_units_per_hour: float  # rough throughput proxy
    hourly_cost: float
    meta: dict[str, Any] = field(default_factory=dict)


PROCESSES = {
    "cnc": Process("cnc", "cnc", 0.5, 1.0, 85.0),
    "sheet": Process("sheet", "sheet", 0.3, 5.0, 55.0),
    "tube": Process("tube", "tube", 0.2, 8.0, 50.0),
    "weld": Process("weld", "weld", 0.25, 2.0, 65.0),
    "cast": Process("cast", "cast", 2.0, 0.5, 70.0),
    "forge": Process("forge", "forge", 3.0, 0.3, 90.0),
    "composite": Process("composite", "composite", 1.0, 0.4, 75.0),
    "infusion": Process("infusion", "composite", 1.5, 0.3, 80.0),
    "am": Process("am", "am", 0.1, 0.2, 40.0),
}


def select_process(part_category: str, material: str) -> str:
    """Heuristic process selection."""
    mat = material.lower()
    cat = part_category.lower()
    if cat in ("chassis", "suspension") and mat in ("steel", "aluminum"):
        return "tube" if cat == "chassis" else "cnc"
    if cat in ("body",) and mat == "aluminum":
        return "sheet"
    if cat in ("wing", "body") and mat in ("cfrp", "gfrp"):
        return "composite"
    if mat in ("abs_am",) or cat in ("prototype",):
        return "am"
    if cat in ("powertrain", "engine"):
        return "cnc"
    return "cnc"
