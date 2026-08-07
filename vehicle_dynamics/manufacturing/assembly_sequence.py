"""Assembly sequence planning."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class AssemblyStep:
    order: int
    part: str
    action: str
    tools: list[str] = field(default_factory=list)
    time_min: float = 5.0


@dataclass
class AssemblyPlan:
    steps: list[AssemblyStep]
    total_time_hours: float

    @property
    def part_order(self) -> list[str]:
        return [s.part for s in self.steps]


# Preferred dependency order for vehicle build
DEFAULT_ORDER = [
    "chassis",
    "suspension",
    "powertrain",
    "battery",
    "fuel",
    "cooling",
    "brake",
    "wheel",
    "body",
    "aero",
    "cockpit",
    "electronics",
]


def plan_assembly(part_names: list[str], categories: dict[str, str] | None = None) -> AssemblyPlan:
    categories = categories or {}
    # sort by category priority
    def key(name: str) -> tuple[int, str]:
        cat = categories.get(name, name.split("_")[0] if "_" in name else name)
        try:
            return (DEFAULT_ORDER.index(cat) if cat in DEFAULT_ORDER else 50, name)
        except ValueError:
            return (50, name)

    ordered = sorted(part_names, key=key)
    steps = []
    for i, p in enumerate(ordered, 1):
        cat = categories.get(p, "generic")
        action = "install"
        tools = ["hand"]
        t = 5.0
        if cat in ("chassis",):
            action = "fixture_and_weld"
            tools = ["fixture", "welder"]
            t = 30.0
        elif cat in ("powertrain", "engine"):
            action = "mount"
            tools = ["hoist", "torque_wrench"]
            t = 45.0
        elif cat in ("body",):
            action = "bond_or_bolt"
            tools = ["adhesive", "rivet_gun"]
            t = 20.0
        steps.append(AssemblyStep(i, p, action, tools, t))
    total = sum(s.time_min for s in steps) / 60.0
    return AssemblyPlan(steps, total)
