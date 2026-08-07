"""Calibratable parameter definitions and bounds."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class CalibParameter:
    name: str
    value: float
    low: float
    high: float
    group: str = "general"  # tire | suspension | aero | mass | brake | steering
    unit: str = ""

    def clip(self, x: float) -> float:
        return float(min(self.high, max(self.low, x)))


@dataclass
class ParameterSet:
    params: list[CalibParameter] = field(default_factory=list)

    def names(self) -> list[str]:
        return [p.name for p in self.params]

    def values(self) -> dict[str, float]:
        return {p.name: p.value for p in self.params}

    def vector(self) -> list[float]:
        return [p.value for p in self.params]

    def set_vector(self, x: list[float] | tuple[float, ...]) -> None:
        for p, v in zip(self.params, x):
            p.value = p.clip(float(v))

    def bounds(self) -> list[tuple[float, float]]:
        return [(p.low, p.high) for p in self.params]

    def update(self, values: dict[str, float]) -> None:
        by = {p.name: p for p in self.params}
        for k, v in values.items():
            if k in by:
                by[k].value = by[k].clip(float(v))

    def to_dict(self) -> dict[str, Any]:
        return {p.name: {"value": p.value, "low": p.low, "high": p.high, "group": p.group} for p in self.params}

    @classmethod
    def default_vehicle(cls) -> "ParameterSet":
        return cls(params=[
            CalibParameter("mass", 1400, 1000, 2000, "mass", "kg"),
            CalibParameter("Cd", 0.34, 0.20, 0.60, "aero"),
            CalibParameter("Cl_front", -0.45, -1.5, 0.2, "aero"),
            CalibParameter("Cl_rear", -0.70, -2.0, 0.2, "aero"),
            CalibParameter("front_spring", 28000, 15000, 60000, "suspension", "N/m"),
            CalibParameter("rear_spring", 32000, 15000, 70000, "suspension", "N/m"),
            CalibParameter("tire_mu", 1.0, 0.5, 1.8, "tire"),
            CalibParameter("tire_Cx", 80000, 40000, 150000, "tire", "N"),
            CalibParameter("brake_gain", 1.0, 0.5, 2.0, "brake"),
            CalibParameter("rolling_resistance", 0.015, 0.005, 0.04, "aero"),
        ])
