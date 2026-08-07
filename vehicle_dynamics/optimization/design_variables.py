"""Design variable definitions for DOE / optimization."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class DesignVariable:
    name: str
    low: float
    high: float
    unit: str = ""
    description: str = ""

    def __post_init__(self) -> None:
        if self.high < self.low:
            raise ValueError(f"{self.name}: high < low")

    def clip(self, x: float) -> float:
        return float(min(self.high, max(self.low, x)))

    def scale01(self, x: float) -> float:
        if abs(self.high - self.low) < 1e-15:
            return 0.0
        return (x - self.low) / (self.high - self.low)

    def from01(self, u: float) -> float:
        return self.low + float(u) * (self.high - self.low)

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "low": self.low, "high": self.high, "unit": self.unit}
