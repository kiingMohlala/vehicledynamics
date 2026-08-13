"""Roll-up cost estimation."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class CostBreakdown:
    material: float = 0.0
    machining: float = 0.0
    welding: float = 0.0
    composite: float = 0.0
    additive: float = 0.0
    assembly: float = 0.0
    purchased: float = 0.0
    overhead: float = 0.0

    @property
    def total(self) -> float:
        return (
            self.material + self.machining + self.welding + self.composite
            + self.additive + self.assembly + self.purchased + self.overhead
        )

    def to_dict(self) -> dict[str, float]:
        return {
            "material": self.material,
            "machining": self.machining,
            "welding": self.welding,
            "composite": self.composite,
            "additive": self.additive,
            "assembly": self.assembly,
            "purchased": self.purchased,
            "overhead": self.overhead,
            "total": self.total,
        }
