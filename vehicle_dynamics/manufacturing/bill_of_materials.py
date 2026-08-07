"""Bill of materials generation."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import csv


@dataclass
class BOMItem:
    part_number: str
    name: str
    qty: int
    material: str
    process: str
    mass_kg: float
    unit_cost: float
    level: int = 0
    supplier: str = "TBD"

    @property
    def extended_cost(self) -> float:
        return self.qty * self.unit_cost


@dataclass
class BOM:
    items: list[BOMItem] = field(default_factory=list)

    @property
    def total_cost(self) -> float:
        return sum(i.extended_cost for i in self.items)

    @property
    def total_mass(self) -> float:
        return sum(i.mass_kg * i.qty for i in self.items)

    def to_csv(self, path: str | Path) -> Path:
        path = Path(path)
        with path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["level", "part_number", "name", "qty", "material", "process", "mass_kg", "unit_cost", "ext_cost", "supplier"])
            for i in self.items:
                w.writerow([i.level, i.part_number, i.name, i.qty, i.material, i.process, f"{i.mass_kg:.3f}", f"{i.unit_cost:.2f}", f"{i.extended_cost:.2f}", i.supplier])
        return path
