"""Material library and region assignment."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class Material:
    name: str
    E: float  # Pa
    nu: float
    rho: float  # kg/m3
    meta: dict[str, Any] = field(default_factory=dict)


MATERIALS = {
    "steel": Material("steel", 210e9, 0.30, 7850),
    "aluminum": Material("aluminum", 70e9, 0.33, 2700),
    "titanium": Material("titanium", 110e9, 0.34, 4500),
    "cfrp": Material("cfrp", 70e9, 0.30, 1550),
    "gfrp": Material("gfrp", 20e9, 0.28, 1900),
    "magnesium": Material("magnesium", 45e9, 0.35, 1800),
}


@dataclass
class MaterialAssignment:
    region: str
    material: str

    @property
    def props(self) -> Material:
        return MATERIALS[self.material]


def default_vehicle_materials() -> list[MaterialAssignment]:
    return [
        MaterialAssignment("chassis", "steel"),
        MaterialAssignment("body", "aluminum"),
        MaterialAssignment("suspension", "steel"),
        MaterialAssignment("wing", "cfrp"),
    ]
