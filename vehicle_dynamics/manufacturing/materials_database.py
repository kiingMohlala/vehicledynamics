"""Manufacturing-oriented material properties."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class MfgMaterial:
    name: str
    density: float  # kg/m3
    cost_per_kg: float
    processes: list[str] = field(default_factory=list)
    min_wall_mm: float = 1.0
    machinability: float = 1.0  # higher = easier
    weldable: bool = True
    composite: bool = False
    meta: dict[str, Any] = field(default_factory=dict)


MATERIALS = {
    "steel": MfgMaterial("steel", 7850, 1.2, ["cnc", "weld", "sheet", "tube"], 1.5, 0.7, True),
    "aluminum": MfgMaterial("aluminum", 2700, 3.5, ["cnc", "sheet", "cast", "weld"], 1.2, 1.2, True),
    "titanium": MfgMaterial("titanium", 4500, 35.0, ["cnc", "am"], 1.0, 0.3, False),
    "cfrp": MfgMaterial("cfrp", 1550, 45.0, ["composite", "infusion"], 0.8, 0.2, False, True),
    "gfrp": MfgMaterial("gfrp", 1900, 12.0, ["composite"], 1.0, 0.4, False, True),
    "abs_am": MfgMaterial("abs_am", 1050, 8.0, ["am"], 1.5, 1.5, False),
}


def get_material(name: str) -> MfgMaterial:
    return MATERIALS[name.lower()]
