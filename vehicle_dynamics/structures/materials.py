"""Structural materials (wrap FEM + extra design data)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

try:
    from vehicle_dynamics.fem.material import (
        Material as FEMMaterial,
        steel as fem_steel,
        aluminum as fem_aluminum,
        AISI_4130,
        stainless_304,
        aluminium_6061,
        custom_material,
    )
    _HAS_FEM = True
except Exception:
    _HAS_FEM = False


@dataclass
class StructuralMaterial:
    name: str
    E: float
    nu: float
    rho: float
    Sy: float  # yield
    Su: float  # ultimate
    fatigue_endurance: float = 0.0  # approx endurance limit

    @property
    def G(self) -> float:
        return self.E / (2 * (1 + self.nu))


def steel() -> StructuralMaterial:
    return StructuralMaterial("steel", 210e9, 0.30, 7850, 350e6, 450e6, 175e6)


def aluminum() -> StructuralMaterial:
    return StructuralMaterial("aluminum", 70e9, 0.33, 2700, 250e6, 310e6, 90e6)


def titanium() -> StructuralMaterial:
    return StructuralMaterial("titanium", 110e9, 0.34, 4500, 880e6, 950e6, 400e6)


def cfrp() -> StructuralMaterial:
    return StructuralMaterial("cfrp", 70e9, 0.30, 1550, 600e6, 800e6, 250e6)


def aisi_4130() -> StructuralMaterial:
    return StructuralMaterial("AISI_4130", 205e9, 0.29, 7850, 460e6, 670e6, 230e6)


MATERIALS = {
    "steel": steel,
    "aluminum": aluminum,
    "titanium": titanium,
    "cfrp": cfrp,
    "aisi_4130": aisi_4130,
}
