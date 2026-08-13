"""Composite ply schedule and laminate estimates."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Ply:
    orientation_deg: float
    thickness_mm: float
    material: str = "cfrp"


@dataclass
class Laminate:
    plies: list[Ply] = field(default_factory=list)
    area_m2: float = 1.0

    @property
    def thickness_mm(self) -> float:
        return sum(p.thickness_mm for p in self.plies)

    @property
    def fiber_mass_kg(self) -> float:
        # ~1.55 g/cm3 * area * thickness
        return 1550 * self.area_m2 * (self.thickness_mm / 1000.0) * 0.6  # 60% fiber fraction proxy


@dataclass
class CompositeEstimate:
    laminate: Laminate
    layup_hours: float
    cure_hours: float
    cost: float


def estimate_composite(
    area_m2: float,
    n_plies: int = 8,
    ply_thickness_mm: float = 0.25,
    orientations: list[float] | None = None,
    labor_rate: float = 75.0,
    material_cost_per_kg: float = 45.0,
) -> CompositeEstimate:
    orients = orientations or [0, 45, -45, 90] * ((n_plies + 3) // 4)
    plies = [Ply(orients[i % len(orients)], ply_thickness_mm) for i in range(n_plies)]
    lam = Laminate(plies, area_m2)
    layup = area_m2 * n_plies * 0.15  # hours
    cure = 4.0
    cost = layup * labor_rate + cure * 20.0 + lam.fiber_mass_kg * material_cost_per_kg
    return CompositeEstimate(lam, layup, cure, cost)
