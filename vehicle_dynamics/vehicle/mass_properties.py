"""Mass and inertia properties."""
from __future__ import annotations
from dataclasses import dataclass


@dataclass
class MassProperties:
    mass_kg: float = 1400.0
    Iz_kgm2: float = 2500.0
    Ix_kgm2: float = 500.0
    Iy_kgm2: float = 2200.0
    unsprung_front_kg: float = 40.0
    unsprung_rear_kg: float = 40.0
    fuel_capacity_kg: float = 60.0
    fuel_mass_kg: float = 30.0

    @property
    def total_mass_kg(self) -> float:
        return self.mass_kg + self.fuel_mass_kg
