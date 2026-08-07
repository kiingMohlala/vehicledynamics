"""Engineering comparison between two vehicle definitions / twins."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .vehicle_definition import VehicleDefinition
from .digital_twin import DigitalTwin


@dataclass
class ComparisonResult:
    name_a: str
    name_b: str
    deltas: dict[str, float]
    notes: list[str]

    def as_table(self) -> str:
        lines = [f"Comparison: {self.name_a} vs {self.name_b}", "-" * 40]
        for k, v in self.deltas.items():
            lines.append(f"  {k:28s} {v:+.4g}")
        for n in self.notes:
            lines.append(f"  note: {n}")
        return "\n".join(lines)


def compare_definitions(a: VehicleDefinition, b: VehicleDefinition) -> ComparisonResult:
    deltas = {
        "mass_kg": b.mass.total_mass_kg - a.mass.total_mass_kg,
        "wheelbase_m": b.geometry.L - a.geometry.L,
        "track_front_m": b.geometry.track_front_m - a.geometry.track_front_m,
        "Iz": b.mass.Iz_kgm2 - a.mass.Iz_kgm2,
        "peak_power_kw": b.subsystems.powertrain.peak_power_kw - a.subsystems.powertrain.peak_power_kw,
        "Cd": b.subsystems.aero.Cd - a.subsystems.aero.Cd,
        "Cl_rear": b.subsystems.aero.Cl_rear - a.subsystems.aero.Cl_rear,
        "ks_front": b.subsystems.suspension.ks_front - a.subsystems.suspension.ks_front,
        "brake_torque": b.subsystems.brakes.max_torque_Nm - a.subsystems.brakes.max_torque_Nm,
    }
    notes = []
    if a.subsystems.powertrain.architecture != b.subsystems.powertrain.architecture:
        notes.append(
            f"architecture: {a.subsystems.powertrain.architecture} → {b.subsystems.powertrain.architecture}"
        )
    if a.subsystems.tire.model != b.subsystems.tire.model:
        notes.append(f"tire model: {a.subsystems.tire.model} → {b.subsystems.tire.model}")
    return ComparisonResult(name_a=a.name, name_b=b.name, deltas=deltas, notes=notes)


def compare_twins(a: DigitalTwin, b: DigitalTwin) -> ComparisonResult:
    return compare_definitions(a.definition, b.definition)
