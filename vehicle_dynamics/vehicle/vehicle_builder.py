"""Build a runnable vehicle package from a VehicleDefinition."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .vehicle_definition import VehicleDefinition
from .subsystem_registry import DEFAULT_REGISTRY, SubsystemRegistry


@dataclass
class BuiltVehicle:
    """Concrete vehicle instance ready for simulation binding."""
    definition: VehicleDefinition
    config_hash: str
    simulation_kwargs: dict[str, Any] = field(default_factory=dict)
    subsystem_handles: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    @property
    def name(self) -> str:
        return self.definition.name

    @property
    def mass_kg(self) -> float:
        return self.definition.mass.total_mass_kg

    @property
    def wheelbase_m(self) -> float:
        return self.definition.geometry.L


class VehicleBuilder:
    def __init__(self, registry: SubsystemRegistry | None = None) -> None:
        self.registry = registry or DEFAULT_REGISTRY

    def build(self, definition: VehicleDefinition) -> BuiltVehicle:
        d = definition
        g = d.geometry
        m = d.mass
        s = d.subsystems

        # Map definition → SimulationConfig-compatible kwargs
        sim_kwargs = {
            "mass": m.total_mass_kg,
            "Iz": m.Iz_kgm2,
            "wheelbase": g.L,
            "track": 0.5 * (g.track_front_m + g.track_rear_m),
            "wheel_radius": g.wheel_radius_m,
            "CdA": s.aero.Cd * s.aero.frontal_area_m2 if s.aero.enabled else 0.0,
            "controls_enabled": s.controls.abs or s.controls.tc or s.controls.esc,
            "powertrain_enabled": True,
            "strategy_enabled": s.controls.strategy_enabled,
            "drive_mode": s.controls.drive_mode,
            "aero_enabled": s.aero.enabled,
        }

        handles: dict[str, Any] = {}
        notes: list[str] = []

        # Resolve registered subsystem stubs
        tire_name = s.tire.model
        if self.registry.has("tire", tire_name):
            handles["tire"] = self.registry.create("tire", tire_name, mu=s.tire.mu)
        else:
            notes.append(f"tire model '{tire_name}' not in registry; using definition only")

        pt_arch = s.powertrain.architecture
        if self.registry.has("powertrain", pt_arch):
            handles["powertrain"] = self.registry.create("powertrain", pt_arch)
        else:
            notes.append(f"powertrain '{pt_arch}' not registered")

        diff = s.powertrain.differential
        if self.registry.has("differential", diff):
            handles["differential"] = self.registry.create("differential", diff)

        handles["geometry"] = {
            "a": g.a_m, "b": g.b_m, "h_cg": g.h_cg_m,
            "track_f": g.track_front_m, "track_r": g.track_rear_m,
        }
        handles["brakes"] = {
            "max_torque": s.brakes.max_torque_Nm,
            "bias_front": s.brakes.bias_front,
            "abs": s.brakes.abs_enabled,
        }
        handles["aero"] = {
            "Cd": s.aero.Cd, "Cl_f": s.aero.Cl_front, "Cl_r": s.aero.Cl_rear,
            "area": s.aero.frontal_area_m2, "rho": s.aero.rho,
        }

        return BuiltVehicle(
            definition=d,
            config_hash=d.configuration_hash(),
            simulation_kwargs=sim_kwargs,
            subsystem_handles=handles,
            notes=notes,
        )
