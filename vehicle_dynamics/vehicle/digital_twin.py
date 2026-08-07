"""Digital twin wrapper: definition + build + validation metadata."""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from .vehicle_definition import VehicleDefinition
from .vehicle_builder import BuiltVehicle, VehicleBuilder


@dataclass
class DigitalTwin:
    definition: VehicleDefinition
    built: BuiltVehicle
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    validation_status: str = "unvalidated"
    calibration: dict[str, Any] = field(default_factory=dict)
    version: str = "12.0.0"
    notes: list[str] = field(default_factory=list)

    @property
    def name(self) -> str:
        return self.definition.name

    @property
    def config_hash(self) -> str:
        return self.built.config_hash

    def mark_validated(self) -> None:
        self.validation_status = "validated"

    def summary(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "class": self.definition.vehicle_class,
            "hash": self.config_hash,
            "status": self.validation_status,
            "mass_kg": self.built.mass_kg,
            "wheelbase_m": self.built.wheelbase_m,
            "architecture": self.definition.subsystems.powertrain.architecture,
            "created_at": self.created_at,
            "version": self.version,
        }


def create_digital_twin(definition: VehicleDefinition) -> DigitalTwin:
    built = VehicleBuilder().build(definition)
    return DigitalTwin(definition=definition, built=built, notes=list(built.notes))
