"""Complete vehicle definition (digital twin blueprint)."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any
import hashlib
import json

from .geometry import GeometryConfig
from .mass_properties import MassProperties
from .configuration import SubsystemBundle


@dataclass
class VehicleDefinition:
    name: str = "generic_sedan"
    vehicle_class: str = "passenger"   # passenger | gt | formula | rally | hypercar | truck | ev | kart
    version: str = "1.0.0"
    description: str = ""
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    mass: MassProperties = field(default_factory=MassProperties)
    subsystems: SubsystemBundle = field(default_factory=SubsystemBundle)
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "VehicleDefinition":
        geo = GeometryConfig(**d.get("geometry", {}))
        mass = MassProperties(**d.get("mass", {}))
        sub_raw = d.get("subsystems", {})
        # nested rebuild
        from .configuration import (
            TireConfig, SuspensionConfig, BrakeConfig, AeroConfigBlock,
            PowertrainConfigBlock, ControlsConfigBlock, DriverConfigBlock,
            ChassisConfig, SubsystemBundle,
        )
        tire = TireConfig(**sub_raw.get("tire", {}))
        susp = SuspensionConfig(**sub_raw.get("suspension", {}))
        brakes = BrakeConfig(**sub_raw.get("brakes", {}))
        aero = AeroConfigBlock(**sub_raw.get("aero", {}))
        pt = PowertrainConfigBlock(**sub_raw.get("powertrain", {}))
        ctrl = ControlsConfigBlock(**sub_raw.get("controls", {}))
        drv = DriverConfigBlock(**sub_raw.get("driver", {}))
        chassis = ChassisConfig(**sub_raw.get("chassis", {}))
        bundle = SubsystemBundle(
            tire=tire, suspension=susp, brakes=brakes, aero=aero,
            powertrain=pt, controls=ctrl, driver=drv, chassis=chassis,
            extras=sub_raw.get("extras", {}),
        )
        return cls(
            name=d.get("name", "unnamed"),
            vehicle_class=d.get("vehicle_class", "passenger"),
            version=d.get("version", "1.0.0"),
            description=d.get("description", ""),
            geometry=geo,
            mass=mass,
            subsystems=bundle,
            metadata=d.get("metadata", {}),
        )

    def configuration_hash(self) -> str:
        payload = json.dumps(self.to_dict(), sort_keys=True, default=str)
        return hashlib.sha256(payload.encode()).hexdigest()[:16]

    @classmethod
    def from_json(cls, path: str) -> "VehicleDefinition":
        with open(path) as f:
            return cls.from_dict(json.load(f))

    def to_json(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
