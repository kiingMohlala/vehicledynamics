"""Parametric vehicle assembly."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import copy
import numpy as np

from .component import Component
from .chassis import build_chassis
from .body import build_body
from .suspension_assembly import suspension_mounts
from .powertrain_layout import layout_ice, layout_ev
from .cockpit import build_cockpit
from .parametric_parts import wheel_tire, aero_wing, cooling_radiator
from .mass_properties import compute_mass_properties, MassProperties
from .packaging_solver import evaluate_packaging, PackagingReport
from .interference import detect_interferences
from .export import export_obj, export_stl, export_json_assembly


@dataclass
class AssemblyConfig:
    wheelbase: float = 2.70
    track: float = 1.55
    ride_height: float = 0.12
    tire_radius: float = 0.32
    tire_width: float = 0.25
    body_width: float = 1.90
    body_height: float = 1.15
    powertrain: str = "ice"  # ice | ev
    driver_mass: float = 75.0
    chassis_mass: float = 180.0
    body_mass: float = 120.0
    front_axle_x: float = 0.0
    enabled: bool = True


@dataclass
class VehicleAssembly:
    config: AssemblyConfig = field(default_factory=AssemblyConfig)
    components: list[Component] = field(default_factory=list)
    _built: bool = False

    def build(self) -> "VehicleAssembly":
        cfg = self.config
        parts: list[Component] = []
        parts.append(build_chassis(cfg.wheelbase, cfg.track, cfg.ride_height, cfg.chassis_mass))
        parts.append(build_body(cfg.wheelbase, cfg.body_width, cfg.body_height, cfg.body_mass))
        parts.extend(suspension_mounts(cfg.wheelbase, cfg.track, cfg.ride_height))
        if cfg.powertrain == "ev":
            parts.extend(layout_ev(cfg.wheelbase))
        else:
            parts.extend(layout_ice(cfg.wheelbase))
        parts.extend(build_cockpit(cfg.wheelbase, cfg.driver_mass))
        # wheels
        half_t = cfg.track * 0.5
        fx = cfg.front_axle_x
        rx = fx - cfg.wheelbase
        for name, pos in {
            "wheel_FL": np.array([fx, half_t, cfg.tire_radius]),
            "wheel_FR": np.array([fx, -half_t, cfg.tire_radius]),
            "wheel_RL": np.array([rx, half_t, cfg.tire_radius]),
            "wheel_RR": np.array([rx, -half_t, cfg.tire_radius]),
        }.items():
            parts.append(wheel_tire(name, pos, cfg.tire_radius, cfg.tire_width))
        parts.append(aero_wing("rear_wing", position=(rx - 0.15, 0.0, 0.95)))
        parts.append(aero_wing("front_wing", position=(fx + 0.35, 0.0, 0.20), span=1.4, chord=0.25, mass=5.0))
        parts.append(cooling_radiator(position=(fx + 0.45, 0.0, 0.40)))
        self.components = parts
        self._built = True
        return self

    def build_from_digital_twin(self, twin: Any = None) -> "VehicleAssembly":
        """Optional: pull wheelbase/track from a digital twin object if available."""
        if twin is not None:
            for attr, key in (("wheelbase", "wheelbase"), ("track", "track"), ("ride_height", "ride_height")):
                if hasattr(twin, attr):
                    setattr(self.config, key, float(getattr(twin, attr)))
                elif isinstance(twin, dict) and key in twin:
                    setattr(self.config, key, float(twin[key]))
        return self.build()

    def update(self, **kwargs) -> "VehicleAssembly":
        for k, v in kwargs.items():
            if hasattr(self.config, k):
                setattr(self.config, k, v)
            elif k == "engine_position":
                # rebuild then shift engine
                self.config.meta_engine_pos = v  # type: ignore
        self.build()
        if "engine_position" in kwargs:
            for c in self.components:
                if c.name == "engine":
                    c.position = np.asarray(kwargs["engine_position"], dtype=float)
        return self

    def get(self, name: str) -> Component | None:
        for c in self.components:
            if c.name == name:
                return c
        return None

    @property
    def mass_properties(self) -> MassProperties:
        if not self._built:
            self.build()
        return compute_mass_properties(
            self.components,
            wheelbase=self.config.wheelbase,
            front_axle_x=self.config.front_axle_x,
        )

    def packaging(self) -> PackagingReport:
        if not self._built:
            self.build()
        return evaluate_packaging(self.components)

    def interferences(self):
        if not self._built:
            self.build()
        return detect_interferences(self.components)

    def export(self, path: str):
        if not self._built:
            self.build()
        p = str(path).lower()
        if p.endswith(".obj"):
            return export_obj(self.components, path)
        if p.endswith(".stl"):
            return export_stl(self.components, path)
        if p.endswith(".json"):
            return export_json_assembly(self.components, path, meta=self.config.__dict__)
        # default JSON
        return export_json_assembly(self.components, path, meta=self.config.__dict__)

    def to_dict(self) -> dict[str, Any]:
        return {
            "config": self.config.__dict__,
            "components": [c.to_dict() for c in self.components],
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "VehicleAssembly":
        cfg = AssemblyConfig(**{k: v for k, v in d.get("config", {}).items() if k in AssemblyConfig.__dataclass_fields__})
        asm = cls(config=cfg)
        asm.components = [Component.from_dict(c) for c in d.get("components", [])]
        asm._built = len(asm.components) > 0
        return asm
