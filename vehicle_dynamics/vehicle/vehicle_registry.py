"""In-memory registry of named vehicles / digital twins."""
from __future__ import annotations

from typing import Dict

from .vehicle_definition import VehicleDefinition
from .vehicle_builder import BuiltVehicle, VehicleBuilder
from .presets import PRESETS, load_preset


class VehicleRegistry:
    def __init__(self) -> None:
        self._defs: Dict[str, VehicleDefinition] = {}
        self._built: Dict[str, BuiltVehicle] = {}

    def register(self, defn: VehicleDefinition, build: bool = True) -> BuiltVehicle | None:
        self._defs[defn.name] = defn
        if build:
            bv = VehicleBuilder().build(defn)
            self._built[defn.name] = bv
            return bv
        return None

    def get_definition(self, name: str) -> VehicleDefinition:
        if name not in self._defs:
            raise KeyError(name)
        return self._defs[name]

    def get_built(self, name: str) -> BuiltVehicle:
        if name not in self._built:
            raise KeyError(name)
        return self._built[name]

    def list_names(self) -> list[str]:
        return sorted(self._defs.keys())

    def load_preset(self, name: str) -> BuiltVehicle:
        defn = load_preset(name)
        bv = self.register(defn, build=True)
        # Also index under the preset key when it differs from defn.name
        if name != defn.name:
            self._defs[name] = defn
            if bv is not None:
                self._built[name] = bv
        return bv  # type: ignore

    def load_all_presets(self) -> list[str]:
        names = []
        for n in PRESETS:
            self.load_preset(n)
            names.append(n)
        return names


DEFAULT_VEHICLE_REGISTRY = VehicleRegistry()
