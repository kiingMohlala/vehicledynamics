"""Store named geometry entities (hardpoints, curves, surfaces, meshes)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import numpy as np


@dataclass
class GeometryDatabase:
    hardpoints: dict[str, np.ndarray] = field(default_factory=dict)
    curves: dict[str, Any] = field(default_factory=dict)
    surfaces: dict[str, Any] = field(default_factory=dict)
    meshes: dict[str, Any] = field(default_factory=dict)
    meta: dict[str, Any] = field(default_factory=dict)

    def add_hardpoint(self, name: str, point) -> None:
        self.hardpoints[name] = np.asarray(point, dtype=float)

    def add_curve(self, name: str, curve) -> None:
        self.curves[name] = curve

    def add_surface(self, name: str, surface) -> None:
        self.surfaces[name] = surface

    def add_mesh(self, name: str, mesh) -> None:
        self.meshes[name] = mesh

    def list_entities(self) -> dict[str, list[str]]:
        return {
            "hardpoints": list(self.hardpoints.keys()),
            "curves": list(self.curves.keys()),
            "surfaces": list(self.surfaces.keys()),
            "meshes": list(self.meshes.keys()),
        }

    def __len__(self) -> int:
        return len(self.hardpoints) + len(self.curves) + len(self.surfaces) + len(self.meshes)
