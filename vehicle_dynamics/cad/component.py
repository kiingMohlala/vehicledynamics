"""Parametric CAD component with pose, bounds, and mass."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import numpy as np


@dataclass
class Component:
    name: str
    category: str = "generic"  # chassis, body, suspension, powertrain, aero, cockpit, wheel, battery, fuel, cooling
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation_deg: np.ndarray = field(default_factory=lambda: np.zeros(3))  # euler xyz
    size: np.ndarray = field(default_factory=lambda: np.array([0.1, 0.1, 0.1]))  # Lx, Ly, Lz AABB
    mass: float = 0.0
    cg_local: np.ndarray = field(default_factory=lambda: np.zeros(3))
    meta: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float).ravel()[:3]
        self.rotation_deg = np.asarray(self.rotation_deg, dtype=float).ravel()[:3]
        self.size = np.asarray(self.size, dtype=float).ravel()[:3]
        self.cg_local = np.asarray(self.cg_local, dtype=float).ravel()[:3]

    @property
    def cg_global(self) -> np.ndarray:
        return self.position + self.cg_local

    @property
    def aabb_min(self) -> np.ndarray:
        return self.position - 0.5 * self.size

    @property
    def aabb_max(self) -> np.ndarray:
        return self.position + 0.5 * self.size

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "category": self.category,
            "position": self.position.tolist(),
            "rotation_deg": self.rotation_deg.tolist(),
            "size": self.size.tolist(),
            "mass": self.mass,
            "cg_local": self.cg_local.tolist(),
            "meta": self.meta,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Component":
        return cls(
            name=d["name"],
            category=d.get("category", "generic"),
            position=d.get("position", [0, 0, 0]),
            rotation_deg=d.get("rotation_deg", [0, 0, 0]),
            size=d.get("size", [0.1, 0.1, 0.1]),
            mass=float(d.get("mass", 0.0)),
            cg_local=d.get("cg_local", [0, 0, 0]),
            meta=d.get("meta", {}),
        )
