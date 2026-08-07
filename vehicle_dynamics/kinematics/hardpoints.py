"""Suspension hardpoint definitions and import."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any
import json
import numpy as np


@dataclass
class HardpointSet:
    """Named 3D points for one corner (or axle). Coordinates in vehicle frame (m)."""
    name: str = "FL"
    points: dict[str, np.ndarray] = field(default_factory=dict)

    def set(self, key: str, xyz) -> None:
        self.points[key] = np.asarray(xyz, dtype=float).ravel()[:3]

    def get(self, key: str) -> np.ndarray:
        if key not in self.points:
            raise KeyError(f"Hardpoint '{key}' not in {self.name}")
        return self.points[key]

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "points": {k: v.tolist() for k, v in self.points.items()}}

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "HardpointSet":
        hp = cls(name=d.get("name", "corner"))
        for k, v in d.get("points", {}).items():
            hp.set(k, v)
        return hp


@dataclass
class HardpointModel:
    """Full-vehicle hardpoint model."""
    suspension_type: str = "double_wishbone"
    corners: dict[str, HardpointSet] = field(default_factory=dict)
    meta: dict[str, Any] = field(default_factory=dict)

    def add_corner(self, corner: HardpointSet) -> None:
        self.corners[corner.name] = corner

    def to_dict(self) -> dict[str, Any]:
        return {
            "suspension_type": self.suspension_type,
            "corners": {k: v.to_dict() for k, v in self.corners.items()},
            "meta": self.meta,
        }

    def to_json(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "HardpointModel":
        m = cls(suspension_type=d.get("suspension_type", "double_wishbone"), meta=d.get("meta", {}))
        for c in d.get("corners", {}).values():
            m.add_corner(HardpointSet.from_dict(c))
        return m

    @classmethod
    def from_json(cls, path: str | Path) -> "HardpointModel":
        return cls.from_dict(json.loads(Path(path).read_text()))

    @classmethod
    def default_double_wishbone(cls) -> "HardpointModel":
        """Generic passenger-car style DWB hardpoints (left front)."""
        m = cls(suspension_type="double_wishbone")
        for side, sy in (("FL", 1.0), ("FR", -1.0)):
            hp = HardpointSet(name=side)
            # chassis pickups (y sign flips)
            hp.set("LCA_front", [0.05, 0.35 * sy, -0.15])
            hp.set("LCA_rear", [-0.25, 0.35 * sy, -0.15])
            hp.set("UCA_front", [0.02, 0.45 * sy, 0.20])
            hp.set("UCA_rear", [-0.20, 0.45 * sy, 0.20])
            hp.set("LCA_outer", [0.00, 0.75 * sy, -0.12])
            hp.set("UCA_outer", [0.00, 0.72 * sy, 0.18])
            hp.set("wheel_center", [0.00, 0.80 * sy, 0.00])
            hp.set("tierod_inner", [0.15, 0.30 * sy, -0.05])
            hp.set("tierod_outer", [0.12, 0.78 * sy, -0.05])
            hp.set("strut_lower", [0.00, 0.70 * sy, -0.05])
            hp.set("strut_upper", [0.02, 0.55 * sy, 0.45])
            m.add_corner(hp)
        # rear simplified mirror in x
        for side, sy in (("RL", 1.0), ("RR", -1.0)):
            hp = HardpointSet(name=side)
            hp.set("LCA_front", [-2.50, 0.35 * sy, -0.15])
            hp.set("LCA_rear", [-2.80, 0.35 * sy, -0.15])
            hp.set("UCA_front", [-2.52, 0.45 * sy, 0.18])
            hp.set("UCA_rear", [-2.75, 0.45 * sy, 0.18])
            hp.set("LCA_outer", [-2.65, 0.75 * sy, -0.12])
            hp.set("UCA_outer", [-2.65, 0.72 * sy, 0.16])
            hp.set("wheel_center", [-2.65, 0.80 * sy, 0.00])
            hp.set("tierod_inner", [-2.50, 0.30 * sy, -0.05])
            hp.set("tierod_outer", [-2.55, 0.78 * sy, -0.05])
            m.add_corner(hp)
        return m

    @classmethod
    def default_macpherson(cls) -> "HardpointModel":
        m = cls(suspension_type="macpherson")
        for side, sy in (("FL", 1.0), ("FR", -1.0)):
            hp = HardpointSet(name=side)
            hp.set("LCA_front", [0.05, 0.35 * sy, -0.15])
            hp.set("LCA_rear", [-0.20, 0.35 * sy, -0.15])
            hp.set("LCA_outer", [0.00, 0.75 * sy, -0.10])
            hp.set("strut_lower", [0.00, 0.70 * sy, 0.00])
            hp.set("strut_upper", [0.03, 0.55 * sy, 0.50])
            hp.set("wheel_center", [0.00, 0.80 * sy, 0.00])
            hp.set("tierod_inner", [0.15, 0.30 * sy, -0.05])
            hp.set("tierod_outer", [0.12, 0.78 * sy, -0.05])
            m.add_corner(hp)
        return m
