"""Track segment primitives."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal
import numpy as np

SurfaceType = Literal["dry", "wet", "ice", "gravel", "curb"]


@dataclass
class SurfaceProperties:
    mu: float = 1.0
    roughness: float = 0.0
    water_depth_m: float = 0.0
    temperature_C: float = 25.0
    rubber_level: float = 0.0
    wind_exposure: float = 0.0
    surface: SurfaceType = "dry"


@dataclass
class TrackSegment:
    kind: str                          # straight | constant_radius | hairpin | esses | chicane | clothoid | elevation | jump | banked
    length_m: float
    radius_m: float = 0.0              # 0 = straight
    banking_deg: float = 0.0
    elevation_change_m: float = 0.0
    width_m: float = 12.0
    surface: SurfaceProperties = field(default_factory=SurfaceProperties)
    name: str = ""

    @property
    def curvature(self) -> float:
        if abs(self.radius_m) < 1e-6:
            return 0.0
        return 1.0 / self.radius_m


def straight(length: float, width: float = 12.0, **kw) -> TrackSegment:
    return TrackSegment(kind="straight", length_m=length, width_m=width, name=kw.get("name", "straight"), **{k: v for k, v in kw.items() if k != "name"})


def constant_radius(length: float, radius: float, banking_deg: float = 0.0, width: float = 12.0, **kw) -> TrackSegment:
    return TrackSegment(
        kind="banked" if abs(banking_deg) > 1e-6 else "constant_radius",
        length_m=length, radius_m=radius, banking_deg=banking_deg, width_m=width,
        name=kw.get("name", "corner"),
        **{k: v for k, v in kw.items() if k != "name"},
    )


def hairpin(radius: float = 15.0, width: float = 10.0, **kw) -> TrackSegment:
    length = abs(np.pi * radius)  # ~180 deg
    return TrackSegment(kind="hairpin", length_m=length, radius_m=radius, width_m=width, name=kw.get("name", "hairpin"))
