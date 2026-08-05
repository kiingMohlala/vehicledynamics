"""
Suspension hardpoint definitions (pickup points) in vehicle coordinates.

Convention (vehicle frame):
  +x forward
  +y left
  +z up

All coordinates in metres relative to a convenient origin (e.g. ground plane
at design ride height, mid-track).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class Point3:
    x: float
    y: float
    z: float

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    def __add__(self, other: "Point3") -> "Point3":
        return Point3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Point3") -> "Point3":
        return Point3(self.x - other.x, self.y - other.y, self.z - other.z)


@dataclass
class WishboneHardpoints:
    """One double-wishbone corner (typically left front)."""
    # Upper control arm
    upper_front: Point3
    upper_rear: Point3
    upper_outer: Point3   # ball joint / upright attachment

    # Lower control arm
    lower_front: Point3
    lower_rear: Point3
    lower_outer: Point3   # ball joint

    # Steering / upright
    tierod_inner: Point3
    tierod_outer: Point3  # steering arm on upright

    # Wheel geometry
    wheel_center: Point3
    contact_patch: Point3  # design ride height ground contact


def default_front_left() -> WishboneHardpoints:
    """
    Generic passenger-car-ish left-front double wishbone (illustrative).
    Not a specific production car — used for solver validation.
    """
    return WishboneHardpoints(
        upper_front=Point3(0.05, 0.35, 0.55),
        upper_rear=Point3(-0.15, 0.35, 0.55),
        upper_outer=Point3(-0.02, 0.68, 0.52),
        lower_front=Point3(0.12, 0.30, 0.18),
        lower_rear=Point3(-0.20, 0.30, 0.18),
        lower_outer=Point3(0.00, 0.72, 0.15),
        tierod_inner=Point3(0.05, 0.28, 0.28),
        tierod_outer=Point3(0.05, 0.70, 0.28),
        wheel_center=Point3(0.00, 0.78, 0.33),
        contact_patch=Point3(0.00, 0.78, 0.00),
    )


def mirror_y(p: Point3) -> Point3:
    return Point3(p.x, -p.y, p.z)


def mirror_corner(hp: WishboneHardpoints) -> WishboneHardpoints:
    """Mirror left corner to right (negate y)."""
    return WishboneHardpoints(
        upper_front=mirror_y(hp.upper_front),
        upper_rear=mirror_y(hp.upper_rear),
        upper_outer=mirror_y(hp.upper_outer),
        lower_front=mirror_y(hp.lower_front),
        lower_rear=mirror_y(hp.lower_rear),
        lower_outer=mirror_y(hp.lower_outer),
        tierod_inner=mirror_y(hp.tierod_inner),
        tierod_outer=mirror_y(hp.tierod_outer),
        wheel_center=mirror_y(hp.wheel_center),
        contact_patch=mirror_y(hp.contact_patch),
    )
