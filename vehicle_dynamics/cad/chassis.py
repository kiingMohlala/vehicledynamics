"""Chassis layout helpers."""
from __future__ import annotations

from .parametric_parts import chassis_tub
from .component import Component


def build_chassis(wheelbase: float, track: float, ride_height: float, mass: float = 180.0) -> Component:
    c = chassis_tub(wheelbase=wheelbase, width=track * 0.9, height=0.30 + ride_height * 0.5, mass=mass)
    c.position[2] = ride_height + c.size[2] * 0.5
    c.meta = {"wheelbase": wheelbase, "track": track, "ride_height": ride_height}
    return c
