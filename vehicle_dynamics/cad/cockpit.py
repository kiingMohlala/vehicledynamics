"""Cockpit / driver packaging."""
from __future__ import annotations

from .parametric_parts import cockpit
from .component import Component


def build_cockpit(wheelbase: float = 2.70, driver_mass: float = 75.0) -> list[Component]:
    c = cockpit(position=(0.40 * wheelbase, 0.0, 0.55))
    seat = Component(
        name="seat",
        category="cockpit",
        position=(0.38 * wheelbase, 0.0, 0.40),
        size=(0.55, 0.50, 0.70),
        mass=12.0 + driver_mass * 0.1,
    )
    driver = Component(
        name="driver",
        category="cockpit",
        position=(0.38 * wheelbase, 0.0, 0.55),
        size=(0.50, 0.45, 0.90),
        mass=driver_mass,
    )
    return [c, seat, driver]
