"""Powertrain packaging positions."""
from __future__ import annotations

from .parametric_parts import engine_block, transmission_box, differential_unit, battery_pack, fuel_tank
from .component import Component


def layout_ice(wheelbase: float = 2.70) -> list[Component]:
    return [
        engine_block(position=(0.35 * wheelbase, 0.0, 0.35)),
        transmission_box(position=(0.55 * wheelbase, 0.0, 0.28)),
        differential_unit(position=(0.95 * wheelbase, 0.0, 0.22)),
        fuel_tank(position=(0.70 * wheelbase, 0.0, 0.18)),
    ]


def layout_ev(wheelbase: float = 2.70) -> list[Component]:
    return [
        battery_pack(position=(0.50 * wheelbase, 0.0, 0.15), size=(wheelbase * 0.55, 1.10, 0.16)),
        Component(name="motor_rear", category="powertrain", position=(0.95 * wheelbase, 0.0, 0.22), size=(0.35, 0.35, 0.30), mass=55.0),
        differential_unit(position=(0.95 * wheelbase, 0.0, 0.22)),
    ]
