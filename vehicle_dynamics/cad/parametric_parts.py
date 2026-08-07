"""Factory helpers for common vehicle parts."""
from __future__ import annotations

import numpy as np
from .component import Component


def chassis_tub(wheelbase: float = 2.70, width: float = 1.40, height: float = 0.35, mass: float = 180.0) -> Component:
    return Component(
        name="chassis",
        category="chassis",
        position=np.array([wheelbase * 0.5, 0.0, height * 0.5]),
        size=np.array([wheelbase * 0.95, width, height]),
        mass=mass,
    )


def body_shell(wheelbase: float = 2.70, width: float = 1.90, height: float = 1.15, mass: float = 120.0) -> Component:
    return Component(
        name="body",
        category="body",
        position=np.array([wheelbase * 0.5, 0.0, height * 0.45]),
        size=np.array([wheelbase + 0.8, width, height]),
        mass=mass,
    )


def engine_block(position=(1.40, 0.0, 0.30), size=(0.70, 0.55, 0.50), mass: float = 160.0) -> Component:
    return Component(name="engine", category="powertrain", position=position, size=size, mass=mass)


def transmission_box(position=(1.90, 0.0, 0.25), size=(0.45, 0.35, 0.30), mass: float = 55.0) -> Component:
    return Component(name="transmission", category="powertrain", position=position, size=size, mass=mass)


def differential_unit(position=(2.55, 0.0, 0.20), size=(0.30, 0.40, 0.25), mass: float = 35.0) -> Component:
    return Component(name="differential", category="powertrain", position=position, size=size, mass=mass)


def battery_pack(position=(1.35, 0.0, 0.15), size=(1.20, 1.00, 0.18), mass: float = 320.0) -> Component:
    return Component(name="battery", category="battery", position=position, size=size, mass=mass)


def fuel_tank(position=(2.20, 0.0, 0.20), size=(0.50, 0.70, 0.30), mass: float = 40.0) -> Component:
    return Component(name="fuel_tank", category="fuel", position=position, size=size, mass=mass)


def cockpit(position=(1.10, 0.0, 0.55), size=(1.00, 1.20, 0.90), mass: float = 80.0) -> Component:
    return Component(name="cockpit", category="cockpit", position=position, size=size, mass=mass)


def wheel_tire(name: str, position, radius: float = 0.32, width: float = 0.25, mass: float = 25.0) -> Component:
    return Component(
        name=name,
        category="wheel",
        position=position,
        size=np.array([radius * 2, width, radius * 2]),
        mass=mass,
    )


def aero_wing(name: str, position, span: float = 1.6, chord: float = 0.30, mass: float = 8.0) -> Component:
    return Component(name=name, category="aero", position=position, size=np.array([chord, span, 0.08]), mass=mass)


def cooling_radiator(position=(0.40, 0.0, 0.35), size=(0.25, 0.70, 0.40), mass: float = 12.0) -> Component:
    return Component(name="radiator", category="cooling", position=position, size=size, mass=mass)
