"""Body shell layout."""
from __future__ import annotations

from .parametric_parts import body_shell
from .component import Component


def build_body(wheelbase: float, width: float = 1.90, height: float = 1.15, mass: float = 120.0) -> Component:
    b = body_shell(wheelbase=wheelbase, width=width, height=height, mass=mass)
    return b
