"""Beam cross-section properties."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class Section:
    """
    A  : cross-sectional area [m²]
    Iy : second moment about local y [m⁴]
    Iz : second moment about local z [m⁴]
    J  : torsional constant [m⁴]
    """

    A: float
    Iy: float
    Iz: float
    J: float


def circular(diameter: float) -> Section:
    """Solid circular cross-section."""
    r = diameter / 2.0
    A = math.pi * r**2
    I = math.pi * r**4 / 4.0
    J = math.pi * r**4 / 2.0
    return Section(A=A, Iy=I, Iz=I, J=J)


def rectangular(b: float, h: float) -> Section:
    """Solid rectangle: b along local y, h along local z."""
    A = b * h
    Iy = b * h**3 / 12.0  # bending about y (deflection in z)
    Iz = h * b**3 / 12.0  # bending about z (deflection in y)
    # Approximate torsion constant for rectangle
    a, c = max(b, h), min(b, h)
    J = a * c**3 * (1.0 / 3.0 - 0.21 * (c / a) * (1.0 - c**4 / (12.0 * a**4)))
    return Section(A=A, Iy=Iy, Iz=Iz, J=J)


def tube(od: float, wall: float) -> Section:
    """Circular tube: outer diameter and wall thickness."""
    ro = od / 2.0
    ri = max(ro - wall, 0.0)
    A = math.pi * (ro**2 - ri**2)
    I = math.pi * (ro**4 - ri**4) / 4.0
    J = math.pi * (ro**4 - ri**4) / 2.0
    return Section(A=A, Iy=I, Iz=I, J=J)
