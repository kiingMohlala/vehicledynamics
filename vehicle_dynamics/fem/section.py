"""Beam cross-section properties (SI: m, m², m⁴)."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class Section:
    A: float
    Iy: float
    Iz: float
    J: float
    # Optional metadata for reporting / mass
    od: float | None = None
    wall: float | None = None
    label: str = ""

    @property
    def mass_per_metre(self) -> float | None:
        """Requires density to be supplied externally; placeholder."""
        return None


def circular(diameter: float) -> Section:
    r = diameter / 2.0
    A = math.pi * r**2
    I = math.pi * r**4 / 4.0
    J = math.pi * r**4 / 2.0
    return Section(A=A, Iy=I, Iz=I, J=J, od=diameter, wall=None, label=f"solid_d{diameter*1e3:.1f}")


def rectangular(b: float, h: float) -> Section:
    A = b * h
    Iy = b * h**3 / 12.0
    Iz = h * b**3 / 12.0
    a, c = max(b, h), min(b, h)
    J = a * c**3 * (1.0 / 3.0 - 0.21 * (c / a) * (1.0 - c**4 / (12.0 * a**4)))
    return Section(A=A, Iy=Iy, Iz=Iz, J=J, label=f"rect_{b*1e3:.0f}x{h*1e3:.0f}")


def tube(od: float, wall: float) -> Section:
    """Circular tube: outer diameter and wall thickness [m]."""
    if wall <= 0 or wall >= od / 2.0:
        raise ValueError(f"Invalid tube wall={wall} for od={od}")
    ro = od / 2.0
    ri = ro - wall
    A = math.pi * (ro**2 - ri**2)
    I = math.pi * (ro**4 - ri**4) / 4.0
    J = math.pi * (ro**4 - ri**4) / 2.0
    return Section(
        A=A,
        Iy=I,
        Iz=I,
        J=J,
        od=od,
        wall=wall,
        label=f"tube_{od*1e3:.1f}x{wall*1e3:.1f}",
    )
