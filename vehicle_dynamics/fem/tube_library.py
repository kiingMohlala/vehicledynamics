"""Reusable circular-tube section library (SI metres)."""

from __future__ import annotations

from .section import Section, tube
from .material import Material, steel


def tube_custom(od_mm: float, wall_mm: float) -> Section:
    """od and wall specified in millimetres, converted to metres."""
    return tube(od_mm * 1e-3, wall_mm * 1e-3)


def tube_25x2() -> Section:
    return tube_custom(25.0, 2.0)


def tube_32x2() -> Section:
    return tube_custom(32.0, 2.0)


def tube_38x2() -> Section:
    return tube_custom(38.1, 2.0)


def tube_45x2_5() -> Section:
    return tube_custom(45.0, 2.5)


def mass_per_metre(section: Section, material: Material | None = None) -> float:
    """Linear density [kg/m]."""
    mat = material or steel()
    return mat.rho * section.A


def tube_properties_summary(section: Section, material: Material | None = None) -> dict:
    mat = material or steel()
    return {
        "A_m2": section.A,
        "Iy_m4": section.Iy,
        "Iz_m4": section.Iz,
        "J_m4": section.J,
        "od_m": section.od,
        "wall_m": section.wall,
        "mass_per_m_kg": mass_per_metre(section, mat),
        "label": section.label,
    }
