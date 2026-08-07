"""Simplified shell section properties."""
from __future__ import annotations

from dataclasses import dataclass
from .materials import StructuralMaterial, aluminum


@dataclass
class ShellSection:
    thickness: float
    material: StructuralMaterial

    @property
    def membrane_stiffness(self) -> float:
        """Extensional rigidity A = E t / (1-nu^2) proxy (isotropic)."""
        m = self.material
        return m.E * self.thickness / (1 - m.nu**2)

    @property
    def bending_rigidity(self) -> float:
        """D = E t^3 / (12(1-nu^2))."""
        m = self.material
        return m.E * self.thickness**3 / (12 * (1 - m.nu**2))

    @property
    def areal_mass(self) -> float:
        return self.material.rho * self.thickness


def default_body_shell(thickness: float = 0.0015) -> ShellSection:
    return ShellSection(thickness=thickness, material=aluminum())
