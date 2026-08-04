"""Linear elastic isotropic material."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Material:
    E: float = 210e9          # Young's modulus [Pa]
    nu: float = 0.30          # Poisson ratio
    rho: float = 7850.0       # density [kg/m³] (for future dynamics)

    @property
    def G(self) -> float:
        """Shear modulus."""
        return self.E / (2.0 * (1.0 + self.nu))


def steel() -> Material:
    return Material(E=210e9, nu=0.30, rho=7850.0)


def aluminum() -> Material:
    return Material(E=70e9, nu=0.33, rho=2700.0)
