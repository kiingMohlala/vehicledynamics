"""Linear elastic isotropic material library (SI units)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Material:
    name: str = "generic"
    E: float = 210e9
    nu: float = 0.30
    rho: float = 7850.0
    yield_strength: float = 250e6
    ultimate_strength: float = 400e6

    @property
    def G(self) -> float:
        return self.E / (2.0 * (1.0 + self.nu))


def steel() -> Material:
    return Material(name="steel", E=210e9, nu=0.30, rho=7850.0, yield_strength=250e6, ultimate_strength=400e6)


def AISI_4130() -> Material:
    return Material(name="AISI_4130", E=205e9, nu=0.29, rho=7850.0, yield_strength=460e6, ultimate_strength=670e6)


def stainless_304() -> Material:
    return Material(name="stainless_304", E=193e9, nu=0.29, rho=8000.0, yield_strength=215e6, ultimate_strength=505e6)


def aluminium_6061() -> Material:
    return Material(name="aluminium_6061", E=68.9e9, nu=0.33, rho=2700.0, yield_strength=276e6, ultimate_strength=310e6)


def aluminum() -> Material:
    return aluminium_6061()


def custom_material(name: str, E: float, nu: float, rho: float, yield_strength: float = 250e6, ultimate_strength: float = 400e6) -> Material:
    return Material(name=name, E=E, nu=nu, rho=rho, yield_strength=yield_strength, ultimate_strength=ultimate_strength)
