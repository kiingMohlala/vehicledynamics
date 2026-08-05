"""Plastic material properties for crash engineering models."""

from __future__ import annotations

from dataclasses import dataclass
from vehicle_dynamics.fem.material import Material


@dataclass
class PlasticMaterial:
    """
    Elastic–perfectly plastic / bilinear idealization.
    Stresses in Pa; E, Et in Pa; rho in kg/m³.
    """

    name: str
    E: float
    nu: float
    rho: float
    yield_strength: float
    ultimate_strength: float
    plastic_tangent: float = 0.0  # Et; 0 → perfect plasticity

    @property
    def G(self) -> float:
        return self.E / (2.0 * (1.0 + self.nu))

    def to_elastic(self) -> Material:
        return Material(
            name=self.name,
            E=self.E,
            nu=self.nu,
            rho=self.rho,
            yield_strength=self.yield_strength,
            ultimate_strength=self.ultimate_strength,
        )


def plastic_steel() -> PlasticMaterial:
    return PlasticMaterial(
        name="mild_steel",
        E=210e9,
        nu=0.30,
        rho=7850.0,
        yield_strength=250e6,
        ultimate_strength=400e6,
        plastic_tangent=2.1e9,
    )


def plastic_4130() -> PlasticMaterial:
    return PlasticMaterial(
        name="AISI_4130",
        E=205e9,
        nu=0.29,
        rho=7850.0,
        yield_strength=460e6,
        ultimate_strength=670e6,
        plastic_tangent=4.0e9,
    )


def plastic_6061() -> PlasticMaterial:
    return PlasticMaterial(
        name="aluminium_6061",
        E=68.9e9,
        nu=0.33,
        rho=2700.0,
        yield_strength=276e6,
        ultimate_strength=310e6,
        plastic_tangent=1.0e9,
    )


def plastic_stainless() -> PlasticMaterial:
    return PlasticMaterial(
        name="stainless_304",
        E=193e9,
        nu=0.29,
        rho=8000.0,
        yield_strength=215e6,
        ultimate_strength=505e6,
        plastic_tangent=2.0e9,
    )
