"""Gear mesh stiffness / damping (tooth elasticity)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class GearMeshState:
    theta: float = 0.0
    torque: float = 0.0
    stiffness: float = 0.0
    ripple: float = 0.0


class GearMesh:
    """
    Simple mesh: T = k_mesh * θ + c_mesh * ω
    Optional first-harmonic torque ripple for NVH studies.
    """

    def __init__(
        self,
        stiffness: float = 5.0e4,
        damping: float = 20.0,
        ripple_amp: float = 0.0,
        teeth: int = 30,
    ):
        self.k = float(stiffness)
        self.c = float(damping)
        self.ripple_amp = float(ripple_amp)
        self.teeth = int(teeth)

    def torque(self, theta: float, omega: float) -> float:
        T = self.k * theta + self.c * omega
        if self.ripple_amp > 0.0:
            T += self.ripple_amp * np.sin(self.teeth * theta)
        return float(T)

    def evaluate(self, theta: float, omega: float) -> GearMeshState:
        T = self.torque(theta, omega)
        ripple = self.ripple_amp * np.sin(self.teeth * theta) if self.ripple_amp else 0.0
        return GearMeshState(
            theta=float(theta),
            torque=T,
            stiffness=self.k,
            ripple=float(ripple),
        )
