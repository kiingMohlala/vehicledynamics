"""Elastic propshaft / driveshaft with torsional stiffness and damping."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class ShaftState:
    theta: float = 0.0       # rad twist
    omega: float = 0.0       # rad/s relative rate
    torque: float = 0.0      # N·m transmitted
    energy: float = 0.0      # J stored
    stress_proxy: float = 0.0  # N·m / (arbitrary section) diagnostic


@dataclass
class ElasticShaft:
    """
    Torsional shaft: T = k θ + c ω  (with optional saturation).

    Equation of relative twist (when used with inertias on both ends):
        J_rel * θ_ddot = T_in - T_out - c θ_dot - k θ
    Here we expose the elastic torque law; integration lives in the solver.
    """

    stiffness: float = 12000.0   # N·m/rad
    damping: float = 40.0        # N·m·s/rad
    max_torque: float = 8000.0   # N·m soft limit

    def torque(self, theta: float, omega: float) -> float:
        T = self.stiffness * theta + self.damping * omega
        return float(np.clip(T, -self.max_torque, self.max_torque))

    def energy(self, theta: float) -> float:
        return 0.5 * self.stiffness * theta * theta

    def evaluate(self, theta: float, omega: float) -> ShaftState:
        T = self.torque(theta, omega)
        return ShaftState(
            theta=float(theta),
            omega=float(omega),
            torque=T,
            energy=self.energy(theta),
            stress_proxy=abs(T),
        )
