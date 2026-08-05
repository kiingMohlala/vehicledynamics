"""Axle inertia / average speed tracking."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class AxleState:
    omega: float = 0.0          # average rad/s
    torque_in: float = 0.0
    torque_L: float = 0.0
    torque_R: float = 0.0


class AxleModel:
    def __init__(self, inertia: float = 0.8):
        self.inertia = inertia
        self.state = AxleState()

    def step(self, T_in: float, T_L: float, T_R: float, omega_L: float, omega_R: float, dt: float) -> AxleState:
        omega_avg = 0.5 * (omega_L + omega_R)
        # Residual torque accelerates axle carrier (diagnostic)
        residual = T_in - (T_L + T_R)
        omega = omega_avg  # kinematic average is authoritative for open/LSD
        self.state = AxleState(
            omega=float(omega),
            torque_in=float(T_in),
            torque_L=float(T_L),
            torque_R=float(T_R),
        )
        return self.state
