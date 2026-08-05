"""
Phase 5.2 – Four independent ABS controllers.

Reuses the validated Phase 3.2 ABSController FSM per wheel.
"""

from __future__ import annotations

import numpy as np
from ..braking.abs_controller import ABSController, ABSParams


class FourWheelABS:
    """One ABSController instance per wheel (FL, FR, RL, RR)."""

    def __init__(self, params: ABSParams = None):
        self.controllers = [ABSController(params) for _ in range(4)]

    def reset(self):
        for c in self.controllers:
            c.pressure = 1.0
            c.state = "build"

    def update(self, kappas: np.ndarray, dt: float, active: bool = True) -> np.ndarray:
        """
        Parameters
        ----------
        kappas : array (4,)
            Longitudinal slip per wheel (κ > 0 braking).
        dt : float
            Timestep [s].
        active : bool
            If False, return unity pressure (no modulation).

        Returns
        -------
        pressures : array (4,) in [min_pressure, 1]
        """
        if not active:
            return np.ones(4)
        out = np.zeros(4)
        for i in range(4):
            out[i] = self.controllers[i].update(max(float(kappas[i]), 0.0), dt)
        return out

    @property
    def pressures(self) -> np.ndarray:
        return np.array([c.pressure for c in self.controllers])

    @property
    def states(self) -> list[str]:
        return [c.state for c in self.controllers]
