"""Drag Reduction System (rear wing flap)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import numpy as np


class DRSState(str, Enum):
    CLOSED = "closed"
    TRANSITION = "transition"
    OPEN = "open"


@dataclass
class DRSParams:
    open_Cl_factor: float = 0.35      # remaining Cl when fully open
    open_Cd_factor: float = 0.40      # remaining Cd when fully open
    transition_time: float = 0.25     # s


@dataclass
class DRSController:
    params: DRSParams
    position: float = 0.0             # 0=closed, 1=open
    state: DRSState = DRSState.CLOSED
    _target: float = 0.0

    def command(self, open_drs: bool) -> None:
        self._target = 1.0 if open_drs else 0.0
        if abs(self._target - self.position) < 1e-6:
            self.state = DRSState.OPEN if self.position > 0.5 else DRSState.CLOSED
        else:
            self.state = DRSState.TRANSITION

    def step(self, dt: float) -> float:
        if dt <= 0:
            return self.position
        rate = 1.0 / max(self.params.transition_time, 1e-3)
        if self.position < self._target:
            self.position = min(self.position + rate * dt, self._target)
        elif self.position > self._target:
            self.position = max(self.position - rate * dt, self._target)
        if abs(self.position - self._target) < 1e-6:
            self.state = DRSState.OPEN if self.position > 0.5 else DRSState.CLOSED
        else:
            self.state = DRSState.TRANSITION
        return self.position

    def factors(self) -> tuple[float, float]:
        """Return (Cl_factor, Cd_factor) blended closed→open."""
        p = self.position
        cl_f = 1.0 * (1 - p) + self.params.open_Cl_factor * p
        cd_f = 1.0 * (1 - p) + self.params.open_Cd_factor * p
        return cl_f, cd_f
