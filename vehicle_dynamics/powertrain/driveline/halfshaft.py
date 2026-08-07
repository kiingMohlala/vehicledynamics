"""Independent left/right half-shaft compliance."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .shaft import ElasticShaft, ShaftState


@dataclass
class HalfShaftState:
    left: ShaftState
    right: ShaftState

    @property
    def torque_left(self) -> float:
        return self.left.torque

    @property
    def torque_right(self) -> float:
        return self.right.torque


@dataclass
class HalfShaftPair:
    """Two half-shafts sharing nominal stiffness (may be asymmetric)."""

    k_left: float = 8000.0
    k_right: float = 8000.0
    c_left: float = 25.0
    c_right: float = 25.0
    max_torque: float = 5000.0

    def __post_init__(self) -> None:
        self._L = ElasticShaft(self.k_left, self.c_left, self.max_torque)
        self._R = ElasticShaft(self.k_right, self.c_right, self.max_torque)

    def evaluate(
        self,
        theta_L: float,
        omega_L: float,
        theta_R: float,
        omega_R: float,
    ) -> HalfShaftState:
        return HalfShaftState(
            left=self._L.evaluate(theta_L, omega_L),
            right=self._R.evaluate(theta_R, omega_R),
        )
