"""
Phase 5.2 – Four-wheel brake torque distribution.

Maps a pedal command (and optional per-wheel scale factors) to independent
brake torque commands for FL, FR, RL, RR.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from ..braking.parameters import BrakeParams

# Wheel order: FL, FR, RL, RR
N_WHEELS = 4


@dataclass
class WheelBrakeCommand:
    """Desired brake torque [N·m] per wheel (magnitude, resisting)."""
    T: np.ndarray  # shape (4,)


class FourWheelBrakeDistributor:
    """
    Front/rear bias from BrakeParams, equal left/right by default.

    Optional per-wheel scale in [0, 1] for ESC / split testing later.
    """

    def __init__(self, params: BrakeParams = None):
        self.p = params or BrakeParams()

    def desired(
        self,
        pedal: float,
        wheel_scale: np.ndarray | None = None,
    ) -> WheelBrakeCommand:
        pedal = float(np.clip(pedal, 0.0, 1.0))
        T_f_axle = self.p.front_bias * self.p.max_front_torque * pedal
        T_r_axle = (1.0 - self.p.front_bias) * self.p.max_rear_torque * pedal
        T = np.array([
            0.5 * T_f_axle,
            0.5 * T_f_axle,
            0.5 * T_r_axle,
            0.5 * T_r_axle,
        ], dtype=float)
        if wheel_scale is not None:
            scale = np.asarray(wheel_scale, dtype=float).reshape(4)
            scale = np.clip(scale, 0.0, 1.0)
            T *= scale
        return WheelBrakeCommand(T=T)
