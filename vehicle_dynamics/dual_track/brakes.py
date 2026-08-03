"""
Phase 5.2 / 5.3 – Four-wheel brake torque distribution.

Driver pedal uses front/rear bias.
ESC may add independent per-wheel torque via esc_scale (even when pedal = 0).
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from ..braking.parameters import BrakeParams

N_WHEELS = 4


@dataclass
class WheelBrakeCommand:
    """Desired brake torque [N·m] per wheel (magnitude, resisting)."""
    T: np.ndarray  # shape (4,)


class FourWheelBrakeDistributor:
    def __init__(self, params: BrakeParams = None):
        self.p = params or BrakeParams()

    def desired(
        self,
        pedal: float,
        wheel_scale: np.ndarray | None = None,
        esc_scale: np.ndarray | None = None,
    ) -> WheelBrakeCommand:
        """
        Parameters
        ----------
        pedal : float
            Driver brake pedal [0, 1].
        wheel_scale : optional (4,)
            Multiplier on driver torque (legacy / testing).
        esc_scale : optional (4,)
            ESC additional brake fraction in [0, 1] relative to axle max
            per side (0.5 * axle max torque). Applied even if pedal = 0.
        """
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
            scale = np.clip(np.asarray(wheel_scale, dtype=float).reshape(4), 0.0, 1.0)
            T *= scale

        if esc_scale is not None:
            esc = np.clip(np.asarray(esc_scale, dtype=float).reshape(4), 0.0, 1.0)
            # Nominal ESC capacity = half of max axle torque at full pedal
            T_esc_cap = np.array([
                0.5 * self.p.front_bias * self.p.max_front_torque,
                0.5 * self.p.front_bias * self.p.max_front_torque,
                0.5 * (1.0 - self.p.front_bias) * self.p.max_rear_torque,
                0.5 * (1.0 - self.p.front_bias) * self.p.max_rear_torque,
            ])
            T = T + esc * T_esc_cap

        return WheelBrakeCommand(T=T)
