"""High-level energy strategy modes."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import numpy as np

from .torque_blending import BlendMode


class EnergyMode(str, Enum):
    EV = "ev"
    HYBRID = "hybrid"
    CHARGE_SUSTAIN = "charge_sustain"
    CHARGE_DEPLETE = "charge_deplete"
    PERFORMANCE = "performance"
    ECONOMY = "economy"


@dataclass
class EnergyManager:
    mode: EnergyMode = EnergyMode.HYBRID
    soc_target: float = 0.50
    soc_min_ev: float = 0.15

    def decide(self, soc: float, throttle: float, brake: float) -> tuple[BlendMode, bool, float]:
        """
        Returns (blend_mode, engine_on, motor_bias).
        motor_bias in [-1, 1]: positive prefers motor assist, negative prefers charge.
        """
        mode = self.mode
        engine_on = True
        motor_bias = 0.0

        if mode == EnergyMode.EV:
            if soc > self.soc_min_ev:
                return BlendMode.EV, False, 1.0
            return BlendMode.HYBRID, True, 0.2

        if mode == EnergyMode.CHARGE_DEPLETE:
            if soc > self.soc_min_ev:
                return BlendMode.EV if throttle < 0.7 else BlendMode.HYBRID, throttle >= 0.7, 0.8
            return BlendMode.HYBRID, True, 0.0

        if mode == EnergyMode.CHARGE_SUSTAIN:
            err = self.soc_target - soc
            motor_bias = float(np.clip(-err * 2.0, -0.5, 0.5))
            return BlendMode.HYBRID, True, motor_bias

        if mode == EnergyMode.PERFORMANCE:
            return BlendMode.PERFORMANCE, True, 1.0

        if mode == EnergyMode.ECONOMY:
            return BlendMode.ECONOMY, throttle > 0.15, 0.3

        # HYBRID
        engine_on = throttle > 0.1 or soc < self.soc_min_ev
        return BlendMode.HYBRID, engine_on, 0.4
