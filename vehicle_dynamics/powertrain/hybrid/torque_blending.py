"""ICE + motor torque blending."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import numpy as np


class BlendMode(str, Enum):
    EV = "ev"
    HYBRID = "hybrid"
    PERFORMANCE = "performance"
    ECONOMY = "economy"
    ICE_ONLY = "ice_only"


@dataclass
class TorqueBlender:
    mode: BlendMode = BlendMode.HYBRID

    def blend(
        self,
        driver_torque_request: float,
        ice_available: float,
        motor_available: float,
        mode: BlendMode | None = None,
    ) -> tuple[float, float]:
        """
        Returns (ice_torque_cmd, motor_torque_cmd).
        driver_torque_request is desired total (positive drive).
        """
        mode = mode or self.mode
        req = float(driver_torque_request)
        ice_a = max(float(ice_available), 0.0)
        mot_a = float(motor_available)  # can be signed for regen handled elsewhere

        if mode == BlendMode.EV:
            return 0.0, float(np.clip(req, -abs(mot_a), abs(mot_a)))
        if mode == BlendMode.ICE_ONLY:
            return float(np.clip(req, 0.0, ice_a)), 0.0
        if mode == BlendMode.PERFORMANCE:
            # Motor fills first, ICE adds for boost
            m = float(np.clip(req, 0.0, abs(mot_a)))
            ice = float(np.clip(req - m, 0.0, ice_a))
            return ice, m
        if mode == BlendMode.ECONOMY:
            # Prefer ICE at efficient mid-load if available; else motor
            if req < 0.3 * ice_a and ice_a > 0:
                return float(np.clip(req, 0.0, ice_a)), 0.0
            m = float(np.clip(req * 0.4, 0.0, abs(mot_a)))
            ice = float(np.clip(req - m, 0.0, ice_a))
            return ice, m
        # HYBRID default: split
        if req <= 0:
            return 0.0, float(np.clip(req, -abs(mot_a), 0.0))
        share_m = 0.45
        m = float(np.clip(req * share_m, 0.0, abs(mot_a)))
        ice = float(np.clip(req - m, 0.0, ice_a))
        return ice, m
