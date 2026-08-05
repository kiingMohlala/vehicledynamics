"""Ride-height / rake sensitivity multipliers."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .coefficients import AeroConfig


@dataclass
class RideHeightState:
    h_front: float = 0.080   # m
    h_rear: float = 0.100
    pitch_rad: float = 0.0   # positive nose-up
    yaw_rad: float = 0.0

    @property
    def rake(self) -> float:
        """Rear − front ride height (m). Positive = higher rear."""
        return self.h_rear - self.h_front


def _clamp_smooth(x: float, lo: float, hi: float) -> float:
    return float(np.clip(x, lo, hi))


def ride_height_factors(state: RideHeightState, cfg: AeroConfig) -> dict[str, float]:
    """
    Multipliers on reference Cl_front, Cl_rear, Cd.

    Ground effect: lower ride height → more |Cl| (until stall).
    Diffuser stall: very low rear height reduces rear downforce.
    """
    hf = state.h_front
    hr = state.h_rear
    hf_ref = cfg.h_front_ref
    hr_ref = cfg.h_rear_ref

    # Front: ~ 1 / (1 + k * h/h_ref) style boost when low
    front_boost = (hf_ref + 0.02) / (hf + 0.02)
    front_boost = _clamp_smooth(front_boost, 0.5, 1.8)

    # Rear / diffuser: boost when lowered, stall below ~20 mm
    rear_boost = (hr_ref + 0.025) / (hr + 0.025)
    rear_boost = _clamp_smooth(rear_boost, 0.4, 1.9)
    if hr < 0.025:
        # Diffuser stall: sharp loss
        stall = (hr / 0.025) ** 2
        rear_boost *= stall

    # Drag rises slightly with downforce / low ride height
    cd_mult = 1.0 + 0.08 * (front_boost + rear_boost - 2.0)
    cd_mult = _clamp_smooth(cd_mult, 0.85, 1.35)

    # Pitch (nose-up positive): shifts load rearward
    pitch = state.pitch_rad
    front_pitch = 1.0 - 2.5 * pitch
    rear_pitch = 1.0 + 2.0 * pitch
    front_pitch = _clamp_smooth(front_pitch, 0.4, 1.6)
    rear_pitch = _clamp_smooth(rear_pitch, 0.4, 1.6)

    return {
        "Cl_front": front_boost * front_pitch,
        "Cl_rear": rear_boost * rear_pitch,
        "Cd": cd_mult,
        "Cy": 1.0 + 0.1 * abs(state.yaw_rad),
    }
