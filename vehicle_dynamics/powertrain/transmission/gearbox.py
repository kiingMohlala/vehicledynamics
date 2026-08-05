"""Manual / sequential gearbox state."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import numpy as np

from .gear_ratios import GearRatios, default_ratios


class GearboxType(str, Enum):
    MANUAL = "manual"
    SEQUENTIAL = "sequential"


@dataclass
class Gearbox:
    ratios: GearRatios
    gtype: GearboxType = GearboxType.SEQUENTIAL
    current_gear: int = 0          # 0=N, -1=R, 1..n forward
    omega_input: float = 0.0       # rad/s after clutch
    omega_output: float = 0.0      # after final drive (wheel side total)
    torque_output: float = 0.0

    def set_gear(self, gear: int) -> None:
        if gear < 0:
            self.current_gear = -1
        elif gear == 0:
            self.current_gear = 0
        else:
            self.current_gear = int(np.clip(gear, 0, self.ratios.n_forward))

    def apply(self, torque_in: float, omega_in: float) -> tuple[float, float]:
        """Return (wheel_torque, gearbox_input_omega reference from wheels inverted)."""
        g = self.current_gear
        if g == 0:
            self.omega_input = omega_in
            self.omega_output = 0.0
            self.torque_output = 0.0
            return 0.0, omega_in
        t_out = self.ratios.output_torque(torque_in, g)
        # omega_out if input spinning (for reporting)
        w_out = self.ratios.output_omega(omega_in, g)
        self.omega_input = omega_in
        self.omega_output = w_out
        self.torque_output = t_out
        return t_out, omega_in

    def input_omega_from_output(self, omega_wheel: float) -> float:
        r = self.ratios.overall(self.current_gear)
        if abs(r) < 1e-9:
            return 0.0
        return omega_wheel * r
