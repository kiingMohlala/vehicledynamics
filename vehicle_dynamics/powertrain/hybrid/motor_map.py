"""Motor torque-speed / efficiency map."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class MotorMap:
    rpm_break: np.ndarray
    torque_peak: np.ndarray       # N·m at each rpm
    eta_peak: float = 0.94

    def peak_torque(self, rpm: float) -> float:
        rpm = float(np.clip(rpm, self.rpm_break[0], self.rpm_break[-1]))
        return float(np.interp(rpm, self.rpm_break, self.torque_peak))

    def efficiency(self, rpm: float, torque: float) -> float:
        t_pk = max(self.peak_torque(rpm), 1.0)
        load = abs(torque) / t_pk
        # Peak eta near 0.5–0.8 load
        eta = self.eta_peak * (0.7 + 0.3 * float(np.clip(1.0 - abs(load - 0.6), 0, 1)))
        return float(np.clip(eta, 0.70, self.eta_peak))


def default_motor_map(peak_torque: float = 350.0, peak_rpm: float = 12000.0) -> MotorMap:
    rpm = np.array([0, 1000, 3000, 5000, 8000, 10000, 12000], dtype=float)
    # Flat torque then power-limited
    t = np.array([peak_torque, peak_torque, peak_torque, peak_torque * 0.85,
                  peak_torque * 0.55, peak_torque * 0.40, peak_torque * 0.30])
    rpm = np.clip(rpm, 0, peak_rpm)
    return MotorMap(rpm_break=rpm, torque_peak=t)
