"""Engine torque map (RPM × throttle → torque)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class EngineMap:
    rpm: np.ndarray          # 1D
    throttle: np.ndarray     # 1D 0..1
    torque: np.ndarray       # 2D [n_rpm, n_thr] N·m

    def torque_at(self, rpm: float, throttle: float) -> float:
        rpm = float(np.clip(rpm, self.rpm[0], self.rpm[-1]))
        thr = float(np.clip(throttle, 0.0, 1.0))
        # Bilinear interpolation
        i = int(np.searchsorted(self.rpm, rpm) - 1)
        j = int(np.searchsorted(self.throttle, thr) - 1)
        i = int(np.clip(i, 0, len(self.rpm) - 2))
        j = int(np.clip(j, 0, len(self.throttle) - 2))
        r0, r1 = self.rpm[i], self.rpm[i + 1]
        t0, t1 = self.throttle[j], self.throttle[j + 1]
        fr = 0.0 if r1 <= r0 else (rpm - r0) / (r1 - r0)
        ft = 0.0 if t1 <= t0 else (thr - t0) / (t1 - t0)
        q00 = self.torque[i, j]
        q01 = self.torque[i, j + 1]
        q10 = self.torque[i + 1, j]
        q11 = self.torque[i + 1, j + 1]
        q0 = q00 * (1 - ft) + q01 * ft
        q1 = q10 * (1 - ft) + q11 * ft
        return float(q0 * (1 - fr) + q1 * fr)

    def power_kw(self, rpm: float, throttle: float) -> float:
        tq = self.torque_at(rpm, throttle)
        omega = rpm * 2.0 * np.pi / 60.0
        return tq * omega / 1000.0


def default_na_map(
    idle_rpm: float = 900.0,
    redline_rpm: float = 7500.0,
    peak_torque: float = 400.0,
    peak_torque_rpm: float = 4500.0,
) -> EngineMap:
    """Synthetic NA torque map peaking near peak_torque_rpm."""
    rpm = np.linspace(idle_rpm * 0.7, redline_rpm * 1.05, 25)
    thr = np.array([0.0, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0])
    torque = np.zeros((len(rpm), len(thr)))
    for i, r in enumerate(rpm):
        # Bell-shaped WOT curve
        x = (r - peak_torque_rpm) / (0.45 * peak_torque_rpm)
        wot = peak_torque * np.exp(-0.5 * x * x)
        # Fall toward redline
        if r > peak_torque_rpm:
            wot *= max(0.55, 1.0 - 0.35 * (r - peak_torque_rpm) / (redline_rpm - peak_torque_rpm + 1e-9))
        if r < idle_rpm:
            wot *= 0.6
        for j, t in enumerate(thr):
            # Nonlinear throttle fill
            fill = t ** 0.85
            friction = 25.0 + 0.008 * r  # closed-throttle friction proxy at thr=0
            if t < 1e-6:
                torque[i, j] = -friction * 0.3  # mild negative at closed throttle in map
            else:
                torque[i, j] = wot * fill - friction * (1.0 - fill) * 0.15
    return EngineMap(rpm=rpm, throttle=thr, torque=torque)
