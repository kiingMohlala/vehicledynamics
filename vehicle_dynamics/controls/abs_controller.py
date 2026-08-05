"""Four-channel ABS (command layer — does not embed brake physics)."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .sensor_model import SensorReading


@dataclass
class ABSController:
    target_slip: float = 0.18
    release_slip: float = 0.22
    reapply_slip: float = 0.12
    mu_gain: float = 0.05
    enabled: bool = True
    pressure: np.ndarray = field(default_factory=lambda: np.ones(4))

    def step(
        self,
        sensors: SensorReading,
        driver_brake: float,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """
        Returns (brake_pressures 0..1, active flags, mu_est).
        """
        if not self.enabled or driver_brake < 0.05:
            self.pressure = np.ones(4) * driver_brake
            return self.pressure.copy(), np.zeros(4, dtype=bool), 1.0

        mu_est = 1.0
        active = np.zeros(4, dtype=bool)
        for i in range(4):
            s = abs(sensors.slip_ratio[i])
            # Adaptive target with simple mu proxy from peak slip usage
            tgt = self.target_slip * (0.7 + 0.3 * mu_est)
            p = self.pressure[i]
            if s > self.release_slip:
                p = max(0.1, p - 0.5 * dt)  # release
                active[i] = True
            elif s > tgt:
                p = max(0.2, p - 0.2 * dt)  # hold/reduce
                active[i] = True
            elif s < self.reapply_slip:
                p = min(1.0, p + 0.4 * dt)  # build
            p = float(np.clip(p * driver_brake + (1 - driver_brake) * 0, 0.0, driver_brake))
            # Blend with driver demand
            p = min(driver_brake, max(p, driver_brake * 0.15 if active[i] else driver_brake))
            if not active[i]:
                p = driver_brake
            self.pressure[i] = p
            if s > 0.25:
                mu_est = max(0.3, mu_est - self.mu_gain * dt)

        return self.pressure.copy(), active, float(mu_est)
