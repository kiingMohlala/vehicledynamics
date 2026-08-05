from dataclasses import dataclass
import numpy as np

@dataclass
class ABSParams:
    target_slip: float = 0.18
    slip_threshold_high: float = 0.22
    slip_threshold_low: float = 0.12
    pressure_release_rate: float = 40.0   # 1/s
    pressure_build_rate: float = 15.0     # 1/s
    min_pressure: float = 0.05
    valve_tau: float = 0.015

class ABSController:
    def __init__(self, params: ABSParams = None):
        self.p = params or ABSParams()
        self.pressure = 1.0
        self.state = "build"

    def update(self, slip: float, dt: float) -> float:
        if self.state == "build":
            if slip > self.p.slip_threshold_high:
                self.state = "release"
        elif self.state == "release":
            if slip < self.p.slip_threshold_low:
                self.state = "build"
            else:
                self.state = "hold"
        elif self.state == "hold":
            if slip > self.p.slip_threshold_high:
                self.state = "release"
            elif slip < self.p.slip_threshold_low:
                self.state = "build"

        if self.state == "release":
            target = max(self.p.min_pressure, self.pressure - self.p.pressure_release_rate * dt)
        elif self.state == "build":
            target = min(1.0, self.pressure + self.p.pressure_build_rate * dt)
        else:
            target = self.pressure

        alpha = 1.0 - np.exp(-dt / self.p.valve_tau)
        self.pressure += alpha * (target - self.pressure)
        self.pressure = np.clip(self.pressure, self.p.min_pressure, 1.0)
        return self.pressure
