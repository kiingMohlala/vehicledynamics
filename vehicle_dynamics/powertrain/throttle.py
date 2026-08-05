"""Pedal filtering and throttle-body lag."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class ThrottleState:
    pedal: float = 0.0
    throttle: float = 0.0


@dataclass
class ThrottleModel:
    tau: float = 0.05           # s body lag
    pedal_deadband: float = 0.02
    gamma: float = 1.2          # pedal map exponent (>1 = softer tip-in)

    def pedal_map(self, pedal: float) -> float:
        p = float(np.clip(pedal, 0.0, 1.0))
        if p < self.pedal_deadband:
            return 0.0
        p = (p - self.pedal_deadband) / (1.0 - self.pedal_deadband)
        return float(p ** self.gamma)

    def step(self, pedal: float, state: ThrottleState, dt: float) -> ThrottleState:
        target = self.pedal_map(pedal)
        if dt <= 0 or self.tau <= 0:
            thr = target
        else:
            a = 1.0 - np.exp(-dt / self.tau)
            thr = state.throttle + a * (target - state.throttle)
        return ThrottleState(pedal=float(np.clip(pedal, 0, 1)), throttle=float(np.clip(thr, 0, 1)))
