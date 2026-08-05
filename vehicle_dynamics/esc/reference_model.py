"""
Desired yaw-rate reference from a linear bicycle steady-state model
with optional first-order lag.
"""

from __future__ import annotations

import numpy as np
from .parameters import ESCParameters


class YawReferenceModel:
    def __init__(self, wheelbase: float, params: ESCParameters = None):
        self.L = float(wheelbase)
        self.p = params or ESCParameters()
        self._r_ref = 0.0

    def reset(self):
        self._r_ref = 0.0

    def steady_state(self, vx: float, delta: float) -> float:
        """r_ss = vx * δ / (L * (1 + K_us * vx²))."""
        vx = max(float(vx), 0.5)
        denom = self.L * (1.0 + self.p.understeer_grad * vx * vx)
        r_ss = vx * delta / denom
        return float(np.clip(r_ss, -self.p.r_ref_max, self.p.r_ref_max))

    def update(self, vx: float, delta: float, dt: float) -> float:
        r_ss = self.steady_state(vx, delta)
        if self.p.r_ref_tau <= 1e-6:
            self._r_ref = r_ss
        else:
            alpha = 1.0 - np.exp(-dt / self.p.r_ref_tau)
            self._r_ref += alpha * (r_ss - self._r_ref)
        return self._r_ref

    @property
    def r_ref(self) -> float:
        return self._r_ref
