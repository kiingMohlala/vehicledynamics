"""
ESC yaw-moment controller with hysteresis and sideslip assist.
"""

from __future__ import annotations

import numpy as np
from .parameters import ESCParameters
from .reference_model import YawReferenceModel


class ESCController:
    def __init__(self, wheelbase: float, params: ESCParameters = None):
        self.p = params or ESCParameters()
        self.ref = YawReferenceModel(wheelbase, self.p)
        self.active = False
        self._r_prev = 0.0
        self._Mz = 0.0

    def reset(self):
        self.ref.reset()
        self.active = False
        self._r_prev = 0.0
        self._Mz = 0.0

    def update(
        self,
        vx: float,
        vy: float,
        r: float,
        delta: float,
        dt: float,
    ) -> tuple[float, dict]:
        """
        Returns
        -------
        Mz_des : float
            Desired corrective yaw moment [N·m]. Positive = +yaw (left).
        diag : dict
        """
        dt = max(float(dt), 1e-4)
        r_ref = self.ref.update(vx, delta, dt)
        e_r = r - r_ref
        r_dot = (r - self._r_prev) / dt
        self._r_prev = r

        beta = np.arctan2(vy, max(abs(vx), 0.5))

        # Hysteresis
        if not self.active:
            if abs(e_r) >= self.p.on_threshold and vx >= self.p.min_speed:
                self.active = True
        else:
            if abs(e_r) <= self.p.off_threshold or vx < self.p.min_speed:
                self.active = False

        Mz = 0.0
        if self.active:
            Mz = -self.p.Kp_yaw * e_r - self.p.Kd_yaw * r_dot
            # Sideslip assist: if |β| large in same sense as r, add damping
            if abs(beta) > self.p.beta_limit:
                Mz += -0.3 * self.p.Kp_yaw * np.sign(beta) * (abs(beta) - self.p.beta_limit)
            Mz = float(np.clip(Mz, -self.p.Mz_max, self.p.Mz_max))

        self._Mz = Mz
        diag = {
            "active": self.active,
            "r_ref": r_ref,
            "e_r": e_r,
            "beta": beta,
            "Mz": Mz,
        }
        return Mz, diag
