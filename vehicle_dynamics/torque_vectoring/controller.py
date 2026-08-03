"""
Active rear torque-vectoring controller.

Uses the same yaw-reference structure as ESC, but redistributes drive
torque instead of applying brakes.
"""

from __future__ import annotations

import numpy as np
from .parameters import TVParameters
from .differential import distribute_drive


class TorqueVectoringController:
    def __init__(self, wheelbase: float, params: TVParameters = None):
        self.L = float(wheelbase)
        self.p = params or TVParameters()
        self._r_ref = 0.0
        self._r_prev = 0.0
        self.active = False

    def reset(self):
        self._r_ref = 0.0
        self._r_prev = 0.0
        self.active = False

    def _r_ref_ss(self, vx: float, delta: float) -> float:
        vx = max(float(vx), 0.5)
        denom = self.L * (1.0 + self.p.understeer_grad * vx * vx)
        return float(np.clip(vx * delta / denom, -self.p.r_ref_max, self.p.r_ref_max))

    def update(
        self,
        vx: float,
        vy: float,
        r: float,
        delta: float,
        throttle: float,
        dt: float,
    ) -> tuple[np.ndarray, dict]:
        """
        Returns
        -------
        T_drive : (4,) drive torque per wheel [N·m]
        diag : dict
        """
        dt = max(float(dt), 1e-4)
        r_ss = self._r_ref_ss(vx, delta)
        # simple lag
        alpha = 1.0 - np.exp(-dt / 0.1)
        self._r_ref += alpha * (r_ss - self._r_ref)
        e_r = r - self._r_ref
        r_dot = (r - self._r_prev) / dt
        self._r_prev = r

        rear_delta_T = 0.0
        self.active = False

        if (
            self.p.mode == "active_rear"
            and throttle >= self.p.min_throttle
            and vx >= self.p.min_speed
            and abs(e_r) > self.p.yaw_deadband
        ):
            self.active = True
            # Positive e_r (too much left yaw) → reduce left / increase right
            # delta_T = T_L - T_R; negative delta_T puts more torque on right
            rear_delta_T = float(np.clip(
                -self.p.Kp_yaw * e_r - self.p.Kd_yaw * r_dot,
                -self.p.max_delta_T,
                self.p.max_delta_T,
            ))

        T_drive = distribute_drive(throttle, self.p, rear_delta_T=rear_delta_T)

        diag = {
            "active": self.active,
            "r_ref": self._r_ref,
            "e_r": e_r,
            "rear_delta_T": rear_delta_T,
            "T_drive": T_drive.copy(),
            "throttle": float(np.clip(throttle, 0.0, 1.0)),
        }
        return T_drive, diag
