"""
Map desired yaw moment to per-wheel brake scale factors.

Sign convention (dual-track):
  +Mz  → brake left wheels  (y > 0)
  −Mz  → brake right wheels (y < 0)

Oversteer (too much yaw in turn direction): primarily outer front.
Understeer (too little yaw): primarily inner rear.
"""

from __future__ import annotations

import numpy as np
from .parameters import ESCParameters

# FL, FR, RL, RR
FL, FR, RL, RR = 0, 1, 2, 3


class BrakeAllocator:
    def __init__(self, track_f: float, track_r: float, params: ESCParameters = None):
        self.p = params or ESCParameters()
        self.track_f = float(track_f)
        self.track_r = float(track_r)

    def allocate(
        self,
        Mz_des: float,
        delta: float,
        pedal: float = 0.0,
    ) -> np.ndarray:
        """
        Returns
        -------
        scale : (4,) in [0, max_brake_scale]
            Additional brake demand (0 = no ESC brake). Multiplied onto
            distributor output in the plant; when pedal is low, treated as
            absolute brake fraction for that wheel.
        """
        scale = np.zeros(4)
        if abs(Mz_des) < 1e-3:
            return scale

        # Approximate lever arm ~ half-track
        arm_f = 0.5 * self.track_f
        arm_r = 0.5 * self.track_r
        arm = max(0.5 * (arm_f + arm_r), 0.5)

        # Required longitudinal force magnitude at one side
        F_req = abs(Mz_des) / arm
        # Map to scale using a nominal brake force capacity (~3000 N)
        F_nom = 3000.0
        frac = float(np.clip(F_req / F_nom, 0.0, self.p.max_brake_scale))

        left_turn = delta >= 0.0  # +δ left
        # Oversteer if Mz opposes current turn (trying to reduce yaw)
        # e_r > 0 with +δ → oversteer left → Mz_des < 0 → brake right (outer)
        oversteer = (left_turn and Mz_des < 0) or ((not left_turn) and Mz_des > 0)

        if Mz_des > 0:
            # brake left
            if oversteer:
                # outer for right-turn OS is left
                scale[FL] = self.p.front_os_share * frac
                scale[RL] = (1.0 - self.p.front_os_share) * frac
            else:
                # understeer left: inner rear
                scale[RL] = self.p.rear_us_share * frac
                scale[FL] = (1.0 - self.p.rear_us_share) * frac
        else:
            # brake right
            if oversteer:
                scale[FR] = self.p.front_os_share * frac
                scale[RR] = (1.0 - self.p.front_os_share) * frac
            else:
                scale[RR] = self.p.rear_us_share * frac
                scale[FR] = (1.0 - self.p.rear_us_share) * frac

        return np.clip(scale, 0.0, self.p.max_brake_scale)
