"""
Optional combined load-transfer feedback layer (Phase 6.7).

Applies jacking on top of existing static + lateral + longitudinal Fz.
"""

from __future__ import annotations

import numpy as np
from .jacking import compute_jacking, apply_jacking_to_loads
from .jacking_state import JackingParams, JackingState


class JackingFeedback:
    """
    Stateful optional layer.

    disabled → identity on Fz (Phase 6.6 regression).
    """

    def __init__(self, params: JackingParams = None):
        self.params = params or JackingParams()
        self.last = JackingState()

    def update(
        self,
        Fz: np.ndarray,
        Fy_fl: float,
        Fy_fr: float,
        Fy_rl: float,
        Fy_rr: float,
        rc_front: float,
        rc_rear: float,
    ) -> np.ndarray:
        self.last = compute_jacking(
            Fy_fl, Fy_fr, Fy_rl, Fy_rr,
            rc_front, rc_rear,
            self.params,
        )
        if not self.params.enabled:
            return np.asarray(Fz, dtype=float).reshape(4)
        return apply_jacking_to_loads(Fz, self.last, self.params.Fz_min)
