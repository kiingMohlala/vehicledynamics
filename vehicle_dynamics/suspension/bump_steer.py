"""
Phase 6.3 – Bump-steer kinematics.

Linear model (initial):
  toe_bump_i = gain_i * z_wheel_i

Interface is ready for a geometric hardpoint-based replacement later
without changing the dual-track integration path.

  δ_effective = δ_command + toe_static + toe_bump
"""

from __future__ import annotations

import numpy as np
from .bump_state import BumpSteerParams, BumpSteerState


def compute_toe_bump(
    wheel_travel: np.ndarray,
    params: BumpSteerParams = None,
) -> np.ndarray:
    """
    Parameters
    ----------
    wheel_travel : (4,) wheel vertical travel [m], + = compression (bump)
    params : linear gains [rad/m]

    Returns
    -------
    toe_bump : (4,) radians
    """
    params = params or BumpSteerParams.neutral()
    z = np.asarray(wheel_travel, dtype=float).reshape(4)
    return params.as_array() * z


def update_bump_state(
    wheel_travel: np.ndarray,
    toe_static: np.ndarray,
    params: BumpSteerParams = None,
) -> BumpSteerState:
    """Full bump-steer state for diagnostics and steering integration."""
    params = params or BumpSteerParams.neutral()
    z = np.asarray(wheel_travel, dtype=float).reshape(4)
    toe_s = np.asarray(toe_static, dtype=float).reshape(4)
    toe_b = compute_toe_bump(z, params)
    return BumpSteerState(
        wheel_travel=z.copy(),
        toe_bump=toe_b,
        toe_static=toe_s.copy(),
        toe_total=toe_s + toe_b,
    )


class BumpSteerModel:
    """Stateful wrapper used by SuspensionInterface."""

    def __init__(self, params: BumpSteerParams = None):
        self.params = params or BumpSteerParams.neutral()
        self.last: BumpSteerState = BumpSteerState()

    def reset(self):
        self.last = BumpSteerState()

    def evaluate(
        self,
        wheel_travel: np.ndarray,
        toe_static: np.ndarray,
    ) -> BumpSteerState:
        self.last = update_bump_state(wheel_travel, toe_static, self.params)
        return self.last
