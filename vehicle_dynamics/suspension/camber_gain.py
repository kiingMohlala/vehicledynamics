"""
Phase 6.4 – Camber gain kinematics.

Linear model (initial):
  camber_gain_i = gain_i * z_wheel_i
  camber_total  = camber_static + camber_gain

Camber is logged only — Dugoff tire forces are NOT modified in this phase.
Interface is ready for a geometric hardpoint-based replacement later.
"""

from __future__ import annotations

import numpy as np
from .camber_state import CamberGainParams, CamberState


def compute_camber_gain(
    wheel_travel: np.ndarray,
    params: CamberGainParams = None,
) -> np.ndarray:
    """
    Parameters
    ----------
    wheel_travel : (4,) wheel vertical travel [m], + = compression (bump)
    params : linear gains [rad/m]

    Returns
    -------
    camber_gain : (4,) radians
    """
    params = params or CamberGainParams.neutral()
    z = np.asarray(wheel_travel, dtype=float).reshape(4)
    return params.as_array() * z


def update_camber_state(
    wheel_travel: np.ndarray,
    camber_static: np.ndarray,
    params: CamberGainParams = None,
) -> CamberState:
    """Full camber state for diagnostics."""
    params = params or CamberGainParams.neutral()
    z = np.asarray(wheel_travel, dtype=float).reshape(4)
    cs = np.asarray(camber_static, dtype=float).reshape(4)
    cg = compute_camber_gain(z, params)
    return CamberState(
        wheel_travel=z.copy(),
        camber_static=cs.copy(),
        camber_gain=cg,
        camber_total=cs + cg,
    )


class CamberGainModel:
    """Stateful wrapper used by SuspensionInterface."""

    def __init__(self, params: CamberGainParams = None):
        self.params = params or CamberGainParams.neutral()
        self.last: CamberState = CamberState()

    def reset(self):
        self.last = CamberState()

    def evaluate(
        self,
        wheel_travel: np.ndarray,
        camber_static: np.ndarray,
    ) -> CamberState:
        self.last = update_camber_state(wheel_travel, camber_static, self.params)
        return self.last
