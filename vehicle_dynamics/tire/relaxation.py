"""
First-order relaxation ODE for tire slip inputs (Phase 7.4).

Does not compute forces — only filters κ and α.
"""

from __future__ import annotations

import numpy as np
from .relaxation_parameters import RelaxationParams
from .relaxation_state import RelaxationState


def step_relaxation(
    state: RelaxationState,
    kappa: float,
    alpha: float,
    vx: float,
    dt: float,
    params: RelaxationParams,
) -> RelaxationState:
    """
    Explicit exponential-map integration of the first-order lag.

    kappa_eff += (1 - exp(-dt/tau)) * (kappa - kappa_eff)
    with tau = L / max(|Vx|, v_eps)
    """
    if (not params.enabled) or dt <= 0.0:
        return RelaxationState(float(kappa), float(alpha))

    speed = max(abs(float(vx)), params.v_eps)

    def lag(current: float, target: float, L: float) -> float:
        L = max(float(L), 1e-6)
        tau = L / speed
        alpha_f = 1.0 - np.exp(-float(dt) / tau)
        return float(current + alpha_f * (target - current))

    return RelaxationState(
        kappa_eff=lag(state.kappa_eff, float(kappa), params.Lx),
        alpha_eff=lag(state.alpha_eff, float(alpha), params.Ly),
    )


def rates(
    state: RelaxationState,
    kappa: float,
    alpha: float,
    vx: float,
    params: RelaxationParams,
) -> tuple[float, float]:
    """Continuous-time derivatives (for diagnostics / RK integrators)."""
    if not params.enabled:
        return 0.0, 0.0
    speed = max(abs(float(vx)), params.v_eps)
    Lx = max(params.Lx, 1e-6)
    Ly = max(params.Ly, 1e-6)
    kdot = (speed / Lx) * (float(kappa) - state.kappa_eff)
    adot = (speed / Ly) * (float(alpha) - state.alpha_eff)
    return float(kdot), float(adot)
