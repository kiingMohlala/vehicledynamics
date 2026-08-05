"""Relaxation length parameters (Phase 7.4)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RelaxationParams:
    """
    First-order lag on slip inputs before the steady-state tire model.

    kappa_dot_eff = (Vx / Lx) * (kappa - kappa_eff)
    alpha_dot_eff = (Vx / Ly) * (alpha - alpha_eff)

    enabled=False or L≈0 → instantaneous (baseline regression).
    """

    enabled: bool = True
    Lx: float = 0.30   # longitudinal relaxation length [m]
    Ly: float = 0.50   # lateral relaxation length [m]
    v_eps: float = 0.5  # minimum |Vx| used in the rate [m/s]


def disabled() -> RelaxationParams:
    return RelaxationParams(enabled=False)


def default_passenger() -> RelaxationParams:
    return RelaxationParams()
