"""
Transient tire wrapper (Phase 7.4).

Wraps any steady-state tire that implements:

    longitudinal_lateral_force(slip_ratio, slip_angle, normal_load,
                               camber_rad=0.0) -> TireState

When relaxation is disabled (or L→0), forces match the inner model exactly.
"""

from __future__ import annotations

from .relaxation import step_relaxation
from .relaxation_parameters import RelaxationParams, disabled
from .relaxation_state import RelaxationState
from .dugoff import TireState


class TransientTire:
    """
    Stateful wrapper: measured slips → relaxation → steady-state tire.

    Call `update(...)` once per simulation step with dt and vx.
    `longitudinal_lateral_force` still available for steady-state queries
    (bypasses relaxation, uses instantaneous slips).
    """

    def __init__(self, steady_tire, params: RelaxationParams | None = None):
        self.steady = steady_tire
        self.params = params if params is not None else RelaxationParams()
        self.state = RelaxationState()

    def reset(self, kappa: float = 0.0, alpha: float = 0.0) -> None:
        self.state = RelaxationState(float(kappa), float(alpha))

    def longitudinal_lateral_force(
        self,
        slip_ratio: float,
        slip_angle: float,
        normal_load: float,
        camber_rad: float = 0.0,
    ) -> TireState:
        """Steady-state path (no lag) — preserves TireModel API."""
        return self._call_steady(slip_ratio, slip_angle, normal_load, camber_rad)

    def update(
        self,
        slip_ratio: float,
        slip_angle: float,
        normal_load: float,
        vx: float,
        dt: float,
        camber_rad: float = 0.0,
    ) -> TireState:
        """Advance relaxation state and evaluate forces at effective slips."""
        self.state = step_relaxation(
            self.state, slip_ratio, slip_angle, vx, dt, self.params
        )
        return self._call_steady(
            self.state.kappa_eff,
            self.state.alpha_eff,
            normal_load,
            camber_rad,
        )

    def _call_steady(
        self,
        kappa: float,
        alpha: float,
        Fz: float,
        camber_rad: float,
    ) -> TireState:
        # Support both signatures (with/without camber_rad)
        try:
            return self.steady.longitudinal_lateral_force(
                kappa, alpha, Fz, camber_rad=camber_rad
            )
        except TypeError:
            return self.steady.longitudinal_lateral_force(kappa, alpha, Fz)
