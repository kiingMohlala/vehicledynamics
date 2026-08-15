"""
Phase 15.3 — ESC Stability Envelope & Decision Logic.

Produces a *hypothetical* ΔMz request from observed state.
Does NOT command the plant. Does NOT call BrakeAllocator.

15.1 Observation → 15.3 Decision → (future 15.4) closed loop
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from vehicle_dynamics.controls.esc_observability import ESCObservation


@dataclass
class ESCDecisionConfig:
    # Yaw-rate error hysteresis [rad/s]
    e_enter: float = 0.12
    e_exit: float = 0.06
    # Proportional request gain [N·m / (rad/s)]
    K_Mz: float = 4000.0
    max_delta_Mz: float = 6000.0
    # Inhibit thresholds
    min_vx: float = 8.0
    min_delta: float = 0.015
    max_util: float = 0.98          # near absolute limit → inhibit
    max_beta: float = 0.45          # extreme sideslip → inhibit (open-loop policy)
    # Optional soft gain reduction as util approaches limit
    util_soft_start: float = 0.85


@dataclass
class ESCDecision:
    """Hypothetical request — never applied to plant in 15.3."""

    active: bool = False
    delta_Mz_request: float = 0.0
    reason: str = "idle"
    e_r: float = 0.0
    inhibited: bool = False


class ESCDecisionLogic:
    """
    Open-loop decision policy with hysteresis.

    Sign convention (matches 15.2 plant geometry):
      e_r > 0  (r > r_ref) → excess yaw → request −ΔMz
      e_r < 0  (r < r_ref) → insufficient yaw → request +ΔMz
    """

    def __init__(self, cfg: ESCDecisionConfig | None = None):
        self.cfg = cfg or ESCDecisionConfig()
        self._active = False
        self.last = ESCDecision()

    def reset(self) -> None:
        self._active = False
        self.last = ESCDecision()

    def step(self, obs: ESCObservation) -> ESCDecision:
        cfg = self.cfg
        e = float(obs.e_r)

        # --- Inhibition (no request) ---
        if abs(obs.vx) < cfg.min_vx:
            return self._idle(e, "low_speed")
        if abs(obs.delta) < cfg.min_delta and abs(e) < cfg.e_enter:
            # allow residual-yaw correction only if already active
            if not self._active:
                return self._idle(e, "low_steer")
        if obs.util_max >= cfg.max_util:
            return self._idle(e, "util_limit", inhibited=True)
        if abs(obs.beta) >= cfg.max_beta:
            return self._idle(e, "beta_limit", inhibited=True)

        # --- Hysteresis ---
        if self._active:
            if abs(e) < cfg.e_exit:
                self._active = False
                return self._idle(e, "exit_deadband")
        else:
            if abs(e) < cfg.e_enter:
                return self._idle(e, "enter_deadband")
            self._active = True

        # --- Direction & magnitude ---
        # e > 0 → −ΔMz ; e < 0 → +ΔMz
        raw = -cfg.K_Mz * e
        # Soft util taper
        if obs.util_max > cfg.util_soft_start:
            taper = (cfg.max_util - obs.util_max) / max(cfg.max_util - cfg.util_soft_start, 1e-6)
            taper = float(np.clip(taper, 0.0, 1.0))
            raw *= taper
        M = float(np.clip(raw, -cfg.max_delta_Mz, cfg.max_delta_Mz))

        dec = ESCDecision(
            active=True,
            delta_Mz_request=M,
            reason="correct",
            e_r=e,
            inhibited=False,
        )
        self.last = dec
        return dec

    def _idle(self, e: float, reason: str, inhibited: bool = False) -> ESCDecision:
        if reason in ("util_limit", "beta_limit", "low_speed"):
            self._active = False
        dec = ESCDecision(
            active=False,
            delta_Mz_request=0.0,
            reason=reason,
            e_r=e,
            inhibited=inhibited,
        )
        self.last = dec
        return dec
