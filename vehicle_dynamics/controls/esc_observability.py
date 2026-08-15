"""
Phase 15.1 — ESC Observability & Reference Yaw Model.

Observes the frozen passive plant. Does NOT command brakes, torque, or ARB.

Reference model:
  kinematic:  r_kin = vx / L * tan(δ)     (neutral bicycle)
  understeer: r_ref = r_kin / (1 + K_us * vx²)

K_us is derived from the frozen plant's measured steering gradient
(dδ/d(ay) from Phase 14.9.8), not an arbitrary neutral assumption.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class ESCObservation:
    """Snapshot of signals an ESC would observe — no actuator output."""

    t: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    beta: float = 0.0          # atan2(vy, vx)
    r: float = 0.0             # measured yaw rate
    ay: float = 0.0
    ax: float = 0.0
    delta: float = 0.0         # actual road-wheel centreline approx
    delta_fl: float = 0.0
    delta_fr: float = 0.0
    r_kin: float = 0.0         # neutral kinematic reference
    r_ref: float = 0.0         # understeer-corrected reference
    e_r: float = 0.0           # r - r_ref  (positive: more yaw than ref)
    beta_ref: float = 0.0      # optional kinematic sideslip estimate
    e_beta: float = 0.0
    eligible: bool = False     # intervention eligibility (observe only)
    util_max: float = 0.0
    Fz: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    Fx: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    Fy: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])


@dataclass
class ReferenceYawConfig:
    """
    Understeer gradient K_us [rad · s² / m²] such that:

        δ_req ≈ δ_kin + K_us · ay
        r_ref = (vx / L) · tan(δ) / (1 + K_us · vx²)

    From 14.9.8 hypercar: dδ/d(ay) ≈ 0.0065 rad/(m/s²).
    For a bicycle model, K_us ≈ dδ/d(ay) when ay ≈ r·vx in steady state.
    """
    wheelbase: float = 2.70
    # Understeer gradient from frozen 14.9.8 characterization
    K_us: float = 0.0065
    v_eps: float = 0.5          # m/s — below this, r_ref → 0
    r_ref_max: float = 2.5      # rad/s clamp
    # Eligibility thresholds (observation only — no actuation in 15.1)
    min_speed_eligible: float = 5.0
    min_steer_eligible: float = 0.01


class ESCObservability:
    """
    Read-only observer + reference model.

    step() never returns brake or drive commands.
    """

    def __init__(self, cfg: ReferenceYawConfig | None = None):
        self.cfg = cfg or ReferenceYawConfig()
        self.last = ESCObservation()
        self.history: list[ESCObservation] = []

    def reset(self) -> None:
        self.last = ESCObservation()
        self.history.clear()

    def compute_beta(self, vx: float, vy: float) -> float:
        if abs(vx) < self.cfg.v_eps and abs(vy) < self.cfg.v_eps:
            return 0.0
        return float(np.arctan2(vy, max(abs(vx), 1e-6) * np.sign(vx) if abs(vx) > 1e-9 else 1e-6))

    def compute_r_kin(self, vx: float, delta: float) -> float:
        """Neutral bicycle kinematic yaw rate."""
        if abs(vx) < self.cfg.v_eps:
            return 0.0
        return float(vx / max(self.cfg.wheelbase, 0.1) * np.tan(delta))

    def compute_r_ref(self, vx: float, delta: float) -> float:
        """
        Understeer-corrected reference:

            r_ref = r_kin / (1 + K_us · vx²)

        As K_us → 0 → neutral. K_us > 0 → understeer (lower |r_ref|).
        """
        r_kin = self.compute_r_kin(vx, delta)
        if abs(vx) < self.cfg.v_eps:
            return 0.0
        denom = 1.0 + self.cfg.K_us * vx * vx
        r_ref = r_kin / max(denom, 1e-6)
        return float(np.clip(r_ref, -self.cfg.r_ref_max, self.cfg.r_ref_max))

    def observe(
        self,
        *,
        t: float,
        vx: float,
        vy: float,
        r: float,
        ay: float,
        ax: float = 0.0,
        delta: float = 0.0,
        delta_fl: float = 0.0,
        delta_fr: float = 0.0,
        util: list | None = None,
        Fz: list | None = None,
        Fx: list | None = None,
        Fy: list | None = None,
    ) -> ESCObservation:
        beta = self.compute_beta(vx, vy)
        r_kin = self.compute_r_kin(vx, delta)
        r_ref = self.compute_r_ref(vx, delta)
        e_r = float(r - r_ref)
        # Simple kinematic β_ref for steady circular: β_ref ≈ b/L * δ - m·vx²·a/(L·C) …
        # Keep minimal: β_ref ≈ 0 at this phase (placeholder for 15.x)
        beta_ref = 0.0
        e_beta = float(beta - beta_ref)

        eligible = (
            abs(vx) >= self.cfg.min_speed_eligible
            and abs(delta) >= self.cfg.min_steer_eligible
        )
        util_max = float(max(util)) if util else 0.0

        obs = ESCObservation(
            t=t,
            vx=float(vx),
            vy=float(vy),
            beta=beta,
            r=float(r),
            ay=float(ay),
            ax=float(ax),
            delta=float(delta),
            delta_fl=float(delta_fl),
            delta_fr=float(delta_fr),
            r_kin=r_kin,
            r_ref=r_ref,
            e_r=e_r,
            beta_ref=beta_ref,
            e_beta=e_beta,
            eligible=eligible,
            util_max=util_max,
            Fz=list(Fz) if Fz is not None else [0.0] * 4,
            Fx=list(Fx) if Fx is not None else [0.0] * 4,
            Fy=list(Fy) if Fy is not None else [0.0] * 4,
        )
        self.last = obs
        self.history.append(obs)
        return obs

    def observe_from_simulation(self, sim: Any) -> ESCObservation:
        """Pull signals from a running Simulation without modifying it."""
        v = sim.state.vehicle
        d = sim.dual_track.diagnostics()
        # centreline steer ≈ mean of front road-wheel angles
        dfl = float(d.get("delta_fl", 0.0))
        dfr = float(d.get("delta_fr", 0.0))
        delta = 0.5 * (dfl + dfr)
        return self.observe(
            t=float(sim.state.time),
            vx=float(v.vx),
            vy=float(v.vy),
            r=float(v.yaw_rate),
            ay=float(v.ay),
            ax=float(v.ax),
            delta=delta,
            delta_fl=dfl,
            delta_fr=dfr,
            util=list(d.get("utilization", [0, 0, 0, 0])),
            Fz=[d.get("Fz_FL", 0), d.get("Fz_FR", 0), d.get("Fz_RL", 0), d.get("Fz_RR", 0)],
            Fx=list(d.get("Fx", [0, 0, 0, 0])),
            Fy=list(d.get("Fy", [0, 0, 0, 0])),
        )
