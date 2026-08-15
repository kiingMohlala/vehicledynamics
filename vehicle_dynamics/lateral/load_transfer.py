"""
Explicit wheel normal-load model (Phase 14.4).

ΣFz = m·g + aero_downforce
Longitudinal transfer from ax, h_cg, wheelbase
Lateral transfer from ay, h_cg, track
Aero front/rear split from Cl balance (or explicit shares)

Authority: all geometry/mass/h_cg come from DualTrackConfig / SimulationConfig —
no silent DualTrack defaults on the authoritative path.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class LoadTransferParameters:
    h_cg: float = 0.45
    track_f: float = 1.55
    track_r: float = 1.55
    chi_f: float = 0.55          # front share of total lateral transfer
    Fz_min: float = 50.0         # floor to avoid singular tire model; documented clamp


@dataclass
class LoadTransferState:
    dFz_front: float
    dFz_rear: float
    Fz_fl: float
    Fz_fr: float
    Fz_rl: float
    Fz_rr: float
    wheel_lift_front: bool
    wheel_lift_rear: bool
    # Phase 14.4 diagnostics
    Fz_f_axle: float = 0.0
    Fz_r_axle: float = 0.0
    dFz_long_f: float = 0.0
    dFz_long_r: float = 0.0
    Fz_total: float = 0.0
    residual_Fz: float = 0.0
    clamped: bool = False


def _clamp_axle(Fz_left, Fz_right, Fz_axle, Fz_min):
    """Deterministic unload protection: floor at Fz_min, conserve axle sum when possible."""
    lift = False
    clamped = False
    if Fz_left < Fz_min:
        Fz_left = Fz_min
        Fz_right = max(Fz_axle - Fz_left, Fz_min)
        lift = True
        clamped = True
    if Fz_right < Fz_min:
        Fz_right = Fz_min
        Fz_left = max(Fz_axle - Fz_right, Fz_min)
        lift = True
        clamped = True
    if Fz_axle < 2.0 * Fz_min:
        Fz_left = Fz_min
        Fz_right = Fz_min
        lift = True
        clamped = True
    return Fz_left, Fz_right, lift, clamped


def compute_load_transfer(ay, Fz_f_axle, Fz_r_axle, params=None, mass=1400.0):
    """Lateral-only transfer (legacy API used by dual-track after axle loads set)."""
    if params is None:
        params = LoadTransferParameters()
    dFz_f = (mass * ay * params.h_cg / max(params.track_f, 1e-6)) * params.chi_f
    dFz_r = (mass * ay * params.h_cg / max(params.track_r, 1e-6)) * (1.0 - params.chi_f)
    Fz_fl = Fz_f_axle / 2.0 - dFz_f
    Fz_fr = Fz_f_axle / 2.0 + dFz_f
    Fz_rl = Fz_r_axle / 2.0 - dFz_r
    Fz_rr = Fz_r_axle / 2.0 + dFz_r
    Fz_fl, Fz_fr, lift_f, c1 = _clamp_axle(Fz_fl, Fz_fr, Fz_f_axle, params.Fz_min)
    Fz_rl, Fz_rr, lift_r, c2 = _clamp_axle(Fz_rl, Fz_rr, Fz_r_axle, params.Fz_min)
    total = Fz_fl + Fz_fr + Fz_rl + Fz_rr
    return LoadTransferState(
        dFz_f, dFz_r, Fz_fl, Fz_fr, Fz_rl, Fz_rr, lift_f, lift_r,
        Fz_f_axle=Fz_f_axle, Fz_r_axle=Fz_r_axle,
        Fz_total=total, residual_Fz=0.0, clamped=c1 or c2,
    )


def compute_wheel_loads(
    *,
    mass: float,
    a: float,
    b: float,
    h_cg: float,
    track_f: float,
    track_r: float,
    ax: float,
    ay: float,
    downforce_front: float = 0.0,
    downforce_rear: float = 0.0,
    chi_f: float = 0.55,
    Fz_min: float = 50.0,
    g: float = 9.81,
) -> LoadTransferState:
    """
    Full explicit wheel-load model.

    Static:    Fz_f = m g b/L ,  Fz_r = m g a/L
    Long:      ΔFz_f = -m ax h/L , ΔFz_r = +m ax h/L
    Aero:      added to front/rear axles by downforce_front/rear
    Lateral:   ΔFz = m ay h / track * chi shares
    """
    L = a + b
    if L < 1e-6:
        L = 1.0
    weight = mass * g
    dFz_long_f = -mass * ax * h_cg / L
    dFz_long_r = +mass * ax * h_cg / L
    Fz_f_axle = weight * b / L + dFz_long_f + downforce_front
    Fz_r_axle = weight * a / L + dFz_long_r + downforce_rear
    # Floor axles before lateral split
    Fz_f_axle = max(Fz_f_axle, 2.0 * Fz_min)
    Fz_r_axle = max(Fz_r_axle, 2.0 * Fz_min)

    params = LoadTransferParameters(
        h_cg=h_cg, track_f=track_f, track_r=track_r, chi_f=chi_f, Fz_min=Fz_min
    )
    lt = compute_load_transfer(ay, Fz_f_axle, Fz_r_axle, params, mass=mass)
    lt.dFz_long_f = dFz_long_f
    lt.dFz_long_r = dFz_long_r
    expected = weight + downforce_front + downforce_rear
    # residual vs unclamped expectation
    lt.residual_Fz = lt.Fz_total - expected
    return lt
