"""
Phase 4.1 – Quasi-static lateral load transfer diagnostics.

Level A: compute left/right wheel loads for diagnostics only.
Bicycle axle totals remain unchanged (Phase 4.0 behaviour preserved).

Clamping preserves axle totals: if one wheel is clamped to Fz_min,
the opposite wheel absorbs the remainder.
"""

from dataclasses import dataclass

@dataclass
class LoadTransferParameters:
    h_cg: float = 0.55       # CG height [m]
    track_f: float = 1.55    # front track [m]
    track_r: float = 1.55    # rear track [m]
    chi_f: float = 0.55      # front roll-stiffness ratio [-]
    Fz_min: float = 50.0     # minimum normal load [N]

@dataclass
class LoadTransferState:
    dFz_front: float         # front transfer magnitude [N]
    dFz_rear: float          # rear transfer magnitude [N]
    Fz_fl: float             # front-left normal load [N]
    Fz_fr: float             # front-right normal load [N]
    Fz_rl: float             # rear-left normal load [N]
    Fz_rr: float             # rear-right normal load [N]
    wheel_lift_front: bool   # True if a front wheel was clamped
    wheel_lift_rear: bool    # True if a rear wheel was clamped


def _clamp_axle(Fz_left: float, Fz_right: float, Fz_axle: float, Fz_min: float):
    """
    Clamp individual wheels to Fz_min while preserving the axle total.
    Returns (Fz_left, Fz_right, lift_occurred).
    """
    lift = False
    if Fz_left < Fz_min:
        Fz_left = Fz_min
        Fz_right = Fz_axle - Fz_left
        lift = True
    if Fz_right < Fz_min:
        Fz_right = Fz_min
        Fz_left = Fz_axle - Fz_right
        lift = True
    # Final safety: if axle total itself is below 2*Fz_min, both sit at Fz_min
    # (axle total cannot be preserved in that degenerate case)
    if Fz_axle < 2.0 * Fz_min:
        Fz_left = Fz_min
        Fz_right = Fz_min
        lift = True
    return Fz_left, Fz_right, lift


def compute_load_transfer(
    ay: float,
    Fz_f_axle: float,
    Fz_r_axle: float,
    params: LoadTransferParameters = None,
    mass: float = 1400.0,
) -> LoadTransferState:
    """
    Quasi-static lateral load transfer diagnostics.

    Positive ay (to the left) unloads the left side and loads the right side.

    Parameters
    ----------
    ay : float
        Lateral acceleration at CG [m/s²] (prefer ay_force).
    Fz_f_axle, Fz_r_axle : float
        Axle total normal loads [N] (static in Level A).
    params : LoadTransferParameters
        Geometric and stiffness-distribution parameters.
    mass : float
        Vehicle mass [kg]. Used in the transfer formula:
        dFz ~ mass * ay * h_cg / track.
    """
    if params is None:
        params = LoadTransferParameters()

    # Per-axle transfer (chi_f distributes total transfer between axles)
    dFz_f = (mass * ay * params.h_cg / params.track_f) * params.chi_f
    dFz_r = (mass * ay * params.h_cg / params.track_r) * (1.0 - params.chi_f)

    # Left / right split before clamping
    Fz_fl = Fz_f_axle / 2.0 - dFz_f
    Fz_fr = Fz_f_axle / 2.0 + dFz_f
    Fz_rl = Fz_r_axle / 2.0 - dFz_r
    Fz_rr = Fz_r_axle / 2.0 + dFz_r

    # Clamp while preserving axle totals
    Fz_fl, Fz_fr, lift_f = _clamp_axle(Fz_fl, Fz_fr, Fz_f_axle, params.Fz_min)
    Fz_rl, Fz_rr, lift_r = _clamp_axle(Fz_rl, Fz_rr, Fz_r_axle, params.Fz_min)

    return LoadTransferState(
        dFz_front=float(dFz_f),
        dFz_rear=float(dFz_r),
        Fz_fl=float(Fz_fl),
        Fz_fr=float(Fz_fr),
        Fz_rl=float(Fz_rl),
        Fz_rr=float(Fz_rr),
        wheel_lift_front=lift_f,
        wheel_lift_rear=lift_r,
    )
