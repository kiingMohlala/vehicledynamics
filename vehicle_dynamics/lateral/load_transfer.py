"""
Phase 4.1 – Quasi-static lateral load transfer diagnostics.

Level A: compute left/right wheel loads for diagnostics only.
Bicycle axle totals remain unchanged (Phase 4.0 behaviour preserved).
"""

from dataclasses import dataclass
import numpy as np

@dataclass
class LoadTransferParameters:
    h_cg: float = 0.55       # CG height [m]
    track_f: float = 1.55    # front track [m]
    track_r: float = 1.55    # rear track [m]
    chi_f: float = 0.55      # front roll-stiffness ratio [-]
    Fz_min: float = 50.0     # minimum normal load [N]

@dataclass
class LoadTransferState:
    dFz_front: float         # front transfer magnitude [N] (positive ay → positive = load to right)
    dFz_rear: float          # rear transfer magnitude [N]
    Fz_fl: float             # front-left normal load [N]
    Fz_fr: float             # front-right normal load [N]
    Fz_rl: float             # rear-left normal load [N]
    Fz_rr: float             # rear-right normal load [N]
    wheel_lift_front: bool   # True if either front wheel would lift (clamped)
    wheel_lift_rear: bool    # True if either rear wheel would lift (clamped)


def compute_load_transfer(
    ay: float,
    Fz_f_axle: float,
    Fz_r_axle: float,
    params: LoadTransferParameters = None,
    mass: float = 1400.0,
) -> LoadTransferState:
    """
    Quasi-static lateral load transfer.

    Positive ay (to the left) unloads the left side and loads the right side.

    Parameters
    ----------
    ay : float
        Lateral acceleration at CG [m/s²] (prefer ay_force).
    Fz_f_axle, Fz_r_axle : float
        Static (or current) axle total normal loads [N].
    params : LoadTransferParameters
    mass : float
        Vehicle mass [kg] (used only for documentation / future extensions).
    """
    if params is None:
        params = LoadTransferParameters()

    # Total lateral transfer using average track
    track_avg = 0.5 * (params.track_f + params.track_r)
    # ΔF_total related to m*ay*h/t but we distribute by axle using chi_f
    # Per-axle transfer:
    dFz_f = (mass * ay * params.h_cg / params.track_f) * params.chi_f
    dFz_r = (mass * ay * params.h_cg / params.track_r) * (1.0 - params.chi_f)

    # Left / right split (positive ay → right gains load)
    Fz_fl = Fz_f_axle / 2.0 - dFz_f
    Fz_fr = Fz_f_axle / 2.0 + dFz_f
    Fz_rl = Fz_r_axle / 2.0 - dFz_r
    Fz_rr = Fz_r_axle / 2.0 + dFz_r

    # Clamp and detect lift
    lift_f = False
    lift_r = False
    if Fz_fl < params.Fz_min:
        Fz_fl = params.Fz_min
        lift_f = True
    if Fz_fr < params.Fz_min:
        Fz_fr = params.Fz_min
        lift_f = True
    if Fz_rl < params.Fz_min:
        Fz_rl = params.Fz_min
        lift_r = True
    if Fz_rr < params.Fz_min:
        Fz_rr = params.Fz_min
        lift_r = True

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
