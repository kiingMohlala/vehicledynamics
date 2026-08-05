"""
Phase 6.7 – Roll-center jacking forces.

First-order geometric load transfer:

    dFz_axle ≈ (Fy_left + Fy_right) * h_RC / track

Sign convention (vehicle frame, +y left):
  Positive Fy (force to the left) with positive h_RC raises the left side
  relative to the right in the classic geometric sense; we apply:

    outer/inner from lateral force direction:
    dFz_left  = + Fy_axle * h_RC / track   (half applied each side via pair)

Actually the standard geometric transfer between left and right is:

    ΔFz = Fy * h_RC / t

where the outside wheel gains load for a positive roll-center height during
cornering (same sense as elastic lateral transfer for positive RC).

With +ay (left turn), Fy total is to the left (+), outside is right:
  Fz_right increases, Fz_left decreases for h_RC > 0.

  dFz_right = + Fy_axle * h_RC / track
  dFz_left  = − Fy_axle * h_RC / track

with Fy_axle = Fy_left + Fy_right (body-frame lateral, positive to left).
"""

from __future__ import annotations

import numpy as np
from .jacking_state import JackingParams, JackingState


def axle_jacking_delta(
    Fy_left: float,
    Fy_right: float,
    h_rc: float,
    track: float,
) -> tuple[float, float]:
    """
    Returns (dFz_left, dFz_right) to add to normal loads.
    """
    track = max(float(track), 1e-6)
    Fy_axle = float(Fy_left) + float(Fy_right)
    delta = Fy_axle * float(h_rc) / track
    # left loses, right gains for Fy_axle > 0, h_rc > 0
    return -delta, +delta


def compute_jacking(
    Fy_fl: float,
    Fy_fr: float,
    Fy_rl: float,
    Fy_rr: float,
    rc_front: float,
    rc_rear: float,
    params: JackingParams = None,
) -> JackingState:
    params = params or JackingParams()
    if not params.enabled:
        return JackingState(
            rc_front=float(rc_front),
            rc_rear=float(rc_rear),
            Fy_front=float(Fy_fl + Fy_fr),
            Fy_rear=float(Fy_rl + Fy_rr),
        )

    d_fl, d_fr = axle_jacking_delta(Fy_fl, Fy_fr, rc_front, params.track_f)
    d_rl, d_rr = axle_jacking_delta(Fy_rl, Fy_rr, rc_rear, params.track_r)

    return JackingState(
        rc_front=float(rc_front),
        rc_rear=float(rc_rear),
        Fy_front=float(Fy_fl + Fy_fr),
        Fy_rear=float(Fy_rl + Fy_rr),
        dFz_front=float(abs(d_fl)),  # magnitude of pair transfer
        dFz_rear=float(abs(d_rl)),
        dFz_wheels=np.array([d_fl, d_fr, d_rl, d_rr], dtype=float),
    )


def apply_jacking_to_loads(
    Fz: np.ndarray,
    jacking: JackingState,
    Fz_min: float = 50.0,
) -> np.ndarray:
    """
    Fz_out = Fz + dFz_wheels, then clamp per axle to preserve axle totals.
    """
    Fz = np.asarray(Fz, dtype=float).reshape(4).copy()
    d = jacking.dFz_wheels
    Fz = Fz + d

    def clamp_axle(i0: int, i1: int):
        total = Fz[i0] + Fz[i1]
        Fz[i0] = max(Fz[i0], Fz_min)
        Fz[i1] = total - Fz[i0]
        if Fz[i1] < Fz_min:
            Fz[i1] = Fz_min
            Fz[i0] = max(total - Fz_min, Fz_min)

    clamp_axle(0, 1)
    clamp_axle(2, 3)
    return Fz
