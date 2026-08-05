"""
Four-wheel normal loads with lateral + longitudinal transfer (Phase 5.6).

Order:
  1. Static front/rear split
  2. Longitudinal transfer from ax (accel/brake)
  3. Lateral transfer from ay on the updated axle loads

Sign convention:
  ax > 0  → acceleration (load to rear)
  ax < 0  → braking (load to front)
  ay      → same as Phase 4.1 / 5.0 lateral transfer
"""

from __future__ import annotations

import numpy as np
from ..lateral.load_transfer import LoadTransferParameters, compute_load_transfer


def longitudinal_axle_loads(
    ax: float,
    mass: float,
    a: float,
    b: float,
    h_cg: float | None = None,
    lt_params: LoadTransferParameters | None = None,
) -> tuple[float, float]:
    """
    Quasi-static longitudinal weight transfer.

    ΔFz = m * ax * h_cg / L
    Fz_f = W * b/L - ΔFz
    Fz_r = W * a/L + ΔFz
    """
    L = a + b
    W = mass * 9.81
    if h_cg is None:
        h_cg = (lt_params.h_cg if lt_params is not None else 0.55)
    dFz = mass * float(ax) * float(h_cg) / L
    Fz_f = W * (b / L) - dFz
    Fz_r = W * (a / L) + dFz
    return float(Fz_f), float(Fz_r)


def four_wheel_normal_loads(
    ay: float,
    mass: float,
    a: float,
    b: float,
    lt_params: LoadTransferParameters,
    ax: float = 0.0,
):
    """
    Static split + longitudinal transfer + lateral transfer.

    Returns (Fz_fl, Fz_fr, Fz_rl, Fz_rr).

    ax = 0 → Phase 5.0 behaviour (regression).
    """
    Fz_f, Fz_r = longitudinal_axle_loads(
        ax, mass, a, b, h_cg=lt_params.h_cg, lt_params=lt_params
    )
    # Preserve non-negative axle totals before lateral split (clamp later per wheel)
    Fz_min_axle = 2.0 * lt_params.Fz_min
    Fz_f = max(Fz_f, Fz_min_axle)
    Fz_r = max(Fz_r, Fz_min_axle)
    # If both were clamped, total weight is not conserved — rare extreme ax;
    # renormalize only when needed so sum ≈ m g.
    W = mass * 9.81
    total = Fz_f + Fz_r
    if total > 1e-6 and abs(total - W) > 1.0:
        # Soft renormalize only if one axle hit floor hard
        scale = W / total
        Fz_f *= scale
        Fz_r *= scale

    lt = compute_load_transfer(ay, Fz_f, Fz_r, params=lt_params, mass=mass)
    return lt.Fz_fl, lt.Fz_fr, lt.Fz_rl, lt.Fz_rr
