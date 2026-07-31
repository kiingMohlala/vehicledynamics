"""Lateral load-transfer feedback into four wheel normal loads."""

import numpy as np
from ..lateral.load_transfer import LoadTransferParameters, compute_load_transfer

def four_wheel_normal_loads(
    ay: float,
    mass: float,
    a: float,
    b: float,
    lt_params: LoadTransferParameters,
):
    """
    Static front/rear split + Phase 4.1 lateral transfer feedback.
    Returns Fz_fl, Fz_fr, Fz_rl, Fz_rr.
    No longitudinal load-transfer feedback in Phase 5.0.
    """
    L = a + b
    W = mass * 9.81
    Fz_f = W * (b / L)
    Fz_r = W * (a / L)
    lt = compute_load_transfer(ay, Fz_f, Fz_r, params=lt_params, mass=mass)
    return lt.Fz_fl, lt.Fz_fr, lt.Fz_rl, lt.Fz_rr
