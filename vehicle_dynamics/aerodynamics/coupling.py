"""Apply aero loads to axle tire normals and longitudinal force."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .aero_solver import AeroResult
from vehicle_dynamics.lateral.load_transfer import (
    LoadTransferParameters,
    LoadTransferState,
    compute_load_transfer,
)


@dataclass
class CoupledAxleLoads:
    """Axle and per-wheel vertical loads including aero and lateral transfer."""

    Fz_f_axle: float
    Fz_r_axle: float
    Fz_fl: float
    Fz_fr: float
    Fz_rl: float
    Fz_rr: float
    dFz_aero_f: float
    dFz_aero_r: float
    Fx_aero: float
    Fy_aero: float


def static_axle_loads(mass: float, a: float, b: float, g: float = 9.81) -> tuple[float, float]:
    L = a + b
    W = mass * g
    Fz_f = W * (b / L)
    Fz_r = W * (a / L)
    return Fz_f, Fz_r


def couple_aero_to_tires(
    aero: AeroResult,
    mass: float,
    a: float,
    b: float,
    ay: float = 0.0,
    lt_params: LoadTransferParameters | None = None,
    g: float = 9.81,
) -> CoupledAxleLoads:
    """
    Static weight + aero downforce + lateral load transfer → wheel loads.

    Fz_f_axle = static_front + dFz_aero_front
    Fz_r_axle = static_rear  + dFz_aero_rear
    """
    Fz_f0, Fz_r0 = static_axle_loads(mass, a, b, g=g)
    dF_f = aero.dFz_front if aero.config.enabled else 0.0
    dF_r = aero.dFz_rear if aero.config.enabled else 0.0
    Fz_f = Fz_f0 + dF_f
    Fz_r = Fz_r0 + dF_r

    lt = compute_load_transfer(
        ay, Fz_f, Fz_r, params=lt_params, mass=mass
    )
    return CoupledAxleLoads(
        Fz_f_axle=Fz_f,
        Fz_r_axle=Fz_r,
        Fz_fl=lt.Fz_fl,
        Fz_fr=lt.Fz_fr,
        Fz_rl=lt.Fz_rl,
        Fz_rr=lt.Fz_rr,
        dFz_aero_f=dF_f,
        dFz_aero_r=dF_r,
        Fx_aero=-aero.drag_force if aero.config.enabled else 0.0,
        Fy_aero=aero.side_force if aero.config.enabled else 0.0,
    )
