"""Crosswind side force and moments."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class CrosswindLoads:
    Fy: float = 0.0
    Mx: float = 0.0   # roll
    Mz: float = 0.0   # yaw
    beta_aero: float = 0.0
    q: float = 0.0


def compute_crosswind_loads(
    q: float,
    beta: float,
    *,
    S: float = 1.9,
    L: float = 2.7,
    track: float = 1.55,
    Cy_beta: float = -0.8,
    Cn_beta: float = -0.15,
    Cl_beta: float = -0.05,  # roll from sideslip
) -> CrosswindLoads:
    """
    Fy = Cy(β) q S
    Mz = Cn(β) q S L
    Mx = Cl(β) q S * track
    """
    Cy = Cy_beta * beta
    Cn = Cn_beta * beta
    Cl = Cl_beta * beta
    return CrosswindLoads(
        Fy=Cy * q * S,
        Mx=Cl * q * S * track,
        Mz=Cn * q * S * L,
        beta_aero=beta,
        q=q,
    )
