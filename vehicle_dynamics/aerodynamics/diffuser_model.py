"""Diffuser and undertray / splitter ground-effect models."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class DiffuserParams:
    area: float = 0.8
    Cl_ref: float = -0.55
    Cd_ref: float = 0.04
    h_opt: float = 0.06          # m optimal rear ride height
    h_stall: float = 0.022
    rake_gain: float = 1.5       # sensitivity to positive rake (hr-hf)
    expansion_ratio: float = 2.5


@dataclass
class SplitterParams:
    area: float = 0.25
    Cl_ref: float = -0.25
    Cd_ref: float = 0.02
    h_ref: float = 0.05
    seal_efficiency: float = 0.85


@dataclass
class DeviceAeroForce:
    Fz: float
    Fx: float
    Cl: float
    Cd: float
    stalled: bool = False


def evaluate_diffuser(
    q: float,
    h_rear: float,
    rake: float,
    params: DiffuserParams,
) -> DeviceAeroForce:
    """
    Ground-effect rear downforce with stall at very low ride height.
    Positive rake (hr > hf) improves diffuser performance slightly.
    """
    # Height factor: peak near h_opt, fall off higher, stall lower
    if h_rear < params.h_stall:
        height_fac = (h_rear / params.h_stall) ** 2
        stalled = True
    else:
        # Bell-ish around h_opt
        x = (h_rear - params.h_opt) / max(params.h_opt, 1e-6)
        height_fac = float(np.exp(-0.8 * x * x))
        stalled = False

    rake_fac = 1.0 + params.rake_gain * float(np.clip(rake, -0.05, 0.08))
    rake_fac = float(np.clip(rake_fac, 0.6, 1.6))
    exp_fac = 0.7 + 0.3 * min(params.expansion_ratio / 3.0, 1.0)

    Cl = params.Cl_ref * height_fac * rake_fac * exp_fac
    Cd = params.Cd_ref * (0.8 + 0.4 * height_fac)
    Fz = Cl * q * params.area
    Fx = -Cd * q * params.area
    return DeviceAeroForce(Fz=Fz, Fx=Fx, Cl=Cl, Cd=Cd, stalled=stalled)


def evaluate_splitter(
    q: float,
    h_front: float,
    params: SplitterParams,
) -> DeviceAeroForce:
    """Front ground-effect from splitter / undertray sealing."""
    height_fac = (params.h_ref + 0.02) / (h_front + 0.02)
    height_fac = float(np.clip(height_fac, 0.4, 1.8))
    Cl = params.Cl_ref * height_fac * params.seal_efficiency
    Cd = params.Cd_ref * (0.9 + 0.2 * height_fac)
    Fz = Cl * q * params.area
    Fx = -Cd * q * params.area
    return DeviceAeroForce(Fz=Fz, Fx=Fx, Cl=Cl, Cd=Cd, stalled=False)
