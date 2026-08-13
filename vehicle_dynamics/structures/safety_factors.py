"""Yield / buckling / fatigue safety factors."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from .materials import StructuralMaterial, steel
from .buckling import member_buckling_sf
from .fatigue import miner_damage


@dataclass
class SafetyReport:
    yield_sf: float
    buckling_sf: float
    fatigue_sf: float
    governing: str
    ok: bool


def yield_sf(sigma_vm: float, mat: StructuralMaterial | None = None) -> float:
    mat = mat or steel()
    return float(mat.Sy / (abs(sigma_vm) + 1e-9))


def evaluate_safety(
    sigma_vm: float,
    axial_force: float = 0.0,
    length: float = 0.5,
    I: float = 1e-7,
    stress_amplitudes: list[float] | None = None,
    cycles: list[float] | None = None,
    mat: StructuralMaterial | None = None,
    min_sf: float = 1.5,
) -> SafetyReport:
    mat = mat or steel()
    ys = yield_sf(sigma_vm, mat)
    bs = member_buckling_sf(axial_force, length, mat, I)
    if stress_amplitudes and cycles:
        fat = miner_damage(stress_amplitudes, cycles, mat)
        fs = 1.0 / max(fat.damage, 1e-12)
    else:
        fs = float("inf")
    vals = {"yield": ys, "buckling": bs, "fatigue": fs}
    governing = min(vals, key=lambda k: vals[k] if np.isfinite(vals[k]) else 1e99)
    gov_val = vals[governing]
    return SafetyReport(ys, bs if np.isfinite(bs) else 1e6, fs if np.isfinite(fs) else 1e6, governing, gov_val >= min_sf)
