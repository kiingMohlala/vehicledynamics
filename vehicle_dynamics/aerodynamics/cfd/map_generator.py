"""Build and export aerodynamic maps from sample sets."""

from __future__ import annotations

import csv
from pathlib import Path
import numpy as np

from .cfd_map import AeroSample, AeroMapND


def _dedupe(samples: list[AeroSample], tol: float = 1e-6) -> list[AeroSample]:
    kept: list[AeroSample] = []
    for s in samples:
        v = s.state_vector()
        dup = False
        for k in kept:
            if np.linalg.norm(v - k.state_vector()) < tol:
                dup = True
                break
        if not dup:
            kept.append(s)
    return kept


def build_map_from_samples(
    samples: list[AeroSample],
    name: str = "cfd_map",
    *,
    clean: bool = True,
) -> AeroMapND:
    data = list(samples)
    if clean:
        # Drop non-finite
        data = [
            s for s in data
            if np.isfinite(s.Cd) and np.isfinite(s.Cl_front) and np.isfinite(s.Cl_rear)
        ]
        data = _dedupe(data)
    return AeroMapND(samples=data, name=name)


def export_map_csv(amap: AeroMapND, path: str | Path) -> None:
    path = Path(path)
    fields = [
        "speed", "h_front", "h_rear", "pitch", "yaw", "roll", "wing_angle", "drs",
        "Cd", "Cl_front", "Cl_rear", "Cy", "Cm_pitch", "Cn_yaw", "x_cop", "source",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for s in amap.samples:
            w.writerow({
                "speed": s.speed,
                "h_front": s.h_front,
                "h_rear": s.h_rear,
                "pitch": s.pitch,
                "yaw": s.yaw,
                "roll": s.roll,
                "wing_angle": s.wing_angle,
                "drs": s.drs,
                "Cd": s.Cd,
                "Cl_front": s.Cl_front,
                "Cl_rear": s.Cl_rear,
                "Cy": s.Cy,
                "Cm_pitch": s.Cm_pitch,
                "Cn_yaw": s.Cn_yaw,
                "x_cop": s.x_cop,
                "source": s.source,
            })


def synthetic_cfd_grid(
    cfg_speed: tuple[float, ...] = (20.0, 40.0, 60.0),
    h_fronts: tuple[float, ...] = (0.06, 0.08, 0.10),
    h_rears: tuple[float, ...] = (0.08, 0.10, 0.12),
) -> list[AeroSample]:
    """
    Synthetic OpenFOAM/SU2-like dataset for validation.
    Ground-effect: lower ride → more |Cl|; Cd mild rise.
    """
    samples = []
    for v in cfg_speed:
        for hf in h_fronts:
            for hr in h_rears:
                gf = (0.08 + 0.02) / (hf + 0.02)
                gr = (0.10 + 0.02) / (hr + 0.02)
                Cl_f = -0.42 * gf
                Cl_r = -0.68 * gr
                Cd = 0.32 + 0.02 * (gf + gr - 2.0)
                x_cop = 0.15 * (abs(Cl_r) - abs(Cl_f)) / (abs(Cl_f) + abs(Cl_r) + 1e-9)
                samples.append(
                    AeroSample(
                        speed=v,
                        h_front=hf,
                        h_rear=hr,
                        Cd=Cd,
                        Cl_front=Cl_f,
                        Cl_rear=Cl_r,
                        x_cop=x_cop,
                        source="synthetic_cfd",
                    )
                )
    return samples
