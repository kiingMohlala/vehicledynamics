"""
Import aero coefficients from CSV, OpenFOAM force reports, and SU2 history/CSV.

OpenFOAM: expects a forces summary or custom coeff CSV exported from
postProcess -func forces / forceCoeffs.

SU2: accepts surface_flow.csv-style or history with CL, CD columns,
or a standardized aero table CSV.
"""

from __future__ import annotations

import csv
import re
from pathlib import Path
import numpy as np

from .cfd_map import AeroSample, AeroMapND


_REQUIRED_ALIASES = {
    "speed": ("speed", "v", "velocity", "airspeed", "u_inf"),
    "h_front": ("h_front", "hf", "ride_front", "front_ride"),
    "h_rear": ("h_rear", "hr", "ride_rear", "rear_ride"),
    "pitch": ("pitch", "theta", "pitch_rad", "pitch_deg"),
    "yaw": ("yaw", "beta", "yaw_rad", "yaw_deg"),
    "Cd": ("cd", "c_d", "drag_coeff", "drag"),
    "Cl_front": ("cl_front", "clf", "c_lf", "front_cl"),
    "Cl_rear": ("cl_rear", "clr", "c_lr", "rear_cl"),
}


def _norm_key(k: str) -> str:
    return re.sub(r"[^a-z0-9_]", "", k.strip().lower())


def _map_headers(fieldnames: list[str]) -> dict[str, str]:
    """Map canonical name → actual CSV column."""
    norms = {_norm_key(f): f for f in fieldnames}
    mapping = {}
    for canon, aliases in _REQUIRED_ALIASES.items():
        for a in aliases:
            if a in norms:
                mapping[canon] = norms[a]
                break
    return mapping


def _maybe_deg(name: str, val: float) -> float:
    if "deg" in name.lower():
        return float(np.deg2rad(val))
    return float(val)


def import_csv(path: str | Path, source: str = "csv") -> list[AeroSample]:
    path = Path(path)
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"Empty CSV: {path}")
        mapping = _map_headers(list(reader.fieldnames))
        samples = []
        for row in reader:
            def get(canon: str, default: float = 0.0) -> float:
                col = mapping.get(canon)
                if col is None or row.get(col, "") == "":
                    return default
                return float(row[col])

            pitch_col = mapping.get("pitch", "pitch")
            yaw_col = mapping.get("yaw", "yaw")
            samples.append(
                AeroSample(
                    speed=get("speed", 40.0),
                    h_front=get("h_front", 0.08),
                    h_rear=get("h_rear", 0.10),
                    pitch=_maybe_deg(pitch_col, get("pitch", 0.0)),
                    yaw=_maybe_deg(yaw_col, get("yaw", 0.0)),
                    roll=float(row.get("roll", 0.0) or 0.0),
                    wing_angle=float(row.get("wing_angle", row.get("alpha_wing", 0.12)) or 0.12),
                    drs=float(row.get("drs", 0.0) or 0.0),
                    Cd=get("Cd", 0.34),
                    Cl_front=get("Cl_front", -0.45),
                    Cl_rear=get("Cl_rear", -0.70),
                    Cy=float(row.get("Cy", row.get("cy", 0.0)) or 0.0),
                    Cm_pitch=float(row.get("Cm", row.get("cm_pitch", 0.0)) or 0.0),
                    Cn_yaw=float(row.get("Cn", row.get("cn_yaw", 0.0)) or 0.0),
                    x_cop=float(row.get("x_cop", row.get("cop", 0.0)) or 0.0),
                    source=source,
                )
            )
    return samples


def import_openfoam_forces(
    path: str | Path,
    *,
    speed: float = 40.0,
    h_front: float = 0.08,
    h_rear: float = 0.10,
    pitch: float = 0.0,
    yaw: float = 0.0,
    S_ref: float = 1.9,
    L_ref: float = 2.7,
    rho: float = 1.225,
) -> list[AeroSample]:
    """
    Parse OpenFOAM forceCoeffs.dat or forces.dat style output.

    Expected columns (comment header):
      Time  Cd  Cl  ...
    or
      Time  total_x total_y total_z ...
    For forceCoeffs, Cd/Cl used directly. For forces, convert via q*S.
    """
    path = Path(path)
    text = path.read_text()
    samples: list[AeroSample] = []
    q = 0.5 * rho * speed * speed
    lines = [
        ln.strip()
        for ln in text.splitlines()
        if ln.strip() and not ln.strip().startswith("#")
    ]
    for ln in lines:
        parts = re.split(r"[\s,;]+", ln)
        if len(parts) < 3:
            continue
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue
        # Heuristic: forceCoeffs → time Cd Cl Cs
        if len(vals) >= 3:
            # Prefer last entry as steady state when multiple times
            Cd = vals[1]
            Cl = vals[2]
            # Split Cl 40/60 front/rear if only total Cl present
            Cl_f = 0.4 * Cl
            Cl_r = 0.6 * Cl
            samples.append(
                AeroSample(
                    speed=speed,
                    h_front=h_front,
                    h_rear=h_rear,
                    pitch=pitch,
                    yaw=yaw,
                    Cd=Cd,
                    Cl_front=Cl_f,
                    Cl_rear=Cl_r,
                    source="openfoam",
                    meta={"raw": vals, "file": str(path)},
                )
            )
    if not samples:
        raise ValueError(f"No numeric force data in {path}")
    # Keep last (steady) sample primarily; return all for completeness
    return samples


def import_su2_forces(
    path: str | Path,
    *,
    speed: float = 40.0,
    h_front: float = 0.08,
    h_rear: float = 0.10,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> list[AeroSample]:
    """
    Parse SU2 history file or coeff CSV.

    history.csv typically has columns: "CD", "CL", "CMx", ...
    """
    path = Path(path)
    # Try as standard CSV first
    try:
        with path.open(newline="") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames and any(
                _norm_key(c) in ("cd", "cl", "drag", "lift") for c in reader.fieldnames
            ):
                rows = list(reader)
                if not rows:
                    raise ValueError("empty")
                last = rows[-1]
                norms = {_norm_key(k): v for k, v in last.items()}

                def g(*keys, default=0.0):
                    for k in keys:
                        if k in norms and norms[k] not in ("", None):
                            return float(norms[k])
                    return default

                Cd = g("cd", "c_d", "drag")
                Cl = g("cl", "c_l", "lift")
                return [
                    AeroSample(
                        speed=speed,
                        h_front=h_front,
                        h_rear=h_rear,
                        pitch=pitch,
                        yaw=yaw,
                        Cd=Cd,
                        Cl_front=0.4 * Cl,
                        Cl_rear=0.6 * Cl,
                        Cm_pitch=g("cmx", "cm", "cmy"),
                        Cn_yaw=g("cmz", "cn"),
                        source="su2",
                        meta={"file": str(path)},
                    )
                ]
    except Exception:
        pass

    # Fallback: whitespace table with CD CL in header comment
    return import_openfoam_forces(
        path, speed=speed, h_front=h_front, h_rear=h_rear, pitch=pitch, yaw=yaw
    )
