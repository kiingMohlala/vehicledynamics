"""CFD and FEA boundary condition definitions."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class BoundaryCondition:
    name: str
    bc_type: str  # velocity_inlet, pressure_outlet, wall, symmetry, moving_wall, fixed, load, pressure, gravity
    region: str
    values: dict[str, Any] = field(default_factory=dict)


def default_cfd_bcs(
    speed: float = 40.0,
    rho: float = 1.225,
    ground: bool = True,
    rotating_wheels: bool = True,
) -> list[BoundaryCondition]:
    bcs = [
        BoundaryCondition("inlet", "velocity_inlet", "inlet", {"U": [speed, 0.0, 0.0]}),
        BoundaryCondition("outlet", "pressure_outlet", "outlet", {"p": 0.0}),
        BoundaryCondition("body", "wall", "body", {"type": "noSlip"}),
        BoundaryCondition("symmetry", "symmetry", "symmetry", {}),
    ]
    if ground:
        bcs.append(BoundaryCondition("ground", "moving_wall", "ground", {"U": [speed, 0.0, 0.0]}))
    if rotating_wheels:
        for w in ("wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"):
            bcs.append(BoundaryCondition(w, "moving_wall", w, {"omega": speed / 0.32}))
    return bcs


def default_fea_bcs(
    pressure: float = 1000.0,
    gravity: bool = True,
) -> list[BoundaryCondition]:
    bcs = [
        BoundaryCondition("fixed_mounts", "fixed", "mounts", {}),
        BoundaryCondition("aero_pressure", "pressure", "body", {"p": pressure}),
    ]
    if gravity:
        bcs.append(BoundaryCondition("gravity", "gravity", "all", {"g": [0.0, 0.0, -9.81]}))
    return bcs
