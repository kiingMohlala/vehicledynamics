"""Torsional and bending stiffness metrics."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .static_solver import solve_frame
from .load_cases import LoadCases


@dataclass
class ChassisMetrics:
    torsional_stiffness: float  # Nm/deg
    bending_stiffness: float    # N/m
    max_disp_torsion: float
    max_disp_bending: float


def default_ladder_frame(wheelbase: float = 2.7, track: float = 1.5):
    """Simple ladder frame nodes/elements for metrics."""
    half = track * 0.5
    nodes = {
        "FL": np.array([0.0, half, 0.0]),
        "FR": np.array([0.0, -half, 0.0]),
        "RL": np.array([-wheelbase, half, 0.0]),
        "RR": np.array([-wheelbase, -half, 0.0]),
        "ML": np.array([-0.5 * wheelbase, half, 0.0]),
        "MR": np.array([-0.5 * wheelbase, -half, 0.0]),
    }
    elements = [
        ("FL", "ML"), ("ML", "RL"),
        ("FR", "MR"), ("MR", "RR"),
        ("FL", "FR"), ("ML", "MR"), ("RL", "RR"),
        ("FL", "MR"), ("FR", "ML"),  # light cross braces
    ]
    return nodes, elements


def compute_torsional_stiffness(wheelbase: float = 2.7, track: float = 1.5, force: float = 1000.0) -> tuple[float, float]:
    nodes, elements = default_ladder_frame(wheelbase, track)
    # fix rear, load front diagonal
    loads = {"FL": np.array([0.0, 0.0, force]), "FR": np.array([0.0, 0.0, -force])}
    sol = solve_frame(nodes, elements, loads, fixed=["RL", "RR"])
    if not sol.success:
        return 0.0, 0.0
    idx = {n: i for i, n in enumerate(sol.node_names)}
    z_fl = sol.u[6 * idx["FL"] + 2]
    z_fr = sol.u[6 * idx["FR"] + 2]
    # twist angle (rad) ≈ (z_fl - z_fr) / track
    theta = abs(z_fl - z_fr) / max(track, 1e-9)
    theta_deg = theta * 180.0 / np.pi
    moment = force * track  # Nm approx
    Kt = moment / max(theta_deg, 1e-12)
    return float(Kt), float(sol.max_disp)


def compute_bending_stiffness(wheelbase: float = 2.7, track: float = 1.5, force: float = 2000.0) -> tuple[float, float]:
    nodes, elements = default_ladder_frame(wheelbase, track)
    loads = {"ML": np.array([0.0, 0.0, -force]), "MR": np.array([0.0, 0.0, -force])}
    sol = solve_frame(nodes, elements, loads, fixed=["FL", "FR", "RL", "RR"])
    if not sol.success:
        return 0.0, 0.0
    idx = {n: i for i, n in enumerate(sol.node_names)}
    z = 0.5 * (sol.u[6 * idx["ML"] + 2] + sol.u[6 * idx["MR"] + 2])
    Kb = (2 * force) / max(abs(z), 1e-12)
    return float(Kb), float(sol.max_disp)
