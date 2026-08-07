"""Linear static solve for assembled frames + FEM-backed path."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .stiffness_matrix import assemble_frame
from .materials import steel


@dataclass
class StaticSolution:
    u: np.ndarray
    reactions: np.ndarray
    node_names: list[str]
    success: bool
    message: str = ""
    max_disp: float = 0.0


def solve_frame(
    nodes: dict[str, np.ndarray],
    elements: list[tuple[str, str]],
    loads: dict[str, np.ndarray],
    fixed: list[str],
) -> StaticSolution:
    K, names = assemble_frame(nodes, elements)
    ndof = K.shape[0]
    F = np.zeros(ndof)
    idx = {n: i for i, n in enumerate(names)}
    for tag, force in loads.items():
        if tag not in idx:
            continue
        base = 6 * idx[tag]
        f = np.asarray(force, dtype=float).ravel()
        F[base:base + min(3, len(f))] += f[:3]
    free = np.ones(ndof, dtype=bool)
    for tag in fixed:
        if tag not in idx:
            continue
        base = 6 * idx[tag]
        free[base:base + 6] = False
    if not np.any(free):
        return StaticSolution(np.zeros(ndof), -F, names, False, "no free DOFs")
    Kff = K[np.ix_(free, free)]
    Ff = F[free]
    try:
        uf = np.linalg.solve(0.5 * (Kff + Kff.T), Ff)
    except np.linalg.LinAlgError as e:
        return StaticSolution(np.zeros(ndof), np.zeros(ndof), names, False, str(e))
    u = np.zeros(ndof)
    u[free] = uf
    R = K @ u - F
    # max translational displacement
    trans = np.array([np.linalg.norm(u[i:i+3]) for i in range(0, ndof, 6)])
    return StaticSolution(u, R, names, True, "ok", float(np.max(trans)) if len(trans) else 0.0)


def try_fem_cage_solve(load_case_name: str = "torsion"):
    """Optional: use Phase 8 FEM cage if available."""
    try:
        from vehicle_dynamics.fem import build_default_cage, solve_static, apply_force
        from vehicle_dynamics.fem.load_cases import torsional_rig
        model = build_default_cage()
        # use FEM load helper if present
        loads = np.zeros(model.ndof)
        # leave actual FEM path as optional enrichment
        return model, loads
    except Exception:
        return None, None
