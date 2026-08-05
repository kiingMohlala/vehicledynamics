"""
Quasi-static progressive crash solver with plastic hinge degradation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_dynamics.fem.assembler import Model
from vehicle_dynamics.fem.stiffness import global_stiffness
from vehicle_dynamics.fem.constraints import free_dofs
from vehicle_dynamics.fem.solver import solve_static

from .material_plastic import PlasticMaterial, plastic_steel
from .plastic_hinge import (
    ElementPlasticState,
    HingeState,
    update_hinge_states,
)
from .energy import EnergyAccount, account_energy
from .intrusion import IntrusionMetrics, compute_intrusion


@dataclass
class CrashConfig:
    n_steps: int = 10
    fail_ratio: float = 1.5
    mass_kg: float = 1400.0
    speed_mps: float = 13.9  # ~50 km/h
    default_material: PlasticMaterial = field(default_factory=plastic_steel)


@dataclass
class CrashResult:
    success: bool
    message: str
    u: np.ndarray
    hinge_states: dict[int, ElementPlasticState]
    energy: EnergyAccount
    intrusion: IntrusionMetrics
    n_yielded: int = 0
    n_plastic: int = 0
    n_failed: int = 0
    load_factor: float = 1.0


def _assemble_degraded_K(
    model: Model, states: dict[int, ElementPlasticState]
) -> np.ndarray:
    K = np.zeros((model.ndof, model.ndof))
    for elem in model.elements:
        ke = global_stiffness(elem)
        deg = states.get(elem.id, ElementPlasticState(elem.id)).degradation
        ke = deg * ke
        dofs = np.concatenate([elem.node_i.dof_indices(), elem.node_j.dof_indices()])
        for a in range(12):
            for b in range(12):
                K[dofs[a], dofs[b]] += ke[a, b]
    return 0.5 * (K + K.T)


def _solve_with_K(model: Model, K: np.ndarray, F: np.ndarray) -> tuple[np.ndarray, bool, str]:
    free = free_dofs(model)
    free_idx = np.where(free)[0]
    if free_idx.size == 0:
        return np.zeros(model.ndof), False, "No free DOFs"
    Kff = K[np.ix_(free_idx, free_idx)]
    try:
        u_f = np.linalg.solve(0.5 * (Kff + Kff.T), F[free_idx])
    except np.linalg.LinAlgError as e:
        return np.zeros(model.ndof), False, f"Singular: {e}"
    if not np.all(np.isfinite(u_f)):
        return np.zeros(model.ndof), False, "Non-finite solution"
    u = np.zeros(model.ndof)
    u[free_idx] = u_f
    return u, True, "ok"


def solve_crash_quasistatic(
    model: Model,
    loads: np.ndarray,
    config: CrashConfig | None = None,
    materials: dict[int, PlasticMaterial] | None = None,
) -> CrashResult:
    """
    Incremental load application with hinge-state stiffness degradation.

    At each step λ = i/n:
      1. Assemble degraded K from current hinge states
      2. Solve K u = λ F
      3. Update hinge states from recovered forces
    """
    cfg = config or CrashConfig()
    mats = materials or {-1: cfg.default_material}
    F = np.asarray(loads, dtype=float).reshape(-1)
    if F.size != model.ndof:
        raise ValueError("Load vector size mismatch")

    states: dict[int, ElementPlasticState] = {
        e.id: ElementPlasticState(elem_id=e.id) for e in model.elements
    }
    u = np.zeros(model.ndof)
    ok = True
    msg = "ok"

    for s in range(1, cfg.n_steps + 1):
        lam = s / cfg.n_steps
        K = _assemble_degraded_K(model, states)
        u_step, ok, msg = _solve_with_K(model, K, lam * F)
        if not ok:
            break
        u = u_step
        states = update_hinge_states(
            model.elements, u, mats, prev=states, fail_ratio=cfg.fail_ratio
        )

    # Crush distance: max nodal translational displacement
    crush = 0.0
    for n in model.nodes:
        b = 6 * n.id
        crush = max(crush, float(np.linalg.norm(u[b : b + 3])))

    energy = account_energy(
        model, u, states, cfg.mass_kg, cfg.speed_mps, crush
    )
    intrusion = compute_intrusion(
        model, u, speed_mps=cfg.speed_mps, crush_distance=crush
    )

    n_y = sum(1 for s in states.values() if s.state == HingeState.YIELDING)
    n_p = sum(1 for s in states.values() if s.state == HingeState.PLASTIC)
    n_f = sum(1 for s in states.values() if s.state == HingeState.FAILED)

    return CrashResult(
        success=ok and np.all(np.isfinite(u)),
        message=msg,
        u=u,
        hinge_states=states,
        energy=energy,
        intrusion=intrusion,
        n_yielded=n_y,
        n_plastic=n_p,
        n_failed=n_f,
        load_factor=1.0 if ok else (s / cfg.n_steps),
    )
