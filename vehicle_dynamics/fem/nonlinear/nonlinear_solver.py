"""
Newton–Raphson nonlinear static solver.

R(u) = F_ext - F_int(u) = 0
(K_M + K_G) Δu = R
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_dynamics.fem.assembler import Model
from vehicle_dynamics.fem.constraints import free_dofs
from vehicle_dynamics.fem.result import StaticResult

from .corotational_beam import internal_force_global, material_stiffness_global
from .geometric_stiffness import geometric_stiffness_global
from .convergence import ConvergenceLog


@dataclass
class NonlinearResult:
    u: np.ndarray
    reactions: np.ndarray
    success: bool
    message: str = ""
    n_iter: int = 0
    residual_norm: float = 0.0
    log: ConvergenceLog = field(default_factory=ConvergenceLog)
    load_factor: float = 1.0

    def node_displacement(self, node_id: int) -> np.ndarray:
        base = 6 * node_id
        return self.u[base : base + 6].copy()


def _assemble_internal(model: Model, u: np.ndarray) -> np.ndarray:
    F_int = np.zeros(model.ndof)
    for elem in model.elements:
        fe = internal_force_global(elem, u)
        dofs = np.concatenate([elem.node_i.dof_indices(), elem.node_j.dof_indices()])
        for a in range(12):
            F_int[dofs[a]] += fe[a]
    return F_int


def _assemble_tangent(model: Model, u: np.ndarray) -> np.ndarray:
    K = np.zeros((model.ndof, model.ndof))
    for elem in model.elements:
        km = material_stiffness_global(elem, u)
        kg = geometric_stiffness_global(elem, u)
        ke = km + kg
        dofs = np.concatenate([elem.node_i.dof_indices(), elem.node_j.dof_indices()])
        for a in range(12):
            for b in range(12):
                K[dofs[a], dofs[b]] += ke[a, b]
    return 0.5 * (K + K.T)


def solve_static_nonlinear(
    model: Model,
    loads: np.ndarray,
    tol: float = 1e-6,
    max_iter: int = 30,
    load_factor: float = 1.0,
    u0: np.ndarray | None = None,
) -> NonlinearResult:
    """
    Full-load Newton–Raphson (single load factor).

    For difficult problems prefer solve_nonlinear_stepped().
    """
    F = np.asarray(loads, dtype=float).reshape(-1) * load_factor
    if F.size != model.ndof:
        raise ValueError(f"Load size {F.size} != ndof {model.ndof}")

    free = free_dofs(model)
    free_idx = np.where(free)[0]
    log = ConvergenceLog()

    if free_idx.size == 0:
        return NonlinearResult(
            u=np.zeros(model.ndof),
            reactions=-F,
            success=False,
            message="No free DOFs",
            log=log,
            load_factor=load_factor,
        )

    u = np.zeros(model.ndof) if u0 is None else u0.copy()

    for it in range(max_iter):
        F_int = _assemble_internal(model, u)
        R = F - F_int
        # Zero residual on constrained DOFs (essential BCs: u=0)
        R_f = R[free_idx]
        res_norm = float(np.linalg.norm(R_f))

        if res_norm < tol:
            log.add(it, res_norm, 0.0, load_factor, converged=True)
            reactions = F_int - F  # reaction = internal - applied on fixed
            # Standard: R_reac = K u - F on fixed ≈ F_int - F
            return NonlinearResult(
                u=u,
                reactions=F_int - F,
                success=True,
                message="ok",
                n_iter=it + 1,
                residual_norm=res_norm,
                log=log,
                load_factor=load_factor,
            )

        K = _assemble_tangent(model, u)
        Kff = K[np.ix_(free_idx, free_idx)]
        Kff = 0.5 * (Kff + Kff.T)

        try:
            du_f = np.linalg.solve(Kff, R_f)
        except np.linalg.LinAlgError:
            log.add(it, res_norm, np.inf, load_factor, converged=False)
            return NonlinearResult(
                u=u,
                reactions=np.zeros(model.ndof),
                success=False,
                message=f"Singular tangent at iter {it}",
                n_iter=it + 1,
                residual_norm=res_norm,
                log=log,
                load_factor=load_factor,
            )

        if not np.all(np.isfinite(du_f)):
            return NonlinearResult(
                u=u,
                reactions=np.zeros(model.ndof),
                success=False,
                message="Non-finite correction",
                n_iter=it + 1,
                residual_norm=res_norm,
                log=log,
                load_factor=load_factor,
            )

        corr = float(np.linalg.norm(du_f))
        u[free_idx] += du_f
        log.add(it, res_norm, corr, load_factor, converged=False)

    # Final residual check
    F_int = _assemble_internal(model, u)
    R_f = (F - F_int)[free_idx]
    res_norm = float(np.linalg.norm(R_f))
    ok = res_norm < tol * 10  # soft accept
    return NonlinearResult(
        u=u,
        reactions=F_int - F,
        success=ok,
        message="ok" if ok else f"Max iterations (res={res_norm:.3e})",
        n_iter=max_iter,
        residual_norm=res_norm,
        log=log,
        load_factor=load_factor,
    )
