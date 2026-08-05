"""
Reduced-order compliance: condense FEM to pickup translational DOFs.

Builds a compliance matrix C such that u_pickups = C @ F_pickups
for the free translational DOFs at mapped pickups (static condensation
via the flexibility of the constrained chassis).
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_dynamics.fem.assembler import Model
from vehicle_dynamics.fem.constraints import free_dofs
from vehicle_dynamics.fem.mass import assemble_mass  # noqa: F401 – reserved

from .pickup_mapper import PickupMap


@dataclass
class ReducedComplianceModel:
    """
    C maps stacked [Fx,Fy,Fz] per retained pickup → [ux,uy,uz].
    """

    roles: list[str]
    C: np.ndarray  # (3*n, 3*n)
    node_ids: list[int]

    @property
    def n_pickups(self) -> int:
        return len(self.roles)

    def apply(self, forces: dict[str, tuple[float, float, float]]) -> dict[str, np.ndarray]:
        """
        forces: role → (Fx, Fy, Fz)
        returns: role → (ux, uy, uz)
        """
        n = self.n_pickups
        F = np.zeros(3 * n)
        for i, role in enumerate(self.roles):
            fx, fy, fz = forces.get(role, (0.0, 0.0, 0.0))
            F[3 * i : 3 * i + 3] = [fx, fy, fz]
        U = self.C @ F
        out = {}
        for i, role in enumerate(self.roles):
            out[role] = U[3 * i : 3 * i + 3].copy()
        return out


def build_reduced_compliance(
    model: Model,
    pickup_map: PickupMap,
    roles: list[str] | None = None,
) -> ReducedComplianceModel:
    """
    Static condensation of translational DOFs at selected pickups.

    Method: for each unit load on a retained free translational DOF,
    solve K u = F and read the retained displacements → columns of C.
    """
    if roles is None:
        roles = [r for r in ("susp_fl", "susp_fr", "susp_rl", "susp_rr") if r in pickup_map.nodes]
    if not roles:
        raise ValueError("No pickup roles available for reduction")

    K = model.assemble_stiffness()
    free = free_dofs(model)
    free_idx = np.where(free)[0]
    if free_idx.size == 0:
        raise RuntimeError("No free DOFs for reduced model")
    free_set = set(int(d) for d in free_idx)

    # Only pickups with free translational DOFs (skip constrained supports)
    roles_free: list[str] = []
    node_ids: list[int] = []
    retained_list: list[int] = []
    for r in roles:
        nid = pickup_map.node_id(r)
        base = 6 * nid
        dofs = [base, base + 1, base + 2]
        if all(d in free_set for d in dofs):
            roles_free.append(r)
            node_ids.append(nid)
            retained_list.extend(dofs)
    if not roles_free:
        raise RuntimeError(
            "No free pickup translational DOFs for reduced model "
            "(supports may have constrained all pickups)"
        )
    roles = roles_free
    retained = np.array(retained_list, dtype=int)

    Kff = 0.5 * (K[np.ix_(free_idx, free_idx)] + K[np.ix_(free_idx, free_idx)].T)
    free_pos = {int(d): i for i, d in enumerate(free_idx)}
    retained_in_free = np.array([free_pos[int(d)] for d in retained], dtype=int)

    n_r = len(retained)
    C = np.zeros((n_r, n_r))
    try:
        from numpy.linalg import solve

        for j, j_free in enumerate(retained_in_free):
            F = np.zeros(free_idx.size)
            F[j_free] = 1.0
            u_f = solve(Kff, F)
            C[:, j] = u_f[retained_in_free]
    except np.linalg.LinAlgError as e:
        raise RuntimeError(f"Reduced compliance factorization failed: {e}") from e

    return ReducedComplianceModel(roles=list(roles), C=C, node_ids=node_ids)
