"""
Element and global mass matrices for 3D Euler-Bernoulli beams.

Supports consistent mass (default) and optional lumped mass.
DOF order matches stiffness: [ux,uy,uz,rx,ry,rz] × 2 nodes.
"""

from __future__ import annotations

import numpy as np
from .beam import BeamElement
from .transform import transformation_matrix
from .assembler import Model


def local_mass_consistent(elem: BeamElement) -> np.ndarray:
    """
    12×12 consistent mass matrix in local coordinates.

    Translational parts use classic beam consistent mass.
    Torsional inertia uses polar mass density ρ·J (circular section: J = Ip).
    Rotary inertia of cross-section about y/z is included approximately
    via ρ·Iy, ρ·Iz terms on rotational DOFs (secondary for slender tubes).
    """
    L = elem.length()
    rho = elem.material.rho
    A = elem.section.A
    Iy = elem.section.Iy
    Iz = elem.section.Iz
    J = elem.section.J  # polar / torsion constant

    m = np.zeros((12, 12))

    # --- Axial (ux) consistent mass ---
    # [2 1; 1 2] * (ρ A L / 6)
    ma = rho * A * L / 6.0
    m[0, 0] = 2 * ma
    m[0, 6] = ma
    m[6, 0] = ma
    m[6, 6] = 2 * ma

    # --- Torsion (rx) ---
    # analogous to axial with ρ J instead of ρ A
    mt = rho * J * L / 6.0
    m[3, 3] = 2 * mt
    m[3, 9] = mt
    m[9, 3] = mt
    m[9, 9] = 2 * mt

    # --- Bending in local y (uy, rz) ---
    # Standard consistent mass for Bernoulli beam:
    # ρ A L * matrix with 13/35, 11/210 L, 9/70, ...
    c = rho * A * L
    # uy-uy
    m[1, 1] = 13.0 / 35.0 * c
    m[1, 7] = 9.0 / 70.0 * c
    m[7, 1] = 9.0 / 70.0 * c
    m[7, 7] = 13.0 / 35.0 * c
    # uy-rz
    m[1, 5] = 11.0 / 210.0 * c * L
    m[1, 11] = -13.0 / 420.0 * c * L
    m[5, 1] = 11.0 / 210.0 * c * L
    m[11, 1] = -13.0 / 420.0 * c * L
    m[7, 5] = 13.0 / 420.0 * c * L
    m[7, 11] = -11.0 / 210.0 * c * L
    m[5, 7] = 13.0 / 420.0 * c * L
    m[11, 7] = -11.0 / 210.0 * c * L
    # rz-rz
    m[5, 5] = 1.0 / 105.0 * c * L**2
    m[5, 11] = -1.0 / 140.0 * c * L**2
    m[11, 5] = -1.0 / 140.0 * c * L**2
    m[11, 11] = 1.0 / 105.0 * c * L**2

    # Optional rotary inertia correction about z (ρ Iz)
    ri_z = rho * Iz * L / 6.0
    m[5, 5] += 2 * ri_z
    m[5, 11] += ri_z
    m[11, 5] += ri_z
    m[11, 11] += 2 * ri_z

    # --- Bending in local z (uz, ry) ---
    m[2, 2] = 13.0 / 35.0 * c
    m[2, 8] = 9.0 / 70.0 * c
    m[8, 2] = 9.0 / 70.0 * c
    m[8, 8] = 13.0 / 35.0 * c

    m[2, 4] = -11.0 / 210.0 * c * L
    m[2, 10] = 13.0 / 420.0 * c * L
    m[4, 2] = -11.0 / 210.0 * c * L
    m[10, 2] = 13.0 / 420.0 * c * L
    m[8, 4] = -13.0 / 420.0 * c * L
    m[8, 10] = 11.0 / 210.0 * c * L
    m[4, 8] = -13.0 / 420.0 * c * L
    m[10, 8] = 11.0 / 210.0 * c * L

    m[4, 4] = 1.0 / 105.0 * c * L**2
    m[4, 10] = -1.0 / 140.0 * c * L**2
    m[10, 4] = -1.0 / 140.0 * c * L**2
    m[10, 10] = 1.0 / 105.0 * c * L**2

    ri_y = rho * Iy * L / 6.0
    m[4, 4] += 2 * ri_y
    m[4, 10] += ri_y
    m[10, 4] += ri_y
    m[10, 10] += 2 * ri_y

    return 0.5 * (m + m.T)


def local_mass_lumped(elem: BeamElement) -> np.ndarray:
    """
    Diagonal lumped mass: half translational mass at each node;
    torsional inertia ρ J L / 2 per node; rotary bending inertia ρ I L / 2.
    """
    L = elem.length()
    rho = elem.material.rho
    A = elem.section.A
    Iy = elem.section.Iy
    Iz = elem.section.Iz
    J = elem.section.J

    m = np.zeros((12, 12))
    half_trans = 0.5 * rho * A * L
    half_tors = 0.5 * rho * J * L
    half_ry = 0.5 * rho * Iy * L
    half_rz = 0.5 * rho * Iz * L

    for node in (0, 1):
        b = 6 * node
        m[b + 0, b + 0] = half_trans  # ux
        m[b + 1, b + 1] = half_trans  # uy
        m[b + 2, b + 2] = half_trans  # uz
        m[b + 3, b + 3] = half_tors   # rx
        m[b + 4, b + 4] = half_ry     # ry
        m[b + 5, b + 5] = half_rz     # rz
    return m


def global_mass(elem: BeamElement, consistent: bool = True) -> np.ndarray:
    """m_global = T.T @ m_local @ T"""
    m_loc = local_mass_consistent(elem) if consistent else local_mass_lumped(elem)
    T = transformation_matrix(elem)
    return T.T @ m_loc @ T


def assemble_mass(model: Model, consistent: bool = True) -> np.ndarray:
    """Global mass matrix with same DOF ordering as stiffness."""
    M = np.zeros((model.ndof, model.ndof))
    for elem in model.elements:
        me = global_mass(elem, consistent=consistent)
        dofs = np.concatenate(
            [elem.node_i.dof_indices(), elem.node_j.dof_indices()]
        )
        for a in range(12):
            for b in range(12):
                M[dofs[a], dofs[b]] += me[a, b]
    return 0.5 * (M + M.T)


def total_mass_from_matrix(M: np.ndarray, n_nodes: int) -> float:
    """
    Estimate translational mass by summing ux+uy+uz diagonal contributions
    averaged (each node counted 3 ways → divide by 3).
    """
    total = 0.0
    for i in range(n_nodes):
        base = 6 * i
        total += M[base, base] + M[base + 1, base + 1] + M[base + 2, base + 2]
    return total / 3.0
