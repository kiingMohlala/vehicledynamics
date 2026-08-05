"""
Generalized eigenvalue modal solver: K φ = λ M φ,  ω = √λ,  f = ω/(2π)
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import eigh

from .assembler import Model
from .constraints import free_dofs
from .mass import assemble_mass
from .modal_result import ModalResult, ModeShape


def _classify_mode(shape: np.ndarray, n_nodes: int) -> str:
    """Heuristic classification from translational / rotational energy."""
    ux = uy = uz = rx = ry = rz = 0.0
    for i in range(n_nodes):
        b = 6 * i
        ux += shape[b] ** 2
        uy += shape[b + 1] ** 2
        uz += shape[b + 2] ** 2
        rx += shape[b + 3] ** 2
        ry += shape[b + 4] ** 2
        rz += shape[b + 5] ** 2

    trans = ux + uy + uz
    rot = rx + ry + rz
    total = trans + rot + 1e-30

    # Near-zero frequency handled outside
    if rot > 0.55 * total and rx > 0.4 * rot:
        return "torsion"
    if uz >= uy and uz >= ux:
        return "bending_z"
    if uy >= uz and uy >= ux:
        return "bending_y"
    if ux > uy and ux > uz:
        return "axial"
    if rot > trans:
        return "rotational"
    return "mixed"


def _normalize(phi: np.ndarray, Mff: np.ndarray, free: np.ndarray, method: str) -> np.ndarray:
    """
    method: 'mass' → φᵀ M φ = 1 on free DOFs
            'max'  → max |translational displacement| = 1
    """
    full = np.zeros(free.size)
    # free is boolean mask of length ndof; phi is free-sized
    if method == "mass":
        mnorm = float(phi @ (Mff @ phi))
        if mnorm > 1e-30:
            phi = phi / np.sqrt(mnorm)
    else:
        # map to full for max-disp on translations
        full_tmp = np.zeros(len(free))
        full_tmp[free] = phi
        # max of ux,uy,uz magnitudes
        n = len(free) // 6
        vals = []
        for i in range(n):
            b = 6 * i
            if b + 2 < len(full_tmp):
                vals.extend(
                    [
                        abs(full_tmp[b]),
                        abs(full_tmp[b + 1]),
                        abs(full_tmp[b + 2]),
                    ]
                )
        peak = max(vals) if vals else 1.0
        if peak > 1e-30:
            phi = phi / peak
    return phi


def solve_modal(
    model: Model,
    n_modes: int = 20,
    consistent_mass: bool = True,
    normalize: str = "mass",
    rigid_tol_Hz: float = 0.5,
) -> ModalResult:
    """
    Solve free-vibration modes on free DOFs.

    Parameters
    ----------
    n_modes : int
        Number of lowest modes to return (clamped to available free DOFs).
    consistent_mass : bool
        True → consistent element mass; False → lumped.
    normalize : {'mass', 'max'}
        Mass-orthonormal or unit peak displacement.
    rigid_tol_Hz : float
        Frequencies below this are tagged as rigid-body modes.
    """
    K = model.assemble_stiffness()
    M = assemble_mass(model, consistent=consistent_mass)
    free = free_dofs(model)

    if not np.any(free):
        return ModalResult(
            frequencies_Hz=np.array([]),
            omega=np.array([]),
            eigenvalues=np.array([]),
            mode_shapes=np.zeros((model.ndof, 0)),
            success=False,
            message="No free DOFs",
            mass_type="consistent" if consistent_mass else "lumped",
        )

    Kff = 0.5 * (K[np.ix_(free, free)] + K[np.ix_(free, free)].T)
    Mff = 0.5 * (M[np.ix_(free, free)] + M[np.ix_(free, free)].T)

    # Guard against singular / negative mass diagonals
    if np.any(np.diag(Mff) < 0):
        return ModalResult(
            frequencies_Hz=np.array([]),
            omega=np.array([]),
            eigenvalues=np.array([]),
            mode_shapes=np.zeros((model.ndof, 0)),
            success=False,
            message="Negative mass diagonal entries",
            mass_type="consistent" if consistent_mass else "lumped",
        )

    n_free = int(np.sum(free))
    n_req = min(n_modes, n_free)

    try:
        # Subset of smallest eigenvalues
        evals, evecs = eigh(Kff, Mff, subset_by_index=[0, n_req - 1])
    except Exception as e:
        return ModalResult(
            frequencies_Hz=np.array([]),
            omega=np.array([]),
            eigenvalues=np.array([]),
            mode_shapes=np.zeros((model.ndof, 0)),
            success=False,
            message=f"Eigensolve failed: {e}",
            mass_type="consistent" if consistent_mass else "lumped",
        )

    # Clean tiny negatives from numerical noise
    evals = np.maximum(evals, 0.0)
    omega = np.sqrt(evals)
    freq = omega / (2.0 * np.pi)

    shapes_full = np.zeros((model.ndof, n_req))
    modes: list[ModeShape] = []
    n_rigid = 0

    for i in range(n_req):
        phi_f = evecs[:, i].copy()
        phi_f = _normalize(phi_f, Mff, free, normalize)
        shapes_full[free, i] = phi_f

        if freq[i] < rigid_tol_Hz:
            cls = "rigid_body"
            n_rigid += 1
        else:
            cls = _classify_mode(shapes_full[:, i], len(model.nodes))

        modes.append(
            ModeShape(
                index=i,
                frequency_Hz=float(freq[i]),
                omega_rad_s=float(omega[i]),
                eigenvalue=float(evals[i]),
                period_s=float(1.0 / freq[i]) if freq[i] > 1e-12 else float("inf"),
                shape=shapes_full[:, i].copy(),
                classification=cls,
                mass_normalized=(normalize == "mass"),
            )
        )

    if not np.all(np.isfinite(freq)) or not np.all(np.isfinite(shapes_full)):
        return ModalResult(
            frequencies_Hz=freq,
            omega=omega,
            eigenvalues=evals,
            mode_shapes=shapes_full,
            modes=modes,
            n_rigid_body=n_rigid,
            success=False,
            message="Non-finite modal results",
            mass_type="consistent" if consistent_mass else "lumped",
        )

    return ModalResult(
        frequencies_Hz=freq,
        omega=omega,
        eigenvalues=evals,
        mode_shapes=shapes_full,
        modes=modes,
        n_rigid_body=n_rigid,
        success=True,
        message="ok",
        mass_type="consistent" if consistent_mass else "lumped",
    )


def modal_orthogonality(result: ModalResult, model: Model, consistent_mass: bool = True) -> float:
    """
    Return max off-diagonal |φᵢᵀ M φⱼ| for i≠j (should be ~0 for mass-normalized modes).
    """
    from .mass import assemble_mass

    M = assemble_mass(model, consistent=consistent_mass)
    free = free_dofs(model)
    Mff = M[np.ix_(free, free)]
    n = result.mode_shapes.shape[1]
    max_off = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            vi = result.mode_shapes[free, i]
            vj = result.mode_shapes[free, j]
            off = abs(float(vi @ (Mff @ vj)))
            max_off = max(max_off, off)
    return max_off
