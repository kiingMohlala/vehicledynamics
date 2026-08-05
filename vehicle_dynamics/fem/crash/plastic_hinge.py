"""Plastic hinge state machine for beam elements."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import numpy as np

from vehicle_dynamics.fem.beam import BeamElement
from vehicle_dynamics.fem.stiffness import local_stiffness
from vehicle_dynamics.fem.transform import transformation_matrix
from .material_plastic import PlasticMaterial


class HingeState(str, Enum):
    ELASTIC = "elastic"
    YIELDING = "yielding"
    PLASTIC = "plastic"
    FAILED = "failed"


@dataclass
class ElementPlasticState:
    elem_id: int
    state: HingeState = HingeState.ELASTIC
    M_ratio: float = 0.0          # |M| / Mp
    N_ratio: float = 0.0
    plastic_work: float = 0.0     # J
    degradation: float = 1.0      # stiffness scale (1=elastic, →0 failed)


def section_plastic_moment(elem: BeamElement, mat: PlasticMaterial) -> float:
    """
    Approximate plastic moment for circular tube / rectangle.
    Tube: Mp ≈ Sy * (Do^3 - Di^3)/6
    Rectangle: Mp ≈ Sy * b h^2 / 4
    """
    sec = elem.section
    Sy = mat.yield_strength
    if sec.od is not None and sec.wall is not None:
        Do = sec.od
        Di = max(Do - 2 * sec.wall, 0.0)
        return Sy * (Do**3 - Di**3) / 6.0
    # From I and section modulus estimate for rectangle-like
    # Mp = 1.5 * My for rectangle; My = Sy * I / (h/2)
    h = 0.1
    if sec.Iy > 0:
        c = np.sqrt(sec.Iy * 12 / max(sec.A / 0.05, 1e-6)) if sec.A > 0 else 0.05
        # use elastic modulus then shape factor 1.5
        My = Sy * sec.Iy / max(c / 2, 1e-6)
        return 1.5 * My
    return Sy * sec.A * 0.02  # fallback


def section_axial_yield(elem: BeamElement, mat: PlasticMaterial) -> float:
    return mat.yield_strength * elem.section.A


def recover_local_forces(elem: BeamElement, u: np.ndarray) -> np.ndarray:
    """Linear local end forces from current global u."""
    T = transformation_matrix(elem)
    k = local_stiffness(elem)
    dofs = np.concatenate(
        [u[elem.node_i.dof_indices()], u[elem.node_j.dof_indices()]]
    )
    return k @ (T @ dofs)


def update_hinge_states(
    elements: list[BeamElement],
    u: np.ndarray,
    materials: dict[int, PlasticMaterial],
    prev: dict[int, ElementPlasticState] | None = None,
    fail_ratio: float = 1.5,
) -> dict[int, ElementPlasticState]:
    """
    Update plastic hinge state from moment / axial utilization.
    Interaction: utilization = max(|M|/Mp, |N|/Ny) (conservative).
    """
    prev = prev or {}
    out: dict[int, ElementPlasticState] = {}

    for elem in elements:
        mat = materials.get(elem.id) or materials.get(-1)
        if mat is None:
            out[elem.id] = ElementPlasticState(elem_id=elem.id)
            continue

        f_loc = recover_local_forces(elem, u)
        My = max(abs(f_loc[4]), abs(f_loc[10]))
        Mz = max(abs(f_loc[5]), abs(f_loc[11]))
        M = max(My, Mz)
        N = abs(f_loc[0])

        Mp = section_plastic_moment(elem, mat)
        Ny = section_axial_yield(elem, mat)
        m_ratio = M / max(Mp, 1e-12)
        n_ratio = N / max(Ny, 1e-12)
        util = max(m_ratio, n_ratio)

        old = prev.get(elem.id, ElementPlasticState(elem_id=elem.id))
        state = old.state
        deg = old.degradation
        work = old.plastic_work

        if util >= fail_ratio:
            state = HingeState.FAILED
            deg = 0.05
        elif util >= 1.0:
            if state in (HingeState.ELASTIC, HingeState.YIELDING):
                state = HingeState.PLASTIC
            deg = max(0.15, 1.0 / util)
            # incremental plastic work proxy
            work += max(util - 1.0, 0.0) * Mp * 1e-4
        elif util >= 0.9:
            state = HingeState.YIELDING
            deg = 0.85
        else:
            if state not in (HingeState.PLASTIC, HingeState.FAILED):
                state = HingeState.ELASTIC
                deg = 1.0

        out[elem.id] = ElementPlasticState(
            elem_id=elem.id,
            state=state,
            M_ratio=m_ratio,
            N_ratio=n_ratio,
            plastic_work=work,
            degradation=deg,
        )
    return out
