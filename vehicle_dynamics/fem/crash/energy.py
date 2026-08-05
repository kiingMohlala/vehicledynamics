"""Crash energy accounting."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_dynamics.fem.assembler import Model
from vehicle_dynamics.fem.stiffness import global_stiffness
from .plastic_hinge import ElementPlasticState


@dataclass
class EnergyAccount:
    kinetic_initial: float = 0.0
    elastic_strain: float = 0.0
    plastic_work: float = 0.0
    absorbed: float = 0.0
    residual_kinetic: float = 0.0
    crush_distance: float = 0.0

    @property
    def balance_error(self) -> float:
        """|E_k - (U_e + U_p + E_k_res)| / max(E_k, eps)"""
        rhs = self.elastic_strain + self.plastic_work + self.residual_kinetic
        return abs(self.kinetic_initial - rhs) / max(self.kinetic_initial, 1e-9)


def elastic_strain_energy(model: Model, u: np.ndarray) -> float:
    """½ uᵀ K u using linear assembly."""
    K = np.zeros((model.ndof, model.ndof))
    for elem in model.elements:
        ke = global_stiffness(elem)
        dofs = np.concatenate([elem.node_i.dof_indices(), elem.node_j.dof_indices()])
        for a in range(12):
            for b in range(12):
                K[dofs[a], dofs[b]] += ke[a, b]
    return 0.5 * float(u @ (K @ u))


def total_plastic_work(states: dict[int, ElementPlasticState]) -> float:
    return float(sum(s.plastic_work for s in states.values()))


def impact_kinetic_energy(mass_kg: float, speed_mps: float) -> float:
    return 0.5 * mass_kg * speed_mps**2


def account_energy(
    model: Model,
    u: np.ndarray,
    states: dict[int, ElementPlasticState],
    mass_kg: float,
    speed_mps: float,
    crush_distance: float,
) -> EnergyAccount:
    Ek = impact_kinetic_energy(mass_kg, speed_mps)
    Ue = elastic_strain_energy(model, u)
    Up = total_plastic_work(states)
    absorbed = Ue + Up
    # residual KE proxy: energy not absorbed (clamped ≥ 0)
    Ek_res = max(Ek - absorbed, 0.0)
    return EnergyAccount(
        kinetic_initial=Ek,
        elastic_strain=Ue,
        plastic_work=Up,
        absorbed=absorbed,
        residual_kinetic=Ek_res,
        crush_distance=crush_distance,
    )
