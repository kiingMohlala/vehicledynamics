"""High-level structural solver for chassis engineering."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import numpy as np

from .materials import steel, StructuralMaterial
from .load_cases import LoadCase, LoadCases
from .static_solver import solve_frame, StaticSolution
from .chassis_metrics import (
    default_ladder_frame,
    compute_torsional_stiffness,
    compute_bending_stiffness,
)
from .modal_analysis import solve_modes, ModalResult
from .safety_factors import evaluate_safety, SafetyReport
from .beam_elements import cantilever_tip_deflection
from .shell_elements import default_body_shell


@dataclass
class StructuralConfig:
    wheelbase: float = 2.70
    track: float = 1.55
    material: StructuralMaterial = field(default_factory=steel)
    total_mass_proxy: float = 80.0  # frame mass for modal
    min_safety_factor: float = 1.5


@dataclass
class StructuralResult:
    load_case: str
    solution: StaticSolution
    torsional_stiffness: float
    bending_stiffness: float
    max_displacement: float
    max_von_mises: float
    safety: SafetyReport
    modal: ModalResult | None
    reactions_summary: dict[str, float]
    meta: dict[str, Any] = field(default_factory=dict)


class StructuralSolver:
    def __init__(self, config: StructuralConfig | None = None):
        self.config = config or StructuralConfig()

    def solve(
        self,
        chassis: Any = None,
        load_case: LoadCase | None = None,
    ) -> StructuralResult:
        cfg = self.config
        nodes, elements = default_ladder_frame(cfg.wheelbase, cfg.track)
        # optional chassis from assembly: ignore detailed geometry, use metrics frame
        lc = load_case or LoadCases.cornering(1.5)
        # map load tags onto frame nodes
        loads = {}
        for tag, f in lc.forces.items():
            if tag in nodes:
                loads[tag] = f
            elif tag in ("front_aero",):
                loads["FL"] = loads.get("FL", np.zeros(3)) + 0.5 * f
                loads["FR"] = loads.get("FR", np.zeros(3)) + 0.5 * f
            elif tag in ("rear_aero",):
                loads["RL"] = loads.get("RL", np.zeros(3)) + 0.5 * f
                loads["RR"] = loads.get("RR", np.zeros(3)) + 0.5 * f
            elif tag == "engine":
                loads["ML"] = loads.get("ML", np.zeros(3)) + 0.5 * f
                loads["MR"] = loads.get("MR", np.zeros(3)) + 0.5 * f
        # supports: fix one corner fully + stabilize
        fixed = ["RL"]
        # add soft constraints via fixing RR uz only through full fix if few loads
        if len(loads) <= 2:
            fixed = ["RL", "RR"]
        sol = solve_frame(nodes, elements, loads, fixed=fixed)
        Kt, _ = compute_torsional_stiffness(cfg.wheelbase, cfg.track)
        Kb, _ = compute_bending_stiffness(cfg.wheelbase, cfg.track)
        # stress proxy from max displacement * scale
        # sigma ~ E * (disp / characteristic length)
        char = max(cfg.wheelbase, 0.1)
        sigma = cfg.material.E * (sol.max_disp / char) * 0.1
        safety = evaluate_safety(sigma, axial_force=-abs(sum(np.linalg.norm(f) for f in loads.values()) * 0.1),
                                 length=0.5 * cfg.wheelbase, mat=cfg.material, min_sf=cfg.min_safety_factor)
        modal = solve_modes(nodes, elements, fixed=["RL", "RR"], total_mass=cfg.total_mass_proxy, n_modes=5)
        # reactions
        reactions = {}
        if sol.success:
            idx = {n: i for i, n in enumerate(sol.node_names)}
            for tag in fixed:
                if tag in idx:
                    base = 6 * idx[tag]
                    reactions[tag] = float(np.linalg.norm(sol.reactions[base:base+3]))
        return StructuralResult(
            load_case=lc.name,
            solution=sol,
            torsional_stiffness=Kt,
            bending_stiffness=Kb,
            max_displacement=sol.max_disp,
            max_von_mises=float(sigma),
            safety=safety,
            modal=modal,
            reactions_summary=reactions,
            meta={"n_nodes": len(nodes), "n_elements": len(elements)},
        )
