"""Phase 13.5 – Structural Analysis & Chassis Engineering."""

from .materials import StructuralMaterial, steel, aluminum, titanium, cfrp, MATERIALS
from .load_cases import LoadCase, LoadCases
from .structural_solver import StructuralSolver, StructuralConfig, StructuralResult
from .chassis_metrics import compute_torsional_stiffness, compute_bending_stiffness
from .modal_analysis import solve_modes, ModalResult
from .safety_factors import evaluate_safety, SafetyReport
from .structures_report import format_structures_report
from .beam_elements import local_beam_stiffness, cantilever_tip_deflection
from .shell_elements import ShellSection, default_body_shell
from .buckling import euler_critical_load
from .fatigue import basquin_life, miner_damage

__all__ = [
    "StructuralMaterial", "steel", "aluminum", "titanium", "cfrp", "MATERIALS",
    "LoadCase", "LoadCases",
    "StructuralSolver", "StructuralConfig", "StructuralResult",
    "compute_torsional_stiffness", "compute_bending_stiffness",
    "solve_modes", "ModalResult",
    "evaluate_safety", "SafetyReport",
    "format_structures_report",
    "local_beam_stiffness", "cantilever_tip_deflection",
    "ShellSection", "default_body_shell",
    "euler_critical_load",
    "basquin_life", "miner_damage",
]
