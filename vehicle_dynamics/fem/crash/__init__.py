"""Phase 8.5 – Crash & energy absorption (engineering-level)."""

from .material_plastic import PlasticMaterial, plastic_steel, plastic_4130, plastic_6061, plastic_stainless
from .plastic_hinge import HingeState, ElementPlasticState, update_hinge_states
from .crash_solver import CrashConfig, CrashResult, solve_crash_quasistatic
from .crash_load_cases import (
    frontal_impact,
    rear_impact,
    side_impact,
    roof_crush,
    harness_pull,
)
from .energy import EnergyAccount
from .intrusion import IntrusionMetrics, compute_intrusion
from .crash_report import format_crash_report

__all__ = [
    "PlasticMaterial",
    "plastic_steel",
    "plastic_4130",
    "plastic_6061",
    "plastic_stainless",
    "HingeState",
    "ElementPlasticState",
    "update_hinge_states",
    "CrashConfig",
    "CrashResult",
    "solve_crash_quasistatic",
    "frontal_impact",
    "rear_impact",
    "side_impact",
    "roof_crush",
    "harness_pull",
    "EnergyAccount",
    "IntrusionMetrics",
    "compute_intrusion",
    "format_crash_report",
]
