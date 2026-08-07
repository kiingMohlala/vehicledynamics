"""Phase 13.2 – Parametric Vehicle Assembly & CAD Architecture."""

from .component import Component
from .assembly import VehicleAssembly, AssemblyConfig
from .mass_properties import MassProperties, compute_mass_properties
from .interference import InterferenceHit, detect_interferences
from .packaging_solver import PackagingReport, evaluate_packaging
from .export import export_obj, export_stl, export_json_assembly
from .cad_report import format_cad_report
from .parametric_parts import (
    chassis_tub, body_shell, engine_block, battery_pack, wheel_tire, aero_wing,
)

__all__ = [
    "Component",
    "VehicleAssembly", "AssemblyConfig",
    "MassProperties", "compute_mass_properties",
    "InterferenceHit", "detect_interferences",
    "PackagingReport", "evaluate_packaging",
    "export_obj", "export_stl", "export_json_assembly",
    "format_cad_report",
    "chassis_tub", "body_shell", "engine_block", "battery_pack", "wheel_tire", "aero_wing",
]
