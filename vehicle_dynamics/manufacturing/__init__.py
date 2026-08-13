"""Phase 13.6 – Manufacturing Engineering & Production Planning."""

from .manufacturing_planner import ManufacturingPlanner, ManufacturingConfig, ManufacturingResult
from .dfm import evaluate_dfm, DFMReport
from .dfa import evaluate_dfa, DFAReport
from .assembly_sequence import plan_assembly, AssemblyPlan
from .bill_of_materials import BOM, BOMItem
from .cost_estimation import CostBreakdown
from .tolerances import Tolerance, stack_up, clearance_analysis
from .welding import estimate_weld
from .composites import estimate_composite
from .machining import estimate_cnc
from .additive import estimate_am
from .manufacturing_report import format_manufacturing_report
from .materials_database import MATERIALS, get_material
from .manufacturing_processes import select_process, PROCESSES

__all__ = [
    "ManufacturingPlanner", "ManufacturingConfig", "ManufacturingResult",
    "evaluate_dfm", "DFMReport",
    "evaluate_dfa", "DFAReport",
    "plan_assembly", "AssemblyPlan",
    "BOM", "BOMItem",
    "CostBreakdown",
    "Tolerance", "stack_up", "clearance_analysis",
    "estimate_weld", "estimate_composite", "estimate_cnc", "estimate_am",
    "format_manufacturing_report",
    "MATERIALS", "get_material",
    "select_process", "PROCESSES",
]
