"""End-to-end manufacturing planner."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import numpy as np

from .materials_database import get_material, MATERIALS
from .manufacturing_processes import select_process, PROCESSES
from .dfm import evaluate_dfm, DFMReport
from .dfa import evaluate_dfa, DFAReport
from .assembly_sequence import plan_assembly, AssemblyPlan
from .welding import estimate_weld
from .composites import estimate_composite
from .machining import estimate_cnc
from .additive import estimate_am
from .cost_estimation import CostBreakdown
from .bill_of_materials import BOM, BOMItem
from .tolerances import Tolerance, stack_up, clearance_analysis


@dataclass
class ManufacturingConfig:
    labor_rate: float = 65.0
    overhead_fraction: float = 0.25
    weld_length_m: float = 25.0
    default_material: str = "aluminum"


@dataclass
class ManufacturingResult:
    bom: BOM
    cost: CostBreakdown
    dfm: DFMReport
    dfa: DFAReport
    assembly_plan: AssemblyPlan
    manufacturability_score: float
    assembly_time_hours: float
    total_cost: float
    meta: dict[str, Any] = field(default_factory=dict)

    def export_bom(self, path: str):
        return self.bom.to_csv(path)


class ManufacturingPlanner:
    def __init__(self, config: ManufacturingConfig | None = None):
        self.config = config or ManufacturingConfig()

    def evaluate(self, assembly: Any = None) -> ManufacturingResult:
        cfg = self.config
        # Build part list from assembly components or defaults
        parts_info = []
        if assembly is not None and hasattr(assembly, "components"):
            for c in assembly.components:
                cat = getattr(c, "category", "generic")
                mass = float(getattr(c, "mass", 1.0) or 1.0)
                mat = cfg.default_material
                if cat in ("chassis", "suspension"):
                    mat = "steel"
                elif cat in ("aero", "wing"):
                    mat = "cfrp"
                elif cat == "battery":
                    mat = "aluminum"
                process = select_process(cat, mat)
                size = getattr(c, "size", np.array([0.1, 0.1, 0.1]))
                thickness_mm = float(min(size) * 1000) if hasattr(size, "__len__") else 2.0
                thickness_mm = min(max(thickness_mm / 20.0, 1.0), 8.0)  # rough panel thickness proxy
                parts_info.append({
                    "name": c.name,
                    "category": cat,
                    "material": mat,
                    "process": process,
                    "mass_kg": mass,
                    "thickness_mm": thickness_mm,
                    "depth_mm": float(size[2] * 1000) if hasattr(size, "__len__") else 20.0,
                    "diameter_mm": 8.0,
                    "spacing_mm": 20.0,
                })
        else:
            parts_info = [
                {"name": "chassis", "category": "chassis", "material": "steel", "process": "tube", "mass_kg": 180, "thickness_mm": 2.0, "depth_mm": 30, "diameter_mm": 10, "spacing_mm": 25},
                {"name": "body", "category": "body", "material": "aluminum", "process": "sheet", "mass_kg": 120, "thickness_mm": 1.5, "depth_mm": 15, "diameter_mm": 6, "spacing_mm": 20},
                {"name": "wishbone_FL", "category": "suspension", "material": "steel", "process": "cnc", "mass_kg": 4, "thickness_mm": 5, "depth_mm": 40, "diameter_mm": 12, "spacing_mm": 30},
                {"name": "rear_wing", "category": "aero", "material": "cfrp", "process": "composite", "mass_kg": 8, "thickness_mm": 2.0, "depth_mm": 10, "diameter_mm": 5, "spacing_mm": 15},
                {"name": "upright_FL", "category": "suspension", "material": "aluminum", "process": "cnc", "mass_kg": 3.5, "thickness_mm": 8, "depth_mm": 50, "diameter_mm": 10, "spacing_mm": 22},
                {"name": "bracket_proto", "category": "prototype", "material": "abs_am", "process": "am", "mass_kg": 0.2, "thickness_mm": 2.0, "depth_mm": 20, "diameter_mm": 5, "spacing_mm": 12},
            ]

        # BOM + costs
        bom = BOM()
        cost = CostBreakdown()
        for i, p in enumerate(parts_info, 1):
            mat = get_material(p["material"])
            unit = mat.cost_per_kg * p["mass_kg"]
            process = p["process"]
            # process adders
            if process == "cnc":
                est = estimate_cnc(p["mass_kg"] / mat.density * 1e6 * 1.5, p["mass_kg"] / mat.density * 1e6, features=12)
                cost.machining += est.cost
                unit += est.cost
            elif process == "weld" or process == "tube":
                pass  # frame weld rolled up later
            elif process in ("composite", "infusion"):
                area = max(p["mass_kg"] / 3.0, 0.2)
                est = estimate_composite(area, n_plies=6)
                cost.composite += est.cost
                unit += est.cost * 0.3
            elif process == "am":
                vol = p["mass_kg"] / mat.density * 1e6
                est = estimate_am(vol)
                cost.additive += est.cost
                unit += est.cost
            cost.material += mat.cost_per_kg * p["mass_kg"]
            bom.items.append(BOMItem(
                part_number=f"P{i:04d}",
                name=p["name"],
                qty=1,
                material=p["material"],
                process=process,
                mass_kg=p["mass_kg"],
                unit_cost=unit,
            ))

        weld = estimate_weld(cfg.weld_length_m)
        cost.welding += weld.cost
        cost.assembly += evaluate_dfa(parts_info).estimated_time_hours * cfg.labor_rate
        cost.overhead = cfg.overhead_fraction * (cost.total - cost.overhead)

        dfm = evaluate_dfm(parts_info)
        dfa = evaluate_dfa(parts_info)
        categories = {p["name"]: p["category"] for p in parts_info}
        plan = plan_assembly([p["name"] for p in parts_info], categories)

        mfg_score = 0.5 * dfm.score + 0.5 * dfa.score
        return ManufacturingResult(
            bom=bom,
            cost=cost,
            dfm=dfm,
            dfa=dfa,
            assembly_plan=plan,
            manufacturability_score=mfg_score,
            assembly_time_hours=plan.total_time_hours,
            total_cost=cost.total,
            meta={"n_parts": len(parts_info), "weld_hours": weld.time_hours},
        )
