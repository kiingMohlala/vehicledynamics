"""Phase 13.6 – Manufacturing Engineering validation (20 gates from task card core set)."""
from __future__ import annotations

import math
import tempfile
from pathlib import Path
import numpy as np

from .materials_database import MATERIALS, get_material
from .manufacturing_processes import select_process, PROCESSES
from .machining import estimate_cnc
from .welding import estimate_weld
from .composites import estimate_composite
from .additive import estimate_am
from .dfm import evaluate_dfm, check_wall_thickness, check_tool_access
from .dfa import evaluate_dfa
from .assembly_sequence import plan_assembly
from .tolerances import Tolerance, stack_up, clearance_analysis
from .cost_estimation import CostBreakdown
from .bill_of_materials import BOM, BOMItem
from .manufacturing_planner import ManufacturingPlanner, ManufacturingConfig
from .manufacturing_report import format_manufacturing_report


def _ok(name: str, passed: bool, detail: str = "") -> tuple[str, bool, str]:
    return name, passed, detail


def gate_material_database() -> tuple[str, bool, str]:
    return _ok("material_database", "steel" in MATERIALS and get_material("cfrp").composite)


def gate_process_selection() -> tuple[str, bool, str]:
    p = select_process("chassis", "steel")
    return _ok("process_selection", p in PROCESSES)


def gate_machining_estimation() -> tuple[str, bool, str]:
    e = estimate_cnc(100, 40, features=8)
    return _ok("machining_estimation", e.time_hours > 0 and e.cost > 0)


def gate_welding_estimation() -> tuple[str, bool, str]:
    e = estimate_weld(10.0, "MIG")
    return _ok("welding_estimation", e.time_hours > 0 and e.mass_kg > 0)


def gate_composite_layup() -> tuple[str, bool, str]:
    e = estimate_composite(1.5, n_plies=8)
    return _ok("composite_layup", e.laminate.thickness_mm > 0 and e.cost > 0)


def gate_additive_process() -> tuple[str, bool, str]:
    e = estimate_am(50, "FDM")
    return _ok("additive_process", e.time_hours > 0)


def gate_dfm_wall_thickness() -> tuple[str, bool, str]:
    issues = check_wall_thickness(0.5, "steel", "test")
    return _ok("dfm_wall_thickness", any(i.code == "WALL_THIN" for i in issues))


def gate_dfm_tool_access() -> tuple[str, bool, str]:
    issues = check_tool_access(100, 5, "deep_hole")
    return _ok("dfm_tool_access", any(i.severity in ("error", "warn") for i in issues))


def gate_dfa_part_count() -> tuple[str, bool, str]:
    r = evaluate_dfa([{"name": f"p{i}"} for i in range(20)])
    return _ok("dfa_part_count", r.part_count == 20 and r.estimated_time_hours > 0)


def gate_assembly_sequence() -> tuple[str, bool, str]:
    plan = plan_assembly(["body", "chassis", "engine"], {"engine": "powertrain", "body": "body", "chassis": "chassis"})
    return _ok("assembly_sequence", plan.part_order[0] == "chassis" and plan.total_time_hours > 0)


def gate_tolerance_stackup() -> tuple[str, bool, str]:
    t1 = Tolerance("a", 10.0, 0.1, 0.1)
    t2 = Tolerance("b", 5.0, 0.05, 0.05)
    s = stack_up([t1, t2])
    return _ok("tolerance_stackup", abs(s.nominal - 15.0) < 1e-9 and s.span > 0)


def gate_clearance_analysis() -> tuple[str, bool, str]:
    hole = Tolerance("H7", 20.0, 0.021, 0.0)
    shaft = Tolerance("g6", 20.0, 0.0, 0.020)
    r = clearance_analysis(hole, shaft)
    return _ok("clearance_analysis", r.fit in ("clearance", "transition", "interference"))


def gate_cost_estimation() -> tuple[str, bool, str]:
    c = CostBreakdown(material=100, machining=50, overhead=20)
    return _ok("cost_estimation", c.total == 170)


def gate_bom_generation() -> tuple[str, bool, str]:
    bom = BOM([BOMItem("P1", "arm", 2, "steel", "cnc", 1.5, 40.0)])
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "bom.csv"
        bom.to_csv(p)
        text = p.read_text()
    return _ok("bom_generation", "part_number" in text and bom.total_cost == 80.0)


def gate_manufacturing_report() -> tuple[str, bool, str]:
    r = ManufacturingPlanner().evaluate()
    text = format_manufacturing_report(r)
    return _ok("manufacturing_report", "Cost breakdown" in text and "BOM" in text)


def gate_digital_twin_integration() -> tuple[str, bool, str]:
    try:
        from vehicle_dynamics.cad import VehicleAssembly, AssemblyConfig
        asm = VehicleAssembly(AssemblyConfig()).build()
        r = ManufacturingPlanner().evaluate(asm)
        return _ok("digital_twin_integration", r.bom.items and r.total_cost > 0)
    except Exception as e:
        return _ok("digital_twin_integration", False, str(e))


def gate_repeatability() -> tuple[str, bool, str]:
    a = ManufacturingPlanner().evaluate()
    b = ManufacturingPlanner().evaluate()
    return _ok("repeatability", abs(a.total_cost - b.total_cost) < 1e-6)


def gate_performance_regression() -> tuple[str, bool, str]:
    import time
    t0 = time.perf_counter()
    for _ in range(30):
        ManufacturingPlanner().evaluate()
    ms = (time.perf_counter() - t0) * 1000
    return _ok("performance_regression", ms < 10000.0, f"{ms:.1f} ms")


def gate_no_nan_inf() -> tuple[str, bool, str]:
    r = ManufacturingPlanner().evaluate()
    ok = math.isfinite(r.total_cost) and math.isfinite(r.manufacturability_score)
    return _ok("no_nan_inf", ok)


def gate_regression_contract() -> tuple[str, bool, str]:
    try:
        from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
        sim = Simulation(SimulationConfig(dt=0.02))
        sim.load_scenario(ScenarioLibrary.straight_acceleration(0.2))
        res = sim.run(0.2)
        return _ok("regression_contract", len(res.telemetry.samples) > 0)
    except Exception as e:
        return _ok("regression_contract", False, str(e))


GATES = [
    gate_material_database,
    gate_process_selection,
    gate_machining_estimation,
    gate_welding_estimation,
    gate_composite_layup,
    gate_additive_process,
    gate_dfm_wall_thickness,
    gate_dfm_tool_access,
    gate_dfa_part_count,
    gate_assembly_sequence,
    gate_tolerance_stackup,
    gate_clearance_analysis,
    gate_cost_estimation,
    gate_bom_generation,
    gate_manufacturing_report,
    gate_digital_twin_integration,
    gate_repeatability,
    gate_performance_regression,
    gate_no_nan_inf,
    gate_regression_contract,
]


def run_phase136_validation(verbose: bool = True) -> bool:
    results = []
    for g in GATES:
        name, passed, detail = g()
        results.append((name, passed, detail))
        if verbose:
            status = "PASS" if passed else "FAIL"
            extra = f"  ({detail})" if detail else ""
            print(f"  {name:28s}: {status}{extra}")
    n_pass = sum(1 for _, p, _ in results if p)
    n = len(results)
    if verbose:
        print()
        print("=" * 41)
        if n_pass == n:
            print("ALL TESTS PASSED")
            print("Phase 13.6 Status: IMPLEMENTATION VALIDATED ✓")
        else:
            print(f"FAILED: {n_pass}/{n}")
            print("Phase 13.6 Status: NOT VALIDATED")
    return n_pass == n


if __name__ == "__main__":
    raise SystemExit(0 if run_phase136_validation() else 1)
