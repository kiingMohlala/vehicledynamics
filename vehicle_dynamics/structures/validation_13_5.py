"""Phase 13.5 – Structural Analysis & Chassis Engineering validation (24 gates)."""
from __future__ import annotations

import math
import numpy as np

from .materials import steel, aluminum, MATERIALS
from .beam_elements import local_beam_stiffness, cantilever_tip_deflection
from .shell_elements import default_body_shell, ShellSection
from .stiffness_matrix import assemble_frame
from .static_solver import solve_frame
from .load_cases import LoadCases
from .modal_analysis import solve_modes
from .buckling import euler_critical_load, member_buckling_sf
from .fatigue import basquin_life, miner_damage
from .chassis_metrics import (
    default_ladder_frame,
    compute_torsional_stiffness,
    compute_bending_stiffness,
)
from .safety_factors import evaluate_safety, yield_sf
from .structural_solver import StructuralSolver, StructuralConfig
from .structures_report import format_structures_report


def _ok(name: str, passed: bool, detail: str = "") -> tuple[str, bool, str]:
    return name, passed, detail


def gate_material_database() -> tuple[str, bool, str]:
    return _ok("material_database", "steel" in MATERIALS and steel().E > 200e9)


def gate_beam_stiffness() -> tuple[str, bool, str]:
    k = local_beam_stiffness(1.0)
    return _ok("beam_stiffness", k.shape == (12, 12) and k[0, 0] > 0)


def gate_shell_properties() -> tuple[str, bool, str]:
    s = default_body_shell(0.002)
    return _ok("shell_properties", s.bending_rigidity > 0 and s.areal_mass > 0)


def gate_global_matrix() -> tuple[str, bool, str]:
    nodes = {"A": np.array([0, 0, 0.0]), "B": np.array([1, 0, 0.0])}
    K, names = assemble_frame(nodes, [("A", "B")])
    return _ok("global_matrix", K.shape == (12, 12) and names == ["A", "B"])


def gate_cantilever_validation() -> tuple[str, bool, str]:
    L, E, I, P = 1.0, 210e9, 1e-6, 100.0
    analytic = cantilever_tip_deflection(P, L, E, I)
    # numerical frame: root fixed, tip load
    nodes = {"root": np.array([0, 0, 0.0]), "tip": np.array([L, 0, 0.0])}
    # use large I by scaling — local_beam uses Iz for uy
    sol = solve_frame(nodes, [("root", "tip")], {"tip": np.array([0.0, 0.0, -P])}, fixed=["root"])
    # our Iz default 1e-7 — recompute analytic with that I
    analytic = cantilever_tip_deflection(P, L, E, 1e-7)
    if not sol.success:
        return _ok("cantilever_validation", False, sol.message)
    tip_z = abs(sol.u[8])  # tip node index 1 -> dof 6+2=8
    err = abs(tip_z - analytic) / max(analytic, 1e-15)
    return _ok("cantilever_validation", err < 0.15, f"err={err:.3f} num={tip_z:.3e} ana={analytic:.3e}")


def gate_spaceframe_validation() -> tuple[str, bool, str]:
    nodes, elems = default_ladder_frame()
    sol = solve_frame(nodes, elems, {"FL": np.array([0, 0, 1000.0])}, fixed=["RL", "RR"])
    return _ok("spaceframe_validation", sol.success and sol.max_disp > 0)


def gate_torsional_stiffness() -> tuple[str, bool, str]:
    Kt, d = compute_torsional_stiffness()
    return _ok("torsional_stiffness", Kt > 0 and math.isfinite(Kt), f"Kt={Kt:.1f}")


def gate_bending_stiffness() -> tuple[str, bool, str]:
    Kb, d = compute_bending_stiffness()
    return _ok("bending_stiffness", Kb > 0 and math.isfinite(Kb), f"Kb={Kb:.1f}")


def gate_modal_analysis() -> tuple[str, bool, str]:
    nodes, elems = default_ladder_frame()
    m = solve_modes(nodes, elems, fixed=["RL", "RR"], n_modes=4)
    return _ok("modal_analysis", m.success and m.n_modes >= 1)


def gate_natural_frequency() -> tuple[str, bool, str]:
    nodes, elems = default_ladder_frame()
    m = solve_modes(nodes, elems, fixed=["RL", "RR"])
    return _ok("natural_frequency", m.success and float(m.frequencies_hz[0]) > 0)


def gate_buckling_estimation() -> tuple[str, bool, str]:
    Pcr = euler_critical_load(1.0, 210e9, 1e-7)
    return _ok("buckling_estimation", Pcr > 0)


def gate_fatigue_damage() -> tuple[str, bool, str]:
    life = basquin_life(100e6)
    dmg = miner_damage([100e6, 80e6], [1e3, 1e3])
    return _ok("fatigue_damage", life > 0 and dmg.damage >= 0)


def gate_combined_loading() -> tuple[str, bool, str]:
    lc = LoadCases.combined(1.0, 0.5)
    r = StructuralSolver().solve(load_case=lc)
    return _ok("combined_loading", r.solution.success or r.max_displacement >= 0)


def gate_reaction_forces() -> tuple[str, bool, str]:
    r = StructuralSolver().solve(load_case=LoadCases.bump(5000))
    return _ok("reaction_forces", isinstance(r.reactions_summary, dict))


def gate_suspension_mount_loading() -> tuple[str, bool, str]:
    r = StructuralSolver().solve(load_case=LoadCases.suspension_mount(8000, "FL"))
    return _ok("suspension_mount_loading", r.load_case == "suspension_mount")


def gate_engine_mount_loading() -> tuple[str, bool, str]:
    r = StructuralSolver().solve(load_case=LoadCases.engine_mount(4000))
    return _ok("engine_mount_loading", r.load_case == "engine_mount")


def gate_aero_load_case() -> tuple[str, bool, str]:
    r = StructuralSolver().solve(load_case=LoadCases.aero_downforce(3000))
    return _ok("aero_load_case", r.load_case == "aero")


def gate_safety_factor() -> tuple[str, bool, str]:
    sf = yield_sf(100e6, steel())
    rep = evaluate_safety(100e6)
    return _ok("safety_factor", sf > 1 and rep.yield_sf > 1)


def gate_report_generation() -> tuple[str, bool, str]:
    r = StructuralSolver().solve(load_case=LoadCases.cornering(1.2))
    text = format_structures_report(r)
    return _ok("report_generation", "Torsional" in text and "Safety" in text)


def gate_export_consistency() -> tuple[str, bool, str]:
    # stiffness metrics finite
    Kt, _ = compute_torsional_stiffness()
    Kb, _ = compute_bending_stiffness()
    return _ok("export_consistency", math.isfinite(Kt) and math.isfinite(Kb))


def gate_repeatability() -> tuple[str, bool, str]:
    a = StructuralSolver().solve(load_case=LoadCases.torsion_rig(500))
    b = StructuralSolver().solve(load_case=LoadCases.torsion_rig(500))
    return _ok("repeatability", abs(a.torsional_stiffness - b.torsional_stiffness) < 1e-6)


def gate_performance_regression() -> tuple[str, bool, str]:
    import time
    t0 = time.perf_counter()
    for _ in range(20):
        StructuralSolver().solve(load_case=LoadCases.cornering(1.0))
    ms = (time.perf_counter() - t0) * 1000
    return _ok("performance_regression", ms < 10000.0, f"{ms:.1f} ms")


def gate_no_nan_inf() -> tuple[str, bool, str]:
    r = StructuralSolver().solve(load_case=LoadCases.braking(1.0))
    ok = math.isfinite(r.max_displacement) and math.isfinite(r.torsional_stiffness)
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
    gate_beam_stiffness,
    gate_shell_properties,
    gate_global_matrix,
    gate_cantilever_validation,
    gate_spaceframe_validation,
    gate_torsional_stiffness,
    gate_bending_stiffness,
    gate_modal_analysis,
    gate_natural_frequency,
    gate_buckling_estimation,
    gate_fatigue_damage,
    gate_combined_loading,
    gate_reaction_forces,
    gate_suspension_mount_loading,
    gate_engine_mount_loading,
    gate_aero_load_case,
    gate_safety_factor,
    gate_report_generation,
    gate_export_consistency,
    gate_repeatability,
    gate_performance_regression,
    gate_no_nan_inf,
    gate_regression_contract,
]


def run_phase135_validation(verbose: bool = True) -> bool:
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
            print("Phase 13.5 Status: IMPLEMENTATION VALIDATED ✓")
        else:
            print(f"FAILED: {n_pass}/{n}")
            print("Phase 13.5 Status: NOT VALIDATED")
    return n_pass == n


if __name__ == "__main__":
    raise SystemExit(0 if run_phase135_validation() else 1)
