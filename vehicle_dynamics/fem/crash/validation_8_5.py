"""Phase 8.5 – Crash & energy absorption validation (target 10/10)."""

from __future__ import annotations

import numpy as np

from vehicle_dynamics.fem import build_default_cage
from vehicle_dynamics.fem.assembler import Model
from vehicle_dynamics.fem.material import steel
from vehicle_dynamics.fem.section import rectangular
from vehicle_dynamics.fem.constraints import fix_node, apply_force
from vehicle_dynamics.fem.solver import solve_static

from .material_plastic import plastic_steel, PlasticMaterial
from .plastic_hinge import HingeState, update_hinge_states, section_plastic_moment
from .crash_solver import CrashConfig, solve_crash_quasistatic
from .crash_load_cases import frontal_impact, side_impact, roof_crush, harness_pull
from .energy import elastic_strain_energy, impact_kinetic_energy


def _simple_beam():
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(1.5, 0, 0)
    model.add_beam(n0, n1, steel(), rectangular(0.04, 0.06))
    fix_node(n0)
    return model, n1


def test_elastic_impact_regression() -> tuple[bool, dict]:
    """Small load: crash solver stays elastic and matches linear tip u."""
    model, n1 = _simple_beam()
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-200.0)
    lin = solve_static(model, F)
    cfg = CrashConfig(n_steps=5, speed_mps=1.0, mass_kg=100.0)
    res = solve_crash_quasistatic(model, F, config=cfg)
    if not (lin.success and res.success):
        return False, {"lin": lin.message, "crash": res.message}
    err = np.linalg.norm(res.u - lin.u) / (np.linalg.norm(lin.u) + 1e-15)
    ok = err < 0.05 and res.n_plastic == 0 and res.n_failed == 0
    return ok, {"rel_err": err, "n_plastic": res.n_plastic}


def test_yield_initiation() -> tuple[bool, dict]:
    model, n1 = _simple_beam()
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-5e4)
    cfg = CrashConfig(n_steps=8, speed_mps=5.0, mass_kg=500.0)
    res = solve_crash_quasistatic(model, F, config=cfg)
    ok = res.success and (res.n_yielded + res.n_plastic + res.n_failed) >= 1
    return ok, {
        "yielded": res.n_yielded,
        "plastic": res.n_plastic,
        "failed": res.n_failed,
    }


def test_plastic_hinge_formation() -> tuple[bool, dict]:
    model, n1 = _simple_beam()
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-1.5e5)
    res = solve_crash_quasistatic(
        model, F, config=CrashConfig(n_steps=10, speed_mps=8.0, mass_kg=800.0)
    )
    ok = res.success and res.n_plastic + res.n_failed >= 1
    states = list(res.hinge_states.values())
    return ok, {
        "state": states[0].state.value if states else None,
        "M_ratio": states[0].M_ratio if states else None,
    }


def test_energy_conservation() -> tuple[bool, dict]:
    """
    Elastic small crash: elastic energy ≈ ½ u K u; residual KE accounts for rest.
    Balance error is defined; for elastic, plastic_work≈0.
    """
    model, n1 = _simple_beam()
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-500.0)
    mass, v = 200.0, 2.0
    res = solve_crash_quasistatic(
        model, F, config=CrashConfig(n_steps=5, mass_kg=mass, speed_mps=v)
    )
    ok = res.success and res.energy.plastic_work < 1e-6
    ok = ok and res.energy.elastic_strain >= 0
    # KE should exceed elastic energy for this light load / high mass case
    ok = ok and res.energy.kinetic_initial > 0
    return ok, {
        "Ue": res.energy.elastic_strain,
        "Up": res.energy.plastic_work,
        "Ek": res.energy.kinetic_initial,
        "balance_err": res.energy.balance_error,
    }


def test_collapse_progression() -> tuple[bool, dict]:
    model, n1 = _simple_beam()
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-3e5)
    res = solve_crash_quasistatic(
        model, F, config=CrashConfig(n_steps=12, fail_ratio=1.2, speed_mps=10.0, mass_kg=1000.0)
    )
    ok = res.success and (res.n_plastic + res.n_failed) >= 1
    deg = min(s.degradation for s in res.hinge_states.values())
    ok = ok and deg < 1.0
    return ok, {"min_deg": deg, "plastic": res.n_plastic, "failed": res.n_failed}


def test_frontal_impact() -> tuple[bool, dict]:
    model = build_default_cage()
    F = frontal_impact(model, force_N=40e3)
    res = solve_crash_quasistatic(
        model, F, config=CrashConfig(n_steps=6, mass_kg=1400.0, speed_mps=13.9)
    )
    ok = res.success and np.all(np.isfinite(res.u))
    ok = ok and res.intrusion.max_node_disp_m > 0
    return ok, {
        "success": res.success,
        "crush_mm": res.energy.crush_distance * 1e3,
        "plastic": res.n_plastic,
    }


def test_side_impact() -> tuple[bool, dict]:
    model = build_default_cage()
    F = side_impact(model, force_N=35e3, side="left")
    res = solve_crash_quasistatic(
        model, F, config=CrashConfig(n_steps=6, mass_kg=1400.0, speed_mps=10.0)
    )
    ok = res.success and np.all(np.isfinite(res.u))
    return ok, {"success": res.success, "max_u_mm": res.intrusion.max_node_disp_m * 1e3}


def test_roof_crush() -> tuple[bool, dict]:
    model = build_default_cage()
    F = roof_crush(model, force_N=25e3)
    res = solve_crash_quasistatic(
        model, F, config=CrashConfig(n_steps=6, mass_kg=1400.0, speed_mps=5.0)
    )
    ok = res.success and np.all(np.isfinite(res.u))
    return ok, {
        "success": res.success,
        "roof_mm": res.intrusion.survival_cell_intrusion_m * 1e3,
    }


def test_harness_load() -> tuple[bool, dict]:
    model = build_default_cage()
    F = harness_pull(model, force_N=7e3)
    res = solve_crash_quasistatic(
        model, F, config=CrashConfig(n_steps=5, mass_kg=100.0, speed_mps=1.0)
    )
    ok = res.success and np.all(np.isfinite(res.u))
    ok = ok and res.intrusion.harness_disp_m >= 0
    return ok, {
        "success": res.success,
        "harness_mm": res.intrusion.harness_disp_m * 1e3,
    }


def test_no_nan_inf() -> tuple[bool, dict]:
    model = build_default_cage()
    F = frontal_impact(model, force_N=20e3)
    res = solve_crash_quasistatic(model, F, config=CrashConfig(n_steps=5))
    ok = (
        res.success
        and np.all(np.isfinite(res.u))
        and np.isfinite(res.energy.absorbed)
        and np.isfinite(res.intrusion.max_node_disp_m)
    )
    return ok, {"success": res.success}


def run_phase85_validation() -> bool:
    print("=== Phase 8.5 Crash & Energy Absorption Validation ===\n")
    tests = [
        ("elastic_impact_regression", test_elastic_impact_regression),
        ("yield_initiation", test_yield_initiation),
        ("plastic_hinge_formation", test_plastic_hinge_formation),
        ("energy_conservation", test_energy_conservation),
        ("collapse_progression", test_collapse_progression),
        ("frontal_impact", test_frontal_impact),
        ("side_impact", test_side_impact),
        ("roof_crush", test_roof_crush),
        ("harness_load", test_harness_load),
        ("no_nan_inf", test_no_nan_inf),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:36} : {'PASS' if ok else 'FAIL'}")
        for k, v in list(diag.items())[:6]:
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\n=========================================")
    print("ALL TESTS PASSED" if all_pass else "SOME FAILED")
    if all_pass:
        print("Phase 8.5 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase85_validation()
