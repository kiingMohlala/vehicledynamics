"""Phase 8.1 – Space-frame / roll-cage validation suite."""

from __future__ import annotations

import numpy as np

from .assembler import Model
from .material import steel, AISI_4130, stainless_304, aluminium_6061, custom_material
from .section import tube
from .tube_library import tube_38x2, tube_32x2, tube_properties_summary, mass_per_metre
from .constraints import fix_node, apply_force
from .solver import solve_static
from .cage_builder import CageBuilder, CageParams, build_default_cage
from .load_cases import (
    torsional_rig,
    cornering,
    braking,
    harness_load,
)
from .mass_properties import compute_mass_properties
from .report import recover_element_stresses, format_report
from .visualization import plot_deformed


def test_zero_length_rejected() -> tuple[bool, dict]:
    model = Model()
    n0 = model.add_node(0, 0, 0)
    try:
        model.add_beam(n0, n0, steel(), tube_38x2())
        return False, {"msg": "should have raised"}
    except ValueError as e:
        return True, {"msg": str(e)}


def test_duplicate_node_rejected() -> tuple[bool, dict]:
    model = Model()
    model.add_node(1.0, 2.0, 3.0)
    try:
        model.add_node(1.0, 2.0, 3.0)
        return False, {"msg": "should have raised"}
    except ValueError as e:
        return True, {"msg": str(e)}


def test_underconstrained_detected() -> tuple[bool, dict]:
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(1, 0, 0)
    model.add_beam(n0, n1, steel(), tube_38x2())
    F = np.zeros(model.ndof)
    apply_force(F, n1, fx=10)
    res = solve_static(model, F)
    ok = not res.success and (
        "Under-constrained" in res.message or "Ill-conditioned" in res.message
        or "Singular" in res.message
    )
    return ok, {"message": res.message}


def test_reaction_equilibrium() -> tuple[bool, dict]:
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(2, 0, 0)
    model.add_beam(n0, n1, steel(), tube_38x2())
    fix_node(n0)
    F = np.zeros(model.ndof)
    apply_force(F, n1, fy=500, fz=-800)
    res = solve_static(model, F)
    Ry = res.reactions[n0.dof_indices()[1]]
    Rz = res.reactions[n0.dof_indices()[2]]
    ok = res.success and abs(Ry + 500) < 1e-2 and abs(Rz - 800) < 1e-2
    return ok, {"Ry": Ry, "Rz": Rz}


def test_material_library() -> tuple[bool, dict]:
    mats = [steel(), AISI_4130(), stainless_304(), aluminium_6061()]
    ok = all(m.E > 0 and m.G > 0 and m.rho > 0 and m.yield_strength > 0 for m in mats)
    custom = custom_material("test", E=100e9, nu=0.3, rho=5000)
    ok = ok and abs(custom.G - 100e9 / 2.6) < 1.0
    return ok, {"n_materials": len(mats), "custom_G": custom.G}


def test_tube_properties() -> tuple[bool, dict]:
    sec = tube_38x2()
    props = tube_properties_summary(sec, AISI_4130())
    # Analytical A for 38.1 x 2 mm
    od, wall = 0.0381, 0.002
    ro, ri = od / 2, od / 2 - wall
    A_th = np.pi * (ro**2 - ri**2)
    ok = abs(sec.A - A_th) / A_th < 1e-12
    ok = ok and props["mass_per_m_kg"] > 0
    return ok, {"A": sec.A, "A_th": A_th, "kg_per_m": props["mass_per_m_kg"]}


def test_mass_calculation() -> tuple[bool, dict]:
    model = build_default_cage()
    mass = compute_mass_properties(model)
    ok = mass.total_mass_kg > 10.0 and mass.total_mass_kg < 500.0
    ok = ok and mass.n_elements > 20 and mass.total_length_m > 10.0
    return ok, {
        "mass_kg": mass.total_mass_kg,
        "length_m": mass.total_length_m,
        "n_elem": mass.n_elements,
    }


def test_torsional_rig() -> tuple[bool, dict]:
    model = build_default_cage()
    res, metrics = torsional_rig(model)
    ok = res.success and np.isfinite(metrics["stiffness_Nm_per_deg"])
    ok = ok and abs(metrics["twist_deg"]) > 1e-9
    ok = ok and metrics["stiffness_Nm_per_deg"] > 0
    return ok, metrics


def test_cornering_finite() -> tuple[bool, dict]:
    model = build_default_cage()
    res, metrics = cornering(model)
    ok = res.success and np.all(np.isfinite(res.u))
    return ok, {"success": res.success, **metrics}


def test_braking_finite() -> tuple[bool, dict]:
    model = build_default_cage()
    res, metrics = braking(model)
    ok = res.success and np.all(np.isfinite(res.u))
    return ok, {"success": res.success, **metrics}


def test_harness_finite() -> tuple[bool, dict]:
    model = build_default_cage()
    res, metrics = harness_load(model)
    ok = res.success and np.all(np.isfinite(res.u))
    stresses = recover_element_stresses(model, res)
    ok = ok and len(stresses) > 0
    return ok, {
        "success": res.success,
        "peak_MPa": stresses[0].von_mises_Pa / 1e6 if stresses else None,
        **metrics,
    }


def test_visualization() -> tuple[bool, dict]:
    model = build_default_cage()
    res, _ = torsional_rig(model)
    if not res.success:
        return False, {"msg": res.message}
    out = plot_deformed(model, res, path=None, title="torsion")
    ok = out in ("memory", "matplotlib_unavailable") or out.endswith(".png")
    return ok, {"out": out}


def test_no_nan_inf_cage() -> tuple[bool, dict]:
    model = build_default_cage()
    res, _ = torsional_rig(model)
    ok = res.success and np.all(np.isfinite(res.u)) and np.all(np.isfinite(res.reactions))
    return ok, {"success": res.success, "max_u": res.max_displacement}


def run_phase81_validation() -> bool:
    print("=== Phase 8.1 Space-Frame & Roll-Cage Validation ===\n")
    tests = [
        ("zero_length_rejected", test_zero_length_rejected),
        ("duplicate_node_rejected", test_duplicate_node_rejected),
        ("underconstrained_detected", test_underconstrained_detected),
        ("reaction_equilibrium", test_reaction_equilibrium),
        ("material_library", test_material_library),
        ("tube_properties", test_tube_properties),
        ("mass_calculation", test_mass_calculation),
        ("torsional_rig", test_torsional_rig),
        ("cornering_finite", test_cornering_finite),
        ("braking_finite", test_braking_finite),
        ("harness_finite", test_harness_finite),
        ("visualization", test_visualization),
        ("no_nan_inf_cage", test_no_nan_inf_cage),
    ]
    all_pass = True
    results = []
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        status = "PASS" if ok else "FAIL"
        print(f"{name:36} : {status}")
        for k, v in list(diag.items())[:6]:
            print(f"    {k}: {v}")
        results.append((name, ok))
        if not ok:
            all_pass = False

    n_pass = sum(1 for _, o in results if o)
    print(f"\n=========================================")
    print(f"Tests Passed : {n_pass} / {len(results)}")
    print(f"Tests Failed : {len(results) - n_pass}")
    print("ALL TESTS PASSED" if all_pass else "SOME FAILED")
    if all_pass:
        print("\nPhase 8.1 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase81_validation()
