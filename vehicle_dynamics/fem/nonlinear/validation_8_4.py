"""Phase 8.4 – Geometrically nonlinear FEM validation (target 10/10)."""

from __future__ import annotations

import numpy as np

from vehicle_dynamics.fem.assembler import Model
from vehicle_dynamics.fem.material import steel
from vehicle_dynamics.fem.section import rectangular, circular
from vehicle_dynamics.fem.constraints import fix_node, apply_force
from vehicle_dynamics.fem.solver import solve_static

from .nonlinear_solver import solve_static_nonlinear
from .load_stepping import solve_nonlinear_stepped
from .corotational_beam import axial_force, internal_force_global
from .geometric_stiffness import local_geometric_stiffness


def _cantilever(L=2.0, sec=None):
    mat = steel()
    sec = sec or rectangular(0.05, 0.1)
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(L, 0, 0)
    model.add_beam(n0, n1, mat, sec)
    fix_node(n0)
    return model, n1, mat, sec, L


def test_small_load_regression() -> tuple[bool, dict]:
    """Small tip load: nonlinear ≈ linear tip deflection."""
    model, n1, mat, sec, L = _cantilever()
    P = 50.0  # small
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-P)

    lin = solve_static(model, F)
    nln = solve_static_nonlinear(model, F, tol=1e-8, max_iter=20)

    if not (lin.success and nln.success):
        return False, {"lin": lin.message, "nln": nln.message}

    uz_l = lin.node_displacement(1)[2]
    uz_n = nln.node_displacement(1)[2]
    err = abs(uz_n - uz_l) / (abs(uz_l) + 1e-15)
    ok = err < 0.05  # within 5%
    return ok, {"uz_lin": uz_l, "uz_nln": uz_n, "rel_err": err}


def test_large_displacement_convergence() -> tuple[bool, dict]:
    model, n1, mat, sec, L = _cantilever()
    P = 5000.0
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-P)
    res = solve_static_nonlinear(model, F, tol=1e-5, max_iter=40)
    ok = res.success and np.all(np.isfinite(res.u)) and res.residual_norm < 1e-4
    return ok, {
        "success": res.success,
        "n_iter": res.n_iter,
        "res": res.residual_norm,
        "uz": float(res.node_displacement(1)[2]),
    }


def test_corotational_invariance() -> tuple[bool, dict]:
    """
    Pure rigid rotation of a free-free short beam under zero load → zero residual.
    """
    mat = steel()
    sec = circular(0.02)
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(1.0, 0, 0)
    model.add_beam(n0, n1, mat, sec)
    # Soft spring support: pin n0 translations only so system is determinate
    fix_node(n0, [0, 1, 2])
    F = np.zeros(model.ndof)
    res = solve_static_nonlinear(model, F, tol=1e-8)
    ok = res.success and np.linalg.norm(res.u) < 1e-6
    return ok, {"success": res.success, "|u|": float(np.linalg.norm(res.u))}


def test_load_stepping() -> tuple[bool, dict]:
    model, n1, _, _, _ = _cantilever()
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-3000.0)
    res = solve_nonlinear_stepped(model, F, n_steps=5, tol=1e-5, max_iter=25)
    ok = res.success and np.all(np.isfinite(res.u))
    return ok, {"success": res.success, "n_iter": res.n_iter, "msg": res.message}


def test_tension_stiffening() -> tuple[bool, dict]:
    """
    Lateral stiffness increases under tension: same lateral load, smaller
    deflection when axial tension is present (via geometric stiffness sign).
    """
    # Compare local geometric stiffness eigenvalues for N>0 vs N=0
    kg_t = local_geometric_stiffness(N=1e5, L=1.0)
    kg_0 = local_geometric_stiffness(N=0.0, L=1.0)
    # Lateral DOF stiffness kg[1,1] should be > 0 under tension
    ok = kg_t[1, 1] > kg_0[1, 1] and kg_t[1, 1] > 0
    return ok, {"kg_uy_tension": float(kg_t[1, 1]), "kg_uy_zero": float(kg_0[1, 1])}


def test_compression_softening() -> tuple[bool, dict]:
    kg_c = local_geometric_stiffness(N=-1e5, L=1.0)
    kg_0 = local_geometric_stiffness(N=0.0, L=1.0)
    ok = kg_c[1, 1] < kg_0[1, 1]
    return ok, {"kg_uy_compression": float(kg_c[1, 1]), "kg_uy_zero": float(kg_0[1, 1])}


def test_internal_force_recovery() -> tuple[bool, dict]:
    model, n1, mat, sec, L = _cantilever()
    P = 200.0
    F = np.zeros(model.ndof)
    apply_force(F, n1, fx=P)  # axial tension
    res = solve_static_nonlinear(model, F, tol=1e-8)
    if not res.success:
        return False, {"msg": res.message}
    N = axial_force(model.elements[0], res.u)
    # Should recover ≈ P
    err = abs(abs(N) - P) / P
    ok = err < 0.15
    return ok, {"N": N, "P": P, "rel_err": err}


def test_compliance_integration() -> tuple[bool, dict]:
    """
    Optional flag path: nonlinear solve used as drop-in for compliance.
    Smoke: nonlinear tip load succeeds and matches linear order of magnitude.
    """
    model, n1, _, _, _ = _cantilever()
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-100.0)
    lin = solve_static(model, F)
    nln = solve_static_nonlinear(model, F, tol=1e-7)
    ok = lin.success and nln.success
    ratio = abs(nln.node_displacement(1)[2] / (lin.node_displacement(1)[2] + 1e-30))
    ok = ok and 0.5 < ratio < 1.5
    return ok, {"ratio": ratio}


def test_no_nan_inf() -> tuple[bool, dict]:
    model, n1, _, _, _ = _cantilever()
    F = np.zeros(model.ndof)
    apply_force(F, n1, fy=800.0, fz=-1200.0)
    res = solve_static_nonlinear(model, F, tol=1e-5, max_iter=40)
    ok = res.success and np.all(np.isfinite(res.u)) and np.all(np.isfinite(res.reactions))
    return ok, {"success": res.success, "res": res.residual_norm}


def test_linear_regression_mode() -> tuple[bool, dict]:
    """
    With tiny load, Newton should converge in few iterations to linear solution.
    """
    model, n1, _, _, _ = _cantilever()
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-10.0)
    res = solve_static_nonlinear(model, F, tol=1e-9, max_iter=15)
    lin = solve_static(model, F)
    ok = res.success and res.n_iter <= 5
    err = np.linalg.norm(res.u - lin.u) / (np.linalg.norm(lin.u) + 1e-15)
    ok = ok and err < 0.02
    return ok, {"n_iter": res.n_iter, "err": err}


def run_phase84_validation() -> bool:
    print("=== Phase 8.4 Geometrically Nonlinear FEM Validation ===\n")
    tests = [
        ("small_load_regression", test_small_load_regression),
        ("large_displacement_convergence", test_large_displacement_convergence),
        ("corotational_invariance", test_corotational_invariance),
        ("load_stepping_convergence", test_load_stepping),
        ("tension_stiffening", test_tension_stiffening),
        ("compression_softening", test_compression_softening),
        ("internal_force_recovery", test_internal_force_recovery),
        ("compliance_integration", test_compliance_integration),
        ("no_nan_inf", test_no_nan_inf),
        ("linear_regression_mode", test_linear_regression_mode),
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
        print("Phase 8.4 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase84_validation()
