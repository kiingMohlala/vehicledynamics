"""Phase 8.0 – Beam FEM foundation validation."""

from __future__ import annotations

import numpy as np
from .assembler import Model
from .material import steel
from .section import rectangular, circular
from .constraints import fix_node, pin_node, apply_force
from .solver import solve_static


def test_cantilever():
    L, P = 2.0, 1000.0
    mat = steel()
    sec = rectangular(0.05, 0.1)
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(L, 0, 0)
    model.add_beam(n0, n1, mat, sec)
    fix_node(n0)
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-P)
    res = solve_static(model, F)
    if not res.success:
        return False, {"msg": res.message}
    uz = res.node_displacement(1)[2]
    delta_theory = -P * L**3 / (3 * mat.E * sec.Iy)
    err = abs(uz - delta_theory) / abs(delta_theory)
    return err < 1e-6, {"uz": uz, "theory": delta_theory, "rel_err": err}


def test_simply_supported():
    L, P = 4.0, 2000.0
    mat = steel()
    sec = rectangular(0.04, 0.08)
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(L / 2, 0, 0)
    n2 = model.add_node(L, 0, 0)
    model.add_beam(n0, n1, mat, sec)
    model.add_beam(n1, n2, mat, sec)
    pin_node(n0)
    pin_node(n2)
    n0.fixed[3] = True
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-P)
    res = solve_static(model, F)
    if not res.success:
        return False, {"msg": res.message}
    uz = res.node_displacement(1)[2]
    delta_theory = -P * L**3 / (48 * mat.E * sec.Iy)
    err = abs(uz - delta_theory) / abs(delta_theory)
    return err < 0.02, {"uz": uz, "theory": delta_theory, "rel_err": err}


def test_axial_tension():
    L, P = 1.5, 50e3
    mat = steel()
    sec = circular(0.02)
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(L, 0, 0)
    model.add_beam(n0, n1, mat, sec)
    fix_node(n0)
    F = np.zeros(model.ndof)
    apply_force(F, n1, fx=P)
    res = solve_static(model, F)
    ux = res.node_displacement(1)[0]
    theory = P * L / (mat.E * sec.A)
    err = abs(ux - theory) / theory
    return err < 1e-9 and res.success, {"ux": ux, "theory": theory, "rel_err": err}


def test_pure_bending():
    L, M = 1.0, 500.0
    mat = steel()
    sec = rectangular(0.05, 0.05)
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(L, 0, 0)
    model.add_beam(n0, n1, mat, sec)
    fix_node(n0)
    F = np.zeros(model.ndof)
    apply_force(F, n1, my=M)
    res = solve_static(model, F)
    disp = res.node_displacement(1)
    ry, uz = disp[4], disp[2]
    ry_th = M * L / (mat.E * sec.Iy)
    uz_th = M * L**2 / (2 * mat.E * sec.Iy)
    ok = abs(abs(ry) - abs(ry_th)) / abs(ry_th) < 1e-6
    ok = ok and abs(abs(uz) - abs(uz_th)) / abs(uz_th) < 1e-6
    return ok, {"ry": ry, "ry_th": ry_th, "uz": uz, "uz_th": uz_th}


def test_symmetry():
    L = 2.0
    mat = steel()
    sec = circular(0.03)
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(L / 2, 0, 0)
    n2 = model.add_node(L, 0, 0)
    model.add_beam(n0, n1, mat, sec)
    model.add_beam(n1, n2, mat, sec)
    pin_node(n0)
    pin_node(n2)
    n0.fixed[3] = True
    F = np.zeros(model.ndof)
    apply_force(F, n1, fz=-1000)
    res = solve_static(model, F)
    R0 = res.reactions[n0.dof_indices()[2]]
    R2 = res.reactions[n2.dof_indices()[2]]
    ok = abs(R0 - R2) < 1e-6 and abs(R0 + R2 - 1000) < 1e-3
    return ok, {"R0": R0, "R2": R2}


def test_reaction_equilibrium():
    L, P = 3.0, 1500.0
    mat = steel()
    sec = rectangular(0.06, 0.06)
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(L, 0, 0)
    model.add_beam(n0, n1, mat, sec)
    fix_node(n0)
    F = np.zeros(model.ndof)
    apply_force(F, n1, fy=P, fz=-P)
    res = solve_static(model, F)
    Ry = res.reactions[n0.dof_indices()[1]]
    Rz = res.reactions[n0.dof_indices()[2]]
    ok = abs(Ry + P) < 1e-3 and abs(Rz - P) < 1e-3
    return ok, {"Ry": Ry, "Rz": Rz}


def test_mesh_refinement():
    L, P = 2.0, 1000.0
    mat = steel()
    sec = rectangular(0.05, 0.1)
    theory = -P * L**3 / (3 * mat.E * sec.Iy)

    def tip(n_elem):
        model = Model()
        nodes = [model.add_node(i * L / n_elem, 0, 0) for i in range(n_elem + 1)]
        for i in range(n_elem):
            model.add_beam(nodes[i], nodes[i + 1], mat, sec)
        fix_node(nodes[0])
        F = np.zeros(model.ndof)
        apply_force(F, nodes[-1], fz=-P)
        res = solve_static(model, F)
        return res.node_displacement(n_elem)[2]

    err1 = abs(tip(1) - theory) / abs(theory)
    err4 = abs(tip(4) - theory) / abs(theory)
    return err1 < 1e-6 and err4 < 1e-6, {"err_1": err1, "err_4": err4}


def test_no_singular():
    mat = steel()
    sec = circular(0.02)
    model = Model()
    n0 = model.add_node(0, 0, 0)
    n1 = model.add_node(1, 0, 0)
    model.add_beam(n0, n1, mat, sec)
    F = np.zeros(model.ndof)
    apply_force(F, n1, fx=100)
    res_free = solve_static(model, F)
    fix_node(n0)
    res_fix = solve_static(model, F)
    ok = (not res_free.success) and res_fix.success
    return ok, {"free_msg": res_free.message, "fix_ok": res_fix.success}


def test_no_nan_inf():
    mat = steel()
    sec = circular(0.025)
    model = Model()
    nodes = [model.add_node(i * 0.5, 0.1 * (i % 2), 0) for i in range(5)]
    for i in range(4):
        model.add_beam(nodes[i], nodes[i + 1], mat, sec)
    fix_node(nodes[0])
    F = np.zeros(model.ndof)
    apply_force(F, nodes[-1], fy=200, fz=-300)
    res = solve_static(model, F)
    ok = res.success and np.all(np.isfinite(res.u)) and np.all(np.isfinite(res.reactions))
    return ok, {"success": res.success}


def run_phase80_validation() -> bool:
    print("=== Phase 8.0 Beam FEM Foundation Validation ===\n")
    tests = [
        ("single_cantilever", test_cantilever),
        ("simply_supported", test_simply_supported),
        ("axial_tension", test_axial_tension),
        ("pure_bending", test_pure_bending),
        ("symmetry", test_symmetry),
        ("reaction_equilibrium", test_reaction_equilibrium),
        ("mesh_refinement", test_mesh_refinement),
        ("no_singular_matrices", test_no_singular),
        ("no_nan_inf", test_no_nan_inf),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:36} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_phase80_validation()
