"""
Phase 6.6 – Nonlinear suspension geometry validation.
"""

from __future__ import annotations

import numpy as np
from .hardpoints import default_front_left, mirror_corner
from .wishbone import analyze
from .travel_solver import solve_at_travel, solve_static
from .geometry_curves import build_curves
from .nonlinear_geometry import NonlinearGeometrySolver, FourCornerNonlinearGeometry
from .roll_center import compute_roll_centers


def test_static_matches_phase60() -> tuple[bool, dict]:
    hp = default_front_left()
    base = analyze(hp)
    sol = NonlinearGeometrySolver(hp).solve(0.0)
    ok = abs(sol.camber_rad - np.radians(base.camber_deg)) < 1e-6
    ok = ok and abs(sol.toe_rad - np.radians(base.toe_deg)) < 1e-6
    ok = ok and abs(sol.roll_center_z - base.roll_center_z) < 1e-4
    return ok, {
        "camber_rad": sol.camber_rad,
        "base_camber": np.radians(base.camber_deg),
        "rc": sol.roll_center_z,
        "base_rc": base.roll_center_z,
    }


def test_small_travel_continuity() -> tuple[bool, dict]:
    s = NonlinearGeometrySolver()
    c0 = s.solve(0.0).camber_rad
    c1 = s.solve(0.001).camber_rad
    t0 = s.solve(0.0).toe_rad
    t1 = s.solve(0.001).toe_rad
    ok = abs(c1 - c0) < 0.05 and abs(t1 - t0) < 0.05  # no jump
    return ok, {"d_camber": c1 - c0, "d_toe": t1 - t0}


def test_bump_rebound_finite() -> tuple[bool, dict]:
    s = NonlinearGeometrySolver()
    up = s.solve(0.05)
    dn = s.solve(-0.05)
    ok = all(np.isfinite(x) for x in (
        up.camber_rad, up.toe_rad, up.roll_center_z,
        dn.camber_rad, dn.toe_rad, dn.roll_center_z,
    ))
    return ok, {
        "camber_up": up.camber_rad, "camber_dn": dn.camber_rad,
        "toe_up": up.toe_rad, "toe_dn": dn.toe_rad,
    }


def test_camber_curve_smooth() -> tuple[bool, dict]:
    curves = build_curves(n=41)
    d = np.diff(curves.camber_rad)
    ok = np.all(np.isfinite(curves.camber_rad)) and np.max(np.abs(d)) < 0.5
    return ok, {"max_step": float(np.max(np.abs(d))), "n": len(curves.travel)}


def test_toe_curve_smooth() -> tuple[bool, dict]:
    curves = build_curves(n=41)
    d = np.diff(curves.toe_rad)
    ok = np.all(np.isfinite(curves.toe_rad)) and np.max(np.abs(d)) < 0.5
    return ok, {"max_step": float(np.max(np.abs(d)))}


def test_rc_migration_continuous() -> tuple[bool, dict]:
    curves = build_curves(n=41)
    d = np.diff(curves.roll_center_z)
    ok = np.all(np.isfinite(curves.roll_center_z)) and np.max(np.abs(d)) < 0.5
    return ok, {
        "rc_min": float(np.min(curves.roll_center_z)),
        "rc_max": float(np.max(curves.roll_center_z)),
        "max_step": float(np.max(np.abs(d))),
    }


def test_left_right_symmetry() -> tuple[bool, dict]:
    fl = NonlinearGeometrySolver(default_front_left())
    fr = NonlinearGeometrySolver(mirror_corner(default_front_left()))
    z = 0.03
    a, b = fl.solve(z), fr.solve(z)
    # mirrored: camber/toe should flip sign appropriately
    ok = abs(a.camber_rad + b.camber_rad) < 1e-3 or abs(a.camber_rad - b.camber_rad) < 1e-3
    # RC heights should match magnitude for symmetric travel
    ok = ok and abs(a.roll_center_z - b.roll_center_z) < 1e-3
    return ok, {
        "camber_fl": a.camber_rad, "camber_fr": b.camber_rad,
        "rc_fl": a.roll_center_z, "rc_fr": b.roll_center_z,
    }


def test_no_nan_inf() -> tuple[bool, dict]:
    s = NonlinearGeometrySolver()
    ok = True
    for z in np.linspace(-0.08, 0.08, 17):
        st = s.solve(float(z))
        vals = [st.camber_rad, st.toe_rad, st.kpi_rad, st.caster_rad,
                st.roll_center_z, st.scrub_radius, st.trail]
        if not all(np.isfinite(v) for v in vals):
            ok = False
            break
    return ok, {}


def test_neutral_regression_phase65() -> tuple[bool, dict]:
    """travel=0 → same RC as Phase 6.5 compute_roll_centers(zeros)."""
    st = NonlinearGeometrySolver().solve(0.0)
    rc65 = compute_roll_centers(np.zeros(4))
    ok = abs(st.roll_center_z - rc65.rc_front) < 1e-4
    return ok, {"rc_66": st.roll_center_z, "rc_65": rc65.rc_front}


def run_phase66_validation() -> bool:
    print("=== Phase 6.6 Nonlinear Geometry Validation ===\n")
    tests = [
        ("static_matches_phase60", test_static_matches_phase60),
        ("small_travel_continuity", test_small_travel_continuity),
        ("bump_rebound_finite", test_bump_rebound_finite),
        ("camber_curve_smooth", test_camber_curve_smooth),
        ("toe_curve_smooth", test_toe_curve_smooth),
        ("rc_migration_continuous", test_rc_migration_continuous),
        ("left_right_symmetry", test_left_right_symmetry),
        ("no_nan_inf", test_no_nan_inf),
        ("neutral_regression_phase65", test_neutral_regression_phase65),
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
    run_phase66_validation()
