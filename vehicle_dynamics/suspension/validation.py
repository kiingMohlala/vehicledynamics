"""
Phase 6.0 – Suspension geometry validation.
"""

from __future__ import annotations

import numpy as np
from .hardpoints import default_front_left, mirror_corner, Point3, WishboneHardpoints
from .solver import SuspensionGeometrySolver
from .geometry import line_intersect_2d, instant_center_yz
from .wishbone import analyze


def test_line_intersect() -> tuple[bool, dict]:
    p1 = np.array([0.0, 0.0]); p2 = np.array([1.0, 1.0])
    p3 = np.array([0.0, 1.0]); p4 = np.array([1.0, 0.0])
    ic = line_intersect_2d(p1, p2, p3, p4)
    ok = ic is not None and abs(ic[0] - 0.5) < 1e-9 and abs(ic[1] - 0.5) < 1e-9
    return ok, {"ic": ic.tolist() if ic is not None else None}


def test_ic_not_midpoint() -> tuple[bool, dict]:
    """IC must be line intersection, not midpoint of arm."""
    hp = default_front_left()
    from .geometry import average_inner
    ui = average_inner(hp.upper_front, hp.upper_rear)
    li = average_inner(hp.lower_front, hp.lower_rear)
    ic = instant_center_yz(
        ui.y, ui.z, hp.upper_outer.y, hp.upper_outer.z,
        li.y, li.z, hp.lower_outer.y, hp.lower_outer.z,
    )
    mid_u = 0.5 * (ui.as_array() + hp.upper_outer.as_array())
    ok = ic is not None and (
        abs(ic[0] - mid_u[1]) > 1e-3 or abs(ic[1] - mid_u[2]) > 1e-3
    )
    return ok, {"ic": ic, "mid_upper_yz": (float(mid_u[1]), float(mid_u[2]))}


def test_static_geometry_finite() -> tuple[bool, dict]:
    sol = SuspensionGeometrySolver()
    r = sol.solve()
    vals = [
        r.camber_deg, r.toe_deg, r.caster_deg, r.kpi_deg,
        r.scrub_radius, r.trail, r.roll_center_z,
        r.instant_center_y, r.instant_center_z,
        r.swing_arm_length, r.upper_arm_length, r.lower_arm_length,
    ]
    ok = all(np.isfinite(v) for v in vals)
    return ok, {"summary": r.summary()}


def test_arm_lengths_positive() -> tuple[bool, dict]:
    r = SuspensionGeometrySolver().solve()
    ok = r.upper_arm_length > 0.05 and r.lower_arm_length > 0.05
    return ok, {"upper": r.upper_arm_length, "lower": r.lower_arm_length}


def test_kpi_caster_reasonable() -> tuple[bool, dict]:
    r = SuspensionGeometrySolver().solve()
    ok = 0.0 <= r.kpi_deg <= 20.0 and -5.0 <= r.caster_deg <= 15.0
    return ok, {"kpi": r.kpi_deg, "caster": r.caster_deg}


def test_left_right_symmetry() -> tuple[bool, dict]:
    left, right = SuspensionGeometrySolver().solve_pair()
    # Camber/KPI should match magnitude; scrub should flip sign
    ok = (
        abs(left.kpi_deg - right.kpi_deg) < 1e-6
        and abs(left.caster_deg - right.caster_deg) < 1e-6
        and abs(left.camber_deg + right.camber_deg) < 1e-4  # opposite camber sign
        and abs(left.scrub_radius + right.scrub_radius) < 1e-4
        and abs(left.roll_center_z - right.roll_center_z) < 1e-4
    )
    return ok, {
        "left_camber": left.camber_deg, "right_camber": right.camber_deg,
        "left_scrub": left.scrub_radius, "right_scrub": right.scrub_radius,
        "left_rc_z": left.roll_center_z, "right_rc_z": right.roll_center_z,
    }


def test_roll_center_near_centerline() -> tuple[bool, dict]:
    r = SuspensionGeometrySolver().solve()
    # RC y is forced to 0 by construction; z should be finite and modest
    ok = np.isfinite(r.roll_center_z) and -0.2 < r.roll_center_z < 0.5
    return ok, {"rc_z": r.roll_center_z}


def test_parallel_arms_handled() -> tuple[bool, dict]:
    """Parallel arms → IC at infinity; solver must not crash."""
    hp = default_front_left()
    # Force parallel in YZ: same z for both arms inner/outer
    hp = WishboneHardpoints(
        upper_front=Point3(0.05, 0.35, 0.50),
        upper_rear=Point3(-0.15, 0.35, 0.50),
        upper_outer=Point3(-0.02, 0.68, 0.50),
        lower_front=Point3(0.12, 0.30, 0.20),
        lower_rear=Point3(-0.20, 0.30, 0.20),
        lower_outer=Point3(0.00, 0.72, 0.20),
        tierod_inner=hp.tierod_inner,
        tierod_outer=hp.tierod_outer,
        wheel_center=hp.wheel_center,
        contact_patch=hp.contact_patch,
    )
    try:
        r = analyze(hp)
        # may have nan IC but must not raise
        ok = True
        return ok, {"ic_y": r.instant_center_y, "ic_z": r.instant_center_z}
    except Exception as e:
        return False, {"error": str(e)}


def run_phase60_validation() -> bool:
    print("=== Phase 6.0 Suspension Geometry Validation ===\n")
    tests = [
        ("line_intersect", test_line_intersect),
        ("ic_not_midpoint", test_ic_not_midpoint),
        ("static_geometry_finite", test_static_geometry_finite),
        ("arm_lengths_positive", test_arm_lengths_positive),
        ("kpi_caster_reasonable", test_kpi_caster_reasonable),
        ("left_right_symmetry", test_left_right_symmetry),
        ("roll_center_near_centerline", test_roll_center_near_centerline),
        ("parallel_arms_handled", test_parallel_arms_handled),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:30} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            if k == "summary":
                print(v)
            else:
                print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_phase60_validation()
