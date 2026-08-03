"""
Phase 6.1 – Wheel rate & motion ratio validation.
"""

from __future__ import annotations

import numpy as np
from .wheel_rate import (
    SpringDamperParams,
    MotionRatioParams,
    compute_wheel_rate,
    motion_ratio_from_ir,
    installation_ratio_from_travels,
    effective_wheel_rate,
    effective_wheel_damping,
    wheel_rate_curve,
)


def test_mr_equals_one() -> tuple[bool, dict]:
    r = compute_wheel_rate(
        SpringDamperParams(Ks=25000, Cs=1500),
        MotionRatioParams(installation_ratio=1.0, layout="direct"),
    )
    ok = (
        abs(r.motion_ratio - 1.0) < 1e-12
        and abs(r.Kw - 25000) < 1e-6
        and abs(r.Cw - 1500) < 1e-6
    )
    return ok, {"result": r.summary()}


def test_mr_less_than_one() -> tuple[bool, dict]:
    """Pushrod IR=0.7 → MR≈1.429, Kw = 0.49 * Ks."""
    Ks, Cs, ir = 30000.0, 2000.0, 0.7
    r = compute_wheel_rate(
        SpringDamperParams(Ks=Ks, Cs=Cs),
        MotionRatioParams(installation_ratio=ir, layout="pushrod"),
    )
    Kw_expected = Ks * ir ** 2
    Cw_expected = Cs * ir ** 2
    ok = (
        abs(r.motion_ratio - 1.0 / ir) < 1e-12
        and abs(r.Kw - Kw_expected) < 1e-6
        and abs(r.Cw - Cw_expected) < 1e-6
        and r.Kw < Ks
    )
    return ok, {"Kw": r.Kw, "expected": Kw_expected, "MR": r.motion_ratio}


def test_mr_greater_than_one() -> tuple[bool, dict]:
    """IR=1.2 → MR<1, Kw > Ks."""
    Ks, ir = 30000.0, 1.2
    r = compute_wheel_rate(
        SpringDamperParams(Ks=Ks, Cs=2000),
        MotionRatioParams(installation_ratio=ir, layout="custom"),
    )
    ok = r.motion_ratio < 1.0 and abs(r.Kw - Ks * ir ** 2) < 1e-6 and r.Kw > Ks
    return ok, {"MR": r.motion_ratio, "Kw": r.Kw}


def test_analytical_crosscheck() -> tuple[bool, dict]:
    """Kw = Ks * (zs/zw)^2 from virtual work."""
    zw, zs, Ks = 0.05, 0.035, 28000.0
    ir = installation_ratio_from_travels(zw, zs)
    Kw = effective_wheel_rate(Ks, ir)
    # Energy: 0.5*Ks*zs^2 = 0.5*Kw*zw^2  → Kw = Ks*(zs/zw)^2
    Kw_energy = Ks * (zs / zw) ** 2
    ok = abs(Kw - Kw_energy) < 1e-9 and abs(ir - zs / zw) < 1e-12
    return ok, {"Kw": Kw, "Kw_energy": Kw_energy, "IR": ir}


def test_damping_same_ratio() -> tuple[bool, dict]:
    Cs, ir = 1800.0, 0.8
    Cw = effective_wheel_damping(Cs, ir)
    ok = abs(Cw - Cs * ir ** 2) < 1e-9
    return ok, {"Cw": Cw, "expected": Cs * ir ** 2}


def test_left_right_symmetry() -> tuple[bool, dict]:
    """Same IR both sides → identical Kw (geometry solver unchanged)."""
    left = compute_wheel_rate(
        SpringDamperParams(Ks=32000, Cs=2200),
        MotionRatioParams(installation_ratio=0.85, layout="pushrod"),
    )
    right = compute_wheel_rate(
        SpringDamperParams(Ks=32000, Cs=2200),
        MotionRatioParams(installation_ratio=0.85, layout="pushrod"),
    )
    ok = abs(left.Kw - right.Kw) < 1e-12 and abs(left.Cw - right.Cw) < 1e-12
    return ok, {"Kw_L": left.Kw, "Kw_R": right.Kw}


def test_monotonicity() -> tuple[bool, dict]:
    """Higher |IR| → higher Kw for fixed Ks."""
    Ks = 30000.0
    irs = np.array([0.5, 0.7, 0.9, 1.0, 1.1])
    Kws = wheel_rate_curve(Ks, irs)
    ok = bool(np.all(np.diff(Kws) > 0))
    return ok, {"IR": irs.tolist(), "Kw": Kws.tolist()}


def test_mr_inverse_of_ir() -> tuple[bool, dict]:
    ir = 0.65
    mr = motion_ratio_from_ir(ir)
    ok = abs(mr * ir - 1.0) < 1e-12
    return ok, {"IR": ir, "MR": mr}


def test_zero_ir_raises() -> tuple[bool, dict]:
    try:
        compute_wheel_rate(mr_params=MotionRatioParams(installation_ratio=0.0))
        return False, {"error": "did not raise"}
    except ValueError:
        return True, {"raised": "ValueError"}


def run_phase61_validation() -> bool:
    print("=== Phase 6.1 Wheel Rate & Motion Ratio Validation ===\n")
    tests = [
        ("mr_equals_one", test_mr_equals_one),
        ("mr_less_than_one", test_mr_less_than_one),
        ("mr_greater_than_one", test_mr_greater_than_one),
        ("analytical_crosscheck", test_analytical_crosscheck),
        ("damping_same_ratio", test_damping_same_ratio),
        ("left_right_symmetry", test_left_right_symmetry),
        ("monotonicity", test_monotonicity),
        ("mr_inverse_of_ir", test_mr_inverse_of_ir),
        ("zero_ir_raises", test_zero_ir_raises),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:28} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_phase61_validation()
