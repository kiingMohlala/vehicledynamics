"""
Phase 6.7 – Jacking force validation.
"""

from __future__ import annotations

import numpy as np
from .jacking import compute_jacking, apply_jacking_to_loads, axle_jacking_delta
from .jacking_state import JackingParams, JackingState
from .load_transfer_feedback import JackingFeedback


def test_zero_rc_zero_jacking() -> tuple[bool, dict]:
    j = compute_jacking(1000, 1000, 800, 800, rc_front=0.0, rc_rear=0.0)
    ok = np.allclose(j.dFz_wheels, 0.0)
    return ok, {"dFz": j.dFz_wheels.tolist()}


def test_positive_rc_direction() -> tuple[bool, dict]:
    """Fy_axle > 0, h_RC > 0 → right gains, left loses."""
    d_l, d_r = axle_jacking_delta(500.0, 500.0, h_rc=0.1, track=1.55)
    ok = d_l < 0 and d_r > 0 and abs(d_l + d_r) < 1e-12
    expected = 1000.0 * 0.1 / 1.55
    ok = ok and abs(d_r - expected) < 1e-9
    return ok, {"d_left": d_l, "d_right": d_r, "expected": expected}


def test_negative_rc_opposite() -> tuple[bool, dict]:
    d_l, d_r = axle_jacking_delta(500.0, 500.0, h_rc=-0.1, track=1.55)
    ok = d_l > 0 and d_r < 0
    return ok, {"d_left": d_l, "d_right": d_r}


def test_left_right_symmetry() -> tuple[bool, dict]:
    j_pos = compute_jacking(600, 400, 0, 0, rc_front=0.08, rc_rear=0.0)
    j_neg = compute_jacking(-600, -400, 0, 0, rc_front=0.08, rc_rear=0.0)
    ok = np.allclose(j_pos.dFz_wheels, -j_neg.dFz_wheels)
    return ok, {"d_pos": j_pos.dFz_wheels.tolist(), "d_neg": j_neg.dFz_wheels.tolist()}


def test_combined_cornering() -> tuple[bool, dict]:
    j = compute_jacking(
        Fy_fl=800, Fy_fr=900, Fy_rl=600, Fy_rr=700,
        rc_front=0.05, rc_rear=0.04,
        params=JackingParams(track_f=1.55, track_r=1.55),
    )
    ok = j.dFz_wheels[1] > 0 and j.dFz_wheels[0] < 0  # front right gains
    ok = ok and j.dFz_wheels[3] > 0 and j.dFz_wheels[2] < 0
    return ok, {"dFz": j.dFz_wheels.tolist()}


def test_total_weight_conserved() -> tuple[bool, dict]:
    Fz0 = np.array([3500.0, 3500.0, 3000.0, 3000.0])
    j = compute_jacking(1000, 1000, 800, 800, 0.06, 0.05)
    Fz1 = apply_jacking_to_loads(Fz0, j, Fz_min=50.0)
    ok = abs(Fz1.sum() - Fz0.sum()) < 1e-6
    return ok, {"sum0": float(Fz0.sum()), "sum1": float(Fz1.sum())}


def test_neutral_disabled_regression() -> tuple[bool, dict]:
    Fz0 = np.array([3500.0, 3500.0, 3000.0, 3000.0])
    fb = JackingFeedback(JackingParams(enabled=False))
    Fz1 = fb.update(Fz0, 1000, 1000, 800, 800, 0.1, 0.1)
    ok = np.allclose(Fz0, Fz1)
    return ok, {"Fz0": Fz0.tolist(), "Fz1": Fz1.tolist()}


def test_zero_fy_zero_jacking() -> tuple[bool, dict]:
    j = compute_jacking(0, 0, 0, 0, 0.1, 0.1)
    ok = np.allclose(j.dFz_wheels, 0.0)
    return ok, {"dFz": j.dFz_wheels.tolist()}


def test_no_nan_inf() -> tuple[bool, dict]:
    ok = True
    for h in (-0.1, 0.0, 0.05, 0.2):
        j = compute_jacking(500, -200, 300, 100, h, h)
        if not np.all(np.isfinite(j.dFz_wheels)):
            ok = False
    return ok, {}


def run_phase67_validation() -> bool:
    print("=== Phase 6.7 Jacking Forces Validation ===\n")
    tests = [
        ("zero_rc_zero_jacking", test_zero_rc_zero_jacking),
        ("positive_rc_direction", test_positive_rc_direction),
        ("negative_rc_opposite", test_negative_rc_opposite),
        ("left_right_symmetry", test_left_right_symmetry),
        ("combined_cornering", test_combined_cornering),
        ("total_weight_conserved", test_total_weight_conserved),
        ("neutral_disabled_regression", test_neutral_disabled_regression),
        ("zero_fy_zero_jacking", test_zero_fy_zero_jacking),
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
    run_phase67_validation()
