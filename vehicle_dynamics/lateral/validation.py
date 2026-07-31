"""
Phase 4.0 – Independent validation for the dynamic bicycle model.

Tests are intentionally limited to the pure lateral domain.
No braking, load transfer, or ESC physics are exercised here.
"""

import numpy as np
from .parameters import BicycleParameters
from .bicycle import DynamicBicycleModel

def test_straight_line_stability():
    """Zero steer → states remain near zero."""
    model = DynamicBicycleModel()
    res = model.simulate(vx=20.0, t_span=(0, 5), delta_func=lambda t: 0.0)
    ok = (
        np.max(np.abs(res.vy)) < 1e-3 and
        np.max(np.abs(res.r)) < 1e-3
    )
    return ok, {
        "max_vy": float(np.max(np.abs(res.vy))),
        "max_r": float(np.max(np.abs(res.r))),
    }

def test_step_steer_response():
    """Step steer produces finite, non-zero yaw rate and lateral velocity."""
    model = DynamicBicycleModel()
    delta_final = np.deg2rad(3.0)

    def delta_func(t):
        return delta_final if t >= 1.0 else 0.0

    res = model.simulate(vx=20.0, t_span=(0, 8), delta_func=delta_func)

    # After the step, yaw rate and vy should become non-zero and remain finite
    post = res.time >= 2.0
    ok = (
        np.all(np.isfinite(res.vy)) and
        np.all(np.isfinite(res.r)) and
        np.max(np.abs(res.r[post])) > 1e-3 and
        np.max(np.abs(res.vy[post])) > 1e-3
    )
    return ok, {
        "max_r_post": float(np.max(np.abs(res.r[post]))),
        "max_vy_post": float(np.max(np.abs(res.vy[post]))),
        "final_r": float(res.r[-1]),
        "final_vy": float(res.vy[-1]),
    }

def test_ay_consistency():
    """Check a_y ≈ (Fy_f + Fy_r)/m matches stored ay."""
    model = DynamicBicycleModel()
    delta_final = np.deg2rad(2.0)
    res = model.simulate(
        vx=15.0,
        t_span=(0, 6),
        delta_func=lambda t: delta_final if t >= 0.5 else 0.0,
    )
    ay_from_forces = (res.Fy_f + res.Fy_r) / model.p.m
    err = np.max(np.abs(ay_from_forces - res.ay))
    ok = err < 1e-6
    return ok, {"max_ay_error": float(err)}

def test_no_nan_inf():
    model = DynamicBicycleModel()
    res = model.simulate(
        vx=25.0,
        t_span=(0, 5),
        delta_func=lambda t: np.deg2rad(4.0) if t > 0.2 else 0.0,
    )
    arrays = [res.vy, res.r, res.alpha_f, res.alpha_r, res.Fy_f, res.Fy_r, res.ay]
    ok = all(np.all(np.isfinite(a)) for a in arrays)
    return ok, {}

def run_all_tests():
    print("=== Phase 4.0 Bicycle Model Validation ===\n")
    tests = [
        ("Straight-line stability", test_straight_line_stability),
        ("Step-steer response", test_step_steer_response),
        ("a_y consistency", test_ay_consistency),
        ("No NaN/Inf", test_no_nan_inf),
    ]

    results = {}
    all_pass = True
    for name, fn in tests:
        ok, diag = fn()
        results[name] = ok
        status = "PASS" if ok else "FAIL"
        print(f"{name:30} : {status}")
        if diag:
            for k, v in diag.items():
                print(f"    {k}: {v}")
        if not ok:
            all_pass = False

    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass, results

if __name__ == "__main__":
    run_all_tests()
