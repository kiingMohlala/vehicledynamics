"""
Phase 4.2 – Regression gates before combined scenarios.

1. Pure braking (δ=0) must remain stable and produce finite stopping behaviour.
2. Pure steering (pedal=0) must recover Phase 4.0 qualitative behaviour.
"""

import numpy as np
from .simulation import CombinedVehicleModel

def test_pure_braking_regression():
    """δ = 0, full brake → Vx decreases, lateral states stay near zero."""
    model = CombinedVehicleModel(use_abs=True)
    res = model.simulate(
        vx0=22.22,
        t_span=(0.0, 8.0),
        delta_func=lambda t: 0.0,
        pedal_func=lambda t: 1.0 if t >= 0.1 else 0.0,
    )
    ok = (
        np.all(np.isfinite(res.vx)) and
        res.vx[-1] < 5.0 and                 # substantially slowed
        np.max(np.abs(res.vy)) < 0.5 and     # little lateral motion
        np.max(np.abs(res.r)) < 0.2 and
        res.stopping_distance > 5.0
    )
    return ok, {
        "final_vx": float(res.vx[-1]),
        "max_vy": float(np.max(np.abs(res.vy))),
        "stopping_distance": res.stopping_distance,
    }

def test_pure_steering_regression():
    """pedal = 0, step steer → finite yaw rate, Vx nearly constant."""
    model = CombinedVehicleModel(use_abs=False)
    delta = np.deg2rad(3.0)
    res = model.simulate(
        vx0=20.0,
        t_span=(0.0, 6.0),
        delta_func=lambda t: delta if t >= 0.5 else 0.0,
        pedal_func=lambda t: 0.0,
    )
    post = res.time >= 2.0
    ok = (
        np.all(np.isfinite(res.vy)) and np.all(np.isfinite(res.r)) and
        np.max(np.abs(res.r[post])) > 1e-3 and
        abs(res.vx[-1] - 20.0) < 1.5        # mild variation only (no brake)
    )
    return ok, {
        "final_r": float(res.r[-1]),
        "final_vy": float(res.vy[-1]),
        "final_vx": float(res.vx[-1]),
    }

def test_no_nan_combined_inputs():
    """Light combined input must remain finite (smoke only)."""
    model = CombinedVehicleModel(use_abs=True)
    res = model.simulate(
        vx0=18.0,
        t_span=(0.0, 4.0),
        delta_func=lambda t: np.deg2rad(2.0) if t >= 0.3 else 0.0,
        pedal_func=lambda t: 0.3 if t >= 0.5 else 0.0,
    )
    arrays = [res.vx, res.vy, res.r, res.Fx_f, res.Fy_f, res.kappa_f, res.alpha_f]
    ok = all(np.all(np.isfinite(a)) for a in arrays)
    return ok, {"min_vx": float(np.min(res.vx))}

def run_regression_gates():
    print("=== Phase 4.2 Regression Gates ===\n")
    tests = [
        ("Pure braking regression", test_pure_braking_regression),
        ("Pure steering regression", test_pure_steering_regression),
        ("Combined smoke (finite)", test_no_nan_combined_inputs),
    ]
    all_pass = True
    for name, fn in tests:
        ok, diag = fn()
        print(f"{name:35} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL GATES PASSED" if all_pass else "GATES FAILED")
    return all_pass

if __name__ == "__main__":
    run_regression_gates()
