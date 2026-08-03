"""
Phase 5.1 – Ackermann steering validation.

Checks:
  1. Zero steer → both front angles zero
  2. Left/right command symmetry
  3. Inside |δ| > outside |δ| for non-zero steer
  4. Low-speed geometry residual of classical Ackermann identity
  5. Phase 5.0 regression compatibility with use_ackermann=False
"""

from __future__ import annotations

import numpy as np

from .steering import (
    ackermann_angles,
    equal_angles,
    ideal_ackermann_relation,
    SteeringParameters,
)
from .parameters import DualTrackParameters
from .simulation import DualTrackVehicleModel
from .kinematics import FL, FR


def test_zero_steer() -> tuple[bool, dict]:
    d_fl, d_fr = ackermann_angles(0.0, wheelbase=2.7, track_f=1.55)
    ok = abs(d_fl) < 1e-12 and abs(d_fr) < 1e-12
    return ok, {"delta_fl": d_fl, "delta_fr": d_fr}


def test_left_right_symmetry() -> tuple[bool, dict]:
    L, t = 2.7, 1.55
    cmd = np.deg2rad(5.0)
    fl_p, fr_p = ackermann_angles(cmd, L, t)
    fl_n, fr_n = ackermann_angles(-cmd, L, t)
    # Negating command should swap and negate angles
    ok = (
        abs(fl_p + fr_n) < 1e-9
        and abs(fr_p + fl_n) < 1e-9
    )
    return ok, {
        "fl_pos": float(fl_p), "fr_pos": float(fr_p),
        "fl_neg": float(fl_n), "fr_neg": float(fr_n),
    }


def test_inside_outside() -> tuple[bool, dict]:
    L, t = 2.7, 1.55
    cmd = np.deg2rad(8.0)  # left turn → FL inside
    d_fl, d_fr = ackermann_angles(cmd, L, t)
    ok = abs(d_fl) > abs(d_fr) > 0.0
    return ok, {
        "delta_fl_deg": float(np.rad2deg(d_fl)),
        "delta_fr_deg": float(np.rad2deg(d_fr)),
        "inside": "FL",
    }


def test_low_speed_geometry() -> tuple[bool, dict]:
    L, t = 2.7, 1.55
    residuals = []
    for deg in [1.0, 3.0, 5.0, 10.0, 15.0]:
        cmd = np.deg2rad(deg)
        d_fl, d_fr = ackermann_angles(cmd, L, t)
        residuals.append(ideal_ackermann_relation(d_fl, d_fr, L, t))
    max_res = float(max(residuals))
    ok = max_res < 1e-6
    return ok, {"max_residual": max_res, "residuals": residuals}


def test_phase50_equal_steer_compat() -> tuple[bool, dict]:
    """With use_ackermann=False, behaviour matches Phase 5.0 equal front angles."""
    params = DualTrackParameters()
    params.steering = SteeringParameters(use_ackermann=False)
    model = DualTrackVehicleModel(params=params, use_abs=False)
    delta = np.deg2rad(3.0)
    res = model.simulate(
        vx0=20.0,
        t_span=(0.0, 4.0),
        delta_func=lambda t: delta if t >= 0.5 else 0.0,
        pedal_func=lambda t: 0.0,
    )
    # Front angles must be equal to the command (after clip)
    post = res.time > 1.0
    fl = res.delta_fl[post]
    fr = res.delta_fr[post]
    cmd = res.delta[post]
    ok_equal = bool(np.allclose(fl, fr, atol=1e-9))
    ok_cmd = bool(np.allclose(fl, cmd, atol=1e-9))
    ok_finite = bool(np.all(np.isfinite(res.r)) and np.max(np.abs(res.r[post])) > 1e-3)
    ok = ok_equal and ok_cmd and ok_finite
    return ok, {
        "mean_delta_fl": float(np.mean(fl)),
        "mean_delta_fr": float(np.mean(fr)),
        "mean_cmd": float(np.mean(cmd)),
        "final_r": float(res.r[-1]),
    }


def test_ackermann_simulation_smoke() -> tuple[bool, dict]:
    """Ackermann enabled: inside angle larger, finite dynamics."""
    params = DualTrackParameters()
    params.steering = SteeringParameters(use_ackermann=True)
    model = DualTrackVehicleModel(params=params, use_abs=False)
    delta = np.deg2rad(5.0)
    res = model.simulate(
        vx0=15.0,
        t_span=(0.0, 5.0),
        delta_func=lambda t: delta if t >= 0.3 else 0.0,
        pedal_func=lambda t: 0.0,
    )
    post = res.time > 1.0
    fl = float(np.mean(res.delta_fl[post]))
    fr = float(np.mean(res.delta_fr[post]))
    ok = (
        abs(fl) > abs(fr) > 0.0
        and np.all(np.isfinite(res.vx))
        and np.all(np.isfinite(res.r))
        and float(res.vx[-1]) > 5.0
    )
    return ok, {
        "mean_delta_fl_deg": float(np.rad2deg(fl)),
        "mean_delta_fr_deg": float(np.rad2deg(fr)),
        "final_r": float(res.r[-1]),
        "final_vx": float(res.vx[-1]),
    }


def run_steering_validation() -> bool:
    print("=== Phase 5.1 Ackermann Steering Validation ===\n")
    tests = [
        ("zero_steer", test_zero_steer),
        ("left_right_symmetry", test_left_right_symmetry),
        ("inside_outside", test_inside_outside),
        ("low_speed_geometry", test_low_speed_geometry),
        ("phase50_equal_steer_compat", test_phase50_equal_steer_compat),
        ("ackermann_simulation_smoke", test_ackermann_simulation_smoke),
    ]
    all_pass = True
    for name, fn in tests:
        ok, diag = fn()
        print(f"{name:32} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_steering_validation()
