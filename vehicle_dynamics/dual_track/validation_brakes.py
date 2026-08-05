"""
Phase 5.2 – Independent wheel braking & per-wheel ABS validation.
"""

from __future__ import annotations

import numpy as np
from .parameters import DualTrackParameters
from .simulation import DualTrackVehicleModel
from .brakes import FourWheelBrakeDistributor
from .abs_per_wheel import FourWheelABS
from ..braking.parameters import BrakeParams
from ..braking.abs_controller import ABSParams


def test_zero_brake_wheel_follow() -> tuple[bool, dict]:
    """With pedal=0, wheel peripheral speed tracks vx (κ ≈ 0)."""
    model = DualTrackVehicleModel(use_abs=False)
    res = model.simulate(
        vx0=20.0, t_span=(0.0, 3.0),
        delta_func=lambda t: 0.0,
        pedal_func=lambda t: 0.0,
    )
    # κ should stay small
    max_k = float(np.max(np.abs(res.kappa)))
    ok = res.vx[-1] > 18.0 and max_k < 0.05 and np.all(np.isfinite(res.omega))
    return ok, {"final_vx": float(res.vx[-1]), "max_abs_kappa": max_k}


def test_symmetric_braking_straight() -> tuple[bool, dict]:
    model = DualTrackVehicleModel(use_abs=True)
    res = model.simulate(
        vx0=22.22, t_span=(0.0, 8.0),
        delta_func=lambda t: 0.0,
        pedal_func=lambda t: 1.0 if t >= 0.1 else 0.0,
    )
    ok = (
        res.vx[-1] < 5.0
        and float(np.max(np.abs(res.vy))) < 0.5
        and float(np.max(np.abs(res.r))) < 0.3
        and float(np.max(res.utilization)) <= 1.0 + 1e-3
        and np.all(np.isfinite(res.vx))
    )
    return ok, {
        "final_vx": float(res.vx[-1]),
        "max_vy": float(np.max(np.abs(res.vy))),
        "max_util": float(np.max(res.utilization)),
    }


def test_per_wheel_abs_independent() -> tuple[bool, dict]:
    """Four controllers exist and modulate under high slip."""
    abs4 = FourWheelABS()
    # Drive high slip on FL only
    pressures = []
    for _ in range(300):
        k = np.array([0.35, 0.05, 0.05, 0.05])
        p = abs4.update(k, 0.001, active=True)
        pressures.append(p.copy())
    pressures = np.asarray(pressures)
    # FL pressure should drop more than FR
    ok = float(pressures[-1, 0]) < float(pressures[-1, 1]) - 0.05
    return ok, {
        "p_FL": float(pressures[-1, 0]),
        "p_FR": float(pressures[-1, 1]),
        "states": abs4.states,
    }


def test_wheel_lock_detection() -> tuple[bool, dict]:
    """Without ABS, aggressive brake should drive high κ on at least one wheel."""
    model = DualTrackVehicleModel(use_abs=False)
    res = model.simulate(
        vx0=20.0, t_span=(0.0, 4.0),
        delta_func=lambda t: 0.0,
        pedal_func=lambda t: 1.0 if t >= 0.05 else 0.0,
    )
    max_k = float(np.max(res.kappa))
    ok = max_k > 0.25 and np.all(np.isfinite(res.kappa))
    return ok, {"max_kappa": max_k}


def test_split_mu_braking() -> tuple[bool, dict]:
    """Left μ=0.4, right μ=1.0 → yaw moment under straight brake (with ABS)."""
    mu = np.array([0.4, 1.0, 0.4, 1.0])  # FL, FR, RL, RR
    model = DualTrackVehicleModel(use_abs=True, mu_wheels=mu)
    res = model.simulate(
        vx0=20.0, t_span=(0.0, 5.0),
        delta_func=lambda t: 0.0,
        pedal_func=lambda t: 0.8 if t >= 0.1 else 0.0,
    )
    # Expect some yaw activity due to asymmetric Fx
    max_r = float(np.max(np.abs(res.r)))
    ok = (
        res.vx[-1] < res.vx[0] - 2.0
        and max_r > 1e-3
        and float(np.max(res.utilization)) <= 1.0 + 1e-2
        and np.all(np.isfinite(res.r))
    )
    return ok, {
        "final_vx": float(res.vx[-1]),
        "max_abs_r": max_r,
        "max_util": float(np.max(res.utilization)),
    }


def test_brake_distributor() -> tuple[bool, dict]:
    dist = FourWheelBrakeDistributor(BrakeParams())
    cmd = dist.desired(1.0)
    ok = (
        cmd.T.shape == (4,)
        and cmd.T[0] == cmd.T[1]
        and cmd.T[2] == cmd.T[3]
        and cmd.T[0] > cmd.T[2]  # front bias
    )
    return ok, {"T": cmd.T.tolist()}


def test_phase51_braking_regression() -> tuple[bool, dict]:
    """Symmetric stop still works with per-wheel ABS (vs Phase 5.1 behaviour)."""
    model = DualTrackVehicleModel(use_abs=True)
    res = model.simulate(
        vx0=22.22, t_span=(0.0, 8.0),
        delta_func=lambda t: 0.0,
        pedal_func=lambda t: 1.0 if t >= 0.1 else 0.0,
    )
    ok = res.vx[-1] < 5.0 and float(np.max(np.abs(res.vy))) < 0.5
    return ok, {"final_vx": float(res.vx[-1])}


def run_brake_validation() -> bool:
    print("=== Phase 5.2 Independent Wheel Braking Validation ===\n")
    tests = [
        ("zero_brake_wheel_follow", test_zero_brake_wheel_follow),
        ("symmetric_braking_straight", test_symmetric_braking_straight),
        ("per_wheel_abs_independent", test_per_wheel_abs_independent),
        ("wheel_lock_detection", test_wheel_lock_detection),
        ("split_mu_braking", test_split_mu_braking),
        ("brake_distributor", test_brake_distributor),
        ("phase51_braking_regression", test_phase51_braking_regression),
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
    run_brake_validation()
