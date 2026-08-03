"""
Phase 5.4 – Torque vectoring validation (unit + closed-loop).
"""

from __future__ import annotations

import numpy as np
from .parameters import TVParameters
from .differential import distribute_drive, split_axle_torque
from .controller import TorqueVectoringController
from ..dual_track.fixed_step import FixedStepDualTrack
from ..dual_track.parameters import DualTrackParameters


def test_open_differential() -> tuple[bool, dict]:
    p = TVParameters(mode="open", front_drive_fraction=0.0, max_total_drive_torque=4000)
    T = distribute_drive(1.0, p, rear_delta_T=0.0)
    ok = abs(T[0] - T[1]) < 1e-9 and abs(T[2] - T[3]) < 1e-9 and abs(T.sum() - 4000) < 1e-6
    return ok, {"T": T.tolist()}


def test_fixed_bias() -> tuple[bool, dict]:
    p = TVParameters(mode="fixed_bias", front_drive_fraction=0.0,
                     fixed_left_fraction=0.7, max_total_drive_torque=4000)
    T = distribute_drive(1.0, p)
    ok = abs(T[2] - 0.7 * 4000) < 1e-6 and abs(T[3] - 0.3 * 4000) < 1e-6
    return ok, {"T": T.tolist()}


def test_active_delta_T() -> tuple[bool, dict]:
    Tl, Tr = split_axle_torque(2000.0, "active_rear", delta_T=400.0, max_delta_T=1200)
    ok = abs((Tl - Tr) - 400.0) < 1e-6 and abs(Tl + Tr - 2000) < 1e-6
    return ok, {"T_l": Tl, "T_r": Tr}


def test_controller_yaw_response() -> tuple[bool, dict]:
    """Excessive left yaw → negative rear_delta_T (more torque to right)."""
    p = TVParameters(mode="active_rear", min_throttle=0.05)
    tv = TorqueVectoringController(2.7, p)
    tv.reset()
    T, diag = None, {}
    for _ in range(30):
        T, diag = tv.update(vx=18, vy=0.2, r=0.6, delta=np.deg2rad(4), throttle=0.6, dt=0.01)
    ok = diag["active"] and diag["rear_delta_T"] < 0 and T[3] > T[2]  # RR > RL
    return ok, diag


def test_torque_balance() -> tuple[bool, dict]:
    p = TVParameters(mode="active_rear", max_total_drive_torque=3000, front_drive_fraction=0.3)
    T = distribute_drive(0.5, p, rear_delta_T=200)
    ok = abs(T.sum() - 0.5 * 3000) < 1.0  # delta_T redistributes, does not create net torque
    return ok, {"sum_T": float(T.sum()), "T": T.tolist()}


def test_straight_acceleration() -> tuple[bool, dict]:
    m = FixedStepDualTrack(enable_esc=False, use_abs=False, enable_tv=True, dt=0.002)
    res = m.simulate(
        vx0=5.0, t_span=(0, 4),
        delta_func=lambda t: 0.0,
        pedal_func=lambda t: 0.0,
        throttle_func=lambda t: 0.8 if t >= 0.1 else 0.0,
        dt_out=0.02,
    )
    ok = float(res.vx[-1]) > 8.0 and float(np.max(np.abs(res.r))) < 0.1 and np.all(np.isfinite(res.vx))
    return ok, {"final_vx": float(res.vx[-1]), "max_r": float(np.max(np.abs(res.r)))}


def test_corner_exit() -> tuple[bool, dict]:
    m = FixedStepDualTrack(enable_esc=False, use_abs=False, enable_tv=True, dt=0.002)
    res = m.simulate(
        vx0=12.0, t_span=(0, 5),
        delta_func=lambda t: np.deg2rad(6.0) if t >= 0.3 else 0.0,
        pedal_func=lambda t: 0.0,
        throttle_func=lambda t: 0.7 if t >= 0.5 else 0.0,
        dt_out=0.02,
    )
    ok = np.all(np.isfinite(res.vx)) and float(np.max(res.utilization)) <= 1.05
    return ok, {
        "final_vx": float(res.vx[-1]),
        "max_r": float(np.max(np.abs(res.r))),
        "max_util": float(np.max(res.utilization)),
        "tv_activation": m.tv_diagnostics.activation_fraction if hasattr(m, "tv_diagnostics") else 0.0,
    }


def test_esc_compatibility() -> tuple[bool, dict]:
    m = FixedStepDualTrack(enable_esc=True, use_abs=True, enable_tv=True, dt=0.002)
    res = m.simulate(
        vx0=18.0, t_span=(0, 4),
        delta_func=lambda t: np.deg2rad(8.0) if t >= 0.3 else 0.0,
        pedal_func=lambda t: 0.2 if t >= 1.0 else 0.0,
        throttle_func=lambda t: 0.5 if 0.5 <= t < 1.0 else 0.0,
        dt_out=0.02,
    )
    ok = np.all(np.isfinite(res.vx)) and float(np.max(res.utilization)) <= 1.05
    return ok, {"final_vx": float(res.vx[-1]), "max_util": float(np.max(res.utilization))}


def run_tv_validation() -> bool:
    print("=== Phase 5.4 Torque Vectoring Validation ===\n")
    tests = [
        ("open_differential", test_open_differential),
        ("fixed_bias", test_fixed_bias),
        ("active_delta_T", test_active_delta_T),
        ("controller_yaw_response", test_controller_yaw_response),
        ("torque_balance", test_torque_balance),
        ("straight_acceleration", test_straight_acceleration),
        ("corner_exit", test_corner_exit),
        ("esc_compatibility", test_esc_compatibility),
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
    run_tv_validation()
