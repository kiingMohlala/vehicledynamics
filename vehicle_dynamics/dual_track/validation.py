"""
Phase 5.0 – Symmetric regression against Phase 4.2 bicycle model.

Uses equal front steer (use_ackermann=False) so results remain comparable
to the frozen Phase 5.0 baseline. Ackermann-specific checks live in
validation_steering.py (Phase 5.1).
"""

import numpy as np
from .simulation import DualTrackVehicleModel
from .parameters import DualTrackParameters
from .steering import SteeringParameters
from .kinematics import FL, FR, RL, RR
from ..combined.simulation import CombinedVehicleModel


def _equal_steer_model(use_abs: bool = False) -> DualTrackVehicleModel:
    params = DualTrackParameters()
    params.steering = SteeringParameters(use_ackermann=False)
    return DualTrackVehicleModel(params=params, use_abs=use_abs)


def test_symmetric_pure_steering():
    delta = np.deg2rad(3.0)
    dual = _equal_steer_model(use_abs=False)
    bike = CombinedVehicleModel(use_abs=False)

    res_d = dual.simulate(
        vx0=20.0, t_span=(0, 6),
        delta_func=lambda t: delta if t >= 0.5 else 0.0,
        pedal_func=lambda t: 0.0,
    )
    res_b = bike.simulate(
        vx0=20.0, t_span=(0, 6),
        delta_func=lambda t: delta if t >= 0.5 else 0.0,
        pedal_func=lambda t: 0.0,
    )

    r_d = float(np.mean(res_d.r[res_d.time > 5.0]))
    r_b = float(np.mean(res_b.r[res_b.time > 5.0]))
    if abs(r_b) < 1e-6:
        return False, {"reason": "bike r ~ 0"}
    rel = abs(r_d - r_b) / abs(r_b)
    ok = rel < 0.15
    return ok, {"r_dual": r_d, "r_bike": r_b, "rel_error": rel}


def test_symmetric_pure_braking():
    dual = _equal_steer_model(use_abs=True)
    bike = CombinedVehicleModel(use_abs=True)

    res_d = dual.simulate(
        vx0=22.22, t_span=(0, 8),
        delta_func=lambda t: 0.0,
        pedal_func=lambda t: 1.0 if t >= 0.1 else 0.0,
    )
    res_b = bike.simulate(
        vx0=22.22, t_span=(0, 8),
        delta_func=lambda t: 0.0,
        pedal_func=lambda t: 1.0 if t >= 0.1 else 0.0,
    )

    ok = (
        res_d.vx[-1] < 5.0 and res_b.vx[-1] < 5.0 and
        np.max(np.abs(res_d.vy)) < 0.5 and
        np.all(np.isfinite(res_d.vx))
    )
    return ok, {
        "final_vx_dual": float(res_d.vx[-1]),
        "final_vx_bike": float(res_b.vx[-1]),
        "max_vy_dual": float(np.max(np.abs(res_d.vy))),
    }


def test_load_transfer_feedback():
    dual = _equal_steer_model(use_abs=False)
    delta = np.deg2rad(3.0)
    res = dual.simulate(
        vx0=18.0, t_span=(0, 6),
        delta_func=lambda t: delta,
        pedal_func=lambda t: 0.0,
    )
    mask = res.time > 4.0
    Fz_mean = np.mean(res.Fz[mask], axis=0)
    ok = Fz_mean[FR] > Fz_mean[FL] and Fz_mean[RR] > Fz_mean[RL]
    return ok, {
        "Fz_FL": float(Fz_mean[0]),
        "Fz_FR": float(Fz_mean[1]),
        "Fz_RL": float(Fz_mean[2]),
        "Fz_RR": float(Fz_mean[3]),
    }


def test_no_nan():
    dual = _equal_steer_model(use_abs=True)
    res = dual.simulate(
        vx0=18.0, t_span=(0, 5),
        delta_func=lambda t: np.deg2rad(2.0) if t > 0.3 else 0.0,
        pedal_func=lambda t: 0.3 if t > 0.5 else 0.0,
    )
    ok = all(np.all(np.isfinite(a)) for a in [res.vx, res.vy, res.r, res.Fx, res.Fy, res.Fz])
    return ok, {}


def run_regression_gates():
    print("=== Phase 5.0 Dual-Track Regression Gates (equal steer) ===\n")
    tests = [
        ("Symmetric pure steering", test_symmetric_pure_steering),
        ("Symmetric pure braking", test_symmetric_pure_braking),
        ("Load-transfer feedback", test_load_transfer_feedback),
        ("No NaN/Inf", test_no_nan),
    ]
    all_pass = True
    for name, fn in tests:
        ok, diag = fn()
        print(f"{name:30} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL GATES PASSED" if all_pass else "GATES FAILED")
    return all_pass


if __name__ == "__main__":
    run_regression_gates()
