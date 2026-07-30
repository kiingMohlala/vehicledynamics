"""
Phase 3 Braking Validation Suite
"""

import numpy as np
from .simulation import BrakeSimulation
from .parameters import VehicleLongitudinalParams
from .weight_transfer import WeightTransfer

def test_static_axle_loads():
    params = VehicleLongitudinalParams()
    wt = WeightTransfer(params)
    Fz_f, Fz_r = wt.loads(0.0)
    total = Fz_f + Fz_r
    expected = params.mass * 9.81
    return abs(total - expected) < 1e-3

def test_weight_transfer():
    params = VehicleLongitudinalParams()
    wt = WeightTransfer(params)
    a = 9.81
    Fz_f, Fz_r = wt.loads(a)
    total = Fz_f + Fz_r
    return abs(total - params.mass * 9.81) < 1e-3

def run_full_validation():
    print("=== Phase 3 Braking Validation ===\n")
    results = []

    results.append(("Static axle loads", test_static_axle_loads()))
    results.append(("Weight transfer", test_weight_transfer()))

    # Basic simulation smoke test
    try:
        sim = BrakeSimulation()
        res = sim.run(v0=22.22, use_abs=True)
        ok = res.stopping_distance > 0 and np.isfinite(res.stopping_distance)
        results.append(("ABS emergency stop", ok))
    except Exception as e:
        results.append(("ABS emergency stop", False))
        print("  Simulation error:", e)

    print("\n=== Validation Summary ===")
    all_pass = True
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"{name:30} : {status}")
        if not passed:
            all_pass = False

    print("\n" + ("ALL TESTS PASSED" if all_pass else "SOME TESTS FAILED"))
    return all_pass

if __name__ == "__main__":
    run_full_validation()
