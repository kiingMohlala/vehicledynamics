"""
Phase 3 Braking Validation Suite

Includes:
  - Phase 3.0 plant checks (static axle loads, weight transfer)
  - ABS emergency-stop smoke test
  - Phase 3.2 ABS unit suite (see validation_abs.py)
"""

import numpy as np
from .simulation import BrakeSimulation
from .parameters import VehicleLongitudinalParams
from .weight_transfer import WeightTransfer
from .validation_abs import run_abs_validation


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

    try:
        sim = BrakeSimulation()
        res = sim.run(v0=22.22, use_abs=True)
        ok = res.stopping_distance > 0 and np.isfinite(res.stopping_distance)
        results.append(("ABS emergency stop", ok))
    except Exception as e:
        results.append(("ABS emergency stop", False))
        print("  Simulation error:", e)

    print("\n--- Phase 3.0 / system smoke ---")
    all_pass = True
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"{name:30} : {status}")
        if not passed:
            all_pass = False

    print()
    abs_ok = run_abs_validation()
    all_pass = all_pass and abs_ok

    print("\n" + ("ALL TESTS PASSED" if all_pass else "SOME TESTS FAILED"))
    return all_pass


if __name__ == "__main__":
    run_full_validation()
