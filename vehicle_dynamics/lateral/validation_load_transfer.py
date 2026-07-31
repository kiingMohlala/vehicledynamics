"""
Phase 4.1 – Load-transfer diagnostics validation (Level A).

Does not modify bicycle dynamics. Verifies conservation, symmetry,
bounds, and zero-ay behaviour.
"""

import numpy as np
from .load_transfer import (
    LoadTransferParameters,
    LoadTransferState,
    compute_load_transfer,
)

def test_zero_ay():
    p = LoadTransferParameters()
    Fz_f, Fz_r = 6000.0, 7000.0
    s = compute_load_transfer(0.0, Fz_f, Fz_r, p, mass=1400.0)
    ok = (
        abs(s.dFz_front) < 1e-9 and abs(s.dFz_rear) < 1e-9 and
        abs(s.Fz_fl - Fz_f/2) < 1e-6 and abs(s.Fz_fr - Fz_f/2) < 1e-6 and
        abs(s.Fz_rl - Fz_r/2) < 1e-6 and abs(s.Fz_rr - Fz_r/2) < 1e-6 and
        not s.wheel_lift_front and not s.wheel_lift_rear
    )
    return ok, {"dFz_front": s.dFz_front, "dFz_rear": s.dFz_rear}

def test_total_load_conserved():
    p = LoadTransferParameters()
    Fz_f, Fz_r = 6000.0, 7000.0
    s = compute_load_transfer(5.0, Fz_f, Fz_r, p, mass=1400.0)
    # Before clamp, totals should match axle totals
    # After clamp they may differ slightly if lift occurs; for moderate ay they should match
    total = s.Fz_fl + s.Fz_fr + s.Fz_rl + s.Fz_rr
    expected = Fz_f + Fz_r
    ok = abs(total - expected) < 1.0 or s.wheel_lift_front or s.wheel_lift_rear
    return ok, {"total": total, "expected": expected}

def test_sign_swap():
    p = LoadTransferParameters()
    Fz_f, Fz_r = 6000.0, 7000.0
    sp = compute_load_transfer(4.0, Fz_f, Fz_r, p, mass=1400.0)
    sn = compute_load_transfer(-4.0, Fz_f, Fz_r, p, mass=1400.0)
    ok = (
        abs(sp.dFz_front + sn.dFz_front) < 1e-6 and
        abs(sp.dFz_rear + sn.dFz_rear) < 1e-6 and
        abs(sp.Fz_fl - sn.Fz_fr) < 1e-3 and
        abs(sp.Fz_fr - sn.Fz_fl) < 1e-3
    )
    return ok, {"dFz_f_pos": sp.dFz_front, "dFz_f_neg": sn.dFz_front}

def test_fz_min_respected():
    p = LoadTransferParameters(Fz_min=50.0, h_cg=0.8)
    Fz_f, Fz_r = 3000.0, 3000.0
    # Large ay to force potential lift
    s = compute_load_transfer(15.0, Fz_f, Fz_r, p, mass=1400.0)
    ok = (
        s.Fz_fl >= p.Fz_min - 1e-6 and s.Fz_fr >= p.Fz_min - 1e-6 and
        s.Fz_rl >= p.Fz_min - 1e-6 and s.Fz_rr >= p.Fz_min - 1e-6
    )
    return ok, {
        "Fz_fl": s.Fz_fl, "Fz_fr": s.Fz_fr,
        "wheel_lift_front": s.wheel_lift_front,
    }

def test_parameter_sensitivity():
    p_low = LoadTransferParameters(h_cg=0.4)
    p_high = LoadTransferParameters(h_cg=0.7)
    Fz_f, Fz_r = 6000.0, 7000.0
    s_low = compute_load_transfer(5.0, Fz_f, Fz_r, p_low, mass=1400.0)
    s_high = compute_load_transfer(5.0, Fz_f, Fz_r, p_high, mass=1400.0)
    ok = abs(s_high.dFz_front) > abs(s_low.dFz_front)
    return ok, {"dFz_low": s_low.dFz_front, "dFz_high": s_high.dFz_front}

def run_all_tests():
    print("=== Phase 4.1 Load-Transfer Diagnostics Validation ===\n")
    tests = [
        ("Zero ay", test_zero_ay),
        ("Total load conserved", test_total_load_conserved),
        ("Sign swap (+ay / -ay)", test_sign_swap),
        ("Fz_min respected", test_fz_min_respected),
        ("Parameter sensitivity (h_cg)", test_parameter_sensitivity),
    ]
    all_pass = True
    for name, fn in tests:
        ok, diag = fn()
        print(f"{name:35} : {'PASS' if ok else 'FAIL'}")
        if diag:
            for k, v in diag.items():
                print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass

if __name__ == "__main__":
    run_all_tests()
