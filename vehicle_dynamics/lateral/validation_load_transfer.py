"""
Phase 4.1 – Load-transfer diagnostics validation (Level A).

Strengthened conservation (axle-by-axle) and theoretical transfer cross-check.
"""

import numpy as np
from .load_transfer import (
    LoadTransferParameters,
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

def test_axle_conservation():
    """Fz_fl + Fz_fr == Fz_f_axle and rear equivalent, including under clamp."""
    p = LoadTransferParameters(Fz_min=50.0, h_cg=0.8)
    cases = [
        (0.0, 6000.0, 7000.0),
        (5.0, 6000.0, 7000.0),
        (12.0, 4000.0, 4000.0),   # likely clamp
        (20.0, 3000.0, 3000.0),   # strong clamp
    ]
    tol = 1e-6
    for ay, Fz_f, Fz_r in cases:
        s = compute_load_transfer(ay, Fz_f, Fz_r, p, mass=1400.0)
        if abs((s.Fz_fl + s.Fz_fr) - Fz_f) > tol:
            # Degenerate: axle total < 2*Fz_min
            if Fz_f >= 2 * p.Fz_min:
                return False, {"ay": ay, "front_sum": s.Fz_fl + s.Fz_fr, "Fz_f": Fz_f}
        if abs((s.Fz_rl + s.Fz_rr) - Fz_r) > tol:
            if Fz_r >= 2 * p.Fz_min:
                return False, {"ay": ay, "rear_sum": s.Fz_rl + s.Fz_rr, "Fz_r": Fz_r}
    return True, {"cases_checked": len(cases)}

def test_vehicle_total_conservation():
    p = LoadTransferParameters()
    Fz_f, Fz_r = 6000.0, 7000.0
    s = compute_load_transfer(5.0, Fz_f, Fz_r, p, mass=1400.0)
    total = s.Fz_fl + s.Fz_fr + s.Fz_rl + s.Fz_rr
    ok = abs(total - (Fz_f + Fz_r)) < 1e-6
    return ok, {"total": total, "expected": Fz_f + Fz_r}

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
    s = compute_load_transfer(15.0, 3000.0, 3000.0, p, mass=1400.0)
    ok = all(x >= p.Fz_min - 1e-6 for x in [s.Fz_fl, s.Fz_fr, s.Fz_rl, s.Fz_rr])
    return ok, {
        "Fz_fl": s.Fz_fl, "Fz_fr": s.Fz_fr,
        "wheel_lift_front": s.wheel_lift_front,
        "front_axle_sum": s.Fz_fl + s.Fz_fr,
    }

def test_theoretical_transfer():
    """When track_f == track_r, dFz_front + dFz_rear == m*ay*h/t."""
    track = 1.55
    p = LoadTransferParameters(track_f=track, track_r=track, chi_f=0.55, h_cg=0.55)
    mass = 1400.0
    ay = 5.0
    s = compute_load_transfer(ay, 6000.0, 7000.0, p, mass=mass)
    theoretical = mass * ay * p.h_cg / track
    actual = s.dFz_front + s.dFz_rear
    ok = abs(actual - theoretical) < 1e-6
    return ok, {"actual": actual, "theoretical": theoretical}

def test_parameter_sensitivity():
    Fz_f, Fz_r = 6000.0, 7000.0
    s_low = compute_load_transfer(5.0, Fz_f, Fz_r, LoadTransferParameters(h_cg=0.4), mass=1400.0)
    s_high = compute_load_transfer(5.0, Fz_f, Fz_r, LoadTransferParameters(h_cg=0.7), mass=1400.0)
    ok = abs(s_high.dFz_front) > abs(s_low.dFz_front)
    return ok, {"dFz_low": s_low.dFz_front, "dFz_high": s_high.dFz_front}

def run_all_tests():
    print("=== Phase 4.1 Load-Transfer Diagnostics Validation ===\n")
    tests = [
        ("Zero ay", test_zero_ay),
        ("Axle conservation", test_axle_conservation),
        ("Vehicle total conservation", test_vehicle_total_conservation),
        ("Sign swap (+ay / -ay)", test_sign_swap),
        ("Fz_min respected", test_fz_min_respected),
        ("Theoretical transfer cross-check", test_theoretical_transfer),
        ("Parameter sensitivity (h_cg)", test_parameter_sensitivity),
    ]
    all_pass = True
    for name, fn in tests:
        ok, diag = fn()
        print(f"{name:40} : {'PASS' if ok else 'FAIL'}")
        if diag:
            for k, v in diag.items():
                print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass

if __name__ == "__main__":
    run_all_tests()
