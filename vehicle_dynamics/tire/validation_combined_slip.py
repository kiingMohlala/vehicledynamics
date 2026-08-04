"""
Phase 7.6 – Combined-slip Pacejka validation.
"""

from __future__ import annotations

import numpy as np
from .pacejka import PacejkaTire, combined_weight_x, combined_weight_y
from .pacejka_parameters import PacejkaParams


def test_zero_combined_equals_pure() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(combined_slip=True))
    s = tire.longitudinal_lateral_force(0.15, 0.0, 4000.0)
    ok = abs(s.combined_Gx - 1.0) < 1e-9 and abs(s.Fx - s.Fx_pure) < 1e-6
    s2 = tire.longitudinal_lateral_force(0.0, 0.1, 4000.0)
    ok = ok and abs(s2.combined_Gy - 1.0) < 1e-9 and abs(s2.Fy - s2.Fy_pure) < 1e-6
    return ok, {"Gx": s.combined_Gx, "Gy": s2.combined_Gy}


def test_pure_braking_unchanged() -> tuple[bool, dict]:
    on = PacejkaTire(PacejkaParams(combined_slip=True))
    off = PacejkaTire(PacejkaParams(combined_slip=False))
    errs = []
    for k in np.linspace(-0.5, 0.5, 21):
        s1 = on.longitudinal_lateral_force(k, 0.0, 4000.0)
        s0 = off.longitudinal_lateral_force(k, 0.0, 4000.0)
        errs.append(abs(s1.Fx - s0.Fx) + abs(s1.Fy - s0.Fy))
    ok = max(errs) < 1e-5
    return ok, {"max_err": float(max(errs))}


def test_pure_cornering_unchanged() -> tuple[bool, dict]:
    on = PacejkaTire(PacejkaParams(combined_slip=True))
    off = PacejkaTire(PacejkaParams(combined_slip=False))
    errs = []
    for a in np.linspace(-0.3, 0.3, 21):
        s1 = on.longitudinal_lateral_force(0.0, a, 4000.0)
        s0 = off.longitudinal_lateral_force(0.0, a, 4000.0)
        errs.append(abs(s1.Fx - s0.Fx) + abs(s1.Fy - s0.Fy))
    ok = max(errs) < 1e-5
    return ok, {"max_err": float(max(errs))}


def test_trail_braking_reduces_fx() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(combined_slip=True))
    pure = tire.longitudinal_lateral_force(0.15, 0.0, 4000.0)
    trail = tire.longitudinal_lateral_force(0.15, 0.12, 4000.0)
    ok = abs(trail.Fx) < abs(pure.Fx) * 0.99 and trail.combined_Gx < 1.0
    return ok, {"Fx_pure": pure.Fx, "Fx_trail": trail.Fx, "Gx": trail.combined_Gx}


def test_trail_braking_reduces_fy() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(combined_slip=True))
    pure = tire.longitudinal_lateral_force(0.0, 0.12, 4000.0)
    trail = tire.longitudinal_lateral_force(0.15, 0.12, 4000.0)
    ok = abs(trail.Fy) < abs(pure.Fy) * 0.99 and trail.combined_Gy < 1.0
    return ok, {"Fy_pure": pure.Fy, "Fy_trail": trail.Fy, "Gy": trail.combined_Gy}


def test_weighting_monotonic() -> tuple[bool, dict]:
    alphas = np.linspace(0.0, 0.4, 21)
    Gx = [combined_weight_x(a, 0.15) for a in alphas]
    ok = all(Gx[i] >= Gx[i + 1] - 1e-12 for i in range(len(Gx) - 1))
    kappas = np.linspace(0.0, 0.4, 21)
    Gy = [combined_weight_y(k, 0.12) for k in kappas]
    ok = ok and all(Gy[i] >= Gy[i + 1] - 1e-12 for i in range(len(Gy) - 1))
    ok = ok and abs(Gx[0] - 1.0) < 1e-12 and abs(Gy[0] - 1.0) < 1e-12
    return ok, {"Gx_end": Gx[-1], "Gy_end": Gy[-1]}


def test_safety_clamp_rarely_active() -> tuple[bool, dict]:
    """Combined-slip must reduce clamp activations vs pure MF + clamp."""
    on = PacejkaTire(PacejkaParams(combined_slip=True))
    off = PacejkaTire(PacejkaParams(combined_slip=False))
    act_on = act_off = total = 0
    for k in np.linspace(-0.4, 0.4, 17):
        for a in np.linspace(-0.25, 0.25, 17):
            total += 1
            if on.longitudinal_lateral_force(k, a, 4000.0).clamp_activated:
                act_on += 1
            if off.longitudinal_lateral_force(k, a, 4000.0).clamp_activated:
                act_off += 1
    rate_on = act_on / total
    rate_off = act_off / total
    ok = rate_on < rate_off  # combined slip reduces reliance on clamp
    ok = ok and rate_on < 0.25  # still a minority of samples
    return ok, {
        "rate_combined": rate_on,
        "rate_pure_clamp": rate_off,
        "activations_on": act_on,
        "activations_off": act_off,
    }


def test_combined_utilization_le_1() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(combined_slip=True))
    max_u = 0.0
    for k in np.linspace(-1, 1, 21):
        for a in np.linspace(-0.5, 0.5, 21):
            s = tire.longitudinal_lateral_force(k, a, 4000.0)
            max_u = max(max_u, s.utilization)
    ok = max_u <= 1.02
    return ok, {"max_utilization": max_u}


def test_disabled_phase75() -> tuple[bool, dict]:
    a = PacejkaTire(PacejkaParams(combined_slip=False))
    b = PacejkaTire(PacejkaParams(combined_slip=False, alpha_combined=0.01))
    errs = []
    for k in (0.0, 0.1, 0.3):
        for al in (0.0, 0.05, 0.15):
            s0 = a.longitudinal_lateral_force(k, al, 3500.0)
            s1 = b.longitudinal_lateral_force(k, al, 3500.0)
            errs.append(abs(s0.Fx - s1.Fx) + abs(s0.Fy - s1.Fy))
    ok = max(errs) < 1e-9
    return ok, {"max_err": float(max(errs))}


def test_no_nan_inf() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(combined_slip=True))
    ok = True
    for Fz in (100.0, 4000.0, 12000.0):
        for k in (-1.0, 0.0, 0.5, 1.0):
            for a in (-0.8, 0.0, 0.4):
                s = tire.longitudinal_lateral_force(k, a, Fz)
                vals = [s.Fx, s.Fy, s.combined_Gx, s.combined_Gy, s.utilization]
                if not all(np.isfinite(v) for v in vals):
                    ok = False
    return ok, {}


def test_regression_smoke() -> tuple[bool, dict]:
    tire = PacejkaTire()
    s = tire.longitudinal_lateral_force(0.1, 0.08, 4000.0)
    ok = s.combined_Gx < 1.0 and s.combined_Gy < 1.0
    ok = ok and abs(s.Fx) < abs(s.Fx_pure) and abs(s.Fy) < abs(s.Fy_pure)
    ok = ok and np.isfinite(s.Fx + s.Fy)
    return ok, {
        "Gx": s.combined_Gx,
        "Gy": s.combined_Gy,
        "Fx": s.Fx,
        "Fy": s.Fy,
    }


def run_phase76_validation() -> bool:
    print("=== Phase 7.6 Combined-Slip Pacejka Validation ===\n")
    tests = [
        ("zero_combined_equals_pure", test_zero_combined_equals_pure),
        ("pure_braking_unchanged", test_pure_braking_unchanged),
        ("pure_cornering_unchanged", test_pure_cornering_unchanged),
        ("trail_braking_reduces_fx", test_trail_braking_reduces_fx),
        ("trail_braking_reduces_fy", test_trail_braking_reduces_fy),
        ("weighting_monotonic", test_weighting_monotonic),
        ("safety_clamp_rarely_active", test_safety_clamp_rarely_active),
        ("combined_utilization_le_1", test_combined_utilization_le_1),
        ("disabled_phase75", test_disabled_phase75),
        ("no_nan_inf", test_no_nan_inf),
        ("regression_smoke", test_regression_smoke),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:36} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_phase76_validation()
