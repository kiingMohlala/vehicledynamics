"""
Phase 7.7 – Self-aligning torque (Mz) validation.
"""

from __future__ import annotations

import numpy as np
from .pacejka import PacejkaTire, pneumatic_trail
from .pacejka_parameters import PacejkaParams


def test_disabled_phase76() -> tuple[bool, dict]:
    off = PacejkaTire(PacejkaParams(aligning_torque=False))
    # Force path identical regardless of trail parameters when disabled
    a = PacejkaTire(PacejkaParams(aligning_torque=False, trail0=0.05))
    b = PacejkaTire(PacejkaParams(aligning_torque=False, trail0=0.20))
    errs = []
    for k in (0.0, 0.1):
        for al in (0.0, 0.08, 0.2):
            s0 = a.longitudinal_lateral_force(k, al, 4000.0)
            s1 = b.longitudinal_lateral_force(k, al, 4000.0)
            errs.append(abs(s0.Fx - s1.Fx) + abs(s0.Fy - s1.Fy))
            if abs(s0.Mz) > 1e-12 or abs(s1.Mz) > 1e-12:
                return False, {"error": "Mz not zero when disabled"}
    ok = max(errs) < 1e-9
    return ok, {"max_err": float(max(errs)), "Mz": off.longitudinal_lateral_force(0.1, 0.1, 4000.0).Mz}


def test_zero_slip_mz() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(aligning_torque=True))
    s = tire.longitudinal_lateral_force(0.0, 0.0, 4000.0)
    ok = abs(s.Mz) < 1e-9 and abs(s.Fy) < 1e-9
    # trail at zero alpha is t0
    ok = ok and abs(s.pneumatic_trail - 0.05) < 1e-12
    return ok, {"Mz": s.Mz, "trail": s.pneumatic_trail}


def test_moderate_alpha_finite_mz() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(aligning_torque=True))
    s = tire.longitudinal_lateral_force(0.0, 0.08, 4000.0)
    ok = abs(s.Mz) > 1.0 and np.isfinite(s.Mz)
    ok = ok and abs(s.Mz + s.Fy * s.pneumatic_trail) < 1e-6
    return ok, {"Mz": s.Mz, "Fy": s.Fy, "trail": s.pneumatic_trail}


def test_sign_reversal() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(aligning_torque=True))
    s_pos = tire.longitudinal_lateral_force(0.0, 0.1, 4000.0)
    s_neg = tire.longitudinal_lateral_force(0.0, -0.1, 4000.0)
    ok = s_pos.Mz * s_neg.Mz < 0  # opposite signs
    ok = ok and abs(s_pos.Mz + s_neg.Mz) < 1e-6  # magnitude symmetry
    return ok, {"Mz_pos": s_pos.Mz, "Mz_neg": s_neg.Mz}


def test_trail_decreases_with_alpha() -> tuple[bool, dict]:
    alphas = [0.0, 0.05, 0.10, 0.20, 0.40]
    trails = [pneumatic_trail(a, 0.05, 0.15) for a in alphas]
    ok = all(trails[i] >= trails[i + 1] - 1e-12 for i in range(len(trails) - 1))
    ok = ok and abs(trails[0] - 0.05) < 1e-12
    return ok, {"trails": trails}


def test_large_alpha_mz_reduces() -> tuple[bool, dict]:
    """|Mz| peaks at moderate α then falls as trail collapses."""
    tire = PacejkaTire(PacejkaParams(aligning_torque=True))
    alphas = np.linspace(0.02, 0.6, 30)
    Mz_abs = [
        abs(tire.longitudinal_lateral_force(0.0, a, 4000.0).Mz) for a in alphas
    ]
    # Peak not at the last (large α) sample
    i_peak = int(np.argmax(Mz_abs))
    ok = i_peak < len(alphas) - 3
    ok = ok and Mz_abs[-1] < Mz_abs[i_peak] * 0.5
    return ok, {
        "peak_alpha": float(alphas[i_peak]),
        "peak_Mz": Mz_abs[i_peak],
        "Mz_large": Mz_abs[-1],
    }


def test_combined_slip_compatibility() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(aligning_torque=True, combined_slip=True))
    pure = tire.longitudinal_lateral_force(0.0, 0.1, 4000.0)
    trail = tire.longitudinal_lateral_force(0.15, 0.1, 4000.0)
    # Combined slip reduces |Fy| → |Mz| should also reduce (same trail)
    ok = abs(trail.Mz) < abs(pure.Mz)
    ok = ok and abs(trail.pneumatic_trail - pure.pneumatic_trail) < 1e-12
    return ok, {"Mz_pure": pure.Mz, "Mz_combined": trail.Mz}


def test_no_nan_inf() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(aligning_torque=True))
    ok = True
    for Fz in (100.0, 4000.0, 10000.0):
        for a in (-0.8, -0.1, 0.0, 0.1, 0.8):
            for k in (-0.5, 0.0, 0.3):
                s = tire.longitudinal_lateral_force(k, a, Fz)
                if not all(np.isfinite([s.Mz, s.pneumatic_trail, s.Fx, s.Fy])):
                    ok = False
    return ok, {}


def test_regression_smoke() -> tuple[bool, dict]:
    on = PacejkaTire(PacejkaParams(aligning_torque=True))
    off = PacejkaTire(PacejkaParams(aligning_torque=False))
    s_on = on.longitudinal_lateral_force(0.05, 0.1, 4000.0)
    s_off = off.longitudinal_lateral_force(0.05, 0.1, 4000.0)
    # Forces identical
    ok = abs(s_on.Fx - s_off.Fx) < 1e-9 and abs(s_on.Fy - s_off.Fy) < 1e-9
    ok = ok and abs(s_on.Mz) > 1.0 and abs(s_off.Mz) < 1e-12
    return ok, {"Mz_on": s_on.Mz, "Mz_off": s_off.Mz}


def run_phase77_validation() -> bool:
    print("=== Phase 7.7 Aligning Torque (Mz) Validation ===\n")
    tests = [
        ("disabled_phase76", test_disabled_phase76),
        ("zero_slip_mz", test_zero_slip_mz),
        ("moderate_alpha_finite_mz", test_moderate_alpha_finite_mz),
        ("sign_reversal", test_sign_reversal),
        ("trail_decreases_with_alpha", test_trail_decreases_with_alpha),
        ("large_alpha_mz_reduces", test_large_alpha_mz_reduces),
        ("combined_slip_compatibility", test_combined_slip_compatibility),
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
    run_phase77_validation()
