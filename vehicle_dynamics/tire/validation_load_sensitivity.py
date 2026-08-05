"""
Phase 7.5 – Load-sensitive tire validation.
"""

from __future__ import annotations

import numpy as np
from .load_sensitivity import effective_mu
from .dugoff import DugoffTire, DugoffParams
from .pacejka import PacejkaTire
from .pacejka_parameters import PacejkaParams


def test_disabled_baseline() -> tuple[bool, dict]:
    """load_sensitive=False must match constant-μ model."""
    d0 = DugoffTire(DugoffParams(load_sensitive=False, mu=1.0))
    d1 = DugoffTire(DugoffParams(load_sensitive=False, mu=1.0, load_exponent=0.2))
    errs = []
    for Fz in (2000.0, 4000.0, 8000.0):
        for k, a in [(0.1, 0.0), (0.0, 0.08), (0.12, 0.05)]:
            s0 = d0.longitudinal_lateral_force(k, a, Fz)
            s1 = d1.longitudinal_lateral_force(k, a, Fz)
            errs.append(abs(s0.Fx - s1.Fx) + abs(s0.Fy - s1.Fy))
    ok = max(errs) < 1e-9
    return ok, {"max_err": float(max(errs))}


def test_fz_equals_fz0() -> tuple[bool, dict]:
    mu = effective_mu(1.0, 4000.0, 4000.0, 0.08)
    ok = abs(mu - 1.0) < 1e-12
    return ok, {"mu": mu}


def test_higher_load_lower_mu() -> tuple[bool, dict]:
    mu_lo = effective_mu(1.0, 2000.0, 4000.0, 0.08)
    mu_ref = effective_mu(1.0, 4000.0, 4000.0, 0.08)
    mu_hi = effective_mu(1.0, 8000.0, 4000.0, 0.08)
    ok = mu_lo > mu_ref > mu_hi > 0.0
    return ok, {"mu_lo": mu_lo, "mu_ref": mu_ref, "mu_hi": mu_hi}


def test_lower_load_higher_mu() -> tuple[bool, dict]:
    # covered by higher_load test directionality
    mu_lo = effective_mu(0.9, 1500.0, 4000.0, 0.10)
    mu_ref = effective_mu(0.9, 4000.0, 4000.0, 0.10)
    ok = mu_lo > mu_ref
    return ok, {"mu_lo": mu_lo, "mu_ref": mu_ref}


def test_force_finite() -> tuple[bool, dict]:
    tire = DugoffTire(
        DugoffParams(load_sensitive=True, load_exponent=0.08, Fz0=4000.0)
    )
    ok = True
    for Fz in (50.0, 500.0, 4000.0, 15000.0):
        s = tire.longitudinal_lateral_force(0.3, 0.15, Fz)
        if not all(np.isfinite([s.Fx, s.Fy, s.utilization, s.lambda_])):
            ok = False
    return ok, {}


def test_friction_limit() -> tuple[bool, dict]:
    tire = DugoffTire(
        DugoffParams(mu=1.0, load_sensitive=True, load_exponent=0.08, Fz0=4000.0)
    )
    violations = 0
    max_u = 0.0
    for Fz in (2000.0, 4000.0, 8000.0):
        mu = effective_mu(1.0, Fz, 4000.0, 0.08)
        for k in np.linspace(-1, 1, 11):
            for a in np.linspace(-0.4, 0.4, 9):
                s = tire.longitudinal_lateral_force(k, a, Fz)
                mag = np.hypot(s.Fx, s.Fy)
                max_u = max(max_u, mag / (mu * Fz + 1e-12))
                if mag > mu * Fz * 1.02:
                    violations += 1
    ok = violations == 0 and max_u <= 1.02
    return ok, {"violations": violations, "max_u": max_u}


def test_dugoff_disabled_unchanged() -> tuple[bool, dict]:
    t_off = DugoffTire(DugoffParams(load_sensitive=False))
    # Reference: same params, force at several points
    t_ref = DugoffTire(DugoffParams(load_sensitive=False, mu=1.0))
    errs = []
    for Fz in (2000.0, 6000.0):
        s0 = t_off.longitudinal_lateral_force(0.1, 0.05, Fz)
        s1 = t_ref.longitudinal_lateral_force(0.1, 0.05, Fz)
        errs.append(abs(s0.Fx - s1.Fx) + abs(s0.Fy - s1.Fy))
    ok = max(errs) < 1e-9
    return ok, {"max_err": float(max(errs))}


def test_pacejka_disabled_unchanged() -> tuple[bool, dict]:
    t0 = PacejkaTire(PacejkaParams(load_sensitive=False))
    t1 = PacejkaTire(PacejkaParams(load_sensitive=False, load_exponent=0.2))
    errs = []
    for Fz in (2500.0, 5000.0):
        s0 = t0.longitudinal_lateral_force(0.08, 0.06, Fz)
        s1 = t1.longitudinal_lateral_force(0.08, 0.06, Fz)
        errs.append(abs(s0.Fx - s1.Fx) + abs(s0.Fy - s1.Fy))
    ok = max(errs) < 1e-9
    return ok, {"max_err": float(max(errs))}


def test_no_nan_inf() -> tuple[bool, dict]:
    ok = True
    for n in (0.0, 0.05, 0.08, 0.15):
        for Fz in (1.0, 100.0, 4000.0, 20000.0):
            mu = effective_mu(1.2, Fz, 4000.0, n)
            if not np.isfinite(mu) or mu <= 0:
                ok = False
    return ok, {}


def test_regression_smoke() -> tuple[bool, dict]:
    """Enabled load sensitivity changes force with Fz; disabled does not (beyond Fz scaling)."""
    off = DugoffTire(DugoffParams(load_sensitive=False, mu=1.0))
    on = DugoffTire(
        DugoffParams(load_sensitive=True, mu=1.0, Fz0=4000.0, load_exponent=0.08)
    )
    # At Fz0, forces should match
    s0 = off.longitudinal_lateral_force(0.2, 0.0, 4000.0)
    s1 = on.longitudinal_lateral_force(0.2, 0.0, 4000.0)
    ok_ref = abs(s0.Fx - s1.Fx) < 1e-6

    # At high load, load-sensitive peak force efficiency drops vs linear Fz scaling
    # Compare utilization-normalized: |Fx|/(mu0*Fz) should be lower when sensitive at high Fz
    s_off_hi = off.longitudinal_lateral_force(1.0, 0.0, 8000.0)
    s_on_hi = on.longitudinal_lateral_force(1.0, 0.0, 8000.0)
    # With load sensitivity, available friction force mu_eff*Fz grows slower than mu0*Fz
    ok_hi = abs(s_on_hi.Fx) < abs(s_off_hi.Fx) * 0.99
    ok = ok_ref and ok_hi
    return ok, {
        "Fx_ref_off": s0.Fx,
        "Fx_ref_on": s1.Fx,
        "Fx_hi_off": s_off_hi.Fx,
        "Fx_hi_on": s_on_hi.Fx,
    }


def run_phase75_validation() -> bool:
    print("=== Phase 7.5 Load-Sensitive Tire Validation ===\n")
    tests = [
        ("disabled_baseline", test_disabled_baseline),
        ("fz_equals_fz0", test_fz_equals_fz0),
        ("higher_load_lower_mu", test_higher_load_lower_mu),
        ("lower_load_higher_mu", test_lower_load_higher_mu),
        ("force_finite", test_force_finite),
        ("friction_limit", test_friction_limit),
        ("dugoff_disabled_unchanged", test_dugoff_disabled_unchanged),
        ("pacejka_disabled_unchanged", test_pacejka_disabled_unchanged),
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
    run_phase75_validation()
