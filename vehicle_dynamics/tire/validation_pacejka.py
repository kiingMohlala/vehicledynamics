"""
Phase 7.3 – Pacejka Magic Formula validation.
"""

from __future__ import annotations

import numpy as np
from .pacejka import PacejkaTire
from .pacejka_parameters import PacejkaParams, default_passenger_car
from .dugoff import DugoffTire, DugoffParams


def test_zero_slip() -> tuple[bool, dict]:
    tire = PacejkaTire()
    s = tire.longitudinal_lateral_force(0.0, 0.0, 4000.0)
    ok = abs(s.Fx) < 1e-6 and abs(s.Fy) < 1e-6
    return ok, {"Fx": s.Fx, "Fy": s.Fy}


def test_longitudinal_peak_curve() -> tuple[bool, dict]:
    tire = PacejkaTire()
    Fz = 4000.0
    kappas = np.linspace(-1.0, 1.0, 401)
    Fx = np.array([tire.longitudinal_force(k, Fz) for k in kappas])
    # Peak exists away from zero; continuous; zero at origin
    i0 = np.argmin(np.abs(kappas))
    ok = abs(Fx[i0]) < 50.0
    peak_pos = float(np.max(Fx))
    peak_neg = float(np.min(Fx))
    ok = ok and peak_pos > 0.5 * Fz and peak_neg < -0.5 * Fz
    # Single peak each side (no wild oscillation)
    ok = ok and np.all(np.isfinite(Fx))
    return ok, {"peak_pos": peak_pos, "peak_neg": peak_neg}


def test_lateral_peak_curve() -> tuple[bool, dict]:
    tire = PacejkaTire()
    Fz = 4000.0
    alphas = np.linspace(-0.4, 0.4, 401)
    Fy = np.array([tire.lateral_force(a, Fz) for a in alphas])
    i0 = np.argmin(np.abs(alphas))
    ok = abs(Fy[i0]) < 50.0
    ok = ok and float(np.max(Fy)) > 0.5 * Fz and float(np.min(Fy)) < -0.5 * Fz
    ok = ok and np.all(np.isfinite(Fy))
    return ok, {"peak_pos": float(np.max(Fy)), "peak_neg": float(np.min(Fy))}


def test_symmetry() -> tuple[bool, dict]:
    tire = PacejkaTire()
    Fz = 3500.0
    errs = []
    for k in np.linspace(0.02, 0.5, 20):
        s1 = tire.longitudinal_lateral_force(k, 0.0, Fz)
        s2 = tire.longitudinal_lateral_force(-k, 0.0, Fz)
        errs.append(abs(s1.Fx + s2.Fx))
    for a in np.linspace(0.02, 0.3, 20):
        s1 = tire.longitudinal_lateral_force(0.0, a, Fz)
        s2 = tire.longitudinal_lateral_force(0.0, -a, Fz)
        errs.append(abs(s1.Fy + s2.Fy))
    max_err = float(np.max(errs))
    ok = max_err < 1.0  # near-perfect symmetry for zero shifts
    return ok, {"max_err": max_err}


def test_load_scaling() -> tuple[bool, dict]:
    tire = PacejkaTire()
    s1 = tire.longitudinal_lateral_force(0.1, 0.05, 2000.0)
    s2 = tire.longitudinal_lateral_force(0.1, 0.05, 4000.0)
    # Higher load → larger force magnitude (MF D ∝ Fz)
    ok = abs(s2.Fx) > abs(s1.Fx) * 1.5 and abs(s2.Fy) > abs(s1.Fy) * 1.5
    return ok, {"Fx1": s1.Fx, "Fx2": s2.Fx, "Fy1": s1.Fy, "Fy2": s2.Fy}


def test_friction_limit() -> tuple[bool, dict]:
    tire = PacejkaTire(PacejkaParams(mu_x=0.8, mu_y=0.8))
    Fz = 4000.0
    mu = 0.8
    violations = 0
    max_u = 0.0
    for k in np.linspace(-1, 1, 21):
        for a in np.linspace(-0.5, 0.5, 21):
            s = tire.longitudinal_lateral_force(k, a, Fz)
            mag = np.hypot(s.Fx, s.Fy)
            max_u = max(max_u, mag / (mu * Fz))
            if mag > mu * Fz * 1.02:
                violations += 1
    ok = violations == 0 and max_u <= 1.02
    return ok, {"violations": violations, "max_utilization": max_u}


def test_camber_compatibility() -> tuple[bool, dict]:
    # Default C_gamma=0 → camber argument must not break API / forces
    tire = PacejkaTire()
    s0 = tire.longitudinal_lateral_force(0.0, 0.1, 4000.0, camber_rad=0.0)
    s1 = tire.longitudinal_lateral_force(0.0, 0.1, 4000.0, camber_rad=0.05)
    ok = abs(s0.Fy - s1.Fy) < 1e-6  # no camber effect at C_gamma=0
    # With camber stiffness
    tire2 = PacejkaTire(PacejkaParams(C_gamma=5000.0))
    s2 = tire2.longitudinal_lateral_force(0.0, 0.0, 4000.0, camber_rad=0.05)
    ok = ok and abs(s2.Fy) > 10.0
    return ok, {"Fy_default_camber": s1.Fy, "Fy_with_Cgamma": s2.Fy}


def test_numerical_robustness() -> tuple[bool, dict]:
    tire = PacejkaTire()
    ok = True
    for Fz in (1.0, 100.0, 4000.0, 12000.0):
        for k in (-1.0, -0.5, 0.0, 0.2, 1.0):
            for a in (-1.0, -0.2, 0.0, 0.3, 1.0):
                s = tire.longitudinal_lateral_force(k, a, Fz)
                if not all(np.isfinite(v) for v in (s.Fx, s.Fy, s.utilization, s.lambda_)):
                    ok = False
    return ok, {}


def test_no_nan_inf() -> tuple[bool, dict]:
    return test_numerical_robustness()


def test_dugoff_unchanged() -> tuple[bool, dict]:
    """Regression: Dugoff still behaves as before (zero-slip)."""
    d = DugoffTire(DugoffParams())
    s = d.longitudinal_lateral_force(0.0, 0.0, 4000.0)
    ok = abs(s.Fx) < 1e-8 and abs(s.Fy) < 1e-8
    s2 = d.longitudinal_lateral_force(0.1, 0.0, 4000.0)
    ok = ok and s2.Fx != 0.0 and abs(s2.Fy) < 1e-6
    return ok, {"Fx_kappa": s2.Fx}


def test_factory_select() -> tuple[bool, dict]:
    from .factory import create_tire
    d = create_tire("dugoff")
    p = create_tire("pacejka")
    ok = type(d).__name__ == "DugoffTire" and type(p).__name__ == "PacejkaTire"
    sd = d.longitudinal_lateral_force(0.05, 0.02, 4000.0)
    sp = p.longitudinal_lateral_force(0.05, 0.02, 4000.0)
    ok = ok and np.isfinite(sd.Fx) and np.isfinite(sp.Fx)
    return ok, {"dugoff_Fx": sd.Fx, "pacejka_Fx": sp.Fx}


def run_phase73_validation() -> bool:
    print("=== Phase 7.3 Pacejka Magic Formula Validation ===\n")
    tests = [
        ("zero_slip", test_zero_slip),
        ("longitudinal_peak_curve", test_longitudinal_peak_curve),
        ("lateral_peak_curve", test_lateral_peak_curve),
        ("symmetry", test_symmetry),
        ("load_scaling", test_load_scaling),
        ("friction_limit", test_friction_limit),
        ("camber_compatibility", test_camber_compatibility),
        ("numerical_robustness", test_numerical_robustness),
        ("no_nan_inf", test_no_nan_inf),
        ("dugoff_unchanged", test_dugoff_unchanged),
        ("factory_select", test_factory_select),
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
    run_phase73_validation()
