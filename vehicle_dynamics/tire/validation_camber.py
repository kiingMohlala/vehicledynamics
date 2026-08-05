"""
Phase 7.0 – Camber thrust validation.
"""

from __future__ import annotations

import numpy as np
from .dugoff import DugoffTire, DugoffParams


def test_zero_camber_matches_baseline() -> tuple[bool, dict]:
    """camber=0 → identical Fy/Fx to explicit no-camber path."""
    tire = DugoffTire(DugoffParams())
    ref = DugoffTire(DugoffParams(camber_enabled=False))
    errs = []
    for kappa in (-0.2, 0.0, 0.15):
        for alpha in (-0.1, 0.0, 0.08):
            a = tire.longitudinal_lateral_force(kappa, alpha, 4000.0, camber_rad=0.0)
            b = ref.longitudinal_lateral_force(kappa, alpha, 4000.0, camber_rad=0.0)
            errs.append(abs(a.Fx - b.Fx) + abs(a.Fy - b.Fy))
    ok = max(errs) < 1e-9
    return ok, {"max_err": float(max(errs))}


def test_camber_thrust_sign() -> tuple[bool, dict]:
    tire = DugoffTire(DugoffParams(C_gamma=2000.0))
    pos = tire.longitudinal_lateral_force(0.0, 0.0, 4000.0, camber_rad=0.05)
    neg = tire.longitudinal_lateral_force(0.0, 0.0, 4000.0, camber_rad=-0.05)
    ok = pos.Fy > 0 and neg.Fy < 0 and abs(pos.Fy + neg.Fy) < 1e-9
    ok = ok and abs(pos.Fy_camber - 2000.0 * 0.05) < 1e-6
    return ok, {"Fy_pos": pos.Fy, "Fy_neg": neg.Fy, "Fy_camber": pos.Fy_camber}


def test_load_scaling() -> tuple[bool, dict]:
    tire = DugoffTire(DugoffParams(C_gamma=2000.0, Fz_ref=4000.0))
    a = tire.longitudinal_lateral_force(0.0, 0.0, 4000.0, camber_rad=0.04)
    b = tire.longitudinal_lateral_force(0.0, 0.0, 2000.0, camber_rad=0.04)
    ok = abs(a.Fy_camber - 2.0 * b.Fy_camber) < 1e-6
    return ok, {"Fy_c_4000": a.Fy_camber, "Fy_c_2000": b.Fy_camber}


def test_friction_limit_with_camber() -> tuple[bool, dict]:
    tire = DugoffTire(DugoffParams(mu=1.0, C_gamma=50000.0))
    st = tire.longitudinal_lateral_force(0.0, 0.0, 3000.0, camber_rad=0.3)
    F_mag = np.hypot(st.Fx, st.Fy)
    F_max = 1.0 * 3000.0
    ok = F_mag <= F_max + 1e-6
    return ok, {"F_mag": F_mag, "F_max": F_max, "clamp": st.clamp_activated}


def test_camber_adds_to_cornering() -> tuple[bool, dict]:
    """At small alpha, positive camber increases |Fy| in same sense."""
    tire = DugoffTire(DugoffParams(C_gamma=3000.0))
    base = tire.longitudinal_lateral_force(0.0, 0.02, 4000.0, camber_rad=0.0)
    with_g = tire.longitudinal_lateral_force(0.0, 0.02, 4000.0, camber_rad=0.03)
    ok = with_g.Fy > base.Fy
    return ok, {"Fy_base": base.Fy, "Fy_camber": with_g.Fy}


def test_disable_flag() -> tuple[bool, dict]:
    tire = DugoffTire(DugoffParams(camber_enabled=False, C_gamma=5000.0))
    st = tire.longitudinal_lateral_force(0.0, 0.0, 4000.0, camber_rad=0.1)
    ok = abs(st.Fy) < 1e-12 and abs(st.Fy_camber) < 1e-12
    return ok, {"Fy": st.Fy, "Fy_camber": st.Fy_camber}


def test_state_fields() -> tuple[bool, dict]:
    st = DugoffTire().longitudinal_lateral_force(0.1, 0.05, 3500.0, camber_rad=0.02)
    ok = hasattr(st, "camber_rad") and hasattr(st, "Fy_camber")
    ok = ok and abs(st.camber_rad - 0.02) < 1e-15
    return ok, {"camber_rad": st.camber_rad, "Fy_camber": st.Fy_camber}


def test_no_nan() -> tuple[bool, dict]:
    tire = DugoffTire()
    ok = True
    for g in (-0.1, 0.0, 0.1):
        for a in (-0.2, 0.0, 0.2):
            st = tire.longitudinal_lateral_force(0.05, a, 4000.0, camber_rad=g)
            if not all(np.isfinite(x) for x in (st.Fx, st.Fy, st.Fy_camber, st.utilization)):
                ok = False
    return ok, {}


def run_phase70_validation() -> bool:
    print("=== Phase 7.0 Camber Thrust Validation ===\n")
    tests = [
        ("zero_camber_matches_baseline", test_zero_camber_matches_baseline),
        ("camber_thrust_sign", test_camber_thrust_sign),
        ("load_scaling", test_load_scaling),
        ("friction_limit_with_camber", test_friction_limit_with_camber),
        ("camber_adds_to_cornering", test_camber_adds_to_cornering),
        ("disable_flag", test_disable_flag),
        ("state_fields", test_state_fields),
        ("no_nan", test_no_nan),
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
    run_phase70_validation()
