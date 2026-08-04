"""
Phase 5.6 – Longitudinal load transfer validation.
"""

from __future__ import annotations

import numpy as np
from ..lateral.load_transfer import LoadTransferParameters
from .normal_loads import four_wheel_normal_loads, longitudinal_axle_loads

G = 9.81
MASS = 1400.0
A, B = 1.2, 1.5
L = A + B
H = 0.55


def test_zero_ax_matches_phase50() -> tuple[bool, dict]:
    """ax=0 → same as lateral-only (Phase 5.0)."""
    p = LoadTransferParameters(h_cg=H)
    f0 = four_wheel_normal_loads(0.0, MASS, A, B, p, ax=0.0)
    # static equal split left/right
    W = MASS * G
    Fz_f = W * (B / L)
    Fz_r = W * (A / L)
    ok = (
        abs(f0[0] - Fz_f / 2) < 1e-6
        and abs(f0[1] - Fz_f / 2) < 1e-6
        and abs(f0[2] - Fz_r / 2) < 1e-6
        and abs(f0[3] - Fz_r / 2) < 1e-6
    )
    return ok, {"Fz": f0, "Fz_f_static": Fz_f, "Fz_r_static": Fz_r}


def test_braking_loads_front() -> tuple[bool, dict]:
    """ax < 0 → front axle load increases."""
    p = LoadTransferParameters(h_cg=H)
    f_brake = longitudinal_axle_loads(-5.0, MASS, A, B, h_cg=H)
    f_static = longitudinal_axle_loads(0.0, MASS, A, B, h_cg=H)
    ok = f_brake[0] > f_static[0] and f_brake[1] < f_static[1]
    dFz = MASS * 5.0 * H / L
    ok = ok and abs((f_brake[0] - f_static[0]) - dFz) < 1e-6
    return ok, {"Fz_f_brake": f_brake[0], "Fz_f_static": f_static[0], "dFz": dFz}


def test_accel_loads_rear() -> tuple[bool, dict]:
    p = LoadTransferParameters(h_cg=H)
    f_acc = longitudinal_axle_loads(4.0, MASS, A, B, h_cg=H)
    f_static = longitudinal_axle_loads(0.0, MASS, A, B, h_cg=H)
    ok = f_acc[1] > f_static[1] and f_acc[0] < f_static[0]
    return ok, {"Fz_r_acc": f_acc[1], "Fz_r_static": f_static[1]}


def test_total_weight_conserved() -> tuple[bool, dict]:
    p = LoadTransferParameters(h_cg=H)
    W = MASS * G
    for ax in (-6.0, -2.0, 0.0, 2.0, 5.0):
        for ay in (-4.0, 0.0, 4.0):
            F = four_wheel_normal_loads(ay, MASS, A, B, p, ax=ax)
            total = sum(F)
            if abs(total - W) > 1e-3:
                return False, {"ax": ax, "ay": ay, "total": total, "W": W}
    return True, {"W": W}


def test_theoretical_longitudinal() -> tuple[bool, dict]:
    ax = -6.0
    Fz_f, Fz_r = longitudinal_axle_loads(ax, MASS, A, B, h_cg=H)
    W = MASS * G
    dFz = MASS * ax * H / L  # negative ax → negative dFz → front gains
    ok = abs(Fz_f - (W * B / L - dFz)) < 1e-9
    ok = ok and abs(Fz_r - (W * A / L + dFz)) < 1e-9
    return ok, {"Fz_f": Fz_f, "Fz_r": Fz_r, "dFz": dFz}


def test_combined_lateral_longitudinal() -> tuple[bool, dict]:
    """Braking + left turn: FL highest, RR lowest (typical)."""
    p = LoadTransferParameters(h_cg=H)
    # ax < 0 brake, ay > 0 left turn → load FL (front outside?)
    # ay > 0: load to right wheels (outer in left turn is right)
    # brake: load to front
    # so FR should be high, RL low
    F = four_wheel_normal_loads(ay=5.0, mass=MASS, a=A, b=B, lt_params=p, ax=-5.0)
    Fz_fl, Fz_fr, Fz_rl, Fz_rr = F
    ok = Fz_fr > Fz_fl and Fz_fr > Fz_rr  # front outer high
    ok = ok and (Fz_fl + Fz_fr) > (Fz_rl + Fz_rr)  # front > rear under braking
    return ok, {
        "FL": Fz_fl, "FR": Fz_fr, "RL": Fz_rl, "RR": Fz_rr,
        "front": Fz_fl + Fz_fr, "rear": Fz_rl + Fz_rr,
    }


def test_sign_symmetry_ax() -> tuple[bool, dict]:
    f_p = longitudinal_axle_loads(3.0, MASS, A, B, h_cg=H)
    f_n = longitudinal_axle_loads(-3.0, MASS, A, B, h_cg=H)
    f0 = longitudinal_axle_loads(0.0, MASS, A, B, h_cg=H)
    ok = abs((f_p[0] - f0[0]) + (f_n[0] - f0[0])) < 1e-6
    return ok, {"d_f_pos": f_p[0] - f0[0], "d_f_neg": f_n[0] - f0[0]}


def test_fz_positive() -> tuple[bool, dict]:
    p = LoadTransferParameters(h_cg=H, Fz_min=50.0)
    F = four_wheel_normal_loads(ay=8.0, mass=MASS, a=A, b=B, lt_params=p, ax=-8.0)
    ok = all(f >= p.Fz_min - 1e-6 for f in F)
    return ok, {"F": F}


def run_phase56_validation() -> bool:
    print("=== Phase 5.6 Longitudinal Load Transfer Validation ===\n")
    tests = [
        ("zero_ax_matches_phase50", test_zero_ax_matches_phase50),
        ("braking_loads_front", test_braking_loads_front),
        ("accel_loads_rear", test_accel_loads_rear),
        ("total_weight_conserved", test_total_weight_conserved),
        ("theoretical_longitudinal", test_theoretical_longitudinal),
        ("combined_lateral_longitudinal", test_combined_lateral_longitudinal),
        ("sign_symmetry_ax", test_sign_symmetry_ax),
        ("fz_positive", test_fz_positive),
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
    run_phase56_validation()
