"""
Phase 6.2 – Geometry coupling validation.
"""

from __future__ import annotations

import numpy as np
from .coupling import (
    CoupledSuspension,
    VehicleSuspensionConfig,
    CornerConfig,
    apply_toe_to_delta,
    camber_lateral_force,
)
from .hardpoints import default_front_left, mirror_corner
from .wheel_rate import SpringDamperParams, MotionRatioParams


def test_static_evaluate() -> tuple[bool, dict]:
    susp = CoupledSuspension()
    states = susp.evaluate_all()
    ok = set(states.keys()) == {"FL", "FR", "RL", "RR"}
    ok = ok and all(np.isfinite(s.Kw) and np.isfinite(s.camber_rad) for s in states.values())
    return ok, {k: (v.camber_deg, v.Kw) for k, v in states.items()}


def test_left_right_symmetry() -> tuple[bool, dict]:
    susp = CoupledSuspension()
    s = susp.evaluate_all()
    ok = (
        abs(s["FL"].Kw - s["FR"].Kw) < 1e-9
        and abs(s["FL"].Cw - s["FR"].Cw) < 1e-9
        and abs(s["FL"].camber_rad + s["FR"].camber_rad) < 1e-6
        and abs(s["FL"].scrub_radius + s["FR"].scrub_radius) < 1e-6
    )
    return ok, {
        "camber_FL": s["FL"].camber_deg,
        "camber_FR": s["FR"].camber_deg,
        "Kw_FL": s["FL"].Kw,
        "Kw_FR": s["FR"].Kw,
    }


def test_asymmetric_geometry() -> tuple[bool, dict]:
    """Different IR left vs right → different Kw."""
    hp = default_front_left()
    cfg = VehicleSuspensionConfig(
        fl=CornerConfig(hp, SpringDamperParams(30000, 2000),
                        MotionRatioParams(0.7, "pushrod"), "FL"),
        fr=CornerConfig(mirror_corner(hp), SpringDamperParams(30000, 2000),
                        MotionRatioParams(1.0, "direct"), "FR"),
    )
    s = CoupledSuspension(cfg).evaluate_all()
    ok = s["FL"].Kw < s["FR"].Kw and abs(s["FL"].Kw - 30000 * 0.49) < 1e-6
    return ok, {"Kw_FL": s["FL"].Kw, "Kw_FR": s["FR"].Kw}


def test_ride_frequency_scales_with_mr() -> tuple[bool, dict]:
    """Higher IR → higher Kw → higher ride frequency."""
    hp = default_front_left()
    m_corner = 350.0  # kg

    def freq(ir):
        cfg = VehicleSuspensionConfig(
            fl=CornerConfig(hp, SpringDamperParams(Ks=30000, Cs=2000),
                            MotionRatioParams(ir, "custom"), "FL"),
        )
        return CoupledSuspension(cfg).ride_frequency_hz(m_corner, "FL")

    f1, f2 = freq(0.7), freq(1.0)
    # f ∝ sqrt(Kw) ∝ IR
    ratio = f2 / f1
    expected = 1.0 / 0.7
    ok = f2 > f1 and abs(ratio - expected) < 0.05
    return ok, {"f_IR0.7": f1, "f_IR1.0": f2, "ratio": ratio, "expected": expected}


def test_vertical_force_equilibrium() -> tuple[bool, dict]:
    """At design (compression=0) spring force is 0; damper 0 at rest."""
    susp = CoupledSuspension()
    F = susp.vertical_forces(np.zeros(4), np.zeros(4))
    ok = bool(np.allclose(F, 0.0))
    return ok, {"F": F.tolist()}


def test_vertical_force_bump() -> tuple[bool, dict]:
    """Positive compression → positive restoring force (Kw * z)."""
    susp = CoupledSuspension()
    z = np.array([0.02, 0.02, 0.02, 0.02])
    F = susp.vertical_forces(z, np.zeros(4))
    Kw, _ = susp.wheel_rates()
    ok = bool(np.allclose(F, Kw * z)) and np.all(F > 0)
    return ok, {"F": F.tolist(), "Kw*z": (Kw * z).tolist()}


def test_toe_adds_to_steer() -> tuple[bool, dict]:
    delta = 0.05
    toe = -0.01
    ok = abs(apply_toe_to_delta(delta, toe) - 0.04) < 1e-12
    return ok, {"delta_eff": apply_toe_to_delta(delta, toe)}


def test_camber_force_sign() -> tuple[bool, dict]:
    Fy_pos = camber_lateral_force(0.05, 4000.0)
    Fy_neg = camber_lateral_force(-0.05, 4000.0)
    ok = Fy_pos > 0 and Fy_neg < 0 and abs(Fy_pos + Fy_neg) < 1e-9
    return ok, {"Fy_pos": Fy_pos, "Fy_neg": Fy_neg}


def test_camber_toe_arrays_shape() -> tuple[bool, dict]:
    camber, toe = CoupledSuspension().camber_toe_arrays()
    ok = camber.shape == (4,) and toe.shape == (4,) and np.all(np.isfinite(camber))
    return ok, {"camber_deg": np.degrees(camber).tolist(), "toe_deg": np.degrees(toe).tolist()}


def run_phase62_validation() -> bool:
    print("=== Phase 6.2 Geometry Coupling Validation ===\n")
    tests = [
        ("static_evaluate", test_static_evaluate),
        ("left_right_symmetry", test_left_right_symmetry),
        ("asymmetric_geometry", test_asymmetric_geometry),
        ("ride_frequency_scales_with_mr", test_ride_frequency_scales_with_mr),
        ("vertical_force_equilibrium", test_vertical_force_equilibrium),
        ("vertical_force_bump", test_vertical_force_bump),
        ("toe_adds_to_steer", test_toe_adds_to_steer),
        ("camber_force_sign", test_camber_force_sign),
        ("camber_toe_arrays_shape", test_camber_toe_arrays_shape),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:32} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_phase62_validation()
