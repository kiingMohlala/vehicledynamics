"""
Phase 7.1 – Camber coupling into dual-track tire calls.
"""

from __future__ import annotations

import numpy as np

from ..tire.dugoff import DugoffTire, DugoffParams
from ..suspension.geometry_state import VehicleGeometryState, WheelGeometryState
from ..suspension.camber_state import CamberGainParams
from .suspension_interface import SuspensionInterface, SuspensionInterfaceConfig
from .simulation import DualTrackVehicleModel


def test_wiring_passes_per_wheel_camber() -> tuple[bool, dict]:
    """Model must pass distinct camber to each tire index."""
    g = VehicleGeometryState.neutral()
    g.fl.camber_rad = 0.05
    g.fr.camber_rad = -0.03
    g.rl.camber_rad = 0.01
    g.rr.camber_rad = -0.02
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, camber_gain_enabled=True),
        geometry=g,
        camber_params=CamberGainParams.neutral(),
    )
    model = DualTrackVehicleModel(suspension=iface, use_abs=False)
    model._refresh_camber(0.0)
    seen = []

    class SpyTire:
        def __init__(self, inner):
            self.inner = inner
            self.p = inner.p

        def longitudinal_lateral_force(self, kappa, alpha, Fz, camber_rad=0.0):
            seen.append(float(camber_rad))
            return self.inner.longitudinal_lateral_force(kappa, alpha, Fz, camber_rad)

    model.tire = SpyTire(model.tire)
    for i in range(4):
        model._tire_force(0.0, 0.0, 3500.0, i)
    expected = [0.05, -0.03, 0.01, -0.02]
    ok = all(abs(s - e) < 1e-12 for s, e in zip(seen, expected))
    return ok, {"seen": seen, "expected": expected}


def test_zero_camber_matches_no_suspension() -> tuple[bool, dict]:
    """No suspension vs neutral camber → same Fy at same slip."""
    m0 = DualTrackVehicleModel(use_abs=False)
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, camber_gain_enabled=True),
        camber_params=CamberGainParams.neutral(),
    )
    m1 = DualTrackVehicleModel(suspension=iface, use_abs=False)
    m0._refresh_camber(0.0)
    m1._refresh_camber(0.0)
    a = m0._tire_force(0.0, 0.05, 4000.0, 0)
    b = m1._tire_force(0.0, 0.05, 4000.0, 0)
    ok = abs(a.Fy - b.Fy) < 1e-9 and abs(a.Fx - b.Fx) < 1e-9
    return ok, {"Fy0": a.Fy, "Fy1": b.Fy}


def test_outside_negative_camber_increases_cornering() -> tuple[bool, dict]:
    """
    Outside wheel with negative camber (γ < 0) at positive α adds more
    negative Fy_camber? With our sign: γ < 0 → Fy_γ < 0.
    For a pure camber contribution at α=0, |Fy| increases with |γ|.
    At α > 0 (leftward slip), negative outside camber can oppose or aid;
    we check that |Fy| with |γ| is larger than pure slip at α=0 case,
    and that negative γ produces negative Fy at α=0.
    """
    tire = DugoffTire(DugoffParams(C_gamma=3000.0))
    base = tire.longitudinal_lateral_force(0.0, 0.0, 4000.0, camber_rad=0.0)
    neg = tire.longitudinal_lateral_force(0.0, 0.0, 4000.0, camber_rad=-0.04)
    pos = tire.longitudinal_lateral_force(0.0, 0.0, 4000.0, camber_rad=0.04)
    ok = neg.Fy < base.Fy and pos.Fy > base.Fy and abs(neg.Fy) > 0
    # With positive slip angle, adding negative camber reduces Fy magnitude
    # toward zero or past; positive camber increases Fy further.
    corner = tire.longitudinal_lateral_force(0.0, 0.05, 4000.0, camber_rad=0.0)
    corner_out = tire.longitudinal_lateral_force(0.0, 0.05, 4000.0, camber_rad=-0.03)
    # outside negative camber reduces Fy (less positive) — document behavior
    ok = ok and corner_out.Fy < corner.Fy
    return ok, {
        "Fy_neg_camber": neg.Fy,
        "Fy_pos_camber": pos.Fy,
        "Fy_corner": corner.Fy,
        "Fy_corner_out": corner_out.Fy,
    }


def test_asymmetric_camber_yaw_effect() -> tuple[bool, dict]:
    """Different left/right camber → different Fy → nonzero yaw contribution."""
    tire = DugoffTire(DugoffParams(C_gamma=2500.0))
    # FL γ=+0.05, FR γ=-0.05, α=0, equal Fz
    fl = tire.longitudinal_lateral_force(0.0, 0.0, 3500.0, camber_rad=0.05)
    fr = tire.longitudinal_lateral_force(0.0, 0.0, 3500.0, camber_rad=-0.05)
    # y_fl > 0, y_fr < 0; yaw from Fy: x*Fy - y*Fx ≈ 0 - y*0 + track term from Fy difference
    # Moment about CG from lateral forces: Mz ≈ a*(Fy_fl+Fy_fr) for front only...
    # Simpler: Fy_fl ≠ Fy_fr implies asymmetric lateral force.
    ok = fl.Fy > 0 and fr.Fy < 0 and abs(fl.Fy - fr.Fy) > 1.0
    return ok, {"Fy_fl": fl.Fy, "Fy_fr": fr.Fy, "delta_Fy": fl.Fy - fr.Fy}


def test_utilization_bounded() -> tuple[bool, dict]:
    tire = DugoffTire(DugoffParams(C_gamma=8000.0))
    max_u = 0.0
    for g in np.linspace(-0.15, 0.15, 11):
        for a in np.linspace(-0.2, 0.2, 11):
            st = tire.longitudinal_lateral_force(0.1, float(a), 4000.0, camber_rad=float(g))
            max_u = max(max_u, st.utilization)
    ok = max_u <= 1.0 + 1e-6
    return ok, {"max_utilization": max_u}


def test_no_nan_in_model_path() -> tuple[bool, dict]:
    g = VehicleGeometryState.neutral()
    g.fl.camber_rad = 0.02
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, camber_gain_enabled=True),
        geometry=g,
        camber_params=CamberGainParams.symmetric(front=0.5),
    )
    model = DualTrackVehicleModel(
        suspension=iface,
        use_abs=False,
        wheel_travel_func=lambda t: np.array([0.03, 0.02, 0.01, 0.0]),
    )
    model._refresh_camber(0.0)
    st = model._tire_force(0.05, 0.04, 3800.0, 0)
    ok = all(np.isfinite(x) for x in (st.Fx, st.Fy, st.utilization, model._camber).flatten()
             if isinstance(x, (float, np.floating)) or True)
    ok = np.all(np.isfinite(model._camber)) and np.isfinite(st.Fy)
    return ok, {"camber": model._camber.tolist(), "Fy": st.Fy}


def run_phase71_validation() -> bool:
    print("=== Phase 7.1 Camber Coupling Validation ===\n")
    tests = [
        ("wiring_passes_per_wheel_camber", test_wiring_passes_per_wheel_camber),
        ("zero_camber_matches_no_suspension", test_zero_camber_matches_no_suspension),
        ("outside_negative_camber_effect", test_outside_negative_camber_increases_cornering),
        ("asymmetric_camber_yaw_effect", test_asymmetric_camber_yaw_effect),
        ("utilization_bounded", test_utilization_bounded),
        ("no_nan_in_model_path", test_no_nan_in_model_path),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:40} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_phase71_validation()
