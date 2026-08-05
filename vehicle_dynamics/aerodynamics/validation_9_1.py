"""Phase 9.1 – Closed-loop aero–vehicle coupling validation (target 10/10)."""

from __future__ import annotations

import numpy as np

from .coefficients import AeroConfig
from .vehicle_interface import VehicleAeroInput, ride_from_pitch_heave
from .coupling import couple_aero_to_tires, static_axle_loads
from .aero_solver import solve_aero
from .closed_loop import ClosedLoopAero, PitchHeaveParams
from .ride_height import RideHeightState


def test_zero_speed_closed_loop() -> tuple[bool, dict]:
    cl = ClosedLoopAero(AeroConfig())
    out = cl.step(VehicleAeroInput(speed=0.0))
    ok = out.loads.dFz_aero_f == 0 and out.loads.dFz_aero_r == 0
    ok = ok and out.aero.state.drag == 0
    return ok, {"dFz_f": out.loads.dFz_aero_f}


def test_ride_from_pitch() -> tuple[bool, dict]:
    cfg = AeroConfig()
    # Nose-up pitch → front rises, rear drops
    ride = ride_from_pitch_heave(pitch=0.02, heave=0.0, cfg=cfg)
    ok = ride.h_front > cfg.h_front_ref and ride.h_rear < cfg.h_rear_ref
    return ok, {"hf": ride.h_front, "hr": ride.h_rear}


def test_downforce_increases_with_speed() -> tuple[bool, dict]:
    cl = ClosedLoopAero(AeroConfig())
    o1 = cl.step(VehicleAeroInput(speed=20.0))
    o2 = cl.step(VehicleAeroInput(speed=40.0))
    ok = o2.loads.dFz_aero_f > o1.loads.dFz_aero_f * 3.5
    return ok, {
        "dFz_20": o1.loads.dFz_aero_f + o1.loads.dFz_aero_r,
        "dFz_40": o2.loads.dFz_aero_f + o2.loads.dFz_aero_r,
    }


def test_axle_load_increase() -> tuple[bool, dict]:
    cfg = AeroConfig()
    mass, a, b = 1400.0, 1.2, 1.5
    Fz_f0, Fz_r0 = static_axle_loads(mass, a, b)
    cl = ClosedLoopAero(cfg, mass=mass, a=a, b=b)
    out = cl.step(VehicleAeroInput(speed=50.0))
    ok = out.loads.Fz_f_axle > Fz_f0 and out.loads.Fz_r_axle > Fz_r0
    return ok, {
        "Fz_f0": Fz_f0,
        "Fz_f": out.loads.Fz_f_axle,
        "Fz_r0": Fz_r0,
        "Fz_r": out.loads.Fz_r_axle,
    }


def test_disabled_regression() -> tuple[bool, dict]:
    mass, a, b = 1400.0, 1.2, 1.5
    Fz_f0, Fz_r0 = static_axle_loads(mass, a, b)
    cl = ClosedLoopAero(AeroConfig(enabled=False), mass=mass, a=a, b=b)
    out = cl.step(VehicleAeroInput(speed=80.0, pitch=0.03))
    ok = abs(out.loads.Fz_f_axle - Fz_f0) < 1e-6
    ok = ok and abs(out.loads.Fz_r_axle - Fz_r0) < 1e-6
    ok = ok and out.loads.Fx_aero == 0.0
    return ok, {"Fz_f": out.loads.Fz_f_axle, "Fz_f0": Fz_f0}


def test_pitch_shifts_balance() -> tuple[bool, dict]:
    cfg = AeroConfig()
    cl = ClosedLoopAero(cfg)
    level = cl.step(VehicleAeroInput(speed=50.0, pitch=0.0))
    nose_up = cl.step(VehicleAeroInput(speed=50.0, pitch=0.03))
    # Nose-up → more rear / less front downforce fraction
    bal0 = level.aero.state.front_balance
    bal1 = nose_up.aero.state.front_balance
    ok = bal1 < bal0
    return ok, {"bal_level": bal0, "bal_nose_up": bal1}


def test_lateral_transfer_still_works() -> tuple[bool, dict]:
    cl = ClosedLoopAero(AeroConfig())
    out = cl.step(VehicleAeroInput(speed=40.0), ay=5.0)
    # Outside (right for +ay) should be higher
    ok = out.loads.Fz_fr > out.loads.Fz_fl
    ok = ok and out.loads.Fz_rr > out.loads.Fz_rl
    # Total conserved approximately
    total = out.loads.Fz_fl + out.loads.Fz_fr + out.loads.Fz_rl + out.loads.Fz_rr
    expected = out.loads.Fz_f_axle + out.loads.Fz_r_axle
    ok = ok and abs(total - expected) < 1.0
    return ok, {"Fz_fl": out.loads.Fz_fl, "Fz_fr": out.loads.Fz_fr, "total": total}


def test_pitch_dynamics_stable() -> tuple[bool, dict]:
    cfg = AeroConfig()
    cl = ClosedLoopAero(
        cfg,
        enable_pitch_dynamics=True,
        ph_params=PitchHeaveParams(),
    )
    # Step through time at constant speed
    pitches = []
    for i in range(200):
        out = cl.step(VehicleAeroInput(speed=40.0), dt=0.01)
        pitches.append(out.pitch_heave.pitch)
    pitches = np.array(pitches)
    ok = np.all(np.isfinite(pitches))
    ok = ok and np.max(np.abs(pitches)) < 0.1
    # Should settle (end variance small)
    ok = ok and np.std(pitches[-50:]) < 0.02
    return ok, {
        "pitch_end": float(pitches[-1]),
        "pitch_max": float(np.max(np.abs(pitches))),
        "std_end": float(np.std(pitches[-50:])),
    }


def test_drag_force_sign() -> tuple[bool, dict]:
    cl = ClosedLoopAero(AeroConfig())
    out = cl.step(VehicleAeroInput(speed=30.0))
    # Fx_aero opposes forward motion → negative
    ok = out.loads.Fx_aero < 0
    ok = ok and out.aero.drag_force > 0
    return ok, {"Fx_aero": out.loads.Fx_aero, "drag": out.aero.drag_force}


def test_no_nan_inf() -> tuple[bool, dict]:
    cl = ClosedLoopAero(AeroConfig(), enable_pitch_dynamics=True)
    ok = True
    for v in (0.0, 10.0, 55.0):
        for pitch in (-0.04, 0.0, 0.04):
            out = cl.step(
                VehicleAeroInput(speed=v, pitch=pitch, heave=0.01, yaw_angle=0.05),
                ay=3.0,
                dt=0.01,
            )
            vals = [
                out.loads.Fz_fl, out.loads.Fz_fr, out.loads.Fz_rl, out.loads.Fz_rr,
                out.loads.Fx_aero, out.aero.state.My, out.ride.h_front, out.ride.h_rear,
            ]
            if not all(np.isfinite(vals)):
                ok = False
    return ok, {"ok": ok}


def run_phase91_validation() -> bool:
    print("=== Phase 9.1 Closed-Loop Aero Validation ===\n")
    tests = [
        ("zero_speed_closed_loop", test_zero_speed_closed_loop),
        ("ride_from_pitch", test_ride_from_pitch),
        ("downforce_increases_with_speed", test_downforce_increases_with_speed),
        ("axle_load_increase", test_axle_load_increase),
        ("disabled_regression", test_disabled_regression),
        ("pitch_shifts_balance", test_pitch_shifts_balance),
        ("lateral_transfer_still_works", test_lateral_transfer_still_works),
        ("pitch_dynamics_stable", test_pitch_dynamics_stable),
        ("drag_force_sign", test_drag_force_sign),
        ("no_nan_inf", test_no_nan_inf),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:36} : {'PASS' if ok else 'FAIL'}")
        for k, v in list(diag.items())[:6]:
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\n=========================================")
    print("ALL TESTS PASSED" if all_pass else "SOME FAILED")
    if all_pass:
        print("Phase 9.1 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase91_validation()
