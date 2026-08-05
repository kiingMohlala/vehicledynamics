"""Phase 9.4 – Unsteady aerodynamics validation (target 12/12)."""

from __future__ import annotations

import numpy as np

from vehicle_dynamics.aerodynamics.coefficients import AeroConfig
from vehicle_dynamics.aerodynamics.ride_height import RideHeightState
from vehicle_dynamics.aerodynamics.aero_solver import solve_aero

from .gust_model import StepGust, RampGust, DrydenGust
from .crosswind import compute_crosswind_loads
from .drafting import drafting_factors, DraftingParams
from .wake_model import WakeSource, WakeField, evaluate_wake
from .aero_transients import AeroTransientFilter
from .wake_database import WakeDatabase
from .unsteady_solver import UnsteadyAeroConfig, UnsteadyAeroSolver
from .dynamic_pressure import relative_velocity, air_speed_and_sideslip


def test_zero_wind_regression() -> tuple[bool, dict]:
    cfg = AeroConfig()
    sol = UnsteadyAeroSolver(cfg, UnsteadyAeroConfig(enabled=False))
    out = sol.step(40.0, dt=0.01)
    ref = solve_aero(40.0, cfg=cfg)
    ok = abs(out.aero.state.Fx - ref.state.Fx) < 1e-9
    ok = ok and abs(out.aero.state.Fz_front - ref.state.Fz_front) < 1e-9
    return ok, {"dFx": abs(out.aero.state.Fx - ref.state.Fx)}


def test_constant_crosswind() -> tuple[bool, dict]:
    cfg = AeroConfig()
    gust = StepGust(magnitude=15.0, direction_xy=np.pi / 2, t_onset=0.0)
    sol = UnsteadyAeroSolver(cfg, UnsteadyAeroConfig(use_transients=False), gust=gust)
    out = sol.step(40.0, dt=0.01)
    ok = abs(out.beta_aero) > 0.05
    ok = ok and abs(out.aero.state.Fy) > 10.0
    ok = ok and abs(out.aero.state.Mz) > 1.0
    return ok, {"beta": out.beta_aero, "Fy": out.aero.state.Fy, "Mz": out.aero.state.Mz}


def test_gust_step_response() -> tuple[bool, dict]:
    cfg = AeroConfig()
    gust = StepGust(magnitude=12.0, direction_xy=np.pi / 2, t_onset=0.5)
    sol = UnsteadyAeroSolver(cfg, UnsteadyAeroConfig(use_transients=False), gust=gust)
    # Before onset
    sol._t = 0.0
    pre = sol.step(40.0, dt=0.01)
    # Jump time past onset
    sol._t = 0.49
    post = sol.step(40.0, dt=0.02)
    ok = abs(pre.aero.state.Fy) < abs(post.aero.state.Fy)
    return ok, {"Fy_pre": pre.aero.state.Fy, "Fy_post": post.aero.state.Fy}


def test_gust_ramp_response() -> tuple[bool, dict]:
    cfg = AeroConfig()
    gust = RampGust(magnitude=10.0, direction_xy=np.pi / 2, t_onset=0.0, ramp_time=1.0)
    sol = UnsteadyAeroSolver(cfg, UnsteadyAeroConfig(use_transients=False), gust=gust)
    sol._t = -0.01
    a = sol.step(40.0, dt=0.1)   # t→0.09
    b = sol.step(40.0, dt=0.5)   # mid ramp
    c = sol.step(40.0, dt=1.0)   # after ramp
    ok = abs(a.wind[1]) < abs(b.wind[1]) <= abs(c.wind[1]) + 1e-9
    return ok, {"Wy_a": a.wind[1], "Wy_b": b.wind[1], "Wy_c": c.wind[1]}


def test_drafting_drag_reduction() -> tuple[bool, dict]:
    close = drafting_factors(3.0)
    far = drafting_factors(80.0)
    ok = close["Cd_factor"] < far["Cd_factor"]
    ok = ok and close["Cd_factor"] < 1.0
    return ok, {"Cd_close": close["Cd_factor"], "Cd_far": far["Cd_factor"]}


def test_drafting_downforce_loss() -> tuple[bool, dict]:
    close = drafting_factors(3.0)
    far = drafting_factors(80.0)
    ok = close["Cl_factor"] < far["Cl_factor"]
    return ok, {"Cl_close": close["Cl_factor"], "Cl_far": far["Cl_factor"]}


def test_wake_decay_distance() -> tuple[bool, dict]:
    p = DraftingParams(wake_length=20.0)
    s3 = drafting_factors(3.0, p)["wake_strength"]
    s40 = drafting_factors(40.0, p)["wake_strength"]
    ok = s3 > s40
    return ok, {"str_3m": s3, "str_40m": s40}


def test_transient_lift_delay() -> tuple[bool, dict]:
    filt = AeroTransientFilter(tau_force=0.2)
    filt.reset()
    # Step target Fz
    vals = []
    for _ in range(20):
        _, _, fz, _, _, _ = filt.step(0, 0, -1000.0, -1000.0, 0, 0, 0.05)
        vals.append(fz)
    # First step should not reach full value; later closer
    ok = abs(vals[0]) < abs(vals[-1])
    ok = ok and abs(vals[-1]) > 0.9 * 1000
    return ok, {"fz0": vals[0], "fz_end": vals[-1]}


def test_cornering_pitch_coupling() -> tuple[bool, dict]:
    """Crosswind yaw moment present; ride pitch still affects DF balance."""
    cfg = AeroConfig()
    gust = StepGust(magnitude=8.0, direction_xy=np.pi / 2, t_onset=0.0)
    sol = UnsteadyAeroSolver(cfg, UnsteadyAeroConfig(use_transients=False), gust=gust)
    level = sol.step(40.0, RideHeightState(pitch_rad=0.0), dt=0.01)
    pitched = sol.step(40.0, RideHeightState(pitch_rad=0.03), dt=0.01)
    ok = abs(level.aero.state.Mz) > 0
    ok = ok and pitched.aero.state.front_balance != level.aero.state.front_balance
    return ok, {
        "Mz": level.aero.state.Mz,
        "bal0": level.aero.state.front_balance,
        "bal1": pitched.aero.state.front_balance,
    }


def test_multiple_vehicle_wake() -> tuple[bool, dict]:
    field = WakeField(sources=[
        WakeSource(x=-10.0, y=0.0, strength=1.0),
        WakeSource(x=-5.0, y=3.0, strength=0.5),  # offset laterally
    ])
    # Ego at origin — first source should dominate
    w = evaluate_wake(field, ego_x=0.0, ego_y=0.0)
    ok = w["wake_strength"] > 0 and w["Cd_factor"] < 1.0
    # Far lateral — weak/no wake
    w2 = evaluate_wake(field, ego_x=0.0, ego_y=10.0)
    ok = ok and w2["wake_strength"] < w["wake_strength"]
    return ok, {"str_center": w["wake_strength"], "str_side": w2["wake_strength"]}


def test_no_nan_inf() -> tuple[bool, dict]:
    cfg = AeroConfig()
    gust = DrydenGust(sigma=2.0, seed=1)
    db = WakeDatabase()
    db.add_vehicle(-8.0, 0.0)
    sol = UnsteadyAeroSolver(cfg, gust=gust, wake_db=db)
    ok = True
    for i in range(50):
        out = sol.step(35.0 + 0.1 * i, dt=0.02, ego_x=0.0)
        vals = [
            out.aero.state.Fx, out.aero.state.Fy, out.aero.state.Fz_front,
            out.airspeed, out.beta_aero, out.wake_strength,
        ]
        if not all(np.isfinite(vals)):
            ok = False
    return ok, {"ok": ok}


def test_performance_regression() -> tuple[bool, dict]:
    """With wake, drag should drop vs clear air."""
    cfg = AeroConfig()
    clear = UnsteadyAeroSolver(cfg, UnsteadyAeroConfig(use_transients=False, use_gust=False))
    out0 = clear.step(50.0, dt=0.01)

    db = WakeDatabase()
    db.add_vehicle(-5.0, 0.0)
    draft = UnsteadyAeroSolver(
        cfg, UnsteadyAeroConfig(use_transients=False, use_gust=False), wake_db=db
    )
    out1 = draft.step(50.0, dt=0.01, ego_x=0.0)
    ok = out1.aero.state.drag < out0.aero.state.drag
    ok = ok and out1.aero.state.downforce_total < out0.aero.state.downforce_total
    return ok, {
        "drag_clear": out0.aero.state.drag,
        "drag_draft": out1.aero.state.drag,
        "DF_clear": out0.aero.state.downforce_total,
        "DF_draft": out1.aero.state.downforce_total,
    }


def run_phase94_validation() -> bool:
    print("=== Phase 9.4 Unsteady Aerodynamics & Wake Validation ===\n")
    tests = [
        ("zero_wind_regression", test_zero_wind_regression),
        ("constant_crosswind", test_constant_crosswind),
        ("gust_step_response", test_gust_step_response),
        ("gust_ramp_response", test_gust_ramp_response),
        ("drafting_drag_reduction", test_drafting_drag_reduction),
        ("drafting_downforce_loss", test_drafting_downforce_loss),
        ("wake_decay_distance", test_wake_decay_distance),
        ("transient_lift_delay", test_transient_lift_delay),
        ("cornering_pitch_coupling", test_cornering_pitch_coupling),
        ("multiple_vehicle_wake", test_multiple_vehicle_wake),
        ("no_nan_inf", test_no_nan_inf),
        ("performance_regression", test_performance_regression),
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
        print("Phase 9.4 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase94_validation()
