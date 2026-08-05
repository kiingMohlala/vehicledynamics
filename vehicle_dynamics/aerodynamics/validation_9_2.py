"""Phase 9.2 – Aerodynamic devices & active aero validation (target 12/12)."""

from __future__ import annotations

import numpy as np

from .coefficients import AeroConfig
from .ride_height import RideHeightState
from .aero_devices import AeroDeviceConfig
from .aero_device_solver import AeroDeviceSolver
from .closed_loop import ClosedLoopAero
from .vehicle_interface import VehicleAeroInput
from .active_aero import ActiveAeroMode
from .drs import DRSController, DRSParams


def test_baseline_regression() -> tuple[bool, dict]:
    """devices_enabled=False must match Phase 9.1 ClosedLoopAero."""
    cfg = AeroConfig()
    base = ClosedLoopAero(cfg, device_cfg=AeroDeviceConfig(devices_enabled=False))
    # Explicit no-device path
    from .aero_solver import solve_aero
    from .vehicle_interface import ride_from_vehicle_input

    inp = VehicleAeroInput(speed=45.0, pitch=0.01)
    out_cl = base.step(inp)
    ride = ride_from_vehicle_input(inp, cfg)
    ref = solve_aero(45.0, cfg=cfg, ride=ride)
    err_f = abs(out_cl.aero.dFz_front - ref.dFz_front)
    err_r = abs(out_cl.aero.dFz_rear - ref.dFz_rear)
    err_d = abs(out_cl.aero.drag_force - ref.drag_force)
    ok = err_f < 1e-6 and err_r < 1e-6 and err_d < 1e-6
    return ok, {"err_f": err_f, "err_r": err_r, "err_d": err_d}


def test_front_wing_downforce() -> tuple[bool, dict]:
    cfg = AeroConfig()
    dcfg = AeroDeviceConfig(
        devices_enabled=True,
        use_rear_wing=False,
        use_diffuser=False,
        use_splitter=False,
        use_active_aero=False,
        use_drs=False,
    )
    sol = AeroDeviceSolver(cfg, dcfg)
    ride = RideHeightState()
    off = AeroDeviceSolver(cfg, AeroDeviceConfig(devices_enabled=False)).solve(50.0, ride)
    on = sol.solve(50.0, ride)
    # Front downforce should increase with front wing
    ok = on.state.downforce_front > off.state.downforce_front
    return ok, {
        "DF_f_off": off.state.downforce_front,
        "DF_f_on": on.state.downforce_front,
    }


def test_rear_wing_downforce() -> tuple[bool, dict]:
    cfg = AeroConfig()
    dcfg = AeroDeviceConfig(
        devices_enabled=True,
        use_front_wing=False,
        use_diffuser=False,
        use_splitter=False,
        use_active_aero=False,
        use_drs=False,
    )
    sol = AeroDeviceSolver(cfg, dcfg)
    ride = RideHeightState()
    off = AeroDeviceSolver(cfg, AeroDeviceConfig(devices_enabled=False)).solve(50.0, ride)
    on = sol.solve(50.0, ride)
    ok = on.state.downforce_rear > off.state.downforce_rear
    return ok, {
        "DF_r_off": off.state.downforce_rear,
        "DF_r_on": on.state.downforce_rear,
    }


def test_drs_reduces_drag() -> tuple[bool, dict]:
    cfg = AeroConfig()
    dcfg = AeroDeviceConfig(
        devices_enabled=True,
        use_front_wing=False,
        use_diffuser=False,
        use_splitter=False,
        use_active_aero=False,
        use_drs=True,
    )
    sol = AeroDeviceSolver(cfg, dcfg)
    ride = RideHeightState()
    # Closed
    sol.drs.command(False)
    sol.drs.position = 0.0
    closed = sol.solve(60.0, ride, dt=0.0)
    # Open
    sol.drs.command(True)
    sol.drs.position = 1.0
    opened = sol.solve(60.0, ride, dt=0.0)
    ok = opened.state.drag < closed.state.drag
    return ok, {"drag_closed": closed.state.drag, "drag_open": opened.state.drag}


def test_drs_reduces_downforce() -> tuple[bool, dict]:
    cfg = AeroConfig()
    dcfg = AeroDeviceConfig(
        devices_enabled=True,
        use_front_wing=False,
        use_diffuser=False,
        use_splitter=False,
        use_active_aero=False,
        use_drs=True,
    )
    sol = AeroDeviceSolver(cfg, dcfg)
    ride = RideHeightState()
    sol.drs.position = 0.0
    closed = sol.solve(60.0, ride)
    sol.drs.position = 1.0
    opened = sol.solve(60.0, ride)
    ok = opened.state.downforce_rear < closed.state.downforce_rear
    return ok, {
        "DFr_closed": closed.state.downforce_rear,
        "DFr_open": opened.state.downforce_rear,
    }


def test_diffuser_ground_effect() -> tuple[bool, dict]:
    cfg = AeroConfig()
    dcfg = AeroDeviceConfig(
        devices_enabled=True,
        use_front_wing=False,
        use_rear_wing=False,
        use_splitter=False,
        use_active_aero=False,
        use_drs=False,
    )
    sol = AeroDeviceSolver(cfg, dcfg)
    high = sol.solve(50.0, RideHeightState(h_front=0.10, h_rear=0.14))
    low = sol.solve(50.0, RideHeightState(h_front=0.08, h_rear=0.06))
    ok = low.state.downforce_rear > high.state.downforce_rear
    return ok, {
        "DFr_high": high.state.downforce_rear,
        "DFr_low": low.state.downforce_rear,
    }


def test_diffuser_stall() -> tuple[bool, dict]:
    cfg = AeroConfig()
    dcfg = AeroDeviceConfig(
        devices_enabled=True,
        use_front_wing=False,
        use_rear_wing=False,
        use_splitter=False,
        use_active_aero=False,
    )
    sol = AeroDeviceSolver(cfg, dcfg)
    normal = sol.solve(50.0, RideHeightState(h_rear=0.06))
    stalled = sol.solve(50.0, RideHeightState(h_rear=0.015))
    ok = stalled.breakdown.diffuser_stalled and not normal.breakdown.diffuser_stalled
    ok = ok and stalled.breakdown.diffuser_Fz > normal.breakdown.diffuser_Fz  # less negative / weaker
    # actually stalled should have less |downforce|
    ok = abs(stalled.breakdown.diffuser_Fz) < abs(normal.breakdown.diffuser_Fz)
    return ok, {
        "stalled_flag": stalled.breakdown.diffuser_stalled,
        "Fz_normal": normal.breakdown.diffuser_Fz,
        "Fz_stall": stalled.breakdown.diffuser_Fz,
    }


def test_active_aero_switching() -> tuple[bool, dict]:
    cfg = AeroConfig()
    dcfg = AeroDeviceConfig(devices_enabled=True, use_active_aero=True)
    sol = AeroDeviceSolver(cfg, dcfg)
    ride = RideHeightState()
    corner = sol.solve(40.0, ride, ay=6.0, brake=0.0)
    brake = sol.solve(40.0, ride, ay=0.0, brake=0.5)
    top = sol.solve(70.0, ride, ay=0.0, brake=0.0)
    ok = corner.breakdown.active_mode == "cornering"
    ok = ok and brake.breakdown.active_mode == "braking"
    ok = ok and top.breakdown.active_mode == "top_speed"
    ok = ok and top.breakdown.drs_position >= 0.0
    # top speed should request DRS open
    if top.command:
        ok = ok and top.command.drs_open
    return ok, {
        "corner": corner.breakdown.active_mode,
        "brake": brake.breakdown.active_mode,
        "top": top.breakdown.active_mode,
    }


def test_center_of_pressure_shift() -> tuple[bool, dict]:
    cfg = AeroConfig()
    # Front-heavy devices only
    front = AeroDeviceSolver(
        cfg,
        AeroDeviceConfig(
            devices_enabled=True,
            use_rear_wing=False,
            use_diffuser=False,
            use_active_aero=False,
        ),
    ).solve(50.0, RideHeightState())
    rear = AeroDeviceSolver(
        cfg,
        AeroDeviceConfig(
            devices_enabled=True,
            use_front_wing=False,
            use_splitter=False,
            use_active_aero=False,
        ),
    ).solve(50.0, RideHeightState())
    ok = rear.state.center_of_pressure_x > front.state.center_of_pressure_x
    return ok, {
        "x_cp_front_pkg": front.state.center_of_pressure_x,
        "x_cp_rear_pkg": rear.state.center_of_pressure_x,
    }


def test_vehicle_coupling() -> tuple[bool, dict]:
    cfg = AeroConfig()
    dcfg = AeroDeviceConfig(devices_enabled=True)
    cl = ClosedLoopAero(cfg, device_cfg=dcfg, mass=1400, a=1.2, b=1.5)
    out = cl.step(VehicleAeroInput(speed=50.0), ay=2.0)
    ok = out.loads.dFz_aero_f > 0 and out.loads.dFz_aero_r > 0
    ok = ok and out.loads.Fx_aero < 0
    ok = ok and out.breakdown is not None
    return ok, {
        "dFz_f": out.loads.dFz_aero_f,
        "dFz_r": out.loads.dFz_aero_r,
        "mode": out.breakdown.active_mode if out.breakdown else None,
    }


def test_no_nan_inf() -> tuple[bool, dict]:
    cfg = AeroConfig()
    dcfg = AeroDeviceConfig(devices_enabled=True)
    cl = ClosedLoopAero(cfg, device_cfg=dcfg, enable_pitch_dynamics=True)
    ok = True
    for v in (0.0, 25.0, 70.0):
        for ay, br in ((0.0, 0.0), (5.0, 0.0), (0.0, 0.6)):
            out = cl.step(
                VehicleAeroInput(speed=v, pitch=0.02),
                ay=ay,
                brake=br,
                dt=0.01,
            )
            vals = [
                out.aero.state.Fx, out.aero.state.Fz_front, out.aero.state.Fz_rear,
                out.loads.Fz_fl, out.loads.Fx_aero,
            ]
            if not all(np.isfinite(vals)):
                ok = False
    return ok, {"ok": ok}


def test_performance_regression() -> tuple[bool, dict]:
    """Devices should improve L/D or total DF vs body-only at race speed."""
    cfg = AeroConfig()
    ride = RideHeightState()
    body = AeroDeviceSolver(cfg, AeroDeviceConfig(devices_enabled=False)).solve(55.0, ride)
    full = AeroDeviceSolver(cfg, AeroDeviceConfig(devices_enabled=True)).solve(55.0, ride)
    ok = full.state.downforce_total > body.state.downforce_total
    return ok, {
        "DF_body": body.state.downforce_total,
        "DF_devices": full.state.downforce_total,
        "LD_devices": full.state.L_over_D,
    }


def run_phase92_validation() -> bool:
    print("=== Phase 9.2 Aerodynamic Devices & Active Aero Validation ===\n")
    tests = [
        ("baseline_regression", test_baseline_regression),
        ("front_wing_downforce", test_front_wing_downforce),
        ("rear_wing_downforce", test_rear_wing_downforce),
        ("drs_reduces_drag", test_drs_reduces_drag),
        ("drs_reduces_downforce", test_drs_reduces_downforce),
        ("diffuser_ground_effect", test_diffuser_ground_effect),
        ("diffuser_stall", test_diffuser_stall),
        ("active_aero_switching", test_active_aero_switching),
        ("center_of_pressure_shift", test_center_of_pressure_shift),
        ("vehicle_coupling", test_vehicle_coupling),
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
        print("Phase 9.2 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase92_validation()
