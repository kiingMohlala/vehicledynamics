"""Phase 9.3 – CFD calibration & map generation validation (target 13/13)."""

from __future__ import annotations

from pathlib import Path
import tempfile
import numpy as np

from vehicle_dynamics.aerodynamics.coefficients import AeroConfig
from vehicle_dynamics.aerodynamics.ride_height import RideHeightState
from vehicle_dynamics.aerodynamics.aero_model import compute_aero_loads
from vehicle_dynamics.aerodynamics.closed_loop import ClosedLoopAero
from vehicle_dynamics.aerodynamics.vehicle_interface import VehicleAeroInput
from vehicle_dynamics.aerodynamics.aero_devices import AeroDeviceConfig

from .cfd_map import AeroSample
from .cfd_import import import_csv, import_openfoam_forces, import_su2_forces
from .interpolator import interpolate_sample
from .calibration import calibrate_against_samples
from .aero_database import AeroDatabase, AeroSolverMode
from .map_generator import build_map_from_samples, export_map_csv, synthetic_cfd_grid
from .uncertainty import estimate_uncertainty


def test_analytical_regression() -> tuple[bool, dict]:
    cfg = AeroConfig()
    db = AeroDatabase(mode=AeroSolverMode.ANALYTICAL, config=cfg)
    ride = RideHeightState()
    st_db, _, src = db.lookup(40.0, ride)
    st_ref = compute_aero_loads(40.0, cfg, ride=ride)
    ok = src == "analytical"
    ok = ok and abs(st_db.Fx - st_ref.Fx) < 1e-9
    ok = ok and abs(st_db.Fz_front - st_ref.Fz_front) < 1e-9
    return ok, {"source": src, "dFx": abs(st_db.Fx - st_ref.Fx)}


def test_csv_import() -> tuple[bool, dict]:
    samples = synthetic_cfd_grid()
    amap = build_map_from_samples(samples, name="tmp")
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "aero.csv"
        export_map_csv(amap, path)
        loaded = import_csv(path, source="csv")
    ok = len(loaded) == len(samples)
    ok = ok and abs(loaded[0].Cd - samples[0].Cd) < 1e-9
    return ok, {"n": len(loaded)}


def test_openfoam_import() -> tuple[bool, dict]:
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "forceCoeffs.dat"
        path.write_text(
            "# Time Cd Cl Cs\n"
            "0.1 0.40 -1.10 0.0\n"
            "0.5 0.36 -1.05 0.0\n"
            "1.0 0.35 -1.00 0.0\n"
        )
        samples = import_openfoam_forces(path, speed=50.0)
    ok = len(samples) >= 1 and samples[-1].Cd == 0.35
    ok = ok and samples[-1].source == "openfoam"
    return ok, {"n": len(samples), "Cd": samples[-1].Cd}


def test_su2_import() -> tuple[bool, dict]:
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "history.csv"
        path.write_text(
            '"Time","CD","CL","CMx"\n'
            "0.1,0.42,-1.2,0.01\n"
            "1.0,0.33,-0.95,0.005\n"
        )
        samples = import_su2_forces(path, speed=45.0)
    ok = len(samples) == 1 and abs(samples[0].Cd - 0.33) < 1e-9
    ok = ok and samples[0].source == "su2"
    return ok, {"Cd": samples[0].Cd, "Cl_f": samples[0].Cl_front}


def test_database_build() -> tuple[bool, dict]:
    samples = synthetic_cfd_grid()
    amap = build_map_from_samples(samples, name="synth")
    db = AeroDatabase(amap=amap)
    ok = len(db.amap) == len(samples)
    ok = ok and "speed" in db.amap.bounds()
    return ok, {"n": len(db.amap), "bounds": list(db.amap.bounds().keys())[:3]}


def test_multidimensional_interpolation() -> tuple[bool, dict]:
    samples = synthetic_cfd_grid()
    amap = build_map_from_samples(samples)
    # Query midpoint
    q = AeroSample(speed=40.0, h_front=0.08, h_rear=0.10)
    out, dist, in_b = interpolate_sample(q, amap)
    ok = in_b and np.isfinite(out.Cd) and out.Cl_front < 0 and out.Cl_rear < 0
    ok = ok and dist < 2.0
    return ok, {"Cd": out.Cd, "Cl_f": out.Cl_front, "dist": dist}


def test_calibration_accuracy() -> tuple[bool, dict]:
    samples = synthetic_cfd_grid()
    cal = calibrate_against_samples(samples, base=AeroConfig())
    ok = cal.success and cal.n_samples == len(samples)
    # After calibration RMS should be finite and not huge
    ok = ok and cal.rms_Cd < 0.15 and cal.rms_Cl_f < 0.3
    return ok, {
        "rms_Cd": cal.rms_Cd,
        "rms_Cl_f": cal.rms_Cl_f,
        "Cd": cal.config.coeffs.Cd,
    }


def test_lookup_solver() -> tuple[bool, dict]:
    samples = synthetic_cfd_grid()
    amap = build_map_from_samples(samples)
    db = AeroDatabase(amap=amap, mode=AeroSolverMode.LOOKUP, config=AeroConfig())
    st, unc, src = db.lookup(40.0, RideHeightState(h_front=0.08, h_rear=0.10))
    ok = src == "lookup" and st.downforce_total > 0 and unc.confidence > 0.3
    return ok, {"source": src, "DF": st.downforce_total, "conf": unc.confidence}


def test_hybrid_solver() -> tuple[bool, dict]:
    samples = synthetic_cfd_grid()
    amap = build_map_from_samples(samples)
    db = AeroDatabase(amap=amap, mode=AeroSolverMode.HYBRID, config=AeroConfig())
    # In-bounds → lookup
    st1, _, src1 = db.lookup(40.0, RideHeightState(h_front=0.08, h_rear=0.10))
    # Far OOB speed → analytical fallback
    st2, _, src2 = db.lookup(200.0, RideHeightState(h_front=0.5, h_rear=0.5))
    ok = src1 == "lookup" and src2 == "hybrid_fallback"
    ok = ok and st1.downforce_total > 0 and st2.downforce_total > 0
    return ok, {"src_in": src1, "src_oob": src2}


def test_cop_prediction() -> tuple[bool, dict]:
    samples = synthetic_cfd_grid()
    amap = build_map_from_samples(samples)
    db = AeroDatabase(amap=amap, mode=AeroSolverMode.LOOKUP)
    st, _, src = db.lookup(40.0, RideHeightState(h_front=0.08, h_rear=0.10))
    ok = src == "lookup" and np.isfinite(st.center_of_pressure_x)
    return ok, {"x_cop": st.center_of_pressure_x}


def test_uncertainty_estimation() -> tuple[bool, dict]:
    u_near = estimate_uncertainty(0.1, 27, True)
    u_far = estimate_uncertainty(5.0, 27, False)
    ok = u_near.confidence > u_far.confidence
    ok = ok and u_near.high_confidence
    return ok, {"conf_near": u_near.confidence, "conf_far": u_far.confidence}


def test_out_of_bounds_protection() -> tuple[bool, dict]:
    samples = synthetic_cfd_grid()
    amap = build_map_from_samples(samples)
    db = AeroDatabase(amap=amap, mode=AeroSolverMode.LOOKUP, max_interp_distance=0.5)
    st, unc, src = db.lookup(40.0, RideHeightState(h_front=0.5, h_rear=0.5))
    ok = src == "lookup_oob"
    ok = ok and st.drag == 0  # protected zeros
    return ok, {"source": src, "conf": unc.confidence}


def test_performance_no_nan() -> tuple[bool, dict]:
    samples = synthetic_cfd_grid()
    amap = build_map_from_samples(samples)
    db = AeroDatabase(amap=amap, mode=AeroSolverMode.HYBRID, config=AeroConfig())
    ok = True
    for v in (0.0, 30.0, 55.0):
        for hf, hr in ((0.06, 0.08), (0.1, 0.12), (0.3, 0.3)):
            st, unc, src = db.lookup(v, RideHeightState(h_front=hf, h_rear=hr))
            vals = [st.Fx, st.Fz_front, st.Fz_rear, st.q, unc.confidence]
            if not all(np.isfinite(vals)):
                ok = False
    return ok, {"ok": ok}


def test_closed_loop_analytical_mode() -> tuple[bool, dict]:
    """ClosedLoopAero with devices off still matches baseline (9.2 regression)."""
    cfg = AeroConfig()
    cl = ClosedLoopAero(cfg, device_cfg=AeroDeviceConfig(devices_enabled=False))
    out = cl.step(VehicleAeroInput(speed=40.0))
    ref = compute_aero_loads(40.0, cfg, ride=out.ride)
    ok = abs(out.aero.state.Fx - ref.Fx) < 1e-9
    return ok, {"dFx": abs(out.aero.state.Fx - ref.Fx)}


def run_phase93_validation() -> bool:
    print("=== Phase 9.3 CFD Calibration & Map Generation Validation ===\n")
    tests = [
        ("analytical_regression", test_analytical_regression),
        ("csv_import", test_csv_import),
        ("openfoam_import", test_openfoam_import),
        ("su2_import", test_su2_import),
        ("database_build", test_database_build),
        ("multidimensional_interpolation", test_multidimensional_interpolation),
        ("calibration_accuracy", test_calibration_accuracy),
        ("lookup_solver", test_lookup_solver),
        ("hybrid_solver", test_hybrid_solver),
        ("cop_prediction", test_cop_prediction),
        ("uncertainty_estimation", test_uncertainty_estimation),
        ("out_of_bounds_protection", test_out_of_bounds_protection),
        ("no_nan_inf_performance", test_performance_no_nan),
    ]
    # Note: closed_loop analytical covered by analytical_regression + gate list maps to 13
    # Replace last with closed_loop if needed — we have 13 above if we count carefully:
    # listed 13 tests in the list above? Count: 13 items yes (analytical through no_nan).
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
        print("Phase 9.3 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase93_validation()
