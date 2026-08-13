"""
Phase 14.2H.1 — Runtime Parameter Authority Audit.

Proves VehicleDefinition → SimulationConfig → Runtime Plant with zero silent defaults.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from vehicle_dynamics.simulation.simulation import Simulation
from vehicle_dynamics.demonstration.vehicle_binding import (
    bind_authoritative_hypercar,
    bind_historical_demonstrator,
    HISTORICAL_EXPECTED,
)
from vehicle_dynamics.powertrain.transmission.gear_ratios import default_ratios

ROOT = Path("artifacts/phase_14_2h1")


def _gate(gates, name, ok, detail=""):
    gates.append({"name": name, "pass": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def _t_to(vx, t, speed):
    idx = np.where(np.asarray(vx) >= speed)[0]
    return float(t[idx[0]]) if len(idx) else None


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    sim = Simulation(hyper.simulation_config)
    sim.reset(0.0, 1)
    dt = sim.dual_track
    assert dt is not None

    # --- Definition targets ---
    defn = {
        "mass": 1100.0,
        "power": 750.0,
        "mu": 1.15,
        "Cx": 100000.0,
        "Cy": 90000.0,
        "radius": 0.33,
        "final_drive": 3.9,
        "split": 0.35,
        "abs": True,
        "brake_tmax": 2800.0,
        "h_cg": 0.40,
        "wb": 2.70,
        "track_f": 1.65,
        "track_r": 1.62,
        "Cd": 0.34,
        "Cl_f": -0.55,
        "Cl_r": -0.85,
    }
    cfg = hyper.simulation_config

    rows = []
    def row(param, dval, cval, pval, tol=1e-6):
        match = abs(float(dval) - float(cval)) <= tol and abs(float(cval) - float(pval)) <= tol
        rows.append({
            "parameter": param,
            "definition": dval,
            "simulation_config": cval,
            "runtime_plant": pval,
            "match": match,
        })
        return match

    # Mass / geometry / tire / brake / AWD
    m1 = row("mass", defn["mass"], cfg.mass, dt.cfg.mass, 1.0)
    m2 = row("mu", defn["mu"], cfg.mu_tire, dt.cfg.mu)
    m3 = row("tire_cx", defn["Cx"], cfg.tire_cx, dt.cfg.tire_cx, 1.0)
    m4 = row("tire_cy", defn["Cy"], cfg.tire_cy, dt.cfg.tire_cy, 1.0)
    m5 = row("wheel_radius", defn["radius"], cfg.wheel_radius, dt.cfg.wheel_radius)
    m6 = row("drive_split_front", defn["split"], cfg.drive_split_front, dt.cfg.drive_split_front)
    m7 = row("brake_torque_max", defn["brake_tmax"], cfg.brake_torque_max, dt.cfg.brake_torque_max, 1.0)
    m8 = row("h_cg", defn["h_cg"], cfg.h_cg, dt.cfg.h_cg)
    m9 = row("wheelbase", defn["wb"], cfg.wheelbase, dt.cfg.a + dt.cfg.b, 1e-3)
    m10 = row("track_front", defn["track_f"], cfg.track, dt.cfg.track_f)
    m11 = row("track_rear", defn["track_r"], cfg.track_rear, dt.cfg.track_r)
    m12 = row("abs_enabled", 1.0 if defn["abs"] else 0.0,
              1.0 if cfg.abs_enabled else 0.0,
              1.0 if dt.cfg.abs_enabled else 0.0)

    # Dugoff instances
    cx_ok = all(abs(t.p.Cx - defn["Cx"]) < 1.0 for t in dt.tires)
    cy_ok = all(abs(t.p.Cy - defn["Cy"]) < 1.0 for t in dt.tires)
    mu_ok = all(abs(t.p.mu - defn["mu"]) < 1e-6 for t in dt.tires)
    rows.append({
        "parameter": "DugoffTire.Cx[FL..RR]",
        "definition": defn["Cx"],
        "simulation_config": cfg.tire_cx,
        "runtime_plant": [t.p.Cx for t in dt.tires],
        "match": cx_ok,
    })
    rows.append({
        "parameter": "DugoffTire.Cy[FL..RR]",
        "definition": defn["Cy"],
        "simulation_config": cfg.tire_cy,
        "runtime_plant": [t.p.Cy for t in dt.tires],
        "match": cy_ok,
    })

    # Powertrain runtime
    eng_peak = float(sim.engine.cfg.peak_torque)
    omega_pt = 4500 * 2 * np.pi / 60
    expect_tq = 750000 / omega_pt
    pt_ok = abs(eng_peak - expect_tq) < 5.0
    rows.append({
        "parameter": "engine.peak_torque",
        "definition": round(expect_tq, 1),
        "simulation_config": cfg.peak_torque_nm,
        "runtime_plant": eng_peak,
        "match": pt_ok and abs(cfg.peak_power_kw - 750) < 1,
    })
    fd_rt = float(sim.trans.gearbox.ratios.final_drive)
    fd_ok = abs(fd_rt - 3.9) < 1e-6
    rows.append({
        "parameter": "final_drive",
        "definition": 3.9,
        "simulation_config": cfg.final_drive,
        "runtime_plant": fd_rt,
        "match": fd_ok,
    })
    gears = list(sim.trans.gearbox.ratios.gears)
    rows.append({
        "parameter": "gear_ratios",
        "definition": "[0,3.5,2.2,1.6,1.2,1.0,0.85]",
        "simulation_config": "default_ratios(final_drive)",
        "runtime_plant": gears,
        "match": gears == [0.0, 3.5, 2.2, 1.6, 1.2, 1.0, 0.85],
    })

    # Aero runtime
    aero_ok = (
        abs(sim.aero_cfg.coeffs.Cd - defn["Cd"]) < 1e-6
        and abs(sim.aero_cfg.coeffs.Cl_front - defn["Cl_f"]) < 1e-6
        and abs(sim.aero_cfg.coeffs.Cl_rear - defn["Cl_r"]) < 1e-6
        and sim.aero_cfg.enabled
    )
    rows.append({
        "parameter": "aero_coeffs",
        "definition": f"Cd={defn['Cd']} Clf={defn['Cl_f']} Clr={defn['Cl_r']}",
        "simulation_config": f"Cd={cfg.aero_cd} Clf={cfg.aero_cl_front} Clr={cfg.aero_cl_rear}",
        "runtime_plant": f"Cd={sim.aero_cfg.coeffs.Cd} Clf={sim.aero_cfg.coeffs.Cl_front} Clr={sim.aero_cfg.coeffs.Cl_rear}",
        "match": aero_ok,
    })

    all_match = all(r["match"] for r in rows)
    _gate(gates, "runtime_provenance_complete", all_match,
          f"matched={sum(1 for r in rows if r['match'])}/{len(rows)}")

    # AWD exact + front/rear torque during WOT
    sim._step_plant(1.0, 0, 0, 1.0, 0, 0.01)
    for _ in range(50):
        sim._step_plant(1.0, 0, 0, 1.0, 0, 0.01)
    T = [w.drive_torque for w in dt.wheels]
    T_f = T[0] + T[1]
    T_r = T[2] + T[3]
    split_rt = T_f / (T_f + T_r) if (T_f + T_r) > 1 else -1
    _gate(gates, "runtime_awd_authority",
          abs(cfg.drive_split_front - 0.35) < 1e-6
          and abs(dt.cfg.drive_split_front - 0.35) < 1e-6
          and T_f > 0 and T_r > 0
          and abs(split_rt - 0.35) < 0.05,
          f"cfg={cfg.drive_split_front} plant={dt.cfg.drive_split_front} "
          f"T_f={T_f:.0f} T_r={T_r:.0f} split_rt={split_rt:.3f}")

    _gate(gates, "runtime_tire_parameter_authority", cx_ok and cy_ok and mu_ok,
          f"Cx={[t.p.Cx for t in dt.tires]} Cy={[t.p.Cy for t in dt.tires]} mu={[t.p.mu for t in dt.tires]}")

    _gate(gates, "runtime_brake_parameter_authority",
          abs(dt.cfg.brake_torque_max - 2800) < 1 and dt.cfg.abs_enabled,
          f"Tmax={dt.cfg.brake_torque_max} abs={dt.cfg.abs_enabled}")

    # ABS pressure change
    sim2 = Simulation(hyper.simulation_config)
    sim2.reset(27.78, 3)
    for _ in range(15):
        sim2._step_plant(0, 0, 0, 1, 0, 0.01)
    P = []
    for _ in range(100):
        sim2._step_plant(0, 1.0, 0, 1, 0, 0.01)
        P.append(list(sim2._dual_diag.get("brake_pressure", [1, 1, 1, 1])))
        if sim2.state.vehicle.vx < 1:
            break
    P = np.array(P)
    _gate(gates, "abs_runtime_pressure_authority",
          float(np.std(P)) > 0.02 and float(np.min(P)) < 0.95,
          f"std={np.std(P):.3f} min={np.min(P):.3f}")

    _gate(gates, "runtime_powertrain_parameter_authority",
          pt_ok and fd_ok and abs(cfg.peak_power_kw - 750) < 1,
          f"peak_tq={eng_peak:.1f} expect≈{expect_tq:.1f} fd={fd_rt} P={cfg.peak_power_kw}")

    _gate(gates, "runtime_aero_parameter_authority", aero_ok,
          f"Cd={sim.aero_cfg.coeffs.Cd} Clf={sim.aero_cfg.coeffs.Cl_front} Clr={sim.aero_cfg.coeffs.Cl_rear}")

    # Aero on/off dynamics
    sim_on = Simulation(hyper.simulation_config)
    cfg_off = bind_authoritative_hypercar().simulation_config
    cfg_off.aero_enabled = False
    sim_off = Simulation(cfg_off)
    for s in (sim_on, sim_off):
        s.reset(50.0, 5)
    for _ in range(30):
        sim_on._step_plant(0, 0, 0, 1, 0, 0.01)
        sim_off._step_plant(0, 0, 0, 1, 0, 0.01)
    _gate(gates, "aero_dynamics_authority",
          sim_on.state.vehicle.ax < sim_off.state.vehicle.ax - 0.05,
          f"ax_on={sim_on.state.vehicle.ax:.3f} ax_off={sim_off.state.vehicle.ax:.3f}")

    _gate(gates, "runtime_geometry_parameter_authority",
          m8 and m9 and m10 and m11,
          f"h_cg={dt.cfg.h_cg} L={dt.cfg.a+dt.cfg.b:.3f} tf={dt.cfg.track_f} tr={dt.cfg.track_r}")

    # No authority fallback in hypercar path — plant values must equal cfg
    _gate(gates, "no_authority_fallback",
          all([m1, m2, m3, m4, m5, m6, m7, cx_ok, cy_ok]),
          "Definition→Config→Plant chain closed for mass/μ/Cx/Cy/r/split/brake")

    # Historical regression
    hsim = Simulation(hist.simulation_config)
    hsim.reset(0.0, 1)
    hvx, ht = [], []
    for _ in range(2500):
        hsim._step_plant(1, 0, 0, 1, 0, 0.01)
        hvx.append(hsim.state.vehicle.vx)
        ht.append(hsim.state.time)
    ht100 = _t_to(hvx, ht, 27.78)
    ht200 = _t_to(hvx, ht, 55.56)
    _gate(gates, "historical_regression",
          ht100 is not None and abs(ht100 - 5.36) < 0.15
          and ht200 is not None and abs(ht200 - 19.77) < 0.3,
          f"t100={ht100} t200={ht200}")

    # Authoritative hypercar replay
    as_ = Simulation(hyper.simulation_config)
    as_.reset(0.0, 1)
    avx, at = [], []
    for _ in range(2500):
        as_._step_plant(1, 0, 0, 1, 0, 0.01)
        avx.append(as_.state.vehicle.vx)
        at.append(as_.state.time)
    at100 = _t_to(avx, at, 27.78)
    at200 = _t_to(avx, at, 55.56)
    _gate(gates, "authoritative_hypercar_replay",
          at100 is not None and at200 is not None
          and abs(as_.cfg.mass - 1100) < 1 and abs(as_.cfg.peak_power_kw - 750) < 1,
          f"t100={at100} t200={at200} mass={as_.cfg.mass} P={as_.cfg.peak_power_kw}")

    # Identity: runtime plant mass/power not historical
    _gate(gates, "identity_not_historical",
          abs(dt.cfg.mass - 1100) < 1 and abs(dt.cfg.mass - 1400) > 1,
          f"plant_mass={dt.cfg.mass}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 2 else "FAIL"
    )

    summary = {
        "phase": "14.2H.1",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "provenance_table": rows,
        "historical": {"t100": ht100, "t200": ht200},
        "hypercar": {"t100": at100, "t200": at200, "fingerprint": hyper.config_fingerprint},
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 14.2H.1 — {status} {n_pass}/{len(gates)} ===")
    print("Provenance table:")
    for r in rows:
        print(f"  {r['parameter']}: def={r['definition']} cfg={r['simulation_config']} "
              f"plant={r['runtime_plant']} match={r['match']}")
    return summary


if __name__ == "__main__":
    run_validation()
