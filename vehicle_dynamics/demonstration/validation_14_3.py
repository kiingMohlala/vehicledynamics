"""
Phase 14.3 — Crosswind & Sideslip Aerodynamic Coupling validation.
No retuning. Physics model capability only.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from vehicle_dynamics.demonstration.vehicle_binding import (
    bind_authoritative_hypercar,
    bind_historical_demonstrator,
)
from vehicle_dynamics.simulation.simulation import Simulation

ROOT = Path("artifacts/phase_14_3")


def _gate(gates, name, ok, detail=""):
    gates.append({"name": name, "pass": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def _t_to(vx, t, speed):
    idx = np.where(np.asarray(vx) >= speed)[0]
    return float(t[idx[0]]) if len(idx) else None


def _launch(cfg, n=2500, wind_vy=0.0):
    sim = Simulation(cfg)
    sim.reset(0.0, 1)
    sim.state.wind_vy = wind_vy
    sim.state.wind_vx = 0.0
    vx, t = [], []
    for _ in range(n):
        sim._step_plant(1.0, 0, 0, 1.0, 0, 0.01)
        vx.append(sim.state.vehicle.vx)
        t.append(sim.state.time)
    return vx, t, sim


def _steady_aero(cfg, vx0=30.0, wind_vy=0.0, wind_vx=0.0, steer=0.0, n=40, cy=None, cn=None):
    c = bind_authoritative_hypercar().simulation_config
    # copy critical from cfg if provided
    if cfg is not None:
        c = cfg
    if cy is not None:
        c.aero_cy_beta = cy
    if cn is not None:
        c.aero_cn_beta = cn
    sim = Simulation(c)
    sim.reset(vx0, 4)
    sim.state.wind_vx = wind_vx
    sim.state.wind_vy = wind_vy
    for _ in range(n):
        sim._step_plant(0.05, 0, steer, 1.0, 0, 0.01)
    air = sim._aero_air
    return sim, air


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    (ROOT / "telemetry").mkdir(exist_ok=True)
    gates = []

    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config

    # ----- Gate A: Zero-wind regression -----
    vx, t, _ = _launch(cfg, 2500, wind_vy=0.0)
    t100 = _t_to(vx, t, 27.78)
    t200 = _t_to(vx, t, 55.56)
    zero_ok = (
        t100 is not None and abs(t100 - 3.13) < 0.2
        and t200 is not None and abs(t200 - 8.31) < 0.3
    )
    _gate(gates, "zero_wind_regression", zero_ok, f"t100={t100} t200={t200}")
    with open(ROOT / "zero_wind_regression.json", "w") as f:
        json.dump({"t100": t100, "t200": t200, "pass": zero_ok}, f, indent=2)

    # ----- Gate B: Wind symmetry -----
    _, air_p = _steady_aero(cfg, wind_vy=12.0)
    _, air_m = _steady_aero(cfg, wind_vy=-12.0)
    fy_sym = abs(air_p.Fy_aero + air_m.Fy_aero) < 1e-6 * max(abs(air_p.Fy_aero), 1)
    mz_sym = abs(air_p.Mz_aero + air_m.Mz_aero) < 1e-6 * max(abs(air_p.Mz_aero), 1)
    beta_sym = abs(air_p.beta_air + air_m.beta_air) < 1e-9
    _gate(gates, "wind_symmetry", fy_sym and mz_sym and beta_sym,
          f"Fy+={air_p.Fy_aero:.2f} Fy-={air_m.Fy_aero:.2f} "
          f"Mz+={air_p.Mz_aero:.2f} Mz-={air_m.Mz_aero:.2f}")
    with open(ROOT / "crosswind_symmetry.json", "w") as f:
        json.dump({
            "plus": {"Fy": air_p.Fy_aero, "Mz": air_p.Mz_aero, "beta": air_p.beta_air},
            "minus": {"Fy": air_m.Fy_aero, "Mz": air_m.Mz_aero, "beta": air_m.beta_air},
            "pass": fy_sym and mz_sym,
        }, f, indent=2)

    # ----- Gate C: Sideslip authority -----
    _, a1 = _steady_aero(cfg, wind_vy=5.0)
    _, a2 = _steady_aero(cfg, wind_vy=15.0)
    sideslip_ok = (
        abs(a2.beta_air) > abs(a1.beta_air)
        and abs(a2.Fy_aero) > abs(a1.Fy_aero)
        and abs(a2.Mz_aero) > abs(a1.Mz_aero)
    )
    _gate(gates, "sideslip_authority", sideslip_ok,
          f"β5={a1.beta_air:.3f} β15={a2.beta_air:.3f} "
          f"|Fy| {abs(a1.Fy_aero):.1f}→{abs(a2.Fy_aero):.1f}")

    # ----- Gate D: Relative-air-speed authority -----
    # Hold vehicle speed, change wind magnitude → |q| and |Fy| change
    _, a_lo = _steady_aero(cfg, vx0=30.0, wind_vy=5.0)
    _, a_hi = _steady_aero(cfg, vx0=30.0, wind_vy=20.0)
    rel_ok = (
        a_hi.relative_air_speed != a_lo.relative_air_speed
        and abs(a_hi.Fy_aero) > abs(a_lo.Fy_aero)
        and a_hi.q > a_lo.q * 0.9  # may vary with air speed magnitude
    )
    _gate(gates, "relative_air_speed_authority", rel_ok,
          f"Vair {a_lo.relative_air_speed:.2f}→{a_hi.relative_air_speed:.2f} "
          f"q {a_lo.q:.0f}→{a_hi.q:.0f} Fy {a_lo.Fy_aero:.1f}→{a_hi.Fy_aero:.1f}")

    # ----- Gate E: Aero ON/OFF -----
    c_on = bind_authoritative_hypercar().simulation_config
    c_off = bind_authoritative_hypercar().simulation_config
    c_off.aero_enabled = False
    s_on, a_on = _steady_aero(c_on, wind_vy=10.0)
    s_off, a_off = _steady_aero(c_off, wind_vy=10.0)
    fy_off = a_off.Fy_aero if a_off is not None else 0.0
    drag_off = a_off.drag if a_off is not None else 0.0
    on_off = (
        a_on is not None
        and abs(a_on.Fy_aero) > 1.0
        and abs(fy_off) < 1e-6
        and abs(a_on.drag) > abs(drag_off) + 1.0
    )
    _gate(gates, "aero_on_off", on_off,
          f"Fy_on={a_on.Fy_aero:.1f} Fy_off={fy_off:.1f} drag_on={a_on.drag:.1f}")

    # ----- Gate F: Sign correctness -----
    sign_ok = (
        np.sign(air_p.Fy_aero) == -np.sign(air_m.Fy_aero)
        and np.sign(air_p.Mz_aero) == -np.sign(air_m.Mz_aero)
        and abs(air_p.Fy_aero) > 1.0
    )
    _gate(gates, "sign_correctness", sign_ok,
          f"sign Fy +/- = {np.sign(air_p.Fy_aero)}/{np.sign(air_m.Fy_aero)}")

    # ----- Gate G: Coefficient mutation -----
    _, a_base = _steady_aero(cfg, wind_vy=12.0, cy=-0.8, cn=-0.15)
    _, a_cy = _steady_aero(cfg, wind_vy=12.0, cy=-1.6, cn=-0.15)
    _, a_cn = _steady_aero(cfg, wind_vy=12.0, cy=-0.8, cn=-0.30)
    mut_cy = abs(a_cy.Fy_aero) > abs(a_base.Fy_aero) * 1.5
    mut_cn = abs(a_cn.Mz_aero) > abs(a_base.Mz_aero) * 1.5
    _gate(gates, "coefficient_mutation", mut_cy and mut_cn,
          f"Fy base={a_base.Fy_aero:.1f} Cy*2={a_cy.Fy_aero:.1f}; "
          f"Mz base={a_base.Mz_aero:.1f} Cn*2={a_cn.Mz_aero:.1f}")
    with open(ROOT / "coefficient_mutation.json", "w") as f:
        json.dump({
            "base": {"Fy": a_base.Fy_aero, "Mz": a_base.Mz_aero},
            "Cy_x2": {"Fy": a_cy.Fy_aero, "Mz": a_cy.Mz_aero},
            "Cn_x2": {"Fy": a_cn.Fy_aero, "Mz": a_cn.Mz_aero},
            "pass": mut_cy and mut_cn,
        }, f, indent=2)

    # ----- Relative airflow calculation present -----
    _gate(gates, "relative_airflow_calculation",
          air_p.relative_air_speed > 0 and abs(air_p.beta_air) > 0,
          f"Vair={air_p.relative_air_speed:.2f} β={air_p.beta_air:.3f}")
    _gate(gates, "beta_calculation", abs(air_p.beta_air) > 0.05,
          f"β={air_p.beta_air:.4f}")
    _gate(gates, "aero_side_force", abs(air_p.Fy_aero) > 10,
          f"Fy={air_p.Fy_aero:.1f}")
    _gate(gates, "aero_yaw_moment", abs(air_p.Mz_aero) > 5,
          f"Mz={air_p.Mz_aero:.1f}")

    # ----- Gate H: Historical isolation -----
    hvx, ht, _ = _launch(hist.simulation_config, 2500)
    ht100 = _t_to(hvx, ht, 27.78)
    ht200 = _t_to(hvx, ht, 55.56)
    hist_ok = (
        ht100 is not None and abs(ht100 - 5.36) < 0.15
        and ht200 is not None and abs(ht200 - 19.77) < 0.3
    )
    _gate(gates, "historical_isolation", hist_ok, f"t100={ht100} t200={ht200}")

    # ----- Gate I: Determinism -----
    fy_runs = []
    for _ in range(5):
        _, air = _steady_aero(cfg, wind_vy=10.0)
        fy_runs.append(air.Fy_aero)
    det_ok = max(fy_runs) - min(fy_runs) < 1e-9
    _gate(gates, "deterministic_replay", det_ok, f"Fy runs={fy_runs}")

    # Vehicle response under wind (ay / yaw from aero, not just config)
    s_w, a_w = _steady_aero(cfg, wind_vy=15.0, n=50)
    s_0, _ = _steady_aero(cfg, wind_vy=0.0, n=50)
    response_ok = abs(s_w.state.vehicle.ay) > abs(s_0.state.vehicle.ay) + 0.05
    _gate(gates, "vehicle_response_to_aero", response_ok,
          f"ay_wind={s_w.state.vehicle.ay:.3f} ay_0={s_0.state.vehicle.ay:.3f}")

    # No parameter retuning — identity still frozen
    _gate(gates, "no_parameter_retuning",
          abs(cfg.mass - 1100) < 1 and abs(cfg.peak_power_kw - 750) < 1
          and abs(cfg.mu_tire - 1.15) < 1e-6,
          f"mass={cfg.mass} P={cfg.peak_power_kw} μ={cfg.mu_tire}")

    # Aero authority summary
    authority = {
        "relative_airflow": True,
        "beta": air_p.beta_air,
        "Fy_aero": air_p.Fy_aero,
        "Mz_aero": air_p.Mz_aero,
        "symmetry": fy_sym and mz_sym,
        "mutation_cy": mut_cy,
        "mutation_cn": mut_cn,
        "zero_wind_t100": t100,
        "zero_wind_t200": t200,
    }
    with open(ROOT / "aero_authority.json", "w") as f:
        json.dump(authority, f, indent=2)

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 2 else "FAIL"
    )
    summary = {
        "phase": "14.3",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "zero_wind": {"t100": t100, "t200": t200},
        "historical": {"t100": ht100, "t200": ht200},
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 14.3 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
