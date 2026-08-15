"""
Phase 14.4 — Dynamic Load Transfer & Wheel-Load Authority.
No retuning of frozen 14.2 parameters.
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
from vehicle_dynamics.simulation.dual_track_plant import DualTrackConfig
from vehicle_dynamics.lateral.load_transfer import compute_wheel_loads

ROOT = Path("artifacts/phase_14_4")


def _gate(gates, name, ok, detail=""):
    gates.append({"name": name, "pass": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def _t_to(vx, t, speed):
    idx = np.where(np.asarray(vx) >= speed)[0]
    return float(t[idx[0]]) if len(idx) else None


def _launch(cfg, n=2500):
    sim = Simulation(cfg)
    sim.reset(0.0, 1)
    vx, tt = [], []
    for _ in range(n):
        sim._step_plant(1.0, 0, 0, 1.0, 0, 0.01)
        vx.append(sim.state.vehicle.vx)
        tt.append(sim.state.time)
    return vx, tt, sim


def _fz(sim):
    d = sim.dual_track.diagnostics()
    return np.array([d["Fz_FL"], d["Fz_FR"], d["Fz_RL"], d["Fz_RR"]])


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    provenance = []
    mutations = []

    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config
    sim = Simulation(cfg)
    sim.reset(0.0, 1)
    dt = sim.dual_track

    # ---- Gate: authority map ----
    sources = {
        "mass": ("VehicleDefinition", cfg.mass, dt.cfg.mass),
        "h_cg": ("GeometryConfig", cfg.h_cg, dt.cfg.h_cg),
        "track_f": ("GeometryConfig", cfg.track, dt.cfg.track_f),
        "track_r": ("GeometryConfig", cfg.track_rear, dt.cfg.track_r),
        "a+b": ("GeometryConfig", cfg.wheelbase, dt.cfg.a + dt.cfg.b),
        "chi_f": ("SimulationConfig", getattr(cfg, "chi_f", 0.55), dt.cfg.chi_f),
    }
    auth_ok = True
    for name, (src, cval, pval) in sources.items():
        match = abs(float(cval) - float(pval)) < 1e-6
        provenance.append({"parameter": name, "source": src, "config": cval, "plant": pval, "match": match})
        if not match:
            auth_ok = False
    _gate(gates, "load_transfer_authority", auth_ok,
          f"matched={sum(1 for p in provenance if p['match'])}/{len(provenance)}")

    # ---- Static distribution ----
    lt0 = compute_wheel_loads(
        mass=cfg.mass, a=dt.cfg.a, b=dt.cfg.b, h_cg=cfg.h_cg,
        track_f=dt.cfg.track_f, track_r=dt.cfg.track_r, ax=0, ay=0,
    )
    static_ok = (
        abs(lt0.Fz_fl - lt0.Fz_fr) < 1e-6
        and abs(lt0.Fz_rl - lt0.Fz_rr) < 1e-6
        and abs(lt0.Fz_total - cfg.mass * 9.81) < 1.0
    )
    _gate(gates, "static_wheel_load_distribution", static_ok,
          f"FL={lt0.Fz_fl:.0f} FR={lt0.Fz_fr:.0f} RL={lt0.Fz_rl:.0f} RR={lt0.Fz_rr:.0f} "
          f"sum={lt0.Fz_total:.0f}")

    # ---- Longitudinal symmetry ----
    lt_p = compute_wheel_loads(
        mass=cfg.mass, a=dt.cfg.a, b=dt.cfg.b, h_cg=cfg.h_cg,
        track_f=dt.cfg.track_f, track_r=dt.cfg.track_r, ax=5.0, ay=0,
    )
    lt_m = compute_wheel_loads(
        mass=cfg.mass, a=dt.cfg.a, b=dt.cfg.b, h_cg=cfg.h_cg,
        track_f=dt.cfg.track_f, track_r=dt.cfg.track_r, ax=-5.0, ay=0,
    )
    # +ax → load to rear; -ax → load to front; |ΔFz_long| symmetric about static
    rear_p = lt_p.Fz_rl + lt_p.Fz_rr
    front_p = lt_p.Fz_fl + lt_p.Fz_fr
    rear_m = lt_m.Fz_rl + lt_m.Fz_rr
    front_m = lt_m.Fz_fl + lt_m.Fz_fr
    front_0 = lt0.Fz_fl + lt0.Fz_fr
    d_p = front_0 - front_p   # load lost from front under +ax
    d_m = front_m - front_0   # load gained by front under -ax
    long_sym = (
        rear_p > front_p
        and front_m > rear_m
        and abs(d_p - d_m) < 5.0
        and abs(lt_p.dFz_long_f + lt_m.dFz_long_f) < 1e-6
    )
    _gate(gates, "longitudinal_transfer_symmetry", long_sym,
          f"+ax rear={lt_p.Fz_rl+lt_p.Fz_rr:.0f} front={lt_p.Fz_fl+lt_p.Fz_fr:.0f}; "
          f"-ax rear={lt_m.Fz_rl+lt_m.Fz_rr:.0f} front={lt_m.Fz_fl+lt_m.Fz_fr:.0f}")

    # ---- Lateral symmetry ----
    lt_ay = compute_wheel_loads(
        mass=cfg.mass, a=dt.cfg.a, b=dt.cfg.b, h_cg=cfg.h_cg,
        track_f=dt.cfg.track_f, track_r=dt.cfg.track_r, ax=0, ay=5.0,
    )
    lt_aym = compute_wheel_loads(
        mass=cfg.mass, a=dt.cfg.a, b=dt.cfg.b, h_cg=cfg.h_cg,
        track_f=dt.cfg.track_f, track_r=dt.cfg.track_r, ax=0, ay=-5.0,
    )
    lat_sym = (
        abs(lt_ay.Fz_fr - lt_aym.Fz_fl) < 1.0
        and abs(lt_ay.Fz_fl - lt_aym.Fz_fr) < 1.0
        and lt_ay.Fz_fr > lt_ay.Fz_fl  # +ay → load to right (FR)
    )
    _gate(gates, "lateral_transfer_symmetry", lat_sym,
          f"+ay FL={lt_ay.Fz_fl:.0f} FR={lt_ay.Fz_fr:.0f}; "
          f"-ay FL={lt_aym.Fz_fl:.0f} FR={lt_aym.Fz_fr:.0f}")

    # ---- Conservation ----
    cons_ok = abs(lt0.Fz_total - cfg.mass * 9.81) < 1.0
    _gate(gates, "wheel_load_conservation", cons_ok,
          f"ΣFz={lt0.Fz_total:.1f} mg={cfg.mass*9.81:.1f}")

    # ---- Tire Fz coupling (h_cg mutation changes response) ----
    def peak_ay(hcg, steer=0.08, n=40):
        c = bind_authoritative_hypercar().simulation_config
        c.h_cg = hcg
        s = Simulation(c)
        s.reset(25.0, 3)
        ays = []
        for _ in range(n):
            s._step_plant(0.15, 0, steer, 1.0, 0, 0.01)
            ays.append(s.state.vehicle.ay)
        fz = _fz(s)
        return float(np.max(np.abs(ays))), fz

    ay_lo, fz_lo = peak_ay(0.40)
    ay_hi, fz_hi = peak_ay(0.80)
    # Higher CG → more lateral transfer (larger left-right delta)
    dFz_lo = abs(fz_lo[1] - fz_lo[0])
    dFz_hi = abs(fz_hi[1] - fz_hi[0])
    tire_coup = dFz_hi > dFz_lo * 1.3
    _gate(gates, "tire_fz_coupling", tire_coup,
          f"dFz_hcg0.4={dFz_lo:.0f} dFz_hcg0.8={dFz_hi:.0f}")
    mutations.append({"param": "h_cg×2", "dFz_base": dFz_lo, "dFz_mut": dFz_hi, "ok": tire_coup})

    # ---- Aero Fz coupling ----
    c_on = bind_authoritative_hypercar().simulation_config
    c_off = bind_authoritative_hypercar().simulation_config
    c_off.aero_enabled = False
    s_on = Simulation(c_on)
    s_off = Simulation(c_off)
    for s in (s_on, s_off):
        s.reset(50.0, 5)
        for _ in range(30):
            s._step_plant(0.1, 0, 0, 1, 0, 0.01)
    fz_on = _fz(s_on).sum()
    fz_off = _fz(s_off).sum()
    aero_coup = fz_on > fz_off + 50  # downforce adds normal load
    _gate(gates, "aero_fz_coupling", aero_coup,
          f"ΣFz_on={fz_on:.0f} ΣFz_off={fz_off:.0f}")

    # ---- Combined braking + cornering ----
    s = Simulation(cfg)
    s.reset(30.0, 4)
    for _ in range(10):
        s._step_plant(0, 0, 0, 1, 0, 0.01)
    # brake + steer
    for _ in range(40):
        s._step_plant(0, 0.8, 0.08, 1, 0, 0.01)
    d = s.dual_track.diagnostics()
    util = d.get("utilization", d.get("util", [0]*4))
    if isinstance(util, dict):
        util = [0]*4
    comb_ok = (
        d["min_Fz"] >= 50.0 - 1e-6
        and d["Fz_sum"] > 0
        and not any(np.isnan(x) for x in [d["Fz_FL"], d["Fz_FR"], d["Fz_RL"], d["Fz_RR"]])
        and abs(s.state.vehicle.ax) > 0.1
    )
    _gate(gates, "combined_braking_cornering", comb_ok,
          f"Fz={[round(d[k],0) for k in ('Fz_FL','Fz_FR','Fz_RL','Fz_RR')]} "
          f"min={d['min_Fz']:.0f} ax={s.state.vehicle.ax:.2f}")
    with open(ROOT / "combined_slip_load_transfer.json", "w") as f:
        json.dump({"Fz": {k: d[k] for k in ("Fz_FL","Fz_FR","Fz_RL","Fz_RR")},
                   "min_Fz": d["min_Fz"], "ax": s.state.vehicle.ax, "ay": s.state.vehicle.ay}, f, indent=2)

    # ---- Wheel unloading ----
    # aggressive: high h_cg, hard brake + steer
    c_u = bind_authoritative_hypercar().simulation_config
    c_u.h_cg = 0.9
    s_u = Simulation(c_u)
    s_u.reset(35.0, 4)
    mins = []
    for _ in range(80):
        s_u._step_plant(0, 1.0, 0.12, 1, 0, 0.01)
        mins.append(s_u.dual_track.diagnostics()["min_Fz"])
        if s_u.state.vehicle.vx < 1:
            break
    unload_ok = min(mins) >= 50.0 - 1e-6 and not any(np.isnan(mins))
    _gate(gates, "wheel_unloading_behavior", unload_ok,
          f"min_Fz_over_run={min(mins):.1f} (floor={50})")

    # ---- Mutations ----
    def plant_dFz_lat(track_f):
        c = bind_authoritative_hypercar().simulation_config
        c.track = track_f
        s = Simulation(c)
        s.reset(20.0, 3)
        for _ in range(30):
            s._step_plant(0.1, 0, 0.10, 1, 0, 0.01)
        fz = _fz(s)
        return abs(fz[1] - fz[0])

    dFz_t1 = plant_dFz_lat(1.65)
    dFz_t2 = plant_dFz_lat(1.32)  # ×0.8
    track_ok = dFz_t2 > dFz_t1 * 1.1
    _gate(gates, "track_mutation_authority", track_ok,
          f"dFz_1.65={dFz_t1:.0f} dFz_1.32={dFz_t2:.0f}")
    mutations.append({"param": "track_f×0.8", "dFz_base": dFz_t1, "dFz_mut": dFz_t2, "ok": track_ok})

    # hcg already tested
    _gate(gates, "hcg_mutation_authority", tire_coup, f"dFz ratio={dFz_hi/max(dFz_lo,1):.2f}")

    # wheelbase mutation
    lt_wb = compute_wheel_loads(
        mass=1100, a=1.0, b=1.0, h_cg=0.4, track_f=1.65, track_r=1.62, ax=5, ay=0
    )
    lt_wb2 = compute_wheel_loads(
        mass=1100, a=1.2, b=1.2, h_cg=0.4, track_f=1.65, track_r=1.62, ax=5, ay=0
    )
    # shorter WB → larger long transfer
    wb_ok = abs(lt_wb.dFz_long_f) > abs(lt_wb2.dFz_long_f)
    _gate(gates, "wheelbase_mutation_authority", wb_ok,
          f"|dFz_long| L=2.0={abs(lt_wb.dFz_long_f):.0f} L=2.4={abs(lt_wb2.dFz_long_f):.0f}")

    # aero mutation
    lt_a0 = compute_wheel_loads(
        mass=1100, a=1.2, b=1.5, h_cg=0.4, track_f=1.65, track_r=1.62, ax=0, ay=0,
        downforce_front=0, downforce_rear=0,
    )
    lt_a1 = compute_wheel_loads(
        mass=1100, a=1.2, b=1.5, h_cg=0.4, track_f=1.65, track_r=1.62, ax=0, ay=0,
        downforce_front=2000, downforce_rear=3000,
    )
    aero_mut = lt_a1.Fz_total > lt_a0.Fz_total + 4900
    _gate(gates, "aero_mutation_authority", aero_mut,
          f"ΣFz 0df={lt_a0.Fz_total:.0f} +5kN={lt_a1.Fz_total:.0f}")

    # ---- Negative fallback ----
    DualTrackConfig.__dataclass_fields__["h_cg"].default = 9.99
    DualTrackConfig.__dataclass_fields__["track_f"].default = 0.11
    try:
        s2 = Simulation(bind_authoritative_hypercar().simulation_config)
        fb_ok = abs(s2.dual_track.cfg.h_cg - 0.40) < 1e-6 and abs(s2.dual_track.cfg.track_f - 1.65) < 1e-6
        _gate(gates, "negative_default_fallback", fb_ok,
              f"h_cg={s2.dual_track.cfg.h_cg} track_f={s2.dual_track.cfg.track_f}")
    finally:
        DualTrackConfig.__dataclass_fields__["h_cg"].default = 0.45
        DualTrackConfig.__dataclass_fields__["track_f"].default = 1.65

    # ---- Historical isolation ----
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100 = _t_to(hvx, ht, 27.78)
    ht200 = _t_to(hvx, ht, 55.56)
    avx, at, _ = _launch(cfg)
    at100 = _t_to(avx, at, 27.78)
    at200 = _t_to(avx, at, 55.56)
    hist_ok = (
        ht100 is not None and abs(ht100 - 5.36) < 0.15
        and ht200 is not None and abs(ht200 - 19.77) < 0.3
    )
    zero_ok = (
        at100 is not None and abs(at100 - 3.13) < 0.2
        and at200 is not None and abs(at200 - 8.31) < 0.3
    )
    _gate(gates, "historical_isolation", hist_ok, f"t100={ht100} t200={ht200}")
    _gate(gates, "zero_wind_regression", zero_ok, f"t100={at100} t200={at200}")

    # ---- Determinism ----
    runs = []
    for _ in range(5):
        s = Simulation(cfg)
        s.reset(25.0, 3)
        for __ in range(40):
            s._step_plant(0.1, 0.3, 0.06, 1, 0, 0.01)
        runs.append(tuple(np.round(_fz(s), 6)))
    det_ok = len(set(runs)) == 1
    _gate(gates, "deterministic_replay", det_ok, f"Fz_run0={runs[0]}")

    with open(ROOT / "provenance.json", "w") as f:
        json.dump(provenance, f, indent=2)
    with open(ROOT / "wheel_load_symmetry.json", "w") as f:
        json.dump({
            "static": [lt0.Fz_fl, lt0.Fz_fr, lt0.Fz_rl, lt0.Fz_rr],
            "plus_ax": [lt_p.Fz_fl, lt_p.Fz_fr, lt_p.Fz_rl, lt_p.Fz_rr],
            "minus_ax": [lt_m.Fz_fl, lt_m.Fz_fr, lt_m.Fz_rl, lt_m.Fz_rr],
            "plus_ay": [lt_ay.Fz_fl, lt_ay.Fz_fr, lt_ay.Fz_rl, lt_ay.Fz_rr],
            "minus_ay": [lt_aym.Fz_fl, lt_aym.Fz_fr, lt_aym.Fz_rl, lt_aym.Fz_rr],
        }, f, indent=2)
    with open(ROOT / "mutation_results.json", "w") as f:
        json.dump([{k: (bool(v) if isinstance(v, (bool, np.bool_)) else float(v) if isinstance(v, (np.floating, float)) else v) for k, v in m.items()} for m in mutations], f, indent=2)
    with open(ROOT / "historical_isolation.json", "w") as f:
        json.dump({"hist": {"t100": ht100, "t200": ht200},
                   "hyper": {"t100": at100, "t200": at200}}, f, indent=2)
    with open(ROOT / "deterministic_replay.json", "w") as f:
        json.dump({"runs": [list(r) for r in runs]}, f, indent=2)

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 2 else "FAIL"
    )
    summary = {"phase": "14.4", "status": status, "gates_passed": n_pass,
               "gates_total": len(gates), "gates": gates}
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 14.4 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
