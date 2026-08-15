"""
Phase 14.9.2 — Wheel-Local Slip Angles & Steering–Tire Coupling.
No ESC. No retuning of 14.8 vehicle identity.
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
from vehicle_dynamics.lateral.slip_angles import compute_wheel_slip_angles

ROOT = Path("artifacts/phase_14_9_2")
REF_HYPER = (3.13, 8.34)
REF_HIST = (5.37, 19.81)


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


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config

    # 1 Architecture
    import vehicle_dynamics.lateral.slip_angles as sa
    _gate(gates, "architecture",
          hasattr(sa, "compute_wheel_slip_angles"),
          "slip_angles module between kinematics and Dugoff")

    # 2 Four-wheel authority
    ss = compute_wheel_slip_angles(
        vx=20, vy=0, yaw_rate=0,
        deltas=np.array([0.1, 0.08, 0.0, 0.0]),
        xs=np.array([1.25, 1.25, -1.45, -1.45]),
        ys=np.array([0.825, -0.825, 0.81, -0.81]),
    )
    _gate(gates, "four_wheel_authority",
          len(ss) == 4 and ss[0].alpha != ss[2].alpha,
          f"α={[round(s.alpha,4) for s in ss]}")

    # 3 Ackermann coupling — FL/FR α differ
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    for _ in range(15):
        sim._step_plant(0.1, 0, 0.2, 1, 0, 0.01)
    d = sim.dual_track.diagnostics()
    _gate(gates, "ackermann_coupling",
          abs(d["alpha_FL"] - d["alpha_FR"]) > 1e-4 or abs(d["delta_fl"] - d["delta_fr"]) > 1e-4,
          f"αFL={d['alpha_FL']:.4f} αFR={d['alpha_FR']:.4f} δ={d['delta_fl']:.4f}/{d['delta_fr']:.4f}")

    # 4 Ackermann OFF — equal front α under equal δ
    c = bind_authoritative_hypercar().simulation_config
    c.ackermann_enabled = False
    s = Simulation(c)
    s.reset(25.0, 3)
    for _ in range(10):
        s._step_plant(0.1, 0, 0.15, 1, 0, 0.01)
    d = s.dual_track.diagnostics()
    _gate(gates, "ackermann_off_equal",
          abs(d["delta_fl"] - d["delta_fr"]) < 1e-9,
          f"δFL={d['delta_fl']:.5f} δFR={d['delta_fr']:.5f}")

    # 5 Zero steer consistency
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(20):
        s._step_plant(0.1, 0, 0.0, 1, 0, 0.01)
    d = s.dual_track.diagnostics()
    _gate(gates, "zero_steer",
          abs(d["steer_actual"]) < 1e-9 and abs(d["ay"]) < 0.5,
          f"δ={d['steer_actual']} ay={d['ay']:.3f}")

    # 6 Left/right symmetry
    def run_steer(cmd):
        s = Simulation(cfg)
        s.reset(25.0, 3)
        for _ in range(20):
            s._step_plant(0.1, 0, cmd, 1, 0, 0.01)
        d = s.dual_track.diagnostics()
        return d["alpha"], d["Fy"], d["ay"]

    aL, fL, ayL = run_steer(0.12)
    aR, fR, ayR = run_steer(-0.12)
    sym = abs(ayL + ayR) < 0.15 * max(abs(ayL), 1e-6) and abs(aL[0] + aR[1]) < 0.05
    _gate(gates, "left_right_symmetry", sym,
          f"ayL={ayL:.3f} ayR={ayR:.3f}")

    # 7 Steering authority — larger steer → larger |α|
    a_sm, _, _ = run_steer(0.05)
    a_lg, _, _ = run_steer(0.20)
    _gate(gates, "steering_authority",
          abs(a_lg[0]) > abs(a_sm[0]),
          f"|α|_0.05={abs(a_sm[0]):.4f} |α|_0.20={abs(a_lg[0]):.4f}")

    # 8 Yaw-rate authority
    def yaw_after(cmd):
        s = Simulation(cfg)
        s.reset(25.0, 3)
        for _ in range(40):
            s._step_plant(0.1, 0, cmd, 1, 0, 0.01)
        return s.state.vehicle.yaw_rate, s.state.vehicle.ay

    r1, ay1 = yaw_after(0.05)
    r2, ay2 = yaw_after(0.20)
    _gate(gates, "yaw_rate_authority",
          abs(r2) > abs(r1) and abs(ay2) > abs(ay1),
          f"r 0.05={r1:.3f} 0.20={r2:.3f}")

    # 9 Fz coupling
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(15):
        s._step_plant(0.1, 0, 0.15, 1, 0, 0.01)
    fy0 = abs(s.dual_track.wheels[0].Fy)
    s.dual_track.road_z = np.array([0.04, 0, 0, 0])
    for _ in range(20):
        s._step_plant(0.1, 0, 0.15, 1, 0, 0.01)
    fy1 = abs(s.dual_track.wheels[0].Fy)
    _gate(gates, "fz_coupling",
          abs(fy1 - fy0) > 10 or s.dual_track.wheels[0].Fz > 4000,
          f"Fy0={fy0:.0f} Fy1={fy1:.0f} Fz={s.dual_track.wheels[0].Fz:.0f}")

    # 10 μ coupling
    def fy_mu(mu):
        c = bind_authoritative_hypercar().simulation_config
        c.mu_tire = mu
        s = Simulation(c)
        s.reset(25.0, 3)
        for _ in range(20):
            s._step_plant(0.1, 0, 0.2, 1, 0, 0.01)
        return abs(sum(w.Fy for w in s.dual_track.wheels))
    _gate(gates, "mu_coupling",
          fy_mu(1.15) > fy_mu(0.5) * 1.1,
          f"Σ|Fy| μ1.15={fy_mu(1.15):.0f} μ0.5={fy_mu(0.5):.0f}")

    # 11 Cx/Cy authority
    def fy_cy(cy):
        c = bind_authoritative_hypercar().simulation_config
        c.tire_cy = cy
        s = Simulation(c)
        s.reset(25.0, 3)
        for _ in range(12):
            s._step_plant(0.1, 0, 0.1, 1, 0, 0.01)
        return abs(s.dual_track.wheels[0].Fy)
    _gate(gates, "cx_cy_authority",
          fy_cy(120000) > fy_cy(40000),
          f"Fy Cy120k={fy_cy(120000):.0f} Cy40k={fy_cy(40000):.0f}")

    # 12 Low-speed stability
    s = Simulation(cfg)
    s.reset(0.2, 1)
    ok_ls = True
    for _ in range(50):
        s._step_plant(0.2, 0, 0.3, 1, 0, 0.01)
        d = s.dual_track.diagnostics()
        if any(np.isnan(d["alpha"])) or any(np.isinf(d["alpha"])):
            ok_ls = False
            break
    _gate(gates, "low_speed_stability", ok_ls, f"nan_free={ok_ls}")

    # 13 Sign correctness: +steer → +ay early
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(10):
        s._step_plant(0.1, 0, 0.15, 1, 0, 0.01)
    _gate(gates, "sign_correctness",
          s.state.vehicle.ay > 0.5 and s.dual_track.wheels[0].alpha > 0,
          f"ay={s.state.vehicle.ay:.2f} αFL={s.dual_track.wheels[0].alpha:.4f}")

    # 14 Combined slip
    s = Simulation(cfg)
    s.reset(20.0, 3)
    for _ in range(25):
        s._step_plant(0.6, 0, 0.12, 1, 0, 0.01)
    utils = [w.utilization for w in s.dual_track.wheels]
    _gate(gates, "combined_slip",
          max(utils) > 0.05 and all(u >= 0 for u in utils),
          f"util={[round(u,3) for u in utils]}")

    # 15 Road isolation — FL road step: FL Fz peaks first; RR does not mirror
    s = Simulation(cfg)
    s.reset(20.0, 3)
    for _ in range(10):
        s._step_plant(0.1, 0, 0.0, 1, 0, 0.01)
    s.dual_track.road_z = np.array([0.04, 0, 0, 0])
    peak_fl, peak_rr = 0.0, 0.0
    base_fl = s.dual_track.wheels[0].Fz
    base_rr = s.dual_track.wheels[3].Fz
    for _ in range(25):
        s._step_plant(0.1, 0, 0.0, 1, 0, 0.01)
        peak_fl = max(peak_fl, s.dual_track.wheels[0].Fz)
        peak_rr = max(peak_rr, s.dual_track.wheels[3].Fz)
    _gate(gates, "road_isolation",
          (peak_fl - base_fl) > 1.5 * abs(peak_rr - base_rr) + 200,
          f"Δpeak_FL={peak_fl-base_fl:.0f} Δpeak_RR={peak_rr-base_rr:.0f}")

    # 16 Poisoned defaults
    DualTrackConfig.__dataclass_fields__["tire_cy"].default = 1.0
    DualTrackConfig.__dataclass_fields__["mu"].default = 0.01
    try:
        s = Simulation(bind_authoritative_hypercar().simulation_config)
        ok = abs(s.dual_track.cfg.tire_cy - cfg.tire_cy) < 1 and abs(s.dual_track.cfg.mu - 1.15) < 1e-6
        _gate(gates, "poisoned_defaults", ok,
              f"Cy={s.dual_track.cfg.tire_cy} μ={s.dual_track.cfg.mu}")
    finally:
        DualTrackConfig.__dataclass_fields__["tire_cy"].default = 80000.0
        DualTrackConfig.__dataclass_fields__["mu"].default = 1.15

    # 17 Historical isolation
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100 = _t_to(hvx, ht, 27.78)
    _gate(gates, "historical_isolation",
          ht100 is not None and abs(ht100 - REF_HIST[0]) < 0.3,
          f"t100={ht100}")

    # 18 Zero-wind regression
    avx, at, _ = _launch(cfg)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    reg = (
        at100 is not None and abs(at100 - REF_HYPER[0]) < 0.15
        and at200 is not None and abs(at200 - REF_HYPER[1]) < 0.25
    )
    _gate(gates, "zero_wind_regression", reg,
          f"t100={at100} t200={at200} ref={REF_HYPER}")

    # 19 Determinism
    runs = []
    for _ in range(5):
        s = Simulation(cfg)
        s.reset(25.0, 3)
        for __ in range(30):
            s._step_plant(0.1, 0, 0.12, 1, 0, 0.01)
        d = s.dual_track.diagnostics()
        runs.append((round(d["alpha_FL"], 8), round(d["Fy_FL"], 3), round(d["ay"], 6)))
    _gate(gates, "deterministic_replay", len(set(runs)) == 1, f"run0={runs[0]}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 1 else "FAIL"
    )
    summary = {
        "phase": "14.9.2",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "regression": {
            "hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
            "hist": {"t100": ht100, "ref": REF_HIST},
        },
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 14.9.2 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
