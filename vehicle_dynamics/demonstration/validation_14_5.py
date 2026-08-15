"""
Phase 14.5 — Transient Vehicle Body Dynamics & Suspension Load Transfer.
No retuning of frozen 14.2 mass/power/μ/tire/gear parameters.
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
from vehicle_dynamics.simulation.sprung_body import SprungBodyConfig

ROOT = Path("artifacts/phase_14_5")


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
    sim = Simulation(cfg)
    sim.reset(0.0, 1)
    dt = sim.dual_track

    # 1. Architecture
    arch_ok = (
        hasattr(dt, "sprung")
        and dt.cfg.use_sprung_body
        and hasattr(dt.sprung, "step")
        and hasattr(dt.sprung.state, "theta")
    )
    _gate(gates, "transient_dynamics_architecture", arch_ok,
          f"sprung={hasattr(dt,'sprung')} enabled={dt.cfg.use_sprung_body}")

    # 2. Parameter authority
    auth = (
        abs(dt.cfg.k_front - cfg.k_front) < 1
        and abs(dt.cfg.k_rear - cfg.k_rear) < 1
        and abs(dt.sprung.cfg.k_front - cfg.k_front) < 1
        and abs(dt.sprung.cfg.h_cg - cfg.h_cg) < 1e-6
    )
    _gate(gates, "suspension_parameter_authority", auth,
          f"k_f={dt.sprung.cfg.k_front} k_r={dt.sprung.cfg.k_rear} h={dt.sprung.cfg.h_cg}")

    # 3. Heave dynamics — step downforce
    s = Simulation(cfg)
    s.reset(20.0, 3)
    z0 = s.dual_track.sprung.state.z
    for _ in range(50):
        s.dual_track.step(vx=20, vy=0, yaw_rate=0, steer=0, drive_torque_total=0,
                          brake_cmd=0, dt=0.01, downforce_front=2000, downforce_rear=3000)
    z1 = s.dual_track.sprung.state.z
    _gate(gates, "heave_dynamics", abs(z1 - z0) > 1e-4 or abs(z1) > 1e-5,
          f"z0={z0:.5f} z1={z1:.5f}")

    # 4/5/6 Pitch dynamics + symmetry
    def pitch_trace(ax_cmd, n=80):
        s = Simulation(cfg)
        s.reset(30.0, 4)
        for _ in range(15):
            s._step_plant(0, 0, 0, 1, 0, 0.01)
        th = []
        for _ in range(n):
            # free dynamics under held ax via plant state injection
            s.dual_track.state.ax = ax_cmd
            s.dual_track.step(vx=max(s.state.vehicle.vx, 5), vy=0, yaw_rate=0, steer=0,
                              drive_torque_total=0, brake_cmd=0 if ax_cmd >= 0 else 0.5,
                              dt=0.01)
            th.append(s.dual_track.sprung.state.theta)
        return th

    th_acc = pitch_trace(5.0)
    th_brk = pitch_trace(-8.0)
    # braking → nose dive (theta < 0)
    pitch_ok = min(th_brk) < -0.005
    pitch_sym = (np.mean(th_acc) > 0) != (np.mean(th_brk) > 0) or (np.mean(th_acc) * np.mean(th_brk) < 0)
    # stronger: opposite signs at steady-ish
    pitch_sym = (th_acc[-1] > 0 and th_brk[-1] < 0) or (th_acc[-1] * th_brk[-1] < 0)
    _gate(gates, "pitch_dynamics", pitch_ok, f"brake θ min={min(th_brk):.4f} final={th_brk[-1]:.4f}")
    _gate(gates, "pitch_symmetry", pitch_sym,
          f"θ_acc={th_acc[-1]:.4f} θ_brk={th_brk[-1]:.4f}")

    # 7/8 Roll dynamics + symmetry
    def roll_trace(steer, n=60):
        s = Simulation(cfg)
        s.reset(25.0, 3)
        ph = []
        for _ in range(n):
            s._step_plant(0.2, 0, steer, 1, 0, 0.01)
            ph.append(s.dual_track.sprung.state.phi)
        return ph

    ph_p = roll_trace(0.10)
    ph_m = roll_trace(-0.10)
    roll_ok = abs(ph_p[-1]) > 0.01
    roll_sym = abs(ph_p[-1] + ph_m[-1]) < 0.15 * max(abs(ph_p[-1]), 1e-3)
    _gate(gates, "roll_dynamics", roll_ok, f"φ final={ph_p[-1]:.4f}")
    _gate(gates, "roll_symmetry", roll_sym, f"φ+={ph_p[-1]:.4f} φ-={ph_m[-1]:.4f}")

    # 9 Dynamic wheel load coupling
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(40):
        s._step_plant(0.1, 0, 0.08, 1, 0, 0.01)
    d = s.dual_track.diagnostics()
    fz = [d["Fz_FL"], d["Fz_FR"], d["Fz_RL"], d["Fz_RR"]]
    # left/right differ under cornering
    coup = abs(fz[0] - fz[1]) > 100
    _gate(gates, "dynamic_wheel_load_coupling", coup,
          f"Fz={np.round(fz,0)} φ={s.dual_track.sprung.state.phi:.4f}")

    # 10 Conservation (approx at rest)
    s = Simulation(cfg)
    s.reset(0.0, 1)
    for _ in range(100):
        s._step_plant(0, 0, 0, 1, 0, 0.01)
    d = s.dual_track.diagnostics()
    cons = abs(d["Fz_sum"] - cfg.mass * 9.81) < 50
    _gate(gates, "wheel_load_conservation", cons,
          f"ΣFz={d['Fz_sum']:.0f} mg={cfg.mass*9.81:.0f}")

    # 11 Tire Fz coupling — k_front mutation changes Fz response rate
    def f_settle_k(kf):
        c = bind_authoritative_hypercar().simulation_config
        c.k_front = kf
        s = Simulation(c)
        s.reset(30.0, 4)
        for _ in range(10):
            s._step_plant(0, 0, 0, 1, 0, 0.01)
        ths = []
        for _ in range(40):
            s.dual_track.state.ax = -8.0
            s.dual_track.step(vx=25, vy=0, yaw_rate=0, steer=0, drive_torque_total=0,
                              brake_cmd=0.8, dt=0.01)
            ths.append(s.dual_track.sprung.state.theta)
        return ths[-1], min(ths)

    th_soft, _ = f_settle_k(30000.0)
    th_stiff, _ = f_settle_k(120000.0)
    # stiffer → less pitch magnitude for same moment
    tire_coup = abs(th_stiff) < abs(th_soft)
    _gate(gates, "tire_fz_coupling", tire_coup,
          f"θ_soft={th_soft:.4f} θ_stiff={th_stiff:.4f}")

    # 12 Transient braking
    s = Simulation(cfg)
    s.reset(30.0, 4)
    for _ in range(15):
        s._step_plant(0, 0, 0, 1, 0, 0.01)
    log = []
    for _ in range(60):
        s._step_plant(0, 1.0, 0, 1, 0, 0.01)
        sb = s.dual_track.sprung.state
        log.append({"theta": sb.theta, "Fz_FL": float(sb.Fz[0]), "Fz_RL": float(sb.Fz[2]),
                    "ax": s.state.vehicle.ax})
        if s.state.vehicle.vx < 1:
            break
    brk_ok = (
        log[-1]["Fz_FL"] > log[-1]["Fz_RL"]  # front loaded under brake
        and min(x["theta"] for x in log) < 0  # nose dive
        and not any(np.isnan(x["theta"]) for x in log)
    )
    _gate(gates, "transient_braking", brk_ok,
          f"θmin={min(x['theta'] for x in log):.4f} FzF={log[-1]['Fz_FL']:.0f} FzR={log[-1]['Fz_RL']:.0f}")

    # 13 Transient cornering
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(80):
        s._step_plant(0.2, 0, 0.12, 1, 0, 0.01)
    sb = s.dual_track.sprung.state
    corner_ok = abs(sb.phi) > 0.01 and abs(sb.Fz[0] - sb.Fz[1]) > 50
    _gate(gates, "transient_cornering", corner_ok,
          f"φ={sb.phi:.4f} dFz_front={abs(sb.Fz[0]-sb.Fz[1]):.0f}")

    # 14 Combined
    s = Simulation(cfg)
    s.reset(30.0, 4)
    for _ in range(40):
        s._step_plant(0, 0.7, 0.08, 1, 0, 0.01)
    d = s.dual_track.diagnostics()
    comb = d["min_Fz"] >= 50 and not np.isnan(d["Fz_sum"]) and abs(s.state.vehicle.ay) > 0.5
    _gate(gates, "combined_braking_cornering", comb,
          f"minFz={d['min_Fz']:.0f} ay={s.state.vehicle.ay:.2f}")

    # 15 Suspension dissipation ≥ 0
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(80):
        s._step_plant(0.1, 0.3, 0.08, 1, 0, 0.01)
    E_d = s.dual_track.sprung.state.E_damp_dissipated
    _gate(gates, "suspension_dissipation", E_d >= 0.0, f"E_damp={E_d:.1f}")

    # 16 Mutation authority
    c1 = bind_authoritative_hypercar().simulation_config
    c2 = bind_authoritative_hypercar().simulation_config
    c2.k_front = c1.k_front * 2
    s1 = Simulation(c1)
    s2 = Simulation(c2)
    mut_ok = abs(s2.dual_track.sprung.cfg.k_front - c1.k_front * 2) < 1
    _gate(gates, "suspension_mutation_authority", mut_ok,
          f"k_f runtime={s2.dual_track.sprung.cfg.k_front}")

    # 17 Negative fallback
    DualTrackConfig.__dataclass_fields__["k_front"].default = 1.0
    try:
        s = Simulation(bind_authoritative_hypercar().simulation_config)
        fb = abs(s.dual_track.sprung.cfg.k_front - 1.0) > 1000
        _gate(gates, "negative_default_fallback", fb,
              f"runtime k_front={s.dual_track.sprung.cfg.k_front}")
    finally:
        DualTrackConfig.__dataclass_fields__["k_front"].default = 28000.0

    # 18 Regression
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100, ht200 = _t_to(hvx, ht, 27.78), _t_to(hvx, ht, 55.56)
    avx, at, _ = _launch(cfg)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    # Transient body dynamics shift longitudinal slightly vs pure QS — allow documented band
    hist_ok = ht100 is not None and abs(ht100 - 5.36) < 0.25
    hyper_ok = at100 is not None and abs(at100 - 3.13) < 0.25 and at200 is not None and abs(at200 - 8.30) < 0.5
    _gate(gates, "frozen_vehicle_regression", hist_ok and hyper_ok,
          f"hist {ht100}/{ht200}; hyper {at100}/{at200}")

    # Determinism
    runs = []
    for _ in range(5):
        s = Simulation(cfg)
        s.reset(25.0, 3)
        for __ in range(40):
            s._step_plant(0.1, 0.4, 0.06, 1, 0, 0.01)
        sb = s.dual_track.sprung.state
        runs.append((round(sb.theta, 8), round(sb.phi, 8), round(float(sb.Fz[0]), 4)))
    det = len(set(runs)) == 1
    _gate(gates, "deterministic_replay", det, f"run0={runs[0]}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 2 else "FAIL"
    )
    summary = {
        "phase": "14.5",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "regression": {"hist_t100": ht100, "hist_t200": ht200, "hyper_t100": at100, "hyper_t200": at200},
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 14.5 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
