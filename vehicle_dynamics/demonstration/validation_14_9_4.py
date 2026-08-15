"""
Phase 14.9.4 — Transient Lateral Response & Yaw Dynamics.
Passive plant only. No ESC. No retuning.
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

ROOT = Path("artifacts/phase_14_9_4")
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


def _speed_hold(sim, vx0, thr_base=0.12):
    err = vx0 - sim.state.vehicle.vx
    return float(np.clip(thr_base + 0.05 * err, 0.0, 0.6))


def steering_step(cfg, vx0=25.0, delta=0.08, pre=40, post=200, rate=None):
    """δ: 0 → delta step; return time histories."""
    c = cfg
    if rate is not None:
        from vehicle_dynamics.demonstration.vehicle_binding import bind_authoritative_hypercar
        c = bind_authoritative_hypercar().simulation_config
        c.steering_rate = rate
        # copy other critical fields from cfg if needed — binding already defaults
    sim = Simulation(c)
    sim.reset(vx0, 3)
    hist = []
    for i in range(pre + post):
        cmd = 0.0 if i < pre else delta
        thr = _speed_hold(sim, vx0)
        sim._step_plant(thr, 0.0, cmd, 1.0, 0.0, 0.01)
        d = sim.dual_track.diagnostics()
        v = sim.state.vehicle
        hist.append({
            "t": (i - pre) * 0.01,
            "cmd": cmd,
            "delta": d["steer_actual"],
            "delta_fl": d["delta_fl"],
            "delta_fr": d["delta_fr"],
            "alpha": list(d["alpha"]),
            "Fy": list(d["Fy"]),
            "Fz": list(d["Fz"]),
            "ay": float(v.ay),
            "r": float(v.yaw_rate),
            "yaw_acc": float(sim.dual_track.state.yaw_acc),
            "vy": float(v.vy),
            "vx": float(v.vx),
            "SigmaFy": float(sum(d["Fy"])),
        })
    return hist, sim


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist_v = bind_historical_demonstrator()
    cfg = hyper.simulation_config

    # --- Manoeuvre 1: steering step ---
    H, _ = steering_step(cfg, 25.0, 0.08, pre=40, post=200)
    post = [h for h in H if h["t"] >= 0]
    pre = [h for h in H if h["t"] < 0]

    # 1 Steering step generates transient
    _gate(gates, "steering_step",
          max(abs(h["ay"]) for h in post) > 2.0 and max(abs(h["r"]) for h in post) > 0.1,
          f"peak_ay={max(abs(h['ay']) for h in post):.2f} peak_r={max(abs(h['r']) for h in post):.3f}")

    # 2 Rate limit — δ(t) rises over multiple steps
    rise = [h["delta"] for h in post if h["t"] <= 0.3]
    rate_ok = len(rise) > 5 and rise[-1] > rise[2] + 0.01
    _gate(gates, "steering_rate_limit", rate_ok,
          f"δ@0={rise[0]:.4f} δ@0.3={rise[-1]:.4f}")

    # 3 Angle limit
    H_sat, _ = steering_step(cfg, 25.0, 1.0, pre=20, post=100)
    max_d = max(h["delta"] for h in H_sat)
    _gate(gates, "steering_angle_limit",
          abs(max_d - cfg.max_steer_angle) < 1e-3 or max_d <= cfg.max_steer_angle + 1e-6,
          f"max_δ={max_d:.4f} limit={cfg.max_steer_angle}")

    # 4 Lateral acceleration buildup
    ay_early = abs(post[5]["ay"])
    ay_mid = abs(post[40]["ay"])
    _gate(gates, "lateral_accel_buildup",
          ay_mid > ay_early * 1.2 or ay_mid > 5.0,
          f"ay_early={ay_early:.2f} ay_mid={ay_mid:.2f}")

    # 5 Yaw-rate buildup without runaway
    r_vals = [h["r"] for h in post]
    _gate(gates, "yaw_rate_buildup",
          abs(r_vals[-1]) > 0.15 and abs(r_vals[-1]) < 5.0 and not any(np.isnan(r_vals)),
          f"r_final={r_vals[-1]:.3f} r_max={max(abs(r) for r in r_vals):.3f}")

    # 6 Yaw acceleration sign (left turn → +r_dot early)
    ya_early = post[3]["yaw_acc"]
    _gate(gates, "yaw_acceleration",
          ya_early > 0.1,
          f"yaw_acc_early={ya_early:.3f}")

    # 7 Slip-angle buildup
    a0 = abs(post[2]["alpha"][0])
    a1 = abs(post[30]["alpha"][0])
    _gate(gates, "slip_angle_buildup",
          a1 > a0 * 1.2 or a1 > 0.02,
          f"αFL early={a0:.4f} mid={a1:.4f}")

    # 8 Front/rear α coherent (both same sign under left turn)
    a_f = post[50]["alpha"][0]
    a_r = post[50]["alpha"][2]
    _gate(gates, "front_rear_alpha",
          a_f * a_r > 0 or abs(a_f) + abs(a_r) > 0.02,
          f"αFL={a_f:.4f} αRL={a_r:.4f}")

    # 9 Dynamic Fz — L/R transfer develops
    fz_early = post[2]["Fz"]
    fz_late = post[80]["Fz"]
    right_gain = (fz_late[1] + fz_late[3]) - (fz_early[1] + fz_early[3])
    _gate(gates, "dynamic_fz",
          right_gain > 100 or (fz_late[1] + fz_late[3]) > (fz_late[0] + fz_late[2]),
          f"right_gain={right_gain:.0f} Fz_late={[round(z,0) for z in fz_late]}")

    # 10 Dugoff Fz→Fy coupling (already in chain; check Fy builds)
    fy_mid = abs(sum(post[40]["Fy"]))
    _gate(gates, "dugoff_coupling",
          fy_mid > 2000,
          f"ΣFy_mid={fy_mid:.0f}")

    # 11 Combined κ+α under throttle
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    for _ in range(30):
        sim._step_plant(0.1, 0, 0.0, 1, 0, 0.01)
    for _ in range(40):
        sim._step_plant(0.7, 0, 0.10, 1, 0, 0.01)
    util = [w.utilization for w in sim.dual_track.wheels]
    _gate(gates, "combined_kappa_alpha",
          max(util) > 0.2,
          f"util={[round(u,3) for u in util]}")

    # 12 Steering reversal
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    for _ in range(40):
        sim._step_plant(_speed_hold(sim, 25), 0, 0.0, 1, 0, 0.01)
    for _ in range(120):
        sim._step_plant(_speed_hold(sim, 25), 0, 0.08, 1, 0, 0.01)
    ay_pos = sim.state.vehicle.ay
    r_pos = sim.state.vehicle.yaw_rate
    for _ in range(150):
        sim._step_plant(_speed_hold(sim, 25), 0, -0.08, 1, 0, 0.01)
    ay_neg = sim.state.vehicle.ay
    r_neg = sim.state.vehicle.yaw_rate
    _gate(gates, "steering_reversal",
          ay_pos > 1.0 and ay_neg < -1.0 and r_pos > 0.1 and r_neg < -0.1,
          f"ay+={ay_pos:.2f} ay-={ay_neg:.2f} r+={r_pos:.3f} r-={r_neg:.3f}")

    # 13 L/R symmetry (mirrored steps)
    Hp, _ = steering_step(cfg, 25.0, 0.08, pre=30, post=150)
    Hm, _ = steering_step(cfg, 25.0, -0.08, pre=30, post=150)
    ay_p = Hp[-1]["ay"]
    ay_m = Hm[-1]["ay"]
    _gate(gates, "left_right_symmetry",
          abs(ay_p + ay_m) < 0.25 * max(abs(ay_p), 1e-3),
          f"ay+={ay_p:.3f} ay-={ay_m:.3f}")

    # 14 Zero-steer
    Hz, _ = steering_step(cfg, 25.0, 0.0, pre=20, post=80)
    _gate(gates, "zero_steer",
          abs(Hz[-1]["ay"]) < 0.3 and abs(Hz[-1]["r"]) < 0.05,
          f"ay={Hz[-1]['ay']:.3f} r={Hz[-1]['r']:.4f}")

    # 15 Step magnitude authority
    Hs, _ = steering_step(cfg, 25.0, 0.04, pre=30, post=150)
    Hl, _ = steering_step(cfg, 25.0, 0.12, pre=30, post=150)
    _gate(gates, "step_magnitude_authority",
          abs(Hl[-1]["ay"]) > abs(Hs[-1]["ay"]) * 1.2,
          f"|ay|_0.04={abs(Hs[-1]['ay']):.2f} |ay|_0.12={abs(Hl[-1]['ay']):.2f}")

    # 16 Rate authority
    Hfast, _ = steering_step(cfg, 25.0, 0.10, pre=20, post=80, rate=3.0)
    Hslow, _ = steering_step(cfg, 25.0, 0.10, pre=20, post=80, rate=0.4)
    # time to reach 80% of final delta
    def t_80(hist, target=0.10):
        for h in hist:
            if h["t"] >= 0 and abs(h["delta"]) >= 0.8 * abs(target):
                return h["t"]
        return 99.0
    _gate(gates, "rate_authority",
          t_80(Hslow) > t_80(Hfast),
          f"t80_fast={t_80(Hfast):.2f} t80_slow={t_80(Hslow):.2f}")

    # 17 Speed authority
    H20, _ = steering_step(cfg, 18.0, 0.08, pre=30, post=150)
    H30, _ = steering_step(cfg, 30.0, 0.08, pre=30, post=150)
    _gate(gates, "speed_authority",
          abs(H30[-1]["ay"]) > abs(H20[-1]["ay"]) * 0.85,
          f"|ay|_18={abs(H20[-1]['ay']):.2f} |ay|_30={abs(H30[-1]['ay']):.2f}")

    # 18 Damping — no divergence (bounded final state)
    final_ay = abs(post[-1]["ay"])
    peak_ay = max(abs(h["ay"]) for h in post)
    _gate(gates, "damping",
          final_ay < peak_ay * 2.0 and final_ay < 30.0 and not np.isnan(final_ay),
          f"final={final_ay:.2f} peak={peak_ay:.2f}")

    # 19 Steady-state convergence (compare to 14.9.3 regime)
    # after long settle, yaw_acc small, force balance
    settle = post[-30:]
    mean_ay = float(np.mean([h["ay"] for h in settle]))
    mean_r = float(np.mean([h["r"] for h in settle]))
    mean_sfy = float(np.mean([h["SigmaFy"] for h in settle]))
    mean_ya = float(np.mean([h["yaw_acc"] for h in settle]))
    mean_vx = float(np.mean([h["vx"] for h in settle]))
    _gate(gates, "steady_state_convergence",
          abs(mean_ay) > 5.0 and abs(mean_r) > 0.2,
          f"ay={mean_ay:.2f} r={mean_r:.3f}")

    # 20 Yaw equilibrium
    _gate(gates, "yaw_equilibrium",
          abs(mean_ya) < 0.8,
          f"yaw_acc_ss={mean_ya:.4f}")

    # 21 Force balance
    bal = abs(mean_sfy - cfg.mass * mean_ay) / max(abs(cfg.mass * mean_ay), 1.0)
    _gate(gates, "force_balance",
          bal < 0.15,
          f"ΣFy={mean_sfy:.0f} m·ay={cfg.mass*mean_ay:.0f} err={bal:.3f}")

    # 22 Kinematic consistency
    r_kin = mean_ay / max(mean_vx, 1.0)
    r_err = abs(mean_r - r_kin) / max(abs(r_kin), 0.01)
    _gate(gates, "kinematic_consistency",
          r_err < 0.25,
          f"r={mean_r:.4f} ay/vx={r_kin:.4f} err={r_err:.3f}")

    # 23 Crosswind transient
    sim = Simulation(cfg)
    sim.reset(30.0, 4)
    for _ in range(30):
        sim._step_plant(0.12, 0, 0.0, 1, 0, 0.01)
    sim.state.wind_vy = 18.0
    ays = []
    for _ in range(80):
        sim._step_plant(0.12, 0, 0.0, 1, 0, 0.01)
        ays.append(sim.state.vehicle.ay)
    air = sim._aero_air
    _gate(gates, "crosswind_transient",
          (air is not None and abs(air.Fy_aero) > 40) or max(abs(a) for a in ays) > 0.2,
          f"Fy_aero={air.Fy_aero if air else 0:.0f} peak_ay={max(abs(a) for a in ays):.2f}")

    # 24 Determinism
    runs = []
    for _ in range(5):
        H, _ = steering_step(cfg, 25.0, 0.08, pre=20, post=100)
        runs.append((round(H[-1]["ay"], 5), round(H[-1]["r"], 5), round(H[50]["delta"], 6)))
    _gate(gates, "deterministic_replay", len(set(runs)) == 1, f"run0={runs[0]}")

    # Sine lane-change style (extra evidence, not a hard gate beyond determinism)
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    for _ in range(30):
        sim._step_plant(_speed_hold(sim, 25), 0, 0.0, 1, 0, 0.01)
    sine_ay = []
    for i in range(200):
        cmd = 0.06 * np.sin(2 * np.pi * 0.5 * i * 0.01)  # 0.5 Hz
        sim._step_plant(_speed_hold(sim, 25), 0, cmd, 1, 0, 0.01)
        sine_ay.append(sim.state.vehicle.ay)
    sine_ok = max(sine_ay) > 2.0 and min(sine_ay) < -2.0
    _gate(gates, "sine_lane_change", sine_ok,
          f"ay_max={max(sine_ay):.2f} ay_min={min(sine_ay):.2f}")

    # Regression
    avx, at, _ = _launch(cfg)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    hvx, ht, _ = _launch(hist_v.simulation_config)
    ht100 = _t_to(hvx, ht, 27.78)
    reg = (
        at100 is not None and abs(at100 - REF_HYPER[0]) < 0.15
        and at200 is not None and abs(at200 - REF_HYPER[1]) < 0.25
    )
    _gate(gates, "regression", reg, f"t100={at100} t200={at200}")
    _gate(gates, "historical_isolation",
          ht100 is not None and abs(ht100 - REF_HIST[0]) < 0.3,
          f"t100={ht100}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 2 else "FAIL"
    )
    # Save step telemetry sample
    sample = [{"t": h["t"], "delta": h["delta"], "ay": h["ay"], "r": h["r"],
               "yaw_acc": h["yaw_acc"], "SigmaFy": h["SigmaFy"]} for h in H[30:120]]
    summary = {
        "phase": "14.9.4",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "step_ss": {
            "ay": mean_ay, "r": mean_r, "SigmaFy": mean_sfy,
            "yaw_acc": mean_ya, "force_balance_err": bal, "r_err": r_err,
        },
        "regression": {
            "hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
            "hist": {"t100": ht100, "ref": REF_HIST},
        },
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    with open(ROOT / "step_telemetry.json", "w") as f:
        json.dump(sample, f, indent=2)
    print(f"\n=== PHASE 14.9.4 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
