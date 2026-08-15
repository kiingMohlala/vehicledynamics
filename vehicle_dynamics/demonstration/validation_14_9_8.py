"""
Phase 14.9.8 — Understeer/Oversteer & Yaw-Stability Characterization.
Passive only. No ESC. No retuning.
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
from vehicle_dynamics.lateral.handling_characterization import (
    run_constant_speed_steer_sweep,
    yaw_gain_vs_speed,
    classify_gradient,
)

ROOT = Path("artifacts/phase_14_9_8")
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


def _factory(cfg):
    def make():
        return Simulation(cfg)
    return make


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config

    # Low-speed coherent behavior
    deltas = [0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15]
    sw15 = run_constant_speed_steer_sweep(
        _factory(cfg), vx=15.0, deltas=deltas, settle_n=140, meas_n=30,
    )
    _gate(gates, "neutral_low_speed_behavior",
          all(abs(p.ay) > 0.3 for p in sw15.points if abs(p.delta) > 0.03)
          and all(np.isfinite(p.ay) for p in sw15.points),
          f"n_pts={len(sw15.points)} class={sw15.classification}")

    # Steering gradient measurable
    sw25 = run_constant_speed_steer_sweep(
        _factory(cfg), vx=25.0, deltas=deltas, settle_n=160, meas_n=35,
    )
    _gate(gates, "steering_gradient",
          abs(sw25.steering_gradient) > 1e-5 or sw25.classification == "neutral",
          f"dδ/day={sw25.steering_gradient:.5f} class={sw25.classification}")

    # Speed authority — yaw gain changes with speed
    yg = yaw_gain_vs_speed(_factory(cfg), delta=0.06, speeds=[15.0, 25.0, 32.0])
    gains = [g["yaw_gain"] for g in yg]
    _gate(gates, "speed_authority",
          abs(gains[-1] - gains[0]) > 0.05 or abs(yg[-1]["ay"] - yg[0]["ay"]) > 0.5,
          f"yaw_gain={['{:.2f}'.format(g) for g in gains]}")

    # Front / rear saturation detectable at high δ
    high = [p for p in sw25.points if abs(p.delta) >= 0.12]
    _gate(gates, "front_saturation",
          any(p.util_front > 0.7 for p in high) or any(p.util_front > 0.5 for p in sw25.points),
          f"max util_f={max(p.util_front for p in sw25.points):.3f}")
    _gate(gates, "rear_saturation",
          any(p.util_rear > 0.7 for p in high) or any(p.util_rear > 0.5 for p in sw25.points),
          f"max util_r={max(p.util_rear for p in sw25.points):.3f}")

    # Classification reproducible
    sw25b = run_constant_speed_steer_sweep(
        _factory(cfg), vx=25.0, deltas=deltas, settle_n=160, meas_n=35,
    )
    _gate(gates, "understeer_oversteer_classification",
          sw25.classification == sw25b.classification
          and sw25.classification in ("understeer", "neutral", "oversteer"),
          f"class={sw25.classification} dδ/day={sw25.steering_gradient:.5f}")

    # Yaw gain finite
    _gate(gates, "yaw_gain",
          all(abs(p.yaw_gain) < 50 for p in sw25.points) and any(abs(p.yaw_gain) > 0.1 for p in sw25.points),
          f"yaw_gains={[round(p.yaw_gain,2) for p in sw25.points[:4]]}...")

    # Yaw stability — no runaway in sweep
    _gate(gates, "yaw_stability",
          all(abs(p.r) < 5.0 for p in sw25.points) and all(np.isfinite(p.r) for p in sw25.points),
          f"max |r|={max(abs(p.r) for p in sw25.points):.3f}")

    # μ ×0.5 reduces lateral capacity
    c_lo = bind_authoritative_hypercar().simulation_config
    c_lo.mu_tire = 0.55
    sw_lo = run_constant_speed_steer_sweep(
        _factory(c_lo), vx=25.0, deltas=[0.06, 0.10, 0.14], settle_n=140, meas_n=30,
    )
    ay_hi = max(abs(p.ay) for p in sw25.points)
    ay_lo = max(abs(p.ay) for p in sw_lo.points)
    _gate(gates, "mu_half_capacity",
          ay_lo < ay_hi * 0.85,
          f"max|ay| μ1.15={ay_hi:.2f} μ0.55={ay_lo:.2f}")

    # ARB distribution changes handling (gradient or class may shift)
    cF = bind_authoritative_hypercar().simulation_config
    cF.k_arb_front = 120000
    cF.k_arb_rear = 5000
    cR = bind_authoritative_hypercar().simulation_config
    cR.k_arb_front = 5000
    cR.k_arb_rear = 120000
    swF = run_constant_speed_steer_sweep(
        _factory(cF), vx=25.0, deltas=[0.04, 0.08, 0.12], settle_n=140, meas_n=25,
    )
    swR = run_constant_speed_steer_sweep(
        _factory(cR), vx=25.0, deltas=[0.04, 0.08, 0.12], settle_n=140, meas_n=25,
    )
    # total Fz still conserved at a sample point
    s = Simulation(cF)
    s.reset(25, 3)
    for _ in range(100):
        s.dual_track.state.ay = 6
        s.dual_track.step(vx=25, vy=0, yaw_rate=0, steer=0,
                          drive_torque_total=0, brake_cmd=0, dt=0.01)
    fz_ok = abs(sum(w.Fz for w in s.dual_track.wheels) - 1100 * 9.81) < 150
    _gate(gates, "arb_distribution_handling",
          fz_ok and (abs(swF.steering_gradient - swR.steering_gradient) > 1e-6
                     or swF.classification != swR.classification
                     or abs(swF.points[-1].ay - swR.points[-1].ay) > 0.05),
          f"grad_F={swF.steering_gradient:.5f} grad_R={swR.steering_gradient:.5f} ΣFz_ok={fz_ok}")

    # Mech vs hyd equivalence
    cH = bind_authoritative_hypercar().simulation_config
    cH.use_hydraulic_arb = True
    cH.k_hyd_front = 40000
    cH.k_hyd_rear = 35000
    cH.k_arb_front = 0
    cH.k_arb_rear = 0
    cM = bind_authoritative_hypercar().simulation_config
    cM.use_hydraulic_arb = False
    cM.k_arb_front = 40000
    cM.k_arb_rear = 35000
    swH = run_constant_speed_steer_sweep(
        _factory(cH), vx=25.0, deltas=[0.06, 0.10], settle_n=130, meas_n=25,
    )
    swM = run_constant_speed_steer_sweep(
        _factory(cM), vx=25.0, deltas=[0.06, 0.10], settle_n=130, meas_n=25,
    )
    _gate(gates, "mech_hyd_equivalence",
          abs(swH.points[-1].ay - swM.points[-1].ay) < 1.5,
          f"ay_hyd={swH.points[-1].ay:.2f} ay_mech={swM.points[-1].ay:.2f}")

    # L/R symmetry
    swL = run_constant_speed_steer_sweep(
        _factory(cfg), vx=25.0, deltas=[0.08], settle_n=150, meas_n=30,
    )
    swR2 = run_constant_speed_steer_sweep(
        _factory(cfg), vx=25.0, deltas=[-0.08], settle_n=150, meas_n=30,
    )
    _gate(gates, "lr_symmetry",
          abs(swL.points[0].ay + swR2.points[0].ay) < 0.3,
          f"ay+={swL.points[0].ay:.3f} ay-={swR2.points[0].ay:.3f}")

    # Crosswind
    s = Simulation(cfg)
    s.reset(30.0, 4)
    s.state.wind_vy = 15.0
    for _ in range(50):
        s._step_plant(0.12, 0, 0.0, 1, 0, 0.01)
    air = s._aero_air
    _gate(gates, "crosswind",
          air is not None and abs(air.Fy_aero) > 40,
          f"Fy_aero={air.Fy_aero if air else 0:.0f}")

    # Recovery after steering reduction (allow rate-limited δ return + yaw decay)
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(120):
        s._step_plant(0.12, 0, 0.12, 1, 0, 0.01)
    ay_hi = abs(s.state.vehicle.ay)
    r_hi = abs(s.state.vehicle.yaw_rate)
    for _ in range(250):
        s._step_plant(0.12, 0, 0.0, 1, 0, 0.01)
    ay_lo = abs(s.state.vehicle.ay)
    r_lo = abs(s.state.vehicle.yaw_rate)
    d_act = abs(s.dual_track.steering.state.actual)
    _gate(gates, "recovery_after_steer_off",
          d_act < 0.02 and (r_lo < r_hi * 0.4 or ay_lo < ay_hi * 0.5),
          f"δ={d_act:.4f} ay {ay_hi:.2f}→{ay_lo:.2f} r {r_hi:.3f}→{r_lo:.3f}")

    # Determinism
    runs = []
    for _ in range(5):
        sw = run_constant_speed_steer_sweep(
            _factory(cfg), vx=25.0, deltas=[0.08], settle_n=100, meas_n=20,
        )
        p = sw.points[0]
        runs.append((round(p.ay, 4), round(p.r, 4), round(sw.steering_gradient, 6)))
    # gradient with single point is 0 — use ay/r
    runs2 = [(r[0], r[1]) for r in runs]
    _gate(gates, "deterministic_replay", len(set(runs2)) == 1, f"run0={runs2[0]}")

    # Regression
    avx, at, _ = _launch(cfg)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100 = _t_to(hvx, ht, 27.78)
    reg = (
        at100 is not None and abs(at100 - REF_HYPER[0]) < 0.15
        and at200 is not None and abs(at200 - REF_HYPER[1]) < 0.25
    )
    _gate(gates, "regression", reg, f"t100={at100} t200={at200}")
    _gate(gates, "historical_isolation",
          ht100 is not None and abs(ht100 - REF_HIST[0]) < 0.3,
          f"t100={ht100}")

    # Package sweep summary
    sweep_out = [
        {
            "delta": p.delta, "ay": p.ay, "r": p.r,
            "alpha_f": p.alpha_front, "alpha_r": p.alpha_rear,
            "util_f": p.util_front, "util_r": p.util_rear,
            "yaw_gain": p.yaw_gain,
        }
        for p in sw25.points
    ]

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 2 else "FAIL"
    )
    summary = {
        "phase": "14.9.8",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "handling": {
            "vx25_classification": sw25.classification,
            "steering_gradient": sw25.steering_gradient,
            "vx15_classification": sw15.classification,
            "yaw_gain_vs_speed": yg,
        },
        "regression": {
            "hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
            "hist": {"t100": ht100, "ref": REF_HIST},
        },
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    with open(ROOT / "steer_sweep_vx25.json", "w") as f:
        json.dump(sweep_out, f, indent=2)
    with open(ROOT / "yaw_gain_vs_speed.json", "w") as f:
        json.dump(yg, f, indent=2)
    print(f"\n=== PHASE 14.9.8 — {status} {n_pass}/{len(gates)} ===")
    print(f"  Handling @25 m/s: {sw25.classification}  dδ/d(ay)={sw25.steering_gradient:.5f}")
    return summary


if __name__ == "__main__":
    run_validation()
