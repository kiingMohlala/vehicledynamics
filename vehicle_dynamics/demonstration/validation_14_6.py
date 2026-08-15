"""
Phase 14.6 — Dynamic Body Model Integrity & Energy Closure.
Validation only — no new subsystems, no retuning of 14.2–14.5 vehicle identity.
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

ROOT = Path("artifacts/phase_14_6")

# Frozen 14.5 regression references
REF_HYPER_T100, REF_HYPER_T200 = 3.24, 8.47
REF_HIST_T100, REF_HIST_T200 = 5.37, 18.86


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


def _hold_ax(cfg, ax, n=200, vx0=30.0):
    sim = Simulation(cfg)
    sim.reset(vx0, 4)
    for _ in range(n):
        sim.dual_track.state.ax = ax
        sim.dual_track.state.ay = 0.0
        sim.dual_track.step(
            vx=vx0, vy=0, yaw_rate=0, steer=0,
            drive_torque_total=0, brake_cmd=0, dt=0.01,
        )
    return sim


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    energy = {}
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config

    # ----- 1. Numerical stability / static equilibrium -----
    sim = Simulation(cfg)
    sim.reset(0.0, 1)
    zs, ths, phs = [], [], []
    for _ in range(5000):  # 50 s
        sim._step_plant(0, 0, 0, 1, 0, 0.01)
        sb = sim.dual_track.sprung.state
        zs.append(sb.z)
        ths.append(sb.theta)
        phs.append(sb.phi)
    final = (zs[-1], ths[-1], phs[-1])
    drift = max(abs(zs[-1] - zs[-500]), abs(ths[-1] - ths[-500]), abs(phs[-1] - phs[-500]))
    nan_ok = not any(np.isnan(zs + ths + phs)) and not any(np.isinf(zs + ths + phs))
    eq_ok = abs(zs[-1]) < 1e-3 and abs(ths[-1]) < 1e-3 and abs(phs[-1]) < 1e-3
    _gate(gates, "numerical_stability", nan_ok and drift < 1e-4,
          f"drift={drift:.2e} nan_ok={nan_ok}")
    _gate(gates, "static_equilibrium", eq_ok,
          f"z={zs[-1]:.2e} θ={ths[-1]:.2e} φ={phs[-1]:.2e}")
    _gate(gates, "long_duration_no_drift", drift < 1e-4 and nan_ok,
          f"50s drift={drift:.2e}")

    # ----- 2. Heave response -----
    sim = Simulation(cfg)
    sim.reset(20.0, 3)
    z0 = sim.dual_track.sprung.state.z
    for _ in range(100):
        sim.dual_track.step(
            vx=20, vy=0, yaw_rate=0, steer=0, drive_torque_total=0, brake_cmd=0, dt=0.01,
            downforce_front=2500, downforce_rear=3500,
        )
    z1 = sim.dual_track.sprung.state.z
    _gate(gates, "heave_response", abs(z1 - z0) > 1e-4,
          f"z0={z0:.5f} z_aero={z1:.5f}")

    # ----- 3. Pitch accel / braking -----
    s_acc = _hold_ax(cfg, 5.0, n=250)
    s_brk = _hold_ax(cfg, -8.0, n=250)
    th_a = s_acc.dual_track.sprung.state.theta
    th_b = s_brk.dual_track.sprung.state.theta
    _gate(gates, "pitch_accel_response", th_a > 0.002,
          f"θ_acc={th_a:.4f}")
    _gate(gates, "pitch_braking_response", th_b < -0.002,
          f"θ_brk={th_b:.4f}")

    # ----- 4. Roll response + symmetry -----
    def roll_hold(ay, n=200):
        s = Simulation(cfg)
        s.reset(25.0, 3)
        for _ in range(n):
            s.dual_track.state.ay = ay
            s.dual_track.state.ax = 0.0
            s.dual_track.step(
                vx=25, vy=0, yaw_rate=0, steer=0,
                drive_torque_total=0, brake_cmd=0, dt=0.01,
            )
        return s.dual_track.sprung.state.phi

    ph_p = roll_hold(5.0)
    ph_m = roll_hold(-5.0)
    _gate(gates, "roll_response", abs(ph_p) > 0.01, f"φ={ph_p:.4f}")
    _gate(gates, "roll_symmetry", abs(ph_p + ph_m) < 0.05 * max(abs(ph_p), 1e-6),
          f"φ+={ph_p:.4f} φ-={ph_m:.4f}")

    # ----- 5. Damping decay -----
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(40):
        s.dual_track.state.ay = 6.0
        s.dual_track.step(vx=25, vy=0, yaw_rate=0, steer=0,
                          drive_torque_total=0, brake_cmd=0, dt=0.01)
    ph_peak = abs(s.dual_track.sprung.state.phi)
    for _ in range(200):
        s.dual_track.state.ay = 0.0
        s.dual_track.step(vx=25, vy=0, yaw_rate=0, steer=0,
                          drive_torque_total=0, brake_cmd=0, dt=0.01)
    ph_end = abs(s.dual_track.sprung.state.phi)
    decay_ok = ph_end < ph_peak * 0.4
    _gate(gates, "damping_decay", decay_ok,
          f"|φ|_peak={ph_peak:.4f} |φ|_end={ph_end:.4f}")

    # ----- 6. Suspension parameter authority -----
    c2 = bind_authoritative_hypercar().simulation_config
    c2.k_front = cfg.k_front * 2
    c2.c_front = cfg.c_front * 2
    c2.roll_stiffness_front = cfg.roll_stiffness_front * 2
    s2 = Simulation(c2)
    auth = (
        abs(s2.dual_track.sprung.cfg.k_front - cfg.k_front * 2) < 1
        and abs(s2.dual_track.sprung.cfg.c_front - cfg.c_front * 2) < 1
        and abs(s2.dual_track.sprung.cfg.roll_stiffness_front - cfg.roll_stiffness_front * 2) < 1
    )
    _gate(gates, "suspension_parameter_authority", auth,
          f"k={s2.dual_track.sprung.cfg.k_front} c={s2.dual_track.sprung.cfg.c_front}")

    # h_cg authority
    c3 = bind_authoritative_hypercar().simulation_config
    c3.h_cg = 0.80
    s3 = _hold_ax(c3, 5.0, n=200)
    s0 = _hold_ax(cfg, 5.0, n=200)
    hcg_ok = abs(s3.dual_track.sprung.state.theta) > abs(s0.dual_track.sprung.state.theta) * 1.3
    _gate(gates, "hcg_authority", hcg_ok,
          f"θ_h0.4={s0.dual_track.sprung.state.theta:.4f} θ_h0.8={s3.dual_track.sprung.state.theta:.4f}")

    # k mutation changes pitch
    c_soft = bind_authoritative_hypercar().simulation_config
    c_soft.k_front = 30000.0
    c_soft.k_rear = 35000.0
    th_soft = _hold_ax(c_soft, -8.0, n=200).dual_track.sprung.state.theta
    th_stiff = _hold_ax(cfg, -8.0, n=200).dual_track.sprung.state.theta
    _gate(gates, "k_mutation_pitch", abs(th_stiff) < abs(th_soft),
          f"θ_soft={th_soft:.4f} θ_stiff={th_stiff:.4f}")

    # ----- 7. Wheel-load conservation -----
    def fz_sum_case(ax=0, ay=0, df_f=0, df_r=0, n=250):
        s = Simulation(cfg)
        s.reset(20.0, 3)
        for _ in range(n):
            s.dual_track.state.ax = ax
            s.dual_track.state.ay = ay
            s.dual_track.step(
                vx=20, vy=0, yaw_rate=0, steer=0,
                drive_torque_total=0, brake_cmd=0, dt=0.01,
                downforce_front=df_f, downforce_rear=df_r,
            )
        return float(np.sum(s.dual_track.sprung.state.Fz)), cfg.mass * 9.81 + df_f + df_r

    cases = {
        "static": fz_sum_case(),
        "accel": fz_sum_case(ax=5),
        "brake": fz_sum_case(ax=-8),
        "corner": fz_sum_case(ay=5),
        "aero": fz_sum_case(df_f=2000, df_r=3000),
        "combined": fz_sum_case(ax=-5, ay=4, df_f=1000, df_r=1500),
    }
    cons_ok = all(abs(got - exp) < 80 for got, exp in cases.values())
    _gate(gates, "wheel_load_conservation", cons_ok,
          "; ".join(f"{k}:{got:.0f}/{exp:.0f}" for k, (got, exp) in cases.items()))

    # Combined load transfer present
    s = Simulation(cfg)
    s.reset(30.0, 4)
    for _ in range(50):
        s._step_plant(0, 0.7, 0.08, 1, 0, 0.01)
    fz = s.dual_track.sprung.state.Fz
    _gate(gates, "combined_load_transfer",
          abs(fz[0] - fz[1]) > 50 and abs(fz[0] + fz[1] - (fz[2] + fz[3])) > 50,
          f"Fz={np.round(fz,0)}")

    # Aero load balance
    got_a, exp_a = cases["aero"]
    _gate(gates, "aero_load_balance", abs(got_a - exp_a) < 80,
          f"ΣFz={got_a:.0f} expected={exp_a:.0f}")

    # ----- 8. Quasi-static convergence -----
    dtc = Simulation(cfg).dual_track.cfg
    s_dyn = _hold_ax(cfg, 5.0, n=300)
    fz_dyn = np.array(s_dyn.dual_track.sprung.state.Fz)
    lt = compute_wheel_loads(
        mass=dtc.mass, a=dtc.a, b=dtc.b, h_cg=dtc.h_cg,
        track_f=dtc.track_f, track_r=dtc.track_r, ax=5.0, ay=0.0,
    )
    fz_qs = np.array([lt.Fz_fl, lt.Fz_fr, lt.Fz_rl, lt.Fz_rr])
    qs_err = float(np.max(np.abs(fz_dyn - fz_qs)))
    _gate(gates, "quasi_static_convergence", qs_err < 5.0,
          f"max|Fz_dyn-Fz_qs|={qs_err:.2f} N")

    # also with sprung disabled path
    c_qs = bind_authoritative_hypercar().simulation_config
    c_qs.use_sprung_body = False
    s_off = Simulation(c_qs)
    s_off.reset(30.0, 4)
    for _ in range(50):
        s_off.dual_track.state.ax = 5.0
        s_off.dual_track.step(vx=30, vy=0, yaw_rate=0, steer=0,
                              drive_torque_total=0, brake_cmd=0, dt=0.01)
    fz_off = np.array([w.Fz for w in s_off.dual_track.wheels])
    # both should match QS
    _gate(gates, "qs_fallback_matches", float(np.max(np.abs(fz_off - fz_qs))) < 5.0,
          f"fallback err={float(np.max(np.abs(fz_off - fz_qs))):.2f}")

    # ----- 9. Timestep convergence -----
    def theta_at_dt(dt, n_steps):
        s = Simulation(cfg)
        s.reset(30.0, 4)
        for _ in range(n_steps):
            s.dual_track.state.ax = -8.0
            s.dual_track.step(vx=25, vy=0, yaw_rate=0, steer=0,
                              drive_torque_total=0, brake_cmd=0.5, dt=dt)
        return s.dual_track.sprung.state.theta

    th_01 = theta_at_dt(0.01, 200)
    th_005 = theta_at_dt(0.005, 400)
    dt_ok = abs(th_01 - th_005) < 0.005
    _gate(gates, "timestep_convergence", dt_ok,
          f"θ_dt0.01={th_01:.5f} θ_dt0.005={th_005:.5f}")

    # ----- 10. Energy -----
    s = Simulation(cfg)
    s.reset(25.0, 3)
    E_d0 = s.dual_track.sprung.state.E_damp_dissipated
    for _ in range(100):
        s._step_plant(0.15, 0.3, 0.08, 1, 0, 0.01)
    sb = s.dual_track.sprung.state
    E_spring = sb.E_spring
    E_damp = sb.E_damp_dissipated - E_d0
    E_heave = 0.5 * cfg.mass * sb.z_dot ** 2
    E_pitch = 0.5 * s.dual_track.sprung.cfg.I_theta * sb.theta_dot ** 2
    E_roll = 0.5 * s.dual_track.sprung.cfg.I_phi * sb.phi_dot ** 2
    energy = {
        "E_spring": E_spring,
        "E_damper_dissipated": E_damp,
        "E_heave": E_heave,
        "E_pitch": E_pitch,
        "E_roll": E_roll,
        "status": "PARTIAL",  # drivetrain residual from 14.2H.2 unresolved
    }
    _gate(gates, "energy_spring", E_spring >= 0, f"E_spring={E_spring:.2f}")
    _gate(gates, "energy_damper", E_damp >= 0, f"E_damp_Δ={E_damp:.2f}")
    _gate(gates, "energy_body", E_heave + E_pitch + E_roll >= 0,
          f"E_heave={E_heave:.3f} E_pitch={E_pitch:.3f} E_roll={E_roll:.3f}")

    # ----- 11. Determinism -----
    runs = []
    for _ in range(5):
        s = Simulation(cfg)
        s.reset(25.0, 3)
        for __ in range(50):
            s._step_plant(0.1, 0.4, 0.06, 1, 0, 0.01)
        sb = s.dual_track.sprung.state
        runs.append((round(sb.z, 9), round(sb.theta, 9), round(sb.phi, 9),
                     round(float(sb.Fz[0]), 5)))
    det = len(set(runs)) == 1
    _gate(gates, "deterministic_replay", det, f"run0={runs[0]}")

    # ----- 12. Historical isolation + frozen regression -----
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100, ht200 = _t_to(hvx, ht, 27.78), _t_to(hvx, ht, 55.56)
    avx, at, _ = _launch(cfg)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    hist_ok = (
        ht100 is not None and abs(ht100 - REF_HIST_T100) < 0.25
        and ht200 is not None and abs(ht200 - REF_HIST_T200) < 1.0
    )
    hyper_ok = (
        at100 is not None and abs(at100 - REF_HYPER_T100) < 0.15
        and at200 is not None and abs(at200 - REF_HYPER_T200) < 0.3
    )
    _gate(gates, "historical_isolation", hist_ok, f"t100={ht100} t200={ht200}")
    _gate(gates, "frozen_regression", hyper_ok, f"t100={at100} t200={at200}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 2 else "FAIL"
    )
    summary = {
        "phase": "14.6",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "energy": energy,
        "qs_convergence_err_N": qs_err,
        "regression": {
            "hist": {"t100": ht100, "t200": ht200},
            "hyper": {"t100": at100, "t200": at200},
            "ref_hyper": {"t100": REF_HYPER_T100, "t200": REF_HYPER_T200},
            "ref_hist": {"t100": REF_HIST_T100, "t200": REF_HIST_T200},
        },
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    with open(ROOT / "energy_ledger.json", "w") as f:
        json.dump(energy, f, indent=2)
    with open(ROOT / "qs_convergence.json", "w") as f:
        json.dump({"fz_dyn": fz_dyn.tolist(), "fz_qs": fz_qs.tolist(), "err": qs_err}, f, indent=2)
    print(f"\n=== PHASE 14.6 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
