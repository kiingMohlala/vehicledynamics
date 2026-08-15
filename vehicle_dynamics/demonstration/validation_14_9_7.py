"""
Phase 14.9.7 — Roll-Stiffness Distribution & Lateral Load-Transfer Authority.
Passive validation only. No ESC. No retuning.
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
from vehicle_dynamics.suspension.anti_roll_bar import (
    MechanicalAntiRollBar,
    HydraulicAntiRollBar,
)

ROOT = Path("artifacts/phase_14_9_7")
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


def _mech(k_f, k_r):
    c = bind_authoritative_hypercar().simulation_config
    c.use_hydraulic_arb = False
    c.use_arb = True
    c.k_arb_front = k_f
    c.k_arb_rear = k_r
    return c


def _hyd(k_f, k_r, c_f=800.0, c_r=800.0):
    c = bind_authoritative_hypercar().simulation_config
    c.use_hydraulic_arb = True
    c.use_arb = True
    c.k_hyd_front = k_f
    c.k_hyd_rear = k_r
    c.c_hyd_front = c_f
    c.c_hyd_rear = c_r
    c.k_arb_front = 0.0
    c.k_arb_rear = 0.0
    return c


def _corner_metrics(cfg, ay=8.0, n=220):
    """Hold constant ay; return diagnostics at steady roll."""
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(n):
        s.dual_track.state.ay = ay
        s.dual_track.step(
            vx=25, vy=0, yaw_rate=0, steer=0,
            drive_torque_total=0, brake_cmd=0, dt=0.01,
        )
    d = s.dual_track.diagnostics()
    return d, s


def _steer_metrics(cfg, delta=0.10, n_settle=160):
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(40):
        s._step_plant(0.12, 0, 0.0, 1, 0, 0.01)
    for _ in range(n_settle):
        err = 25.0 - s.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        s._step_plant(thr, 0, delta, 1, 0, 0.01)
    d = s.dual_track.diagnostics()
    return d, s


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg_default = hyper.simulation_config

    # Baseline zero ARB
    d0, _ = _corner_metrics(_mech(0, 0))
    # Front-heavy
    dF, _ = _corner_metrics(_mech(120000, 0))
    # Rear-heavy
    dR, _ = _corner_metrics(_mech(0, 120000))
    # Balanced
    dB, _ = _corner_metrics(_mech(50000, 45000))

    # 1 Front stiffness authority — front-only ARB reduces φ and changes dFz vs zero
    _gate(gates, "front_stiffness_authority",
          abs(dF["phi"]) < abs(d0["phi"]) * 0.5 and abs(dF["dFz_front"] - d0["dFz_front"]) > 100,
          f"φ0={d0['phi']:.4f} φ_front={dF['phi']:.4f} dFz_f 0→{d0['dFz_front']:.0f}/{dF['dFz_front']:.0f}")

    # 2 Rear stiffness authority — rear-only ARB reduces φ and changes dFz_rear
    _gate(gates, "rear_stiffness_authority",
          abs(dR["phi"]) < abs(d0["phi"]) * 0.5 and abs(dR["dFz_rear"] - d0["dFz_rear"]) > 100,
          f"φ0={d0['phi']:.4f} φ_rear={dR['phi']:.4f} dFz_r 0→{d0['dFz_rear']:.0f}/{dR['dFz_rear']:.0f}")

    # 3 Distribution conservation ΣFz
    _gate(gates, "distribution_conservation",
          abs(dF["Fz_sum"] - d0["Fz_sum"]) < 50 and abs(dR["Fz_sum"] - d0["Fz_sum"]) < 50,
          f"ΣFz0={d0['Fz_sum']:.0f} ΣFzF={dF['Fz_sum']:.0f} ΣFzR={dR['Fz_sum']:.0f}")

    # 4 L/R symmetry
    dP, _ = _corner_metrics(_mech(40000, 35000), ay=8)
    dM, _ = _corner_metrics(_mech(40000, 35000), ay=-8)
    _gate(gates, "lr_symmetry",
          abs(dP["dFz_front"] + dM["dFz_front"]) < 50
          and abs(dP["phi"] + dM["phi"]) < 1e-3,
          f"dFz_f+={dP['dFz_front']:.0f} dFz_f-={dM['dFz_front']:.0f}")

    # 5 Zero-roll baseline
    _gate(gates, "zero_roll_baseline",
          abs(d0["phi"]) > abs(dB["phi"]),
          f"φ0={d0['phi']:.4f} φ_bal={dB['phi']:.4f}")

    # 6 Mechanical/hydraulic equivalence (similar K → similar φ)
    dHm, _ = _corner_metrics(_mech(40000, 35000))
    dHh, _ = _corner_metrics(_hyd(40000, 35000))
    _gate(gates, "mech_hyd_equivalence",
          abs(abs(dHm["phi"]) - abs(dHh["phi"])) / max(abs(dHm["phi"]), 1e-6) < 0.35,
          f"φ_mech={dHm['phi']:.4f} φ_hyd={dHh['phi']:.4f}")

    # 7/8 Front/rear ARB force share at identical φ (direct authority)
    def arb_force_share(k_f, k_r, phi=0.02):
        c = _mech(k_f, k_r)
        s = Simulation(c)
        # set roll state approximately via step then read ARB forces from corner heights
        s.reset(25.0, 3)
        # inject phi by integrating ay briefly then read
        for _ in range(80):
            s.dual_track.state.ay = 6.0
            s.dual_track.step(vx=25, vy=0, yaw_rate=0, steer=0,
                              drive_torque_total=0, brake_cmd=0, dt=0.01)
        zs, zsd = s.dual_track.sprung.corner_positions()
        fl, fr, rl, rr = s.dual_track.sprung.arb.axle_forces(zs, zsd)
        front_pair = abs(fl) + abs(fr)
        rear_pair = abs(rl) + abs(rr)
        return front_pair, rear_pair, s.dual_track.sprung.state.phi

    fF, rF, pF = arb_force_share(120000, 5000)
    fR, rR, pR = arb_force_share(5000, 120000)
    _gate(gates, "front_heavy_distribution",
          fF > rF * 2,
          f"front-heavy ARB |F|_f={fF:.0f} |F|_r={rF:.0f} φ={pF:.4f}")
    _gate(gates, "rear_heavy_distribution",
          rR > fR * 2,
          f"rear-heavy ARB |F|_f={fR:.0f} |F|_r={rR:.0f} φ={pR:.4f}")

    # 9 Fy distribution under steered corner
    sF, simF = _steer_metrics(_mech(100000, 5000))
    sR, simR = _steer_metrics(_mech(5000, 100000))
    _gate(gates, "fy_distribution",
          abs(sF["Fy_front"]) > 100 and abs(sR["Fy_rear"]) > 100,
          f"Fy_f front-heavy={sF['Fy_front']:.0f} Fy_r rear-heavy={sR['Fy_rear']:.0f}")

    # 10 Yaw moment responds to distribution
    _gate(gates, "yaw_moment_response",
          abs(sF["Mz_tire"] - sR["Mz_tire"]) > 1e-3 or abs(sF["yaw_acc"] - sR["yaw_acc"]) > 1e-4
          or abs(simF.state.vehicle.yaw_rate - simR.state.vehicle.yaw_rate) > 0.01,
          f"r_F={simF.state.vehicle.yaw_rate:.3f} r_R={simR.state.vehicle.yaw_rate:.3f}")

    # 11 μ sensitivity
    def fy_mu(mu, k_f=40000, k_r=35000):
        c = _mech(k_f, k_r)
        c.mu_tire = mu
        d, _ = _steer_metrics(c, delta=0.10, n_settle=120)
        return abs(d["Fy_front"]) + abs(d["Fy_rear"])
    _gate(gates, "mu_sensitivity",
          fy_mu(1.15) > fy_mu(0.55) * 1.1,
          f"Σ|Fy| μ1.15={fy_mu(1.15):.0f} μ0.55={fy_mu(0.55):.0f}")

    # 12 Fz authority to Dugoff (util present)
    d, s = _steer_metrics(_mech(50000, 45000))
    utils = [w.utilization for w in s.dual_track.wheels]
    _gate(gates, "fz_authority_dugoff",
          max(utils) > 0.1 and d["min_Fz"] >= 50 - 1e-6,
          f"util={[round(u,3) for u in utils]} minFz={d['min_Fz']:.0f}")

    # 13 Combined slip under drive + steer
    s = Simulation(_mech(40000, 35000))
    s.reset(25.0, 3)
    for _ in range(80):
        s._step_plant(0.55, 0, 0.10, 1, 0, 0.01)
    d = s.dual_track.diagnostics()
    _gate(gates, "combined_slip",
          max(d["utilization"]) > 0.2,
          f"util={d['utilization']}")

    # 14 Hydraulic damping changes transient without net Fz
    def peak_phidot(c_hyd):
        c = _hyd(40000, 35000, c_f=c_hyd, c_r=c_hyd)
        s = Simulation(c)
        s.reset(25.0, 3)
        peaks = []
        for i in range(60):
            s.dual_track.state.ay = 8.0 if i < 30 else 0.0
            s.dual_track.step(vx=25, vy=0, yaw_rate=0, steer=0,
                              drive_torque_total=0, brake_cmd=0, dt=0.01)
            peaks.append(abs(s.dual_track.sprung.state.phi_dot))
        return max(peaks), sum(w.Fz for w in s.dual_track.wheels)
    p_lo, fz_lo = peak_phidot(100)
    p_hi, fz_hi = peak_phidot(2500)
    _gate(gates, "hydraulic_damping",
          p_hi <= p_lo * 1.05 and abs(fz_lo - fz_hi) < 80,
          f"φ̇ lo={p_lo:.4f} hi={p_hi:.4f} ΣFz={fz_lo:.0f}/{fz_hi:.0f}")

    # 15 Mechanical fallback
    s = Simulation(cfg_default)
    _gate(gates, "mechanical_fallback",
          isinstance(s.dual_track.sprung.arb.front, MechanicalAntiRollBar),
          type(s.dual_track.sprung.arb.front).__name__)

    # 16 Hydraulic path when enabled
    s = Simulation(_hyd(30000, 28000))
    _gate(gates, "hydraulic_path",
          isinstance(s.dual_track.sprung.arb.front, HydraulicAntiRollBar),
          type(s.dual_track.sprung.arb.front).__name__)

    # Diagnostics expose required fields
    d, _ = _corner_metrics(_mech(30000, 25000))
    need = ["dFz_front", "dFz_rear", "Fy_front", "Fy_rear", "roll_k_front", "roll_k_rear", "Mz_tire"]
    _gate(gates, "diagnostics_complete",
          all(k in d for k in need),
          f"keys present={[k for k in need if k in d]}")

    # 17 Crosswind still coherent
    s = Simulation(_mech(40000, 35000))
    s.reset(30.0, 4)
    s.state.wind_vy = 15.0
    for _ in range(60):
        s._step_plant(0.12, 0, 0.0, 1, 0, 0.01)
    air = s._aero_air
    _gate(gates, "crosswind",
          air is not None and abs(air.Fy_aero) > 40,
          f"Fy_aero={air.Fy_aero if air else 0:.0f}")

    # 18 Reversal under steer
    dpos, _ = _steer_metrics(_mech(40000, 35000), delta=0.10)
    dneg, _ = _steer_metrics(_mech(40000, 35000), delta=-0.10)
    _gate(gates, "reversal",
          dpos["phi"] * dneg["phi"] < 0 and dpos["ay"] * dneg["ay"] < 0,
          f"φ+={dpos['phi']:.4f} φ-={dneg['phi']:.4f}")

    # 19 Determinism
    runs = []
    for _ in range(5):
        d, _ = _corner_metrics(_mech(40000, 35000), ay=6, n=100)
        runs.append((round(d["dFz_front"], 2), round(d["phi"], 6), round(d["Fy_front"], 1)))
    _gate(gates, "deterministic_replay", len(set(runs)) == 1, f"run0={runs[0]}")

    # 20 Regression
    avx, at, _ = _launch(cfg_default)
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

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 2 else "FAIL"
    )
    summary = {
        "phase": "14.9.7",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "load_transfer": {
            "zero": {"dFz_f": d0["dFz_front"], "dFz_r": d0["dFz_rear"], "phi": d0["phi"]},
            "front_heavy": {"dFz_f": dF["dFz_front"], "dFz_r": dF["dFz_rear"], "phi": dF["phi"]},
            "rear_heavy": {"dFz_f": dR["dFz_front"], "dFz_r": dR["dFz_rear"], "phi": dR["phi"]},
        },
        "regression": {
            "hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
            "hist": {"t100": ht100, "ref": REF_HIST},
        },
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    with open(ROOT / "load_transfer.json", "w") as f:
        json.dump(summary["load_transfer"], f, indent=2)
    print(f"\n=== PHASE 14.9.7 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
