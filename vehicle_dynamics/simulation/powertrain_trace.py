"""
Phase 14.2D — full torque-chain instrumentation.

Every sample records the authoritative path:
  engine → clutch → gearbox → differential split → wheels → Dugoff Fx → m·ax
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Optional
import json
import hashlib
import numpy as np


@dataclass
class TorqueChainSample:
    t: float = 0.0
    vx: float = 0.0
    ax: float = 0.0
    engine_rpm: float = 0.0
    engine_torque_nm: float = 0.0
    engine_power_kw: float = 0.0
    clutch_engagement: float = 0.0
    clutch_torque_nm: float = 0.0
    clutch_locked: bool = False
    clutch_slip_rad_s: float = 0.0
    gear: int = 0
    gear_ratio: float = 0.0
    final_drive: float = 0.0
    overall_ratio: float = 0.0
    transmission_efficiency: float = 0.95
    gearbox_wheel_torque_nm: float = 0.0
    T_fl: float = 0.0
    T_fr: float = 0.0
    T_rl: float = 0.0
    T_rr: float = 0.0
    Fx_fl: float = 0.0
    Fx_fr: float = 0.0
    Fx_rl: float = 0.0
    Fx_rr: float = 0.0
    kappa_fl: float = 0.0
    kappa_fr: float = 0.0
    kappa_rl: float = 0.0
    kappa_rr: float = 0.0
    Fx_sum: float = 0.0
    m_ax: float = 0.0
    residual_Fx: float = 0.0  # Fx_sum - m*ax (should ~ drag+rolling)


@dataclass
class PowertrainTrace:
    samples: list[TorqueChainSample] = field(default_factory=list)
    mass: float = 1400.0
    cfg_hash: str = ""

    def append(self, s: TorqueChainSample) -> None:
        self.samples.append(s)

    def stage_power_kw(self, idx: int = -1) -> dict[str, float]:
        """Instantaneous mechanical power at each stage (from telemetry)."""
        if not self.samples:
            return {}
        s = self.samples[idx]
        omega_e = s.engine_rpm * 2.0 * np.pi / 60.0
        P_eng = max(s.engine_torque_nm, 0.0) * omega_e / 1000.0
        # Clutch output ≈ clutch torque * gearbox input speed ≈ engine when locked
        P_clutch = max(s.clutch_torque_nm, 0.0) * omega_e / 1000.0 if s.clutch_locked else max(s.clutch_torque_nm, 0.0) * omega_e / 1000.0
        omega_w = s.vx / 0.32 if s.vx > 0.1 else 0.0
        P_gb = max(s.gearbox_wheel_torque_nm, 0.0) * omega_w / 1000.0
        P_tire = max(s.Fx_sum, 0.0) * max(s.vx, 0.0) / 1000.0
        P_veh = max(s.m_ax * s.vx, 0.0) / 1000.0 if s.vx > 0 else 0.0
        return {
            "engine_kw": float(P_eng),
            "clutch_kw": float(P_clutch),
            "gearbox_kw": float(P_gb),
            "tire_kw": float(P_tire),
            "vehicle_kw": float(P_veh),
        }

    def mean_stage_powers(self, t0: float = 1.0, t1: float = 10.0) -> dict[str, float]:
        acc = {k: [] for k in ("engine_kw", "clutch_kw", "gearbox_kw", "tire_kw", "vehicle_kw")}
        for i, s in enumerate(self.samples):
            if t0 <= s.t <= t1 and s.vx > 1.0:
                p = self.stage_power_kw(i)
                for k, v in p.items():
                    acc[k].append(v)
        return {k: float(np.mean(v)) if v else 0.0 for k, v in acc.items()}

    def to_json(self, path: str) -> None:
        payload = {
            "cfg_hash": self.cfg_hash,
            "mass": self.mass,
            "n_samples": len(self.samples),
            "samples": [asdict(s) for s in self.samples[:: max(1, len(self.samples)//500)]],
        }
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)


def capture_from_simulation(sim, duration: float = 25.0, thr: float = 1.0) -> PowertrainTrace:
    """Run full-throttle launch and capture torque chain at every step."""
    cfg = sim.cfg
    mass = float(cfg.mass)
    r = float(cfg.wheel_radius)
    h = hashlib.sha1(
        f"{mass}:{cfg.peak_torque_nm}:{cfg.peak_power_kw}:{cfg.final_drive}:{cfg.mu_tire}".encode()
    ).hexdigest()[:12]
    trace = PowertrainTrace(mass=mass, cfg_hash=h)
    sim.reset(vx=0.0, gear=1)
    n = int(duration / cfg.dt)
    for _ in range(n):
        sim._step_plant(thr=thr, brk=0.0, steer=0.0, tlim=1.0, tv=0.0, dt=cfg.dt)
        v = sim.state.vehicle
        trc = getattr(sim, "_trace", {})
        diag = getattr(sim, "_dual_diag", {})
        fx = diag.get("Fx", [0, 0, 0, 0])
        kap = diag.get("kappa", [0, 0, 0, 0])
        wheels = getattr(sim.dual_track, "wheels", None) if sim.dual_track else None
        T_w = [0.0, 0.0, 0.0, 0.0]
        if wheels is not None:
            T_w = [float(w.drive_torque) for w in wheels]
        try:
            gear = int(v.gear)
            overall = float(sim.trans.gearbox.ratios.overall(gear)) if gear else 0.0
            gr = float(sim.trans.gearbox.ratios.ratio(gear)) if gear else 0.0
            fd = float(sim.trans.gearbox.ratios.final_drive)
            eff = float(sim.trans.gearbox.ratios.efficiency)
        except Exception:
            overall = gr = fd = 0.0
            eff = 0.95
        omega_e = float(v.engine_rpm) * 2.0 * np.pi / 60.0
        s = TorqueChainSample(
            t=float(sim.state.time),
            vx=float(v.vx),
            ax=float(v.ax),
            engine_rpm=float(v.engine_rpm),
            engine_torque_nm=float(trc.get("engine_torque_nm", 0.0)),
            engine_power_kw=float(max(trc.get("engine_torque_nm", 0.0), 0.0) * omega_e / 1000.0),
            clutch_engagement=float(getattr(sim.trans.state, "clutch_engagement", 0.0)),
            clutch_torque_nm=float(trc.get("clutch_torque_nm", 0.0)),
            clutch_locked=bool(getattr(sim.trans.state, "locked", False)),
            clutch_slip_rad_s=float(getattr(sim.trans.state, "clutch_slip", 0.0)),
            gear=gear,
            gear_ratio=gr,
            final_drive=fd,
            overall_ratio=overall,
            transmission_efficiency=eff,
            gearbox_wheel_torque_nm=float(trc.get("gearbox_wheel_torque_nm", 0.0)),
            T_fl=T_w[0], T_fr=T_w[1], T_rl=T_w[2], T_rr=T_w[3],
            Fx_fl=float(fx[0]), Fx_fr=float(fx[1]), Fx_rl=float(fx[2]), Fx_rr=float(fx[3]),
            kappa_fl=float(kap[0]), kappa_fr=float(kap[1]),
            kappa_rl=float(kap[2]), kappa_rr=float(kap[3]),
            Fx_sum=float(trc.get("Fx_tire_N", 0.0)),
            m_ax=float(v.ax) * mass,
            residual_Fx=float(trc.get("Fx_tire_N", 0.0)) - float(v.ax) * mass,
        )
        trace.append(s)
    return trace
