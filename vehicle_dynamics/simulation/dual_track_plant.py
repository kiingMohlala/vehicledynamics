"""
Four-wheel dual-track plant using authoritative Dugoff tires + wheel dynamics + ABS.

FL=0, FR=1, RL=2, RR=3

Phase 14.2C plant core. Simulation._step_plant delegates here when dual-track is enabled.
Dugoff is the sole source of Fx/Fy — no μFz proxy in this path.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

from vehicle_dynamics.tire.dugoff import DugoffTire, DugoffParams, TireState
from vehicle_dynamics.controls.abs_controller import ABSController
from vehicle_dynamics.controls.sensor_model import SensorReading
from vehicle_dynamics.lateral.load_transfer import (
    LoadTransferParameters,
    compute_load_transfer,
)


@dataclass
class WheelState:
    omega: float = 0.0
    radius: float = 0.33
    inertia: float = 1.8
    drive_torque: float = 0.0
    brake_torque: float = 0.0
    Fz: float = 3000.0
    kappa: float = 0.0
    alpha: float = 0.0
    Fx: float = 0.0
    Fy: float = 0.0
    utilization: float = 0.0
    mu: float = 1.15
    steer: float = 0.0


@dataclass
class DualTrackConfig:
    mass: float = 1100.0
    Iz: float = 2200.0
    a: float = 1.25
    b: float = 1.45
    track_f: float = 1.65
    track_r: float = 1.62
    h_cg: float = 0.45
    wheel_radius: float = 0.33
    wheel_inertia: float = 1.8
    brake_torque_max: float = 2800.0
    drive_split_front: float = 0.35
    mu: float = 1.15
    tire_cx: float = 100000.0
    tire_cy: float = 90000.0
    abs_enabled: bool = True
    v_eps: float = 0.5


@dataclass
class DualTrackState:
    wheels: list = field(default_factory=list)
    ax: float = 0.0
    ay: float = 0.0
    yaw_acc: float = 0.0
    abs_active: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=bool))
    brake_pressure: np.ndarray = field(default_factory=lambda: np.zeros(4))
    Fz_sum: float = 0.0
    residual_Fx: float = 0.0
    residual_Fy: float = 0.0
    residual_Mz: float = 0.0
    residual_Fz: float = 0.0

    def as_arrays(self) -> dict:
        return {
            "omega": np.array([w.omega for w in self.wheels]),
            "kappa": np.array([w.kappa for w in self.wheels]),
            "alpha": np.array([w.alpha for w in self.wheels]),
            "Fx": np.array([w.Fx for w in self.wheels]),
            "Fy": np.array([w.Fy for w in self.wheels]),
            "Fz": np.array([w.Fz for w in self.wheels]),
            "util": np.array([w.utilization for w in self.wheels]),
        }


class DualTrackPlant:
    """Authoritative four-wheel plant: Dugoff + wheel spin + ABS + load transfer."""

    def __init__(self, cfg: Optional[DualTrackConfig] = None):
        self.cfg = cfg or DualTrackConfig()
        self.tires = [
            DugoffTire(
                DugoffParams(
                    mu=self.cfg.mu,
                    Cx=self.cfg.tire_cx,
                    Cy=self.cfg.tire_cy,
                    radius=self.cfg.wheel_radius,
                    v_eps=self.cfg.v_eps,
                )
            )
            for _ in range(4)
        ]
        self.abs = ABSController(enabled=self.cfg.abs_enabled)
        self.lt_params = LoadTransferParameters(
            h_cg=self.cfg.h_cg,
            track_f=self.cfg.track_f,
            track_r=self.cfg.track_r,
            chi_f=0.55,
            Fz_min=50.0,
        )
        self.wheels = [
            WheelState(radius=self.cfg.wheel_radius, inertia=self.cfg.wheel_inertia)
            for _ in range(4)
        ]
        self.state = DualTrackState(wheels=self.wheels)

    def reset(self, vx: float = 0.0) -> None:
        r = self.cfg.wheel_radius
        w0 = vx / max(r, 1e-6)
        for w in self.wheels:
            w.omega = w0
            w.drive_torque = 0.0
            w.brake_torque = 0.0
            w.Fx = w.Fy = 0.0
            w.kappa = w.alpha = 0.0
        g = 9.81
        m = self.cfg.mass
        L = self.cfg.a + self.cfg.b
        Fz_f = m * g * self.cfg.b / L
        Fz_r = m * g * self.cfg.a / L
        for i, w in enumerate(self.wheels):
            w.Fz = (Fz_f / 2.0) if i < 2 else (Fz_r / 2.0)
        self.abs = ABSController(enabled=self.cfg.abs_enabled)
        self.state = DualTrackState(wheels=self.wheels)

    def _kappa_for_fx(self, tire: DugoffTire, alpha: float, Fz: float, Fx_tgt: float) -> float:
        lo, hi = -1.2, 1.2
        for _ in range(14):
            mid = 0.5 * (lo + hi)
            fx = float(tire.longitudinal_lateral_force(mid, alpha, Fz).Fx)
            if fx < Fx_tgt:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def step(
        self,
        *,
        vx: float,
        vy: float,
        yaw_rate: float,
        steer: float,
        drive_torque_total: float,
        brake_cmd: float,
        dt: float,
        downforce: float = 0.0,
        mu_scale: float = 1.0,
        mu_per_wheel: Optional[np.ndarray] = None,
    ) -> DualTrackState:
        cfg = self.cfg
        a, b = cfg.a, cfg.b
        tf, tr = cfg.track_f, cfg.track_r
        r = cfg.wheel_radius
        m = cfg.mass
        g = 9.81

        xs = np.array([a, a, -b, -b])
        ys = np.array([tf / 2, -tf / 2, tr / 2, -tr / 2])
        deltas = np.array([steer, steer, 0.0, 0.0])

        ax_prev = self.state.ax
        ay_prev = self.state.ay
        L = a + b
        dFz_long_f = -m * ax_prev * cfg.h_cg / L
        dFz_long_r = +m * ax_prev * cfg.h_cg / L
        Fz_f_axle = m * g * b / L + dFz_long_f + 0.5 * downforce
        Fz_r_axle = m * g * a / L + dFz_long_r + 0.5 * downforce
        Fz_f_axle = max(Fz_f_axle, 2 * self.lt_params.Fz_min)
        Fz_r_axle = max(Fz_r_axle, 2 * self.lt_params.Fz_min)

        lt = compute_load_transfer(ay_prev, Fz_f_axle, Fz_r_axle, self.lt_params, mass=m)
        Fz_list = [lt.Fz_fl, lt.Fz_fr, lt.Fz_rl, lt.Fz_rr]

        T_f = drive_torque_total * cfg.drive_split_front
        T_r = drive_torque_total * (1.0 - cfg.drive_split_front)
        drive = [T_f / 2, T_f / 2, T_r / 2, T_r / 2]

        slips = np.array([w.kappa for w in self.wheels])
        sensors = SensorReading(
            vx=vx, vy=vy, yaw_rate=yaw_rate,
            ax=ax_prev, ay=ay_prev, steer=steer,
            wheel_omega=np.array([w.omega for w in self.wheels]),
            slip_ratio=slips,
        )
        pressures, abs_active, _ = self.abs.step(
            sensors, float(np.clip(brake_cmd, 0, 1)), dt
        )

        Fx_sum = 0.0
        Fy_sum = 0.0
        Mz_sum = 0.0

        for i, w in enumerate(self.wheels):
            mu_i = float(mu_per_wheel[i]) if mu_per_wheel is not None else cfg.mu * mu_scale
            self.tires[i].p.mu = mu_i
            w.mu = mu_i
            w.Fz = float(Fz_list[i])
            w.steer = float(deltas[i])
            w.drive_torque = float(drive[i])
            w.brake_torque = float(pressures[i] * cfg.brake_torque_max)

            vx_c = vx - yaw_rate * ys[i]
            vy_c = vy + yaw_rate * xs[i]
            c, s = np.cos(deltas[i]), np.sin(deltas[i])
            vx_t = c * vx_c + s * vy_c
            vy_t = -s * vx_c + c * vy_c

            alpha = float(np.arctan2(vy_t, max(abs(vx_t), cfg.v_eps)))

            T_cmd = w.drive_torque - w.brake_torque
            Fx_tgt = T_cmd / max(r, 1e-3)
            Fx_tgt = float(np.clip(Fx_tgt, -mu_i * w.Fz * 0.99, mu_i * w.Fz * 0.99))

            k_eq = self._kappa_for_fx(self.tires[i], alpha, w.Fz, Fx_tgt) if abs(T_cmd) > 0.5 else 0.0

            omega_r = w.omega * r
            denom = max(abs(vx_t), cfg.v_eps)
            k_meas = float(np.clip((omega_r - vx_t) / denom, -1.5, 1.5))

            blend = float(np.clip(abs(T_cmd) / (mu_i * w.Fz * r + 80.0), 0.15, 0.92))
            kappa = (1.0 - blend) * k_meas + blend * k_eq
            kappa = float(np.clip(kappa, -1.5, 1.5))

            ts: TireState = self.tires[i].longitudinal_lateral_force(kappa, alpha, w.Fz)
            Fx_t, Fy_t = float(ts.Fx), float(ts.Fy)
            Fx_v = c * Fx_t - s * Fy_t
            Fy_v = s * Fx_t + c * Fy_t

            w.kappa = float(ts.slip_ratio)
            w.alpha = float(ts.slip_angle)
            w.Fx = Fx_v
            w.Fy = Fy_v
            w.utilization = float(ts.utilization)

            omega_tgt = (vx_t * (1.0 + kappa)) / max(r, 1e-3)
            T_react = Fx_t * r
            T_err = T_cmd - T_react
            Iw = max(w.inertia, 0.8)
            tau_w = 0.05
            w.omega = w.omega + (dt / tau_w) * (omega_tgt - w.omega) + (dt / Iw) * T_err * 0.2
            if vx >= -0.5:
                w.omega = max(w.omega, 0.0)
            w.omega = float(np.clip(w.omega, -250.0, 250.0))

            Fx_sum += Fx_v
            Fy_sum += Fy_v
            Mz_sum += xs[i] * Fy_v - ys[i] * Fx_v

        ax = Fx_sum / m
        ay = Fy_sum / m
        yaw_acc = Mz_sum / cfg.Iz

        Fz_sum = float(sum(Fz_list))
        weight = m * g + downforce

        self.state = DualTrackState(
            wheels=self.wheels,
            ax=float(ax),
            ay=float(ay),
            yaw_acc=float(yaw_acc),
            abs_active=np.array(abs_active, dtype=bool),
            brake_pressure=np.array(pressures, dtype=float),
            Fz_sum=Fz_sum,
            residual_Fx=0.0,
            residual_Fy=0.0,
            residual_Mz=0.0,
            residual_Fz=float(Fz_sum - weight),
        )
        return self.state

    def diagnostics(self) -> dict:
        arr = self.state.as_arrays()
        return {
            "tire_model": "DugoffTire",
            "abs_enabled": self.cfg.abs_enabled,
            "abs_active": self.state.abs_active.tolist(),
            "brake_pressure": self.state.brake_pressure.tolist(),
            "omega": arr["omega"].tolist(),
            "kappa": arr["kappa"].tolist(),
            "alpha": arr["alpha"].tolist(),
            "Fx": arr["Fx"].tolist(),
            "Fy": arr["Fy"].tolist(),
            "Fz": arr["Fz"].tolist(),
            "utilization": arr["util"].tolist(),
            "Fz_sum": self.state.Fz_sum,
            "residual_Fz": self.state.residual_Fz,
            "ax": self.state.ax,
            "ay": self.state.ay,
            "yaw_acc": self.state.yaw_acc,
        }
