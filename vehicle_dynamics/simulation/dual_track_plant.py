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
from vehicle_dynamics.simulation.sprung_body import SprungBodyModel, SprungBodyConfig
from vehicle_dynamics.simulation.unsprung_model import UnsprungModel, UnsprungConfig
from vehicle_dynamics.steering.steering_model import SteeringModel
from vehicle_dynamics.steering.steering_config import SteeringConfig
from vehicle_dynamics.lateral.slip_angles import compute_wheel_slip_angles
from vehicle_dynamics.lateral.load_transfer import (
    LoadTransferParameters,
    compute_load_transfer,
    compute_wheel_loads,
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
    # Phase 14.4 load-transfer authority
    chi_f: float = 0.55
    Fz_min: float = 50.0
    # Phase 14.5 sprung-body suspension
    use_sprung_body: bool = True
    k_front: float = 28000.0
    k_rear: float = 32000.0
    c_front: float = 2500.0
    c_rear: float = 2800.0
    roll_stiffness_front: float = 20000.0
    roll_stiffness_rear: float = 18000.0
    I_theta: float = 1200.0
    I_phi: float = 400.0
    # Phase 14.7 unsprung / tire vertical
    use_unsprung: bool = True
    m_u_front: float = 40.0
    m_u_rear: float = 45.0
    k_tire_front: float = 220000.0
    k_tire_rear: float = 220000.0
    c_tire_front: float = 200.0
    c_tire_rear: float = 200.0
    # Phase 14.9 steering
    max_steer_angle: float = 0.52
    steering_ratio: float = 15.0
    steering_rate: float = 1.2
    ackermann_enabled: bool = True
    k_arb_front: float = 25000.0
    k_arb_rear: float = 22000.0
    c_arb_front: float = 400.0
    c_arb_rear: float = 400.0
    use_arb: bool = True
    use_hydraulic_arb: bool = False
    k_hyd_front: float = 30000.0
    k_hyd_rear: float = 28000.0
    c_hyd_front: float = 800.0
    c_hyd_rear: float = 800.0


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
            chi_f=float(getattr(self.cfg, "chi_f", 0.55)),
            Fz_min=float(getattr(self.cfg, "Fz_min", 50.0)),
        )
        self.wheels = [
            WheelState(radius=self.cfg.wheel_radius, inertia=self.cfg.wheel_inertia)
            for _ in range(4)
        ]
        self.state = DualTrackState(wheels=self.wheels)
        self.sprung = SprungBodyModel(SprungBodyConfig(
            mass=self.cfg.mass,
            a=self.cfg.a,
            b=self.cfg.b,
            track_f=self.cfg.track_f,
            track_r=self.cfg.track_r,
            h_cg=self.cfg.h_cg,
            I_theta=float(getattr(self.cfg, "I_theta", 1200.0)),
            I_phi=float(getattr(self.cfg, "I_phi", 400.0)),
            k_front=float(getattr(self.cfg, "k_front", 28000.0)),
            k_rear=float(getattr(self.cfg, "k_rear", 32000.0)),
            c_front=float(getattr(self.cfg, "c_front", 2500.0)),
            c_rear=float(getattr(self.cfg, "c_rear", 2800.0)),
            roll_stiffness_front=float(getattr(self.cfg, "roll_stiffness_front", 20000.0)),
            roll_stiffness_rear=float(getattr(self.cfg, "roll_stiffness_rear", 18000.0)),
            k_arb_front=float(getattr(self.cfg, "k_arb_front", 25000.0)),
            k_arb_rear=float(getattr(self.cfg, "k_arb_rear", 22000.0)),
            c_arb_front=float(getattr(self.cfg, "c_arb_front", 400.0)),
            c_arb_rear=float(getattr(self.cfg, "c_arb_rear", 400.0)),
            use_arb=bool(getattr(self.cfg, "use_arb", True)),
            use_hydraulic_arb=bool(getattr(self.cfg, "use_hydraulic_arb", False)),
            k_hyd_front=float(getattr(self.cfg, "k_hyd_front", 30000.0)),
            k_hyd_rear=float(getattr(self.cfg, "k_hyd_rear", 28000.0)),
            c_hyd_front=float(getattr(self.cfg, "c_hyd_front", 800.0)),
            c_hyd_rear=float(getattr(self.cfg, "c_hyd_rear", 800.0)),
            Fz_min=float(getattr(self.cfg, "Fz_min", 50.0)),
            enabled=bool(getattr(self.cfg, "use_sprung_body", True)),
        ))
        self.unsprung = UnsprungModel(UnsprungConfig(
            m_u_front=float(getattr(self.cfg, "m_u_front", 40.0)),
            m_u_rear=float(getattr(self.cfg, "m_u_rear", 45.0)),
            k_tire_front=float(getattr(self.cfg, "k_tire_front", 220000.0)),
            k_tire_rear=float(getattr(self.cfg, "k_tire_rear", 220000.0)),
            c_tire_front=float(getattr(self.cfg, "c_tire_front", 200.0)),
            c_tire_rear=float(getattr(self.cfg, "c_tire_rear", 200.0)),
            Fz_min=float(getattr(self.cfg, "Fz_min", 50.0)),
            enabled=bool(getattr(self.cfg, "use_unsprung", True)),
        ))
        self.road_z = np.zeros(4)
        self.road_z_dot = np.zeros(4)
        if hasattr(self, "steering"):
            self.steering.reset()
        self.steering = SteeringModel(SteeringConfig(
            max_steer_angle=float(getattr(self.cfg, "max_steer_angle", 0.52)),
            steering_ratio=float(getattr(self.cfg, "steering_ratio", 15.0)),
            steering_rate=float(getattr(self.cfg, "steering_rate", 1.2)),
            ackermann_enabled=bool(getattr(self.cfg, "ackermann_enabled", True)),
            wheelbase=float(self.cfg.a + self.cfg.b),
            track_front=float(self.cfg.track_f),
        ))

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
        if hasattr(self, "sprung"):
            self.sprung.reset()
        if hasattr(self, "unsprung"):
            g = 9.81
            m = self.cfg.mass
            L = self.cfg.a + self.cfg.b
            Ff = m * g * self.cfg.b / L / 2
            Fr = m * g * self.cfg.a / L / 2
            self.unsprung.reset(np.array([Ff, Ff, Fr, Fr]))
        self.road_z = np.zeros(4)
        self.road_z_dot = np.zeros(4)

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
        downforce_front: Optional[float] = None,
        downforce_rear: Optional[float] = None,
    ) -> DualTrackState:
        kwargs_downforce_front = downforce_front
        kwargs_downforce_rear = downforce_rear
        cfg = self.cfg
        a, b = cfg.a, cfg.b
        tf, tr = cfg.track_f, cfg.track_r
        r = cfg.wheel_radius
        m = cfg.mass
        g = 9.81

        xs = np.array([a, a, -b, -b])
        ys = np.array([tf / 2, -tf / 2, tr / 2, -tr / 2])
        # Phase 14.9.1: rate-limited Ackermann steering
        if hasattr(self, "steering"):
            st = self.steering.step(float(steer), float(dt))
            deltas = np.array([st.delta_fl, st.delta_fr, st.delta_rl, st.delta_rr])
        else:
            deltas = np.array([steer, steer, 0.0, 0.0])

        ax_prev = self.state.ax
        ay_prev = self.state.ay
        df_f = float(kwargs_downforce_front) if kwargs_downforce_front is not None else 0.5 * downforce
        df_r = float(kwargs_downforce_rear) if kwargs_downforce_rear is not None else 0.5 * downforce
        # Phase 14.5/14.7: sprung body (+ optional unsprung wheel-hop)
        if getattr(cfg, "use_sprung_body", True) and hasattr(self, "sprung"):
            z_s, z_s_dot = self.sprung.corner_positions()
            k_c = np.array([
                cfg.k_front / 2, cfg.k_front / 2, cfg.k_rear / 2, cfg.k_rear / 2
            ])
            c_c = np.array([
                cfg.c_front / 2, cfg.c_front / 2, cfg.c_rear / 2, cfg.c_rear / 2
            ])
            Fz_contact = None
            z_u = z_ud = None
            if getattr(cfg, "use_unsprung", True) and hasattr(self, "unsprung"):
                us = self.unsprung.step(
                    z_s=z_s, z_s_dot=z_s_dot,
                    k_susp=k_c, c_susp=c_c,
                    road_z=self.road_z, road_z_dot=self.road_z_dot,
                    dt=dt,
                )
                Fz_contact = us.Fz
                z_u, z_ud = us.z_u, us.z_u_dot
            sb = self.sprung.step(
                ax=ax_prev, ay=ay_prev, dt=dt,
                downforce_front=df_f, downforce_rear=df_r,
                z_u=z_u, z_u_dot=z_ud, Fz_contact=Fz_contact,
            )
            self._last_lt = sb
            Fz_list = [float(sb.Fz[0]), float(sb.Fz[1]), float(sb.Fz[2]), float(sb.Fz[3])]
        else:
            lt = compute_wheel_loads(
                mass=m, a=a, b=b, h_cg=cfg.h_cg,
                track_f=tf, track_r=tr,
                ax=ax_prev, ay=ay_prev,
                downforce_front=df_f, downforce_rear=df_r,
                chi_f=float(getattr(cfg, "chi_f", 0.55)),
                Fz_min=float(getattr(cfg, "Fz_min", 50.0)),
            )
            self._last_lt = lt
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

        slip_states = compute_wheel_slip_angles(
            vx=vx, vy=vy, yaw_rate=yaw_rate, deltas=deltas, xs=xs, ys=ys,
            v_eps=float(getattr(cfg, "v_eps", 0.5)),
        )
        for i, w in enumerate(self.wheels):
            mu_i = float(mu_per_wheel[i]) if mu_per_wheel is not None else cfg.mu * mu_scale
            self.tires[i].p.mu = mu_i
            w.mu = mu_i
            w.Fz = float(Fz_list[i])
            w.steer = float(deltas[i])
            w.drive_torque = float(drive[i])
            w.brake_torque = float(pressures[i] * cfg.brake_torque_max)

            ss = slip_states[i]
            vx_t, vy_t, alpha = ss.vx_t, ss.vy_t, ss.alpha
            c, s = np.cos(deltas[i]), np.sin(deltas[i])

            # Drive torque from open split is non-negative under throttle; brake is separate.
            # Clamping prevents reverse-drive spikes when upstream clutch slip sign flips.
            T_drive = max(float(w.drive_torque), 0.0) if drive_torque_total >= 0.0 else float(w.drive_torque)
            T_cmd = T_drive - w.brake_torque
            Fx_tgt = T_cmd / max(r, 1e-3)
            Fx_cap = mu_i * w.Fz * 0.98
            Fx_tgt = float(np.clip(Fx_tgt, -Fx_cap, Fx_cap))

            # Quasi-static equilibrium kappa for commanded torque (Dugoff is the force source).
            if abs(T_cmd) > 1.0:
                k_eq = self._kappa_for_fx(self.tires[i], alpha, w.Fz, Fx_tgt)
            else:
                k_eq = 0.0

            omega_r = w.omega * r
            denom = max(abs(vx_t), abs(omega_r), cfg.v_eps)
            k_meas = float(np.clip((omega_r - vx_t) / denom, -1.2, 1.2))

            # Prefer equilibrium under significant drive/brake; light blend of measured slip
            # only when near free-rolling (avoids explicit-Euler blow-up of stiff Cx).
            if abs(T_cmd) > 0.15 * mu_i * w.Fz * r:
                kappa = k_eq
            else:
                blend = 0.35
                kappa = (1.0 - blend) * k_meas + blend * k_eq
            kappa = float(np.clip(kappa, -1.2, 1.2))

            ts: TireState = self.tires[i].longitudinal_lateral_force(kappa, alpha, w.Fz)
            Fx_t, Fy_t = float(ts.Fx), float(ts.Fy)
            Fx_v = c * Fx_t - s * Fy_t
            Fy_v = s * Fx_t + c * Fy_t

            w.kappa = float(ts.slip_ratio)
            w.alpha = float(ts.slip_angle)
            w.Fx = Fx_v
            w.Fy = Fy_v
            w.utilization = float(ts.utilization)

            # Wheel speed tracks kinematic target with first-order lag (stable, energy-aware)
            omega_tgt = (vx_t * (1.0 + kappa)) / max(r, 1e-3)
            tau_w = 0.04
            w.omega = w.omega + (dt / tau_w) * (omega_tgt - w.omega)
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
            residual_Fz=float(getattr(self._last_lt, "residual_Fz", Fz_sum - weight)),
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
            "Fz_FL": float(self.wheels[0].Fz),
            "Fz_FR": float(self.wheels[1].Fz),
            "Fz_RL": float(self.wheels[2].Fz),
            "Fz_RR": float(self.wheels[3].Fz),
            "min_Fz": float(min(w.Fz for w in self.wheels)),
            "max_Fz": float(max(w.Fz for w in self.wheels)),
            "lt": getattr(self, "_last_lt", None),
            "z": float(getattr(getattr(self, "sprung", None), "state", type("X",(object,),{"z":0})()).z) if hasattr(self, "sprung") else 0.0,
            "theta": float(self.sprung.state.theta) if hasattr(self, "sprung") else 0.0,
            "phi": float(self.sprung.state.phi) if hasattr(self, "sprung") else 0.0,
            "E_damp": float(self.sprung.state.E_damp_dissipated) if hasattr(self, "sprung") else 0.0,
            "z_u": list(self.unsprung.state.z_u) if hasattr(self, "unsprung") else [0,0,0,0],
            "E_tire_damp": float(self.unsprung.state.E_tire_damp) if hasattr(self, "unsprung") else 0.0,
            "road_z": list(self.road_z) if hasattr(self, "road_z") else [0,0,0,0],
            "delta_fl": float(self.steering.state.delta_fl) if hasattr(self, "steering") else 0.0,
            "delta_fr": float(self.steering.state.delta_fr) if hasattr(self, "steering") else 0.0,
            "steer_actual": float(self.steering.state.actual) if hasattr(self, "steering") else 0.0,
            "alpha_FL": float(self.wheels[0].alpha),
            "alpha_FR": float(self.wheels[1].alpha),
            "alpha_RL": float(self.wheels[2].alpha),
            "alpha_RR": float(self.wheels[3].alpha),
            "Fy_FL": float(self.wheels[0].Fy),
            "Fy_FR": float(self.wheels[1].Fy),
            "Fy_RL": float(self.wheels[2].Fy),
            "Fy_RR": float(self.wheels[3].Fy),
            "ax": self.state.ax,
            "ay": self.state.ay,
            "yaw_acc": self.state.yaw_acc,
        }
