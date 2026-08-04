"""
Phase 5.0–7.1 – Dual-Track (4-wheel) planar vehicle model.

Phase 7.1: per-wheel camber_total from SuspensionInterface → tire camber_rad.
Default (no suspension / zero camber) reproduces Phase 5/6.5 behaviour.
"""

from __future__ import annotations

import numpy as np
from scipy.integrate import solve_ivp

from .parameters import DualTrackParameters
from .result import DualTrackResult
from .kinematics import (
    FL, FR, RL, RR,
    wheel_positions, wheel_body_velocity, wheel_frame_velocity,
    slip_ratio, slip_angle, body_forces_from_wheel,
)
from .steering import front_steer_angles
from .normal_loads import four_wheel_normal_loads
from .brakes import FourWheelBrakeDistributor
from .abs_per_wheel import FourWheelABS
from .suspension_interface import SuspensionInterface, SuspensionInterfaceConfig
from ..tire.factory import TireFactory
from ..tire.dugoff import DugoffParams
from ..lateral.kinematics import inertial_rates


class DualTrackVehicleModel:
    def __init__(
        self,
        params: DualTrackParameters = None,
        use_abs: bool = True,
        mu_wheels: np.ndarray | None = None,
        suspension: SuspensionInterface | None = None,
        wheel_travel_func=None,
    ):
        """
        suspension : optional SuspensionInterface (camber → tire).
        wheel_travel_func : callable(t) -> (4,) wheel travel [m]; default zeros.
        """
        self.p = params or DualTrackParameters()
        self.use_abs = use_abs
        self.tire = TireFactory.create("dugoff_standard", params=self.p.tire)
        self.brake_dist = FourWheelBrakeDistributor(self.p.brake)
        self.abs = FourWheelABS()
        self.R = self.p.longitudinal.wheel_radius
        self.Iw = self.p.longitudinal.Iw
        bp = self.p.bicycle
        lt = self.p.load_transfer
        self.x_w, self.y_w = wheel_positions(bp.a, bp.b, lt.track_f, lt.track_r)
        if mu_wheels is None:
            self.mu_wheels = np.full(4, self.p.tire.mu)
        else:
            self.mu_wheels = np.asarray(mu_wheels, dtype=float).reshape(4)

        self.suspension = suspension
        self.wheel_travel_func = wheel_travel_func or (lambda t: np.zeros(4))
        self._camber = np.zeros(4)

    def _refresh_camber(self, t: float) -> np.ndarray:
        if self.suspension is None:
            self._camber = np.zeros(4)
            return self._camber
        z = np.asarray(self.wheel_travel_func(t), dtype=float).reshape(4)
        self.suspension.set_wheel_travel(z)
        self._camber = self.suspension.camber_total_array()
        return self._camber

    def _steer_pair(self, delta_cmd: float) -> tuple[float, float]:
        bp = self.p.bicycle
        lt = self.p.load_transfer
        return front_steer_angles(
            delta_cmd,
            wheelbase=bp.L,
            track_f=lt.track_f,
            delta_max=bp.delta_max,
            use_ackermann=self.p.steering.use_ackermann,
        )

    def _body_longitudinal_from_tire(self, tire_Fx, kappa):
        if kappa > 0.0:
            return -abs(tire_Fx)
        if kappa < 0.0:
            return abs(tire_Fx)
        return tire_Fx

    def _tire_force(self, kappa, alpha, Fz, wheel_idx):
        mu_saved = self.tire.p.mu
        self.tire.p.mu = float(self.mu_wheels[wheel_idx])
        st = self.tire.longitudinal_lateral_force(
            kappa, alpha, Fz,
            camber_rad=float(self._camber[wheel_idx]),
        )
        self.tire.p.mu = mu_saved
        return st

    def simulate(
        self,
        vx0: float = 20.0,
        t_span=(0.0, 8.0),
        delta_func=None,
        pedal_func=None,
        wheel_scale_func=None,
        dt_out: float = 0.01,
        abs_dt: float = 0.001,
    ) -> DualTrackResult:
        if delta_func is None:
            delta_func = lambda t: 0.0
        if pedal_func is None:
            pedal_func = lambda t: 0.0
        if wheel_scale_func is None:
            wheel_scale_func = lambda t: np.ones(4)

        self.abs.reset()
        w0 = vx0 / self.R
        y0 = [vx0, 0.0, 0.0, 0.0, 0.0, 0.0, w0, w0, w0, w0]
        bp = self.p.bicycle

        abs_pressure = np.ones(4)
        last_abs_t = [t_span[0]]

        def dynamics(t, y):
            nonlocal abs_pressure
            vx, vy, r, psi, X, Y, w_fl, w_fr, w_rl, w_rr = y
            if vx < 0.2:
                return [0.0] * 10

            self._refresh_camber(t)

            delta_cmd = float(np.clip(delta_func(t), -bp.delta_max, bp.delta_max))
            pedal = float(np.clip(pedal_func(t), 0.0, 1.0))
            omegas = [w_fl, w_fr, w_rl, w_rr]

            d_fl, d_fr = self._steer_pair(delta_cmd)
            deltas = [d_fl, d_fr, 0.0, 0.0]

            ay_est = vx * r
            Fzs = four_wheel_normal_loads(
                ay_est, bp.m, bp.a, bp.b, self.p.load_transfer
            )

            kappas = []
            Fx_body = []
            Fy_body = []
            for i in range(4):
                Vx_b, Vy_b = wheel_body_velocity(vx, vy, r, self.x_w[i], self.y_w[i])
                Vx_w, Vy_w = wheel_frame_velocity(Vx_b, Vy_b, deltas[i])
                kappa = slip_ratio(Vx_w, omegas[i], self.R, bp.v_eps)
                alpha = slip_angle(Vx_w, Vy_w, bp.v_eps)
                st = self._tire_force(kappa, alpha, Fzs[i], i)
                Fx_w = self._body_longitudinal_from_tire(st.Fx, kappa)
                Fy_w = st.Fy
                Fxb, Fyb = body_forces_from_wheel(Fx_w, Fy_w, deltas[i])
                kappas.append(kappa)
                Fx_body.append(Fxb)
                Fy_body.append(Fyb)

            dt_abs = max(t - last_abs_t[0], abs_dt)
            last_abs_t[0] = t
            if self.use_abs and pedal > 1e-4:
                abs_pressure = self.abs.update(np.asarray(kappas), dt_abs, active=True)
            else:
                abs_pressure = np.ones(4)

            scale = np.asarray(wheel_scale_func(t), dtype=float).reshape(4)
            cmd = self.brake_dist.desired(pedal, wheel_scale=scale)
            T = cmd.T * abs_pressure

            m = bp.m
            Iz = bp.Iz
            sum_Fx = sum(Fx_body)
            sum_Fy = sum(Fy_body)
            yaw_m = sum(
                self.x_w[i] * Fy_body[i] - self.y_w[i] * Fx_body[i] for i in range(4)
            )

            vx_dot = sum_Fx / m + vy * r
            vy_dot = sum_Fy / m - vx * r
            r_dot = yaw_m / Iz
            X_dot, Y_dot = inertial_rates(vx, vy, psi)
            w_dots = [(-Fx_body[i] * self.R - T[i]) / self.Iw for i in range(4)]

            return [vx_dot, vy_dot, r_dot, r, X_dot, Y_dot,
                    w_dots[0], w_dots[1], w_dots[2], w_dots[3]]

        sol = solve_ivp(
            dynamics, t_span, y0, method="RK45",
            rtol=1e-6, atol=1e-8, dense_output=True, max_step=0.02,
        )
        if not sol.success:
            raise RuntimeError(sol.message)

        t = np.arange(t_span[0], t_span[1] + dt_out, dt_out)
        st = sol.sol(t)
        vx = st[0]
        if np.any(vx < 0.3):
            idx = int(np.argmax(vx < 0.3))
            t = t[: idx + 1]
            st = sol.sol(t)
            vx = st[0]

        n = len(t)
        vy, r, psi, X, Y = st[1], st[2], st[3], st[4], st[5]
        omegas = st[6:10].T

        delta = np.array([
            float(np.clip(delta_func(ti), -bp.delta_max, bp.delta_max)) for ti in t
        ])
        pedal = np.array([float(np.clip(pedal_func(ti), 0.0, 1.0)) for ti in t])

        delta_fl = np.zeros(n)
        delta_fr = np.zeros(n)
        kappa = np.zeros((n, 4))
        alpha = np.zeros((n, 4))
        Fx = np.zeros((n, 4))
        Fy = np.zeros((n, 4))
        Fz = np.zeros((n, 4))
        util = np.zeros((n, 4))
        camber = np.zeros((n, 4))
        brake_torque = np.zeros((n, 4))
        abs_pressure = np.zeros((n, 4))

        abs_log = FourWheelABS()
        for i in range(n):
            self._refresh_camber(t[i])
            camber[i, :] = self._camber
            d_fl, d_fr = self._steer_pair(delta[i])
            delta_fl[i], delta_fr[i] = d_fl, d_fr
            deltas = [d_fl, d_fr, 0.0, 0.0]
            ay_est = vx[i] * r[i]
            Fzs = four_wheel_normal_loads(
                ay_est, bp.m, bp.a, bp.b, self.p.load_transfer
            )
            Fz[i, :] = Fzs
            kappas_i = np.zeros(4)
            for w in range(4):
                Vx_b, Vy_b = wheel_body_velocity(
                    vx[i], vy[i], r[i], self.x_w[w], self.y_w[w]
                )
                Vx_w, Vy_w = wheel_frame_velocity(Vx_b, Vy_b, deltas[w])
                kappa[i, w] = slip_ratio(Vx_w, omegas[i, w], self.R, bp.v_eps)
                kappas_i[w] = kappa[i, w]
                alpha[i, w] = slip_angle(Vx_w, Vy_w, bp.v_eps)
                st_t = self._tire_force(kappa[i, w], alpha[i, w], Fzs[w], w)
                Fx[i, w] = self._body_longitudinal_from_tire(st_t.Fx, kappa[i, w])
                Fy[i, w] = st_t.Fy
                util[i, w] = st_t.utilization

            if self.use_abs and pedal[i] > 1e-4:
                abs_pressure[i, :] = abs_log.update(kappas_i, dt_out, active=True)
            else:
                abs_pressure[i, :] = 1.0
            scale = np.asarray(wheel_scale_func(t[i]), dtype=float).reshape(4)
            cmd = self.brake_dist.desired(pedal[i], wheel_scale=scale)
            brake_torque[i, :] = cmd.T * abs_pressure[i, :]

        return DualTrackResult(
            time=t, vx=vx, vy=vy, r=r, psi=psi,
            delta=delta, delta_fl=delta_fl, delta_fr=delta_fr,
            pedal=pedal,
            kappa=kappa, alpha=alpha, Fx=Fx, Fy=Fy, Fz=Fz,
            omega=omegas, utilization=util,
            brake_torque=brake_torque, abs_pressure=abs_pressure,
            X=X, Y=Y,
            camber=camber,
        )
