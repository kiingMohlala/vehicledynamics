"""
Phase 5.0 / 5.1 – Dual-Track (4-wheel) planar vehicle model.

Phase 5.1: independent front road-wheel angles via Ackermann geometry
(delta_fl, delta_fr). Rear wheels remain unsteered.

RK45, lateral load-transfer feedback only. Tire API unchanged. ABS per-axle.
"""

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
from ..tire.factory import TireFactory
from ..braking.brake_torque import BrakeTorque
from ..braking.abs_controller import ABSController
from ..lateral.kinematics import inertial_rates


class DualTrackVehicleModel:
    def __init__(self, params: DualTrackParameters = None, use_abs: bool = True):
        self.p = params or DualTrackParameters()
        self.use_abs = use_abs
        self.tire = TireFactory.create("dugoff_standard", params=self.p.tire)
        self.brake = BrakeTorque(self.p.brake)
        self.abs_f = ABSController()
        self.abs_r = ABSController()
        self.R = self.p.longitudinal.wheel_radius
        self.Iw = self.p.longitudinal.Iw
        bp = self.p.bicycle
        lt = self.p.load_transfer
        self.x_w, self.y_w = wheel_positions(bp.a, bp.b, lt.track_f, lt.track_r)

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

    def simulate(
        self,
        vx0: float = 20.0,
        t_span=(0.0, 8.0),
        delta_func=None,
        pedal_func=None,
        dt_out: float = 0.01,
    ) -> DualTrackResult:
        if delta_func is None:
            delta_func = lambda t: 0.0
        if pedal_func is None:
            pedal_func = lambda t: 0.0

        w0 = vx0 / self.R
        y0 = [vx0, 0.0, 0.0, 0.0, 0.0, 0.0, w0, w0, w0, w0]
        bp = self.p.bicycle

        def dynamics(t, y):
            vx, vy, r, psi, X, Y, w_fl, w_fr, w_rl, w_rr = y
            if vx < 0.2:
                return [0.0] * 10

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
                st = self.tire.longitudinal_lateral_force(kappa, alpha, Fzs[i])
                Fx_w = self._body_longitudinal_from_tire(st.Fx, kappa)
                Fy_w = st.Fy
                Fxb, Fyb = body_forces_from_wheel(Fx_w, Fy_w, deltas[i])
                kappas.append(kappa)
                Fx_body.append(Fxb)
                Fy_body.append(Fyb)

            kappa_f = 0.5 * (kappas[FL] + kappas[FR])
            kappa_r = 0.5 * (kappas[RL] + kappas[RR])
            if self.use_abs and pedal > 1e-4:
                p_f = self.abs_f.update(max(kappa_f, 0.0), 0.001)
                p_r = self.abs_r.update(max(kappa_r, 0.0), 0.001)
            else:
                p_f = p_r = 1.0

            T_f_des, T_r_des = self.brake.desired(pedal)
            T = [
                0.5 * T_f_des * p_f,
                0.5 * T_f_des * p_f,
                0.5 * T_r_des * p_r,
                0.5 * T_r_des * p_r,
            ]

            m = bp.m
            Iz = bp.Iz
            sum_Fx = sum(Fx_body)
            sum_Fy = sum(Fy_body)
            yaw_m = 0.0
            for i in range(4):
                yaw_m += self.x_w[i] * Fy_body[i] - self.y_w[i] * Fx_body[i]

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

        for i in range(n):
            d_fl, d_fr = self._steer_pair(delta[i])
            delta_fl[i], delta_fr[i] = d_fl, d_fr
            deltas = [d_fl, d_fr, 0.0, 0.0]
            ay_est = vx[i] * r[i]
            Fzs = four_wheel_normal_loads(
                ay_est, bp.m, bp.a, bp.b, self.p.load_transfer
            )
            Fz[i, :] = Fzs
            for w in range(4):
                Vx_b, Vy_b = wheel_body_velocity(
                    vx[i], vy[i], r[i], self.x_w[w], self.y_w[w]
                )
                Vx_w, Vy_w = wheel_frame_velocity(Vx_b, Vy_b, deltas[w])
                kappa[i, w] = slip_ratio(Vx_w, omegas[i, w], self.R, bp.v_eps)
                alpha[i, w] = slip_angle(Vx_w, Vy_w, bp.v_eps)
                st_t = self.tire.longitudinal_lateral_force(
                    kappa[i, w], alpha[i, w], Fzs[w]
                )
                Fx[i, w] = self._body_longitudinal_from_tire(st_t.Fx, kappa[i, w])
                Fy[i, w] = st_t.Fy
                util[i, w] = st_t.utilization

        return DualTrackResult(
            time=t, vx=vx, vy=vy, r=r, psi=psi,
            delta=delta, delta_fl=delta_fl, delta_fr=delta_fr,
            pedal=pedal,
            kappa=kappa, alpha=alpha, Fx=Fx, Fy=Fy, Fz=Fz,
            omega=omegas, utilization=util, X=X, Y=Y,
        )
