"""
Fixed-step dual-track integration with optional ESC and torque vectoring.

RK4 at fixed Δt. Plant physics unchanged; drive torque is additive on the
wheel equation: Iw·ω̇ = −Fx·R − T_brake + T_drive.
"""

from __future__ import annotations

import numpy as np

from .parameters import DualTrackParameters
from .result import DualTrackResult
from .kinematics import (
    wheel_positions, wheel_body_velocity, wheel_frame_velocity,
    slip_ratio, slip_angle, body_forces_from_wheel,
)
from .steering import front_steer_angles
from .normal_loads import four_wheel_normal_loads
from .brakes import FourWheelBrakeDistributor
from .abs_per_wheel import FourWheelABS
from ..tire.factory import TireFactory
from ..lateral.kinematics import inertial_rates
from ..esc.parameters import ESCParameters
from ..esc.controller import ESCController
from ..esc.brake_allocator import BrakeAllocator
from ..esc.diagnostics import ESCDiagnostics
from ..torque_vectoring.parameters import TVParameters
from ..torque_vectoring.controller import TorqueVectoringController
from ..torque_vectoring.diagnostics import TVDiagnostics
from ..torque_vectoring.differential import distribute_drive


class FixedStepDualTrack:
    def __init__(
        self,
        params: DualTrackParameters = None,
        use_abs: bool = True,
        enable_esc: bool = False,
        enable_tv: bool = False,
        esc_params: ESCParameters = None,
        tv_params: TVParameters = None,
        mu_wheels: np.ndarray | None = None,
        dt: float = 0.001,
    ):
        self.p = params or DualTrackParameters()
        self.use_abs = use_abs
        self.enable_esc = enable_esc
        self.enable_tv = enable_tv
        self.dt = float(dt)
        self.tire = TireFactory.create("dugoff_standard", params=self.p.tire)
        self.brake_dist = FourWheelBrakeDistributor(self.p.brake)
        self.abs = FourWheelABS()
        self.R = self.p.longitudinal.wheel_radius
        self.Iw = self.p.longitudinal.Iw
        bp = self.p.bicycle
        lt = self.p.load_transfer
        self.x_w, self.y_w = wheel_positions(bp.a, bp.b, lt.track_f, lt.track_r)
        self.mu_wheels = (
            np.full(4, self.p.tire.mu)
            if mu_wheels is None
            else np.asarray(mu_wheels, dtype=float).reshape(4)
        )
        self.esc = ESCController(bp.L, esc_params or ESCParameters())
        self.alloc = BrakeAllocator(lt.track_f, lt.track_r, esc_params or ESCParameters())
        self.diagnostics = ESCDiagnostics()
        self.tv = TorqueVectoringController(bp.L, tv_params or TVParameters())
        self.tv_params = tv_params or TVParameters()
        self.tv_diagnostics = TVDiagnostics()

    def _steer_pair(self, delta_cmd: float):
        bp = self.p.bicycle
        lt = self.p.load_transfer
        return front_steer_angles(
            delta_cmd, bp.L, lt.track_f, bp.delta_max, self.p.steering.use_ackermann
        )

    def _body_Fx(self, tire_Fx, kappa):
        if kappa > 0.0:
            return -abs(tire_Fx)
        if kappa < 0.0:
            return abs(tire_Fx)
        return tire_Fx

    def _tire(self, kappa, alpha, Fz, i):
        mu_saved = self.tire.p.mu
        self.tire.p.mu = float(self.mu_wheels[i])
        st = self.tire.longitudinal_lateral_force(kappa, alpha, Fz)
        self.tire.p.mu = mu_saved
        return st

    def _derivatives(self, y, delta_cmd, pedal, esc_scale, abs_pressure, T_drive):
        vx, vy, r, psi, X, Y, w_fl, w_fr, w_rl, w_rr = y
        if vx < 0.2:
            return np.zeros(10), np.zeros(4), (0.0, 0.0)

        bp = self.p.bicycle
        d_fl, d_fr = self._steer_pair(delta_cmd)
        deltas = [d_fl, d_fr, 0.0, 0.0]
        omegas = [w_fl, w_fr, w_rl, w_rr]

        ay_est = vx * r
        Fzs = four_wheel_normal_loads(ay_est, bp.m, bp.a, bp.b, self.p.load_transfer)

        kappas, Fx_body, Fy_body = [], [], []
        for i in range(4):
            Vx_b, Vy_b = wheel_body_velocity(vx, vy, r, self.x_w[i], self.y_w[i])
            Vx_w, Vy_w = wheel_frame_velocity(Vx_b, Vy_b, deltas[i])
            kappa = slip_ratio(Vx_w, omegas[i], self.R, bp.v_eps)
            alpha = slip_angle(Vx_w, Vy_w, bp.v_eps)
            st = self._tire(kappa, alpha, Fzs[i], i)
            Fx_w = self._body_Fx(st.Fx, kappa)
            Fy_w = st.Fy
            Fxb, Fyb = body_forces_from_wheel(Fx_w, Fy_w, deltas[i])
            kappas.append(kappa)
            Fx_body.append(Fxb)
            Fy_body.append(Fyb)

        cmd = self.brake_dist.desired(pedal, esc_scale=esc_scale)
        T_brake = cmd.T * abs_pressure
        T_drive = np.asarray(T_drive, dtype=float).reshape(4)

        m, Iz = bp.m, bp.Iz
        sum_Fx = sum(Fx_body)
        sum_Fy = sum(Fy_body)
        yaw_m = sum(self.x_w[i] * Fy_body[i] - self.y_w[i] * Fx_body[i] for i in range(4))

        vx_dot = sum_Fx / m + vy * r
        vy_dot = sum_Fy / m - vx * r
        r_dot = yaw_m / Iz
        X_dot, Y_dot = inertial_rates(vx, vy, psi)
        # Drive increases ω; brake decreases ω
        w_dots = [
            (-Fx_body[i] * self.R - T_brake[i] + T_drive[i]) / self.Iw
            for i in range(4)
        ]

        return np.array([
            vx_dot, vy_dot, r_dot, r, X_dot, Y_dot,
            w_dots[0], w_dots[1], w_dots[2], w_dots[3],
        ]), np.array(kappas), (d_fl, d_fr)

    def simulate(
        self,
        vx0: float = 20.0,
        t_span=(0.0, 8.0),
        delta_func=None,
        pedal_func=None,
        throttle_func=None,
        dt_out: float = 0.01,
    ) -> DualTrackResult:
        if delta_func is None:
            delta_func = lambda t: 0.0
        if pedal_func is None:
            pedal_func = lambda t: 0.0
        if throttle_func is None:
            throttle_func = lambda t: 0.0

        bp = self.p.bicycle
        dt = self.dt
        self.abs.reset()
        self.esc.reset()
        self.tv.reset()
        self.diagnostics = ESCDiagnostics()
        self.tv_diagnostics = TVDiagnostics()

        w0 = vx0 / self.R
        y = np.array([vx0, 0.0, 0.0, 0.0, 0.0, 0.0, w0, w0, w0, w0], dtype=float)

        t0, tf = float(t_span[0]), float(t_span[1])
        n_steps = int(np.ceil((tf - t0) / dt))
        t = t0

        log_t = [t0]
        log_y = [y.copy()]
        log_delta = [0.0]
        log_dfl = [0.0]
        log_dfr = [0.0]
        log_pedal = [0.0]
        log_kappa = [np.zeros(4)]
        log_alpha = [np.zeros(4)]
        log_Fx = [np.zeros(4)]
        log_Fy = [np.zeros(4)]
        log_Fz = [np.zeros(4)]
        log_util = [np.zeros(4)]
        log_T = [np.zeros(4)]
        log_abs = [np.ones(4)]
        next_log = t0 + dt_out

        esc_scale = np.zeros(4)
        abs_pressure = np.ones(4)
        T_drive = np.zeros(4)

        for _ in range(n_steps):
            if y[0] < 0.25:
                break

            delta_cmd = float(np.clip(delta_func(t), -bp.delta_max, bp.delta_max))
            pedal = float(np.clip(pedal_func(t), 0.0, 1.0))
            throttle = float(np.clip(throttle_func(t), 0.0, 1.0))

            if self.enable_esc:
                Mz, diag = self.esc.update(y[0], y[1], y[2], delta_cmd, dt)
                esc_scale = self.alloc.allocate(Mz, delta_cmd, pedal)
                self.diagnostics.log(t, diag, esc_scale)
            else:
                esc_scale = np.zeros(4)

            if self.enable_tv:
                T_drive, tv_diag = self.tv.update(
                    y[0], y[1], y[2], delta_cmd, throttle, dt
                )
                self.tv_diagnostics.log(t, tv_diag)
            else:
                # Open differential drive if throttle without TV controller
                if throttle > 1e-6:
                    T_drive = distribute_drive(throttle, self.tv_params, rear_delta_T=0.0)
                else:
                    T_drive = np.zeros(4)

            _, kappas0, _ = self._derivatives(
                y, delta_cmd, pedal, esc_scale, abs_pressure, T_drive
            )
            if self.use_abs and (pedal > 1e-4 or np.any(esc_scale > 1e-4)):
                abs_pressure = self.abs.update(kappas0, dt, active=True)
            else:
                abs_pressure = np.ones(4)

            def f(yy):
                dydt, _, _ = self._derivatives(
                    yy, delta_cmd, pedal, esc_scale, abs_pressure, T_drive
                )
                return dydt

            k1 = f(y)
            k2 = f(y + 0.5 * dt * k1)
            k3 = f(y + 0.5 * dt * k2)
            k4 = f(y + dt * k3)
            y = y + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
            t += dt

            if t >= next_log - 1e-12 or t >= tf - 1e-12:
                d_fl, d_fr = self._steer_pair(delta_cmd)
                deltas = [d_fl, d_fr, 0.0, 0.0]
                ay_est = y[0] * y[2]
                Fzs = four_wheel_normal_loads(
                    ay_est, bp.m, bp.a, bp.b, self.p.load_transfer
                )
                kappa = np.zeros(4)
                alpha = np.zeros(4)
                Fx = np.zeros(4)
                Fy = np.zeros(4)
                util = np.zeros(4)
                omegas = y[6:10]
                for i in range(4):
                    Vx_b, Vy_b = wheel_body_velocity(
                        y[0], y[1], y[2], self.x_w[i], self.y_w[i]
                    )
                    Vx_w, Vy_w = wheel_frame_velocity(Vx_b, Vy_b, deltas[i])
                    kappa[i] = slip_ratio(Vx_w, omegas[i], self.R, bp.v_eps)
                    alpha[i] = slip_angle(Vx_w, Vy_w, bp.v_eps)
                    st = self._tire(kappa[i], alpha[i], Fzs[i], i)
                    Fx[i] = self._body_Fx(st.Fx, kappa[i])
                    Fy[i] = st.Fy
                    util[i] = st.utilization
                cmd = self.brake_dist.desired(pedal, esc_scale=esc_scale)
                T = cmd.T * abs_pressure

                log_t.append(t)
                log_y.append(y.copy())
                log_delta.append(delta_cmd)
                log_dfl.append(d_fl)
                log_dfr.append(d_fr)
                log_pedal.append(pedal)
                log_kappa.append(kappa)
                log_alpha.append(alpha)
                log_Fx.append(Fx)
                log_Fy.append(Fy)
                log_Fz.append(np.array(Fzs))
                log_util.append(util)
                log_T.append(T)
                log_abs.append(abs_pressure.copy())
                next_log += dt_out

        Ys = np.asarray(log_y)
        return DualTrackResult(
            time=np.asarray(log_t),
            vx=Ys[:, 0], vy=Ys[:, 1], r=Ys[:, 2], psi=Ys[:, 3],
            delta=np.asarray(log_delta),
            delta_fl=np.asarray(log_dfl),
            delta_fr=np.asarray(log_dfr),
            pedal=np.asarray(log_pedal),
            kappa=np.asarray(log_kappa),
            alpha=np.asarray(log_alpha),
            Fx=np.asarray(log_Fx),
            Fy=np.asarray(log_Fy),
            Fz=np.asarray(log_Fz),
            omega=Ys[:, 6:10],
            utilization=np.asarray(log_util),
            brake_torque=np.asarray(log_T),
            abs_pressure=np.asarray(log_abs),
            X=Ys[:, 4], Y=Ys[:, 5],
        )
