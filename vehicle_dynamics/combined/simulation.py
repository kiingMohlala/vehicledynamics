"""
Phase 4.2 – CombinedVehicleModel

Couples:
  - Phase 4.0 bicycle lateral dynamics (vy, r)
  - Phase 3 longitudinal braking (dynamic Vx, optional ABS)
  - Combined-slip Dugoff tires (κ and α both active)

Sign convention (frozen from Phase 3/4):
  κ > 0  → braking slip
  Tire Fx > 0 → force in +x tire axis (forward)
  For vehicle body equations under braking, the longitudinal force
  applied to the vehicle is -Fx when κ > 0 (opposes forward motion).

No ESC, no dual-track, no load-transfer feedback, no Fx yaw moment.
"""

import numpy as np
from scipy.integrate import solve_ivp

from .parameters import CombinedParameters
from .result import CombinedSimulationResult
from ..lateral.kinematics import front_slip_angle, rear_slip_angle, inertial_rates
from ..tire.factory import TireFactory
from ..braking.brake_torque import BrakeTorque
from ..braking.abs_controller import ABSController

class CombinedVehicleModel:
    def __init__(self, params: CombinedParameters = None, use_abs: bool = True):
        self.p = params or CombinedParameters()
        self.use_abs = use_abs

        self.tire = TireFactory.create("dugoff_standard", params=self.p.tire)
        self.brake_torque = BrakeTorque(self.p.brake)
        self.abs_f = ABSController()
        self.abs_r = ABSController()

        W = self.p.bicycle.m * 9.81
        self.Fz_f = W * (self.p.bicycle.b / self.p.bicycle.L)
        self.Fz_r = W * (self.p.bicycle.a / self.p.bicycle.L)
        self.R = self.p.longitudinal.wheel_radius
        self.Iw = self.p.longitudinal.Iw

    def _kappa(self, vx, omega):
        vx_safe = max(abs(vx), self.p.bicycle.v_eps)
        return (vx_safe - omega * self.R) / vx_safe

    def _body_longitudinal_force(self, tire_Fx, kappa):
        """Map tire-frame Fx to vehicle body longitudinal force."""
        # Braking slip (κ>0): tire reports positive Fx in our model;
        # body force must oppose +Vx → negative.
        if kappa > 0.0:
            return -abs(tire_Fx)
        if kappa < 0.0:
            return abs(tire_Fx)
        return tire_Fx

    def simulate(
        self,
        vx0: float = 22.22,
        t_span=(0.0, 10.0),
        delta_func=None,
        pedal_func=None,
        dt_out: float = 0.01,
    ) -> CombinedSimulationResult:
        if delta_func is None:
            delta_func = lambda t: 0.0
        if pedal_func is None:
            pedal_func = lambda t: 0.0

        # State: [vx, vy, r, psi, X, Y, omega_f, omega_r]
        y0 = [vx0, 0.0, 0.0, 0.0, 0.0, 0.0, vx0 / self.R, vx0 / self.R]

        def dynamics(t, y):
            vx, vy, r, psi, X, Y, omega_f, omega_r = y
            # Prevent reverse integration issues
            if vx < 0.2:
                return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            delta = float(np.clip(delta_func(t), -self.p.bicycle.delta_max, self.p.bicycle.delta_max))
            pedal = float(np.clip(pedal_func(t), 0.0, 1.0))

            kappa_f = self._kappa(vx, omega_f)
            kappa_r = self._kappa(vx, omega_r)

            if self.use_abs and pedal > 1e-4:
                p_f = self.abs_f.update(max(kappa_f, 0.0), 0.001)
                p_r = self.abs_r.update(max(kappa_r, 0.0), 0.001)
            else:
                p_f = p_r = 1.0

            T_f_des, T_r_des = self.brake_torque.desired(pedal)
            T_f = T_f_des * p_f
            T_r = T_r_des * p_r

            alpha_f = front_slip_angle(vy, r, vx, delta, self.p.bicycle)
            alpha_r = rear_slip_angle(vy, r, vx, self.p.bicycle)

            st_f = self.tire.longitudinal_lateral_force(kappa_f, alpha_f, self.Fz_f)
            st_r = self.tire.longitudinal_lateral_force(kappa_r, alpha_r, self.Fz_r)

            Fx_body_f = self._body_longitudinal_force(st_f.Fx, kappa_f)
            Fx_body_r = self._body_longitudinal_force(st_r.Fx, kappa_r)
            Fy_f, Fy_r = st_f.Fy, st_r.Fy

            m = self.p.bicycle.m
            Iz = self.p.bicycle.Iz
            a, b = self.p.bicycle.a, self.p.bicycle.b

            vx_dot = (Fx_body_f + Fx_body_r) / m + vy * r
            vy_dot = (Fy_f + Fy_r) / m - vx * r
            r_dot = (a * Fy_f - b * Fy_r) / Iz

            X_dot, Y_dot = inertial_rates(vx, vy, psi)

            # Wheel spin dynamics: brake torque reduces omega;
            # road torque ~ -Fx_body * R when Fx_body is the force on the vehicle
            # (equal and opposite force on the tire contact patch).
            omega_f_dot = (-Fx_body_f * self.R - T_f) / self.Iw
            omega_r_dot = (-Fx_body_r * self.R - T_r) / self.Iw

            return [vx_dot, vy_dot, r_dot, r, X_dot, Y_dot, omega_f_dot, omega_r_dot]

        sol = solve_ivp(
            dynamics, t_span, y0, method="RK45",
            rtol=1e-6, atol=1e-8, dense_output=True, max_step=0.02,
        )
        if not sol.success:
            raise RuntimeError(f"Integrator failed: {sol.message}")

        t = np.arange(t_span[0], t_span[1] + dt_out, dt_out)
        st = sol.sol(t)
        vx = st[0]
        if np.any(vx < 0.3):
            idx = int(np.argmax(vx < 0.3))
            t = t[: idx + 1]
            st = sol.sol(t)
            vx = st[0]

        vy, r, psi, X, Y = st[1], st[2], st[3], st[4], st[5]
        omega_f, omega_r = st[6], st[7]

        n = len(t)
        delta = np.array([float(np.clip(delta_func(ti), -self.p.bicycle.delta_max, self.p.bicycle.delta_max)) for ti in t])
        pedal = np.array([float(np.clip(pedal_func(ti), 0.0, 1.0)) for ti in t])
        alpha_f = np.zeros(n); alpha_r = np.zeros(n)
        kappa_f = np.zeros(n); kappa_r = np.zeros(n)
        Fx_f = np.zeros(n); Fx_r = np.zeros(n)
        Fy_f = np.zeros(n); Fy_r = np.zeros(n)
        ay_force = np.zeros(n)

        for i in range(n):
            kappa_f[i] = self._kappa(vx[i], omega_f[i])
            kappa_r[i] = self._kappa(vx[i], omega_r[i])
            alpha_f[i] = front_slip_angle(vy[i], r[i], vx[i], delta[i], self.p.bicycle)
            alpha_r[i] = rear_slip_angle(vy[i], r[i], vx[i], self.p.bicycle)
            st_f = self.tire.longitudinal_lateral_force(kappa_f[i], alpha_f[i], self.Fz_f)
            st_r = self.tire.longitudinal_lateral_force(kappa_r[i], alpha_r[i], self.Fz_r)
            Fx_f[i] = self._body_longitudinal_force(st_f.Fx, kappa_f[i])
            Fx_r[i] = self._body_longitudinal_force(st_r.Fx, kappa_r[i])
            Fy_f[i], Fy_r[i] = st_f.Fy, st_r.Fy
            ay_force[i] = (Fy_f[i] + Fy_r[i]) / self.p.bicycle.m

        if len(X) > 1:
            stopping_distance = float(np.sum(np.sqrt(np.diff(X)**2 + np.diff(Y)**2)))
        else:
            stopping_distance = 0.0

        return CombinedSimulationResult(
            time=t, vx=vx, vy=vy, r=r, psi=psi,
            delta=delta, pedal=pedal,
            alpha_f=alpha_f, alpha_r=alpha_r,
            kappa_f=kappa_f, kappa_r=kappa_r,
            Fx_f=Fx_f, Fx_r=Fx_r, Fy_f=Fy_f, Fy_r=Fy_r,
            ay_force=ay_force, X=X, Y=Y,
            stopping_distance=stopping_distance,
        )
