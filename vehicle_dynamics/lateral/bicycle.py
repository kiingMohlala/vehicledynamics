
import numpy as np
from scipy.integrate import solve_ivp
from .parameters import BicycleParameters
from .kinematics import front_slip_angle, rear_slip_angle, inertial_rates
from .result import LateralSimulationResult
from vehicle_dynamics.tire.dugoff import DugoffTire, DugoffParams

class DynamicBicycleModel:
    def __init__(self, params=None):
        self.p = params or BicycleParameters()
        self.tire = DugoffTire(DugoffParams())
        total_weight = self.p.m * 9.81
        self.Fz_f = total_weight * (self.p.b / self.p.L)
        self.Fz_r = total_weight * (self.p.a / self.p.L)

    def _tire_forces(self, vy, r, vx, delta):
        alpha_f = front_slip_angle(vy, r, vx, delta, self.p)
        alpha_r = rear_slip_angle(vy, r, vx, self.p)
        state_f = self.tire.longitudinal_lateral_force(0.0, alpha_f, self.Fz_f)
        state_r = self.tire.longitudinal_lateral_force(0.0, alpha_r, self.Fz_r)
        return state_f.Fy, state_r.Fy, alpha_f, alpha_r

    def dynamics(self, t, state, vx, delta_func):
        vy, r, psi, X, Y = state
        delta = delta_func(t)
        Fy_f, Fy_r, _, _ = self._tire_forces(vy, r, vx, delta)
        vy_dot = (Fy_f + Fy_r) / self.p.m - vx * r
        r_dot = (self.p.a * Fy_f - self.p.b * Fy_r) / self.p.Iz
        X_dot, Y_dot = inertial_rates(vx, vy, psi)
        psi_dot = r
        return [vy_dot, r_dot, psi_dot, X_dot, Y_dot]

    def simulate(self, vx=20.0, t_span=(0.0, 10.0), delta_func=None, y0=None, dt_out=0.01):
        if delta_func is None:
            delta_func = lambda t: 0.0
        if y0 is None:
            y0 = [0.0, 0.0, 0.0, 0.0, 0.0]
        sol = solve_ivp(
            fun=lambda t, y: self.dynamics(t, y, vx, delta_func),
            t_span=t_span, y0=y0, method="RK45", rtol=1e-6, atol=1e-8, dense_output=True
        )
        if not sol.success:
            raise RuntimeError(sol.message)
        t = np.arange(t_span[0], t_span[1] + dt_out, dt_out)
        states = sol.sol(t)
        vy, r, psi, X, Y = states
        delta = np.array([delta_func(ti) for ti in t])
        alpha_f = np.zeros_like(t)
        alpha_r = np.zeros_like(t)
        Fy_f = np.zeros_like(t)
        Fy_r = np.zeros_like(t)
        ay = np.zeros_like(t)
        for i in range(len(t)):
            Fy_f[i], Fy_r[i], alpha_f[i], alpha_r[i] = self._tire_forces(vy[i], r[i], vx, delta[i])
            ay[i] = (Fy_f[i] + Fy_r[i]) / self.p.m
        return LateralSimulationResult(
            time=t, vx=np.full_like(t, vx), vy=vy, r=r, psi=psi, delta=delta,
            alpha_f=alpha_f, alpha_r=alpha_r, Fy_f=Fy_f, Fy_r=Fy_r, ay=ay, X=X, Y=Y
        )
