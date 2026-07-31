"""
Phase 4.0 – Dynamic Bicycle Model (2-DOF)

States: vy (lateral velocity), r (yaw rate)
Vx held constant.
Tire forces from the validated combined-slip Dugoff model with κ = 0.
"""

import numpy as np
from scipy.integrate import solve_ivp
from .parameters import BicycleParameters
from .kinematics import front_slip_angle, rear_slip_angle, inertial_rates
from .result import LateralSimulationResult
from ..tire.factory import TireFactory
from ..tire.dugoff import DugoffParams, TireState

class DynamicBicycleModel:
    def __init__(
        self,
        params: BicycleParameters = None,
        tire_model_name: str = "dugoff_standard",
        tire_params: DugoffParams = None,
    ):
        self.p = params or BicycleParameters()
        # Proper parameter injection into the tire model
        self.tire = TireFactory.create(tire_model_name, params=tire_params)

        # Static normal loads (Phase 4.0 – no lateral load transfer yet)
        total_weight = self.p.m * 9.81
        self.Fz_f = total_weight * (self.p.b / self.p.L)
        self.Fz_r = total_weight * (self.p.a / self.p.L)

    def _tire_forces(self, vy, r, vx, delta):
        """Return full front and rear TireState objects."""
        alpha_f = front_slip_angle(vy, r, vx, delta, self.p)
        alpha_r = rear_slip_angle(vy, r, vx, self.p)

        # Pure lateral for Phase 4.0 (κ = 0)
        state_f = self.tire.longitudinal_lateral_force(0.0, alpha_f, self.Fz_f)
        state_r = self.tire.longitudinal_lateral_force(0.0, alpha_r, self.Fz_r)

        return state_f, state_r

    def dynamics(self, t, state, vx, delta_func):
        """
        state = [vy, r, psi, X, Y]
        """
        vy, r, psi, X, Y = state
        delta = float(np.clip(delta_func(t), -self.p.delta_max, self.p.delta_max))

        state_f, state_r = self._tire_forces(vy, r, vx, delta)
        Fy_f = state_f.Fy
        Fy_r = state_r.Fy

        # Equations of motion
        vy_dot = (Fy_f + Fy_r) / self.p.m - vx * r
        r_dot = (self.p.a * Fy_f - self.p.b * Fy_r) / self.p.Iz

        X_dot, Y_dot = inertial_rates(vx, vy, psi)
        psi_dot = r

        return [vy_dot, r_dot, psi_dot, X_dot, Y_dot]

    def simulate(
        self,
        vx: float = 20.0,
        t_span=(0.0, 10.0),
        delta_func=None,
        y0=None,
        dt_out: float = 0.01,
    ) -> LateralSimulationResult:
        """
        Simulate the bicycle model with constant Vx.

        delta_func(t) -> steering angle [rad]
        """
        if delta_func is None:
            delta_func = lambda t: 0.0

        if y0 is None:
            y0 = [0.0, 0.0, 0.0, 0.0, 0.0]  # vy, r, psi, X, Y

        sol = solve_ivp(
            fun=lambda t, y: self.dynamics(t, y, vx, delta_func),
            t_span=t_span,
            y0=y0,
            method="RK45",
            rtol=1e-6,
            atol=1e-8,
            dense_output=True,
        )

        if not sol.success:
            raise RuntimeError(f"Integrator failed: {sol.message}")

        t = np.arange(t_span[0], t_span[1] + dt_out, dt_out)
        states = sol.sol(t)
        vy = states[0]
        r = states[1]
        psi = states[2]
        X = states[3]
        Y = states[4]

        # Reconstruct secondary quantities
        delta = np.array([
            float(np.clip(delta_func(ti), -self.p.delta_max, self.p.delta_max))
            for ti in t
        ])
        alpha_f = np.zeros_like(t)
        alpha_r = np.zeros_like(t)
        Fy_f = np.zeros_like(t)
        Fy_r = np.zeros_like(t)
        ay_force = np.zeros_like(t)
        ay_vehicle = np.zeros_like(t)

        # Approximate vy_dot from gradient for ay_vehicle
        vy_dot = np.gradient(vy, t)

        for i, ti in enumerate(t):
            state_f, state_r = self._tire_forces(vy[i], r[i], vx, delta[i])
            alpha_f[i] = state_f.slip_angle
            alpha_r[i] = state_r.slip_angle
            Fy_f[i] = state_f.Fy
            Fy_r[i] = state_r.Fy
            ay_force[i] = (Fy_f[i] + Fy_r[i]) / self.p.m
            ay_vehicle[i] = vy_dot[i] + vx * r[i]

        return LateralSimulationResult(
            time=t,
            vx=np.full_like(t, vx),
            vy=vy,
            r=r,
            psi=psi,
            delta=delta,
            alpha_f=alpha_f,
            alpha_r=alpha_r,
            Fy_f=Fy_f,
            Fy_r=Fy_r,
            ay_force=ay_force,
            ay_vehicle=ay_vehicle,
            X=X,
            Y=Y,
        )
