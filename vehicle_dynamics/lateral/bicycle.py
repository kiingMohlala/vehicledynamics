"""
Phase 4.0/4.1 – Dynamic Bicycle Model (2-DOF)

States: vy (lateral velocity), r (yaw rate)
Vx held constant.
Tire forces from the validated combined-slip Dugoff model with κ = 0.

Phase 4.1: load-transfer diagnostics are computed and logged only.
They do NOT feed back into the tire normal loads or vehicle dynamics.
"""

import numpy as np
from scipy.integrate import solve_ivp
from .parameters import BicycleParameters
from .kinematics import front_slip_angle, rear_slip_angle, inertial_rates
from .result import LateralSimulationResult
from .load_transfer import LoadTransferParameters, compute_load_transfer
from ..tire.factory import TireFactory
from ..tire.dugoff import DugoffParams, TireState

class DynamicBicycleModel:
    def __init__(
        self,
        params: BicycleParameters = None,
        tire_model_name: str = "dugoff_standard",
        tire_params: DugoffParams = None,
        load_transfer_params: LoadTransferParameters = None,
    ):
        self.p = params or BicycleParameters()
        self.tire = TireFactory.create(tire_model_name, params=tire_params)
        self.lt_params = load_transfer_params or LoadTransferParameters()

        # Static axle normal loads (unchanged by Phase 4.1 Level A)
        total_weight = self.p.m * 9.81
        self.Fz_f = total_weight * (self.p.b / self.p.L)
        self.Fz_r = total_weight * (self.p.a / self.p.L)

    def _tire_forces(self, vy, r, vx, delta):
        """Return full front and rear TireState objects."""
        alpha_f = front_slip_angle(vy, r, vx, delta, self.p)
        alpha_r = rear_slip_angle(vy, r, vx, self.p)
        state_f = self.tire.longitudinal_lateral_force(0.0, alpha_f, self.Fz_f)
        state_r = self.tire.longitudinal_lateral_force(0.0, alpha_r, self.Fz_r)
        return state_f, state_r

    def dynamics(self, t, state, vx, delta_func):
        vy, r, psi, X, Y = state
        delta = float(np.clip(delta_func(t), -self.p.delta_max, self.p.delta_max))
        state_f, state_r = self._tire_forces(vy, r, vx, delta)
        Fy_f, Fy_r = state_f.Fy, state_r.Fy
        vy_dot = (Fy_f + Fy_r) / self.p.m - vx * r
        r_dot = (self.p.a * Fy_f - self.p.b * Fy_r) / self.p.Iz
        X_dot, Y_dot = inertial_rates(vx, vy, psi)
        return [vy_dot, r_dot, r, X_dot, Y_dot]

    def simulate(
        self,
        vx: float = 20.0,
        t_span=(0.0, 10.0),
        delta_func=None,
        y0=None,
        dt_out: float = 0.01,
    ) -> LateralSimulationResult:
        if delta_func is None:
            delta_func = lambda t: 0.0
        if y0 is None:
            y0 = [0.0, 0.0, 0.0, 0.0, 0.0]

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
        vy, r, psi, X, Y = states

        delta = np.array([
            float(np.clip(delta_func(ti), -self.p.delta_max, self.p.delta_max))
            for ti in t
        ])
        n = len(t)
        alpha_f = np.zeros(n)
        alpha_r = np.zeros(n)
        Fy_f = np.zeros(n)
        Fy_r = np.zeros(n)
        ay_force = np.zeros(n)
        ay_vehicle = np.zeros(n)
        dFz_front = np.zeros(n)
        dFz_rear = np.zeros(n)
        Fz_fl = np.zeros(n)
        Fz_fr = np.zeros(n)
        Fz_rl = np.zeros(n)
        Fz_rr = np.zeros(n)
        wheel_lift_front = np.zeros(n, dtype=bool)
        wheel_lift_rear = np.zeros(n, dtype=bool)

        vy_dot = np.gradient(vy, t)

        for i in range(n):
            state_f, state_r = self._tire_forces(vy[i], r[i], vx, delta[i])
            alpha_f[i] = state_f.slip_angle
            alpha_r[i] = state_r.slip_angle
            Fy_f[i] = state_f.Fy
            Fy_r[i] = state_r.Fy
            ay_force[i] = (Fy_f[i] + Fy_r[i]) / self.p.m
            ay_vehicle[i] = vy_dot[i] + vx * r[i]

            # Phase 4.1 diagnostics only – no feedback into dynamics
            lt = compute_load_transfer(
                ay_force[i], self.Fz_f, self.Fz_r,
                params=self.lt_params, mass=self.p.m,
            )
            dFz_front[i] = lt.dFz_front
            dFz_rear[i] = lt.dFz_rear
            Fz_fl[i] = lt.Fz_fl
            Fz_fr[i] = lt.Fz_fr
            Fz_rl[i] = lt.Fz_rl
            Fz_rr[i] = lt.Fz_rr
            wheel_lift_front[i] = lt.wheel_lift_front
            wheel_lift_rear[i] = lt.wheel_lift_rear

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
            dFz_front=dFz_front,
            dFz_rear=dFz_rear,
            Fz_fl=Fz_fl,
            Fz_fr=Fz_fr,
            Fz_rl=Fz_rl,
            Fz_rr=Fz_rr,
            wheel_lift_front=wheel_lift_front,
            wheel_lift_rear=wheel_lift_rear,
        )
