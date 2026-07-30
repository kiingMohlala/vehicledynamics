"""
BrakeSimulation – Phase 3 core
Supports selectable tire models and ABS.
"""

import numpy as np
from .parameters import VehicleLongitudinalParams, BrakeParams, ThermalParams
from .weight_transfer import WeightTransfer
from .brake_torque import BrakeTorque
from .wheel_dynamics import WheelDynamics
from .thermal import BrakeThermal
from .abs_controller import ABSController
from .result import BrakeSimulationResult
from ..tire.factory import TireFactory
from ..tire.dugoff import TireState

class BrakeSimulation:
    def __init__(
        self,
        vehicle_params: VehicleLongitudinalParams = None,
        brake_params: BrakeParams = None,
        thermal_params: ThermalParams = None,
        tire_model_name: str = "dugoff_standard"
    ):
        self.v_params = vehicle_params or VehicleLongitudinalParams()
        self.b_params = brake_params or BrakeParams()
        self.t_params = thermal_params or ThermalParams()

        self.wt = WeightTransfer(self.v_params)
        self.bt = BrakeTorque(self.b_params)
        self.wheel = WheelDynamics(self.v_params)
        self.thermal_f = BrakeThermal(self.t_params)
        self.thermal_r = BrakeThermal(self.t_params)

        self.tire = TireFactory.create(tire_model_name)
        self.tire_model_name = tire_model_name

        self.abs_front = ABSController()
        self.abs_rear = ABSController()

    def run(
        self,
        v0: float = 22.22,
        pedal: float = 1.0,
        use_abs: bool = True,
        dt: float = 0.001,
        t_max: float = 10.0
    ) -> BrakeSimulationResult:

        v = v0
        x = 0.0
        r = self.v_params.wheel_radius
        omega_f = v / r
        omega_r = v / r
        t = 0.0
        a_x = 0.0

        # History lists
        hist = {
            "t": [], "v": [], "omega_f": [], "omega_r": [],
            "slip_f": [], "slip_r": [],
            "p_f": [], "p_r": [],
            "T_f": [], "T_r": [],
            "Fx_f": [], "Fx_r": [],
            "a_x": []
        }

        while v > 0.15 and t < t_max:
            # Weight transfer
            Fz_f, Fz_r = self.wt.loads(a_x)

            # Desired brake torques
            T_f_des, T_r_des = self.bt.desired(pedal)

            # Slip calculation (protected)
            v_safe = max(v, 0.5)
            slip_f = (v_safe - omega_f * r) / v_safe
            slip_r = (v_safe - omega_r * r) / v_safe

            # ABS
            if use_abs:
                p_f = self.abs_front.update(slip_f, dt)
                p_r = self.abs_rear.update(slip_r, dt)
            else:
                p_f = p_r = 1.0

            T_f = T_f_des * p_f
            T_r = T_r_des * p_r

            # Tire forces (pure longitudinal for now)
            state_f = self.tire.longitudinal_lateral_force(slip_f, 0.0, Fz_f)
            state_r = self.tire.longitudinal_lateral_force(slip_r, 0.0, Fz_r)

            # In our sign convention Fx > 0 is traction.
            # Braking force that decelerates the vehicle is therefore -Fx when kappa > 0.
            Fx_f = -state_f.Fx if slip_f > 0 else state_f.Fx
            Fx_r = -state_r.Fx if slip_r > 0 else state_r.Fx

            # Vehicle deceleration
            a_x = (Fx_f + Fx_r) / self.v_params.mass   # already signed

            # Integrate vehicle
            v += a_x * dt
            if v < 0:
                v = 0.0
            x += v * dt

            # Wheel dynamics
            omega_f = self.wheel.step(T_f, state_f.Fx, omega_f, dt)
            omega_r = self.wheel.step(T_r, state_r.Fx, omega_r, dt)

            # Thermal
            self.thermal_f.update(T_f, omega_f, dt)
            self.thermal_r.update(T_r, omega_r, dt)

            # Log
            hist["t"].append(t)
            hist["v"].append(v)
            hist["omega_f"].append(omega_f)
            hist["omega_r"].append(omega_r)
            hist["slip_f"].append(slip_f)
            hist["slip_r"].append(slip_r)
            hist["p_f"].append(p_f)
            hist["p_r"].append(p_r)
            hist["T_f"].append(T_f)
            hist["T_r"].append(T_r)
            hist["Fx_f"].append(Fx_f)
            hist["Fx_r"].append(Fx_r)
            hist["a_x"].append(a_x)

            t += dt

        # Convert to arrays
        for k in hist:
            hist[k] = np.array(hist[k])

        return BrakeSimulationResult(
            time=hist["t"],
            vehicle_speed=hist["v"],
            wheel_speed_front=hist["omega_f"],
            wheel_speed_rear=hist["omega_r"],
            slip_front=hist["slip_f"],
            slip_rear=hist["slip_r"],
            pressure_front=hist["p_f"],
            pressure_rear=hist["p_r"],
            brake_torque_front=hist["T_f"],
            brake_torque_rear=hist["T_r"],
            tire_force_front=hist["Fx_f"],
            tire_force_rear=hist["Fx_r"],
            deceleration=hist["a_x"],
            stopping_distance=float(x),
            peak_slip_front=float(np.max(np.abs(hist["slip_f"]))) if len(hist["slip_f"]) else 0.0,
            peak_slip_rear=float(np.max(np.abs(hist["slip_r"]))) if len(hist["slip_r"]) else 0.0,
            tire_model=self.tire_model_name
        )
