from .parameters import VehicleLongitudinalParams

class WheelDynamics:
    def __init__(self, params: VehicleLongitudinalParams):
        self.Iw = params.Iw
        self.r = params.wheel_radius

    def step(self, T_brake: float, Fx: float, omega: float, dt: float) -> float:
        """
        Integrate wheel angular velocity.
        T_brake is the magnitude of braking torque (positive).
        Fx is the longitudinal tire force in the tire frame.
        """
        # Applied torque on the wheel: tire force contributes +Fx*r,
        # brake torque opposes rotation → -T_brake
        torque_net = Fx * self.r - T_brake
        alpha = torque_net / self.Iw
        omega_new = omega + alpha * dt
        return max(omega_new, 0.0)
