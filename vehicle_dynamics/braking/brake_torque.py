from .parameters import BrakeParams

class BrakeTorque:
    def __init__(self, params: BrakeParams):
        self.p = params

    def desired(self, pedal: float = 1.0):
        """Return desired front and rear brake torques (Nm)"""
        T_f = self.p.front_bias * self.p.max_front_torque * pedal
        T_r = (1.0 - self.p.front_bias) * self.p.max_rear_torque * pedal
        return T_f, T_r
