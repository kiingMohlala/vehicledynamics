from .parameters import VehicleLongitudinalParams

class WeightTransfer:
    def __init__(self, params: VehicleLongitudinalParams):
        self.p = params
        self.g = 9.81

    def loads(self, a_x: float):
        """Return Fz_front, Fz_rear under longitudinal acceleration a_x (m/s²)"""
        static_f = self.p.cg_front_ratio * self.p.mass * self.g
        static_r = (1.0 - self.p.cg_front_ratio) * self.p.mass * self.g
        transfer = (self.p.mass * a_x * self.p.cg_height) / self.p.wheelbase
        return static_f + transfer, static_r - transfer
