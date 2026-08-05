from .parameters import ThermalParams

class BrakeThermal:
    def __init__(self, params: ThermalParams):
        self.p = params
        self.temp = params.ambient

    def update(self, T_brake: float, omega: float, dt: float) -> float:
        """Simple lumped thermal model. Power = T * ω"""
        power = abs(T_brake * omega)
        heat_in = getattr(self.p, "eta_heat", 0.95) * power * dt
        heat_out = self.p.convection * self.p.area * (self.temp - self.p.ambient) * dt
        dT = (heat_in - heat_out) / (self.p.rotor_mass * self.p.rotor_cp)
        self.temp += dT
        return self.temp

    def friction(self) -> float:
        """Temperature-dependent friction coefficient"""
        if self.temp < self.p.fade_start:
            return self.p.mu_cold
        elif self.temp > self.p.fade_full:
            return self.p.mu_hot
        else:
            frac = (self.temp - self.p.fade_start) / (self.p.fade_full - self.p.fade_start)
            return self.p.mu_cold - frac * (self.p.mu_cold - self.p.mu_hot)
