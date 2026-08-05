from dataclasses import dataclass

@dataclass
class VehicleLongitudinalParams:
    mass: float = 1400.0
    wheelbase: float = 2.70
    cg_height: float = 0.45
    cg_front_ratio: float = 0.55
    tire_mu: float = 1.35
    wheel_radius: float = 0.33
    Iw: float = 1.2

@dataclass
class BrakeParams:
    front_bias: float = 0.63
    max_front_torque: float = 4500.0
    max_rear_torque: float = 2800.0
    eta_heat: float = 0.95

@dataclass
class ThermalParams:
    rotor_mass: float = 8.5
    rotor_cp: float = 500.0
    convection: float = 35.0
    area: float = 0.12
    ambient: float = 25.0
    mu_cold: float = 0.42
    mu_hot: float = 0.28
    fade_start: float = 350.0
    fade_full: float = 550.0
