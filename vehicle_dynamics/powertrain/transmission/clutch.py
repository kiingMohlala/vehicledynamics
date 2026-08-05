"""Clutch engagement, slip, and thermal energy."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .clutch_friction import ClutchFrictionParams, clutch_capacity, clutch_heat_power


@dataclass
class ClutchState:
    engagement: float = 0.0
    omega_slip: float = 0.0
    torque: float = 0.0
    capacity: float = 0.0
    locked: bool = False
    temp_C: float = 80.0
    energy_J: float = 0.0
    wear: float = 0.0


class Clutch:
    def __init__(self, params: ClutchFrictionParams | None = None):
        self.params = params or ClutchFrictionParams()
        self.state = ClutchState()
        self.thermal_mass = 800.0
        self.cooling_rate = 0.05
        self.ambient_C = 80.0

    def reset(self) -> None:
        self.state = ClutchState()

    def step(
        self,
        engagement: float,
        omega_engine: float,
        omega_gearbox: float,
        engine_torque: float,
        dt: float,
    ) -> ClutchState:
        e = float(np.clip(engagement, 0.0, 1.0))
        w_slip = float(omega_engine - omega_gearbox)
        T_max = clutch_capacity(e, self.params, self.state.temp_C)

        locked = False
        if e < 0.02 or T_max < 1e-9:
            T = 0.0
        elif e > 0.98 and abs(w_slip) < 2.0:
            T = float(np.clip(engine_torque, -T_max, T_max))
            locked = abs(engine_torque) <= T_max + 1e-6
            if locked:
                w_slip = 0.0
        else:
            # Kinetic friction: capacity limited, direction opposes slip
            # When slip is positive (engine faster), torque on gearbox is positive
            T = float(T_max * np.sign(w_slip if abs(w_slip) > 1e-9 else engine_torque))

        heat = clutch_heat_power(T, w_slip)
        energy = self.state.energy_J + heat * dt
        dT = heat / self.thermal_mass - self.cooling_rate * (self.state.temp_C - self.ambient_C)
        temp = self.state.temp_C + dT * dt
        wear = self.state.wear + 1e-8 * heat * dt

        self.state = ClutchState(
            engagement=e,
            omega_slip=w_slip,
            torque=float(T),
            capacity=float(T_max),
            locked=locked,
            temp_C=float(temp),
            energy_J=float(energy),
            wear=float(wear),
        )
        return self.state
