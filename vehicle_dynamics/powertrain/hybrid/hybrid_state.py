"""Unified hybrid powertrain state."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class HybridState:
    engine_torque: float = 0.0
    motor_torque: float = 0.0
    wheel_torque: float = 0.0
    battery_soc: float = 0.7
    battery_voltage: float = 360.0
    battery_current: float = 0.0
    battery_temperature: float = 25.0
    regen_power_kw: float = 0.0
    motor_speed_rpm: float = 0.0
    motor_efficiency: float = 0.0
    engine_running: bool = False
    power_flow_kw: float = 0.0
    energy_used_kwh: float = 0.0
    energy_recovered_kwh: float = 0.0
    mode: str = "hybrid"
    enabled: bool = True
