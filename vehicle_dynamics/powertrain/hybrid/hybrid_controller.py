"""Supervisor: maps driver + energy decision to component commands."""

from __future__ import annotations

from dataclasses import dataclass

from .energy_manager import EnergyManager, EnergyMode
from .torque_blending import TorqueBlender, BlendMode


@dataclass
class HybridController:
    energy: EnergyManager
    blender: TorqueBlender

    def __init__(self, energy_mode: EnergyMode = EnergyMode.HYBRID):
        self.energy = EnergyManager(mode=energy_mode)
        self.blender = TorqueBlender()

    def step(
        self,
        throttle: float,
        brake: float,
        soc: float,
        ice_torque_cap: float,
        motor_torque_cap: float,
        driver_torque_scale: float = 400.0,
    ) -> dict:
        blend_mode, engine_on, motor_bias = self.energy.decide(soc, throttle, brake)
        req = throttle * driver_torque_scale
        # Bias: shift request toward motor or ICE
        ice_cmd, mot_cmd = self.blender.blend(
            req, ice_torque_cap if engine_on else 0.0, motor_torque_cap, blend_mode
        )
        if motor_bias < 0 and engine_on and ice_cmd > 0:
            # Charge sustain: add load on ICE, negative motor (generate)
            gen = abs(motor_bias) * min(ice_cmd * 0.3, abs(motor_torque_cap) * 0.3)
            mot_cmd -= gen
        return {
            "blend_mode": blend_mode,
            "engine_on": engine_on,
            "ice_torque": ice_cmd,
            "motor_torque": mot_cmd,
            "motor_bias": motor_bias,
        }
