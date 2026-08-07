"""Hybrid engineering report."""

from __future__ import annotations

from .hybrid_state import HybridState


def format_hybrid_report(state: HybridState) -> str:
    lines = [
        "=== Hybrid Powertrain Report ===",
        f"mode                 : {state.mode}",
        f"enabled              : {state.enabled}",
        f"engine running       : {state.engine_running}",
        f"engine torque [N·m]  : {state.engine_torque:.1f}",
        f"motor torque [N·m]   : {state.motor_torque:.1f}",
        f"wheel torque [N·m]   : {state.wheel_torque:.1f}",
        f"motor rpm            : {state.motor_speed_rpm:.0f}",
        f"motor efficiency     : {state.motor_efficiency:.3f}",
        f"battery SOC          : {state.battery_soc:.3f}",
        f"battery V / I        : {state.battery_voltage:.1f} V / {state.battery_current:.1f} A",
        f"battery temp [°C]    : {state.battery_temperature:.1f}",
        f"power flow [kW]      : {state.power_flow_kw:.2f}",
        f"regen power [kW]     : {state.regen_power_kw:.2f}",
        f"energy used [kWh]    : {state.energy_used_kwh:.4f}",
        f"energy recovered     : {state.energy_recovered_kwh:.4f}",
    ]
    return "\n".join(lines)
