"""Strategy engineering report."""

from __future__ import annotations

from .strategy_state import StrategyState


def format_strategy_report(state: StrategyState) -> str:
    lines = [
        "=== Powertrain Strategy Report ===",
        f"drive mode           : {state.drive_mode}",
        f"enabled              : {state.enabled}",
        f"torque factor        : {state.torque_factor:.3f}",
        f"requested torque     : {state.requested_torque:.1f} N·m",
        f"engine / motor req   : {state.engine_request:.3f} / {state.motor_request:.3f}",
        f"regen request        : {state.regen_request:.3f}",
        f"target gear          : {state.target_gear}",
        f"shift request        : {state.shift_request}",
        f"launch active        : {state.launch_active}",
        f"cruise active        : {state.cruise_active}",
        f"pit limiter          : {state.pit_limiter_active}",
        f"hybrid energy mode   : {state.hybrid_energy_mode}",
        f"est. acceleration    : {state.estimated_acceleration:.2f} m/s²",
    ]
    return "\n".join(lines)
