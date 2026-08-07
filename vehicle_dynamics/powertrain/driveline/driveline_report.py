"""Engineering summary for advanced driveline."""

from __future__ import annotations

from .driveline_state import AdvancedDrivelineState
import numpy as np


def format_driveline_report(state: AdvancedDrivelineState) -> str:
    lines = [
        "=== Advanced Driveline Report ===",
        f"enabled              : {state.enabled}",
        f"input torque [N·m]   : {state.torque_input:.2f}",
        f"wheel torque L/R     : {state.torque_left:.2f} / {state.torque_right:.2f}",
        f"propshaft twist [deg]: {np.rad2deg(state.shaft_twist):.3f}",
        f"halfshaft L/R [deg]  : {np.rad2deg(state.halfshaft_twist_L):.3f} / "
        f"{np.rad2deg(state.halfshaft_twist_R):.3f}",
        f"torsional energy [J] : {state.torsional_energy:.3f}",
        f"backlash engaged     : {state.backlash_engaged} (side={state.backlash_side})",
        f"peak torque [N·m]    : {state.peak_torque:.2f}",
        f"oscillation freq [Hz]: {state.oscillation_freq_hz:.2f}",
        f"wheel speeds L/R     : {state.wheel_speed_left:.2f} / {state.wheel_speed_right:.2f}",
    ]
    return "\n".join(lines)
