"""Transmission report."""

from __future__ import annotations

from .transmission_solver import TransmissionState


def format_transmission_report(state: TransmissionState, title: str = "Transmission") -> str:
    lines = [
        f"=== {title} ===",
        f"Gear:            {state.gear}",
        f"Shift phase:     {state.shift_phase}",
        f"Shift active:    {state.shift_active}",
        f"Clutch engage:   {state.clutch_engagement*100:.0f} %",
        f"Clutch slip:     {state.clutch_slip:.2f} rad/s",
        f"Clutch torque:   {state.clutch_torque:.1f} N·m",
        f"Clutch temp:     {state.clutch_temp_C:.1f} °C",
        f"Locked:          {state.locked}",
        f"Gearbox RPM:     {state.gearbox_rpm:.0f}",
        f"Wheel torque:    {state.wheel_torque:.1f} N·m",
        f"Ignition cut:    {state.ignition_cut}",
    ]
    return "\n".join(lines)
