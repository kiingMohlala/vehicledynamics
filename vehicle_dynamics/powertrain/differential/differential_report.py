"""Differential report."""

from __future__ import annotations

from .differential_solver import DifferentialState


def format_differential_report(state: DifferentialState, title: str = "Differential") -> str:
    d = state.driveline
    lines = [
        f"=== {title} ===",
        f"Type:              {d.diff_type}",
        f"Input torque:      {d.torque_input:.1f} N·m",
        f"Left torque:       {d.torque_left:.1f} N·m",
        f"Right torque:      {d.torque_right:.1f} N·m",
        f"Bias:              {d.bias:.1f} N·m",
        f"Locking fraction:  {d.locking_fraction:.3f}",
        f"Axle speed:        {d.axle_speed:.2f} rad/s",
        f"Δω:                {d.delta_omega:.2f} rad/s",
        f"Efficiency:        {d.efficiency:.3f}",
        f"Yaw proxy:         {d.yaw_moment_proxy:.1f} N",
    ]
    return "\n".join(lines)
