"""Unsteady aero report."""

from __future__ import annotations

from .unsteady_solver import UnsteadyAeroState


def format_unsteady_report(st: UnsteadyAeroState, title: str = "Unsteady Aero") -> str:
    a = st.aero.state
    lines = [
        f"=== {title} ===",
        f"Source:     {st.source}",
        f"Airspeed:   {st.airspeed:.2f} m/s",
        f"Beta_aero:  {st.beta_aero*57.3:.2f} deg",
        f"Wind:       [{st.wind[0]:.2f}, {st.wind[1]:.2f}, {st.wind[2]:.2f}] m/s",
        f"Wake str:   {st.wake_strength:.3f}",
        f"Cd/Cl fac:  {st.Cd_factor:.3f} / {st.Cl_factor:.3f}",
        "",
        f"Drag:       {a.drag:.1f} N",
        f"Side force: {a.Fy:.1f} N",
        f"DF front:   {a.downforce_front:.1f} N",
        f"DF rear:    {a.downforce_rear:.1f} N",
        f"Yaw mom:    {a.Mz:.1f} N·m",
        f"Roll mom:   {a.Mx:.1f} N·m",
    ]
    return "\n".join(lines)
