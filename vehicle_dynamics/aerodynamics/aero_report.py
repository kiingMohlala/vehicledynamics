"""Aerodynamics report formatting."""

from __future__ import annotations

from .aero_solver import AeroResult


def format_aero_report(result: AeroResult, title: str = "Aero Report") -> str:
    s = result.state
    lines = [
        f"=== {title} ===",
        f"Enabled: {result.config.enabled}",
        f"Speed:   {result.speed:.1f} m/s  ({result.speed * 3.6:.0f} km/h)",
        f"q:       {s.q:.1f} Pa",
        "",
        "Coefficients (effective)",
        f"  Cd:       {s.Cd_eff:.3f}",
        f"  Cl_front: {s.Cl_front_eff:.3f}",
        f"  Cl_rear:  {s.Cl_rear_eff:.3f}",
        "",
        "Forces",
        f"  Drag:           {s.drag:8.1f} N",
        f"  Cooling drag:   {s.cooling_drag:8.1f} N",
        f"  Side force:     {s.Fy:8.1f} N",
        f"  DF front:       {s.downforce_front:8.1f} N",
        f"  DF rear:        {s.downforce_rear:8.1f} N",
        f"  DF total:       {s.downforce_total:8.1f} N",
        "",
        "Balance",
        f"  Front aero %:   {s.front_balance * 100:.1f} %",
        f"  L/D:            {s.L_over_D:.2f}",
        f"  CoP (from mid): {s.center_of_pressure_x * 1e3:.1f} mm",
        "",
        "Power",
        f"  Drag power:     {result.drag_power / 1000:.2f} kW",
        "",
        "Ride",
        f"  h_front: {result.ride.h_front * 1e3:.1f} mm",
        f"  h_rear:  {result.ride.h_rear * 1e3:.1f} mm",
        f"  rake:    {result.ride.rake * 1e3:.1f} mm",
        f"  yaw:     {np_deg(result.ride.yaw_rad):.1f} deg",
    ]
    return "\n".join(lines)


def np_deg(rad: float) -> float:
    return rad * 180.0 / 3.141592653589793
