"""Report for device-level aero."""

from __future__ import annotations

from .aero_device_solver import DeviceAeroResult


def format_device_report(result: DeviceAeroResult, title: str = "Aero Devices Report") -> str:
    s = result.state
    b = result.breakdown
    lines = [
        f"=== {title} ===",
        f"Active mode: {b.active_mode}  DRS pos: {b.drs_position:.2f}  "
        f"RW alpha: {b.rear_wing_alpha:.3f} rad",
        "",
        "Totals",
        f"  Drag:      {s.drag:8.1f} N",
        f"  DF front:  {s.downforce_front:8.1f} N",
        f"  DF rear:   {s.downforce_rear:8.1f} N",
        f"  DF total:  {s.downforce_total:8.1f} N",
        f"  Front bal: {s.front_balance*100:.1f} %",
        f"  CoP:       {s.center_of_pressure_x*1e3:.1f} mm",
        f"  L/D:       {s.L_over_D:.2f}",
        "",
        "Breakdown Fz (N)",
        f"  Body F/R:   {b.body_Fz_f:8.1f} / {b.body_Fz_r:8.1f}",
        f"  Front wing: {b.front_wing_Fz:8.1f}",
        f"  Splitter:   {b.splitter_Fz:8.1f}",
        f"  Rear wing:  {b.rear_wing_Fz:8.1f}",
        f"  Diffuser:   {b.diffuser_Fz:8.1f}",
        "",
        f"  Diffuser stalled: {b.diffuser_stalled}  RW stalled: {b.rear_wing_stalled}",
    ]
    return "\n".join(lines)
