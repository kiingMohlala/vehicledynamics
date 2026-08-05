"""Driver report."""

from __future__ import annotations

from .driver_state import DriverState
from .telemetry import TelemetryLogger


def format_driver_report(state: DriverState, log: TelemetryLogger | None = None) -> str:
    lines = [
        "=== Driver ===",
        f"Mode:           {state.mode}",
        f"Time:           {state.time:.2f} s",
        f"Throttle:       {state.throttle*100:.0f} %",
        f"Brake:          {state.brake*100:.0f} %",
        f"Steer:          {state.steer:.3f} rad",
        f"Cross-track:    {state.cross_track:.3f} m",
        f"Heading error:  {state.heading_error:.3f} rad",
        f"Speed error:    {state.speed_error:.2f} m/s",
        f"Path s:         {state.s_path:.1f} m",
        f"Target speed:   {state.target_speed:.1f} m/s",
    ]
    if log is not None and log.samples:
        lines.append(f"RMS CTE:        {log.rms_cross_track:.3f} m")
        lines.append(f"Samples:        {len(log.samples)}")
    return "\n".join(lines)
