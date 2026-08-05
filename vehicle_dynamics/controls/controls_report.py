"""Controls report."""

from __future__ import annotations

from .controller_state import ActuatorCommands, ControllerState


def format_controls_report(
    cmd: ActuatorCommands,
    state: ControllerState,
    title: str = "Controls",
) -> str:
    lines = [
        f"=== {title} ===",
        f"Throttle:        {cmd.throttle*100:.0f} %",
        f"Torque limit:    {cmd.engine_torque_limit*100:.0f} %",
        f"Brakes FLFRRLRR: {cmd.brake_pressures}",
        f"TV request:      {cmd.tv_request:.1f} N·m",
        f"Clutch:          {cmd.clutch*100:.0f} %",
        f"ABS active:      {state.abs_active}",
        f"TC active:       {state.tc_active}",
        f"ESC active:      {state.esc_active}",
        f"EBD active:      {state.ebd_active}",
        f"Launch:          {state.launch_active}",
        f"Hill hold:       {state.hill_hold_active}",
        f"Yaw error:       {state.yaw_error:.4f} rad/s",
        f"μ est:           {state.mu_est:.2f}",
    ]
    return "\n".join(lines)
