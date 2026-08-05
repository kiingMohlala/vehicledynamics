"""Compliance analysis text report."""

from __future__ import annotations

from .compliance_solver import ComplianceState
import math


def format_compliance_report(
    state: ComplianceState, title: str = "Compliance Report"
) -> str:
    g = state.geometry
    lines = [
        f"=== {title} ===",
        f"Mode: {state.mode}  Success: {state.success} ({state.message})",
        f"Max node |u|:     {state.max_node_disp * 1e3:.4f} mm",
        f"Max pickup |u|:   {g.max_pickup_disp * 1e3:.4f} mm",
        f"Chassis twist:    {math.degrees(g.chassis_twist_rad):.4f} deg",
        f"Strain energy:    {state.strain_energy:.4f} J",
        "",
        "Alignment deltas (deg):",
        f"  Camber FL/FR: {math.degrees(g.d_camber_fl):+.4f} / {math.degrees(g.d_camber_fr):+.4f}",
        f"  Camber RL/RR: {math.degrees(g.d_camber_rl):+.4f} / {math.degrees(g.d_camber_rr):+.4f}",
        f"  Toe    FL/FR: {math.degrees(g.d_toe_fl):+.4f} / {math.degrees(g.d_toe_fr):+.4f}",
        f"  Toe    RL/RR: {math.degrees(g.d_toe_rl):+.4f} / {math.degrees(g.d_toe_rr):+.4f}",
        "",
        f"Track Δ front/rear: {g.d_track_front*1e3:+.3f} / {g.d_track_rear*1e3:+.3f} mm",
        f"RC height Δ F/R:    {g.d_rc_front*1e3:+.3f} / {g.d_rc_rear*1e3:+.3f} mm",
    ]
    return "\n".join(lines)
