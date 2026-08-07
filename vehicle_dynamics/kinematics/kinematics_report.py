"""Human-readable kinematics report."""
from __future__ import annotations

from typing import Any
import numpy as np


def format_kinematics_report(results: Any, title: str = "Suspension Kinematics Report") -> str:
    lines = [
        f"# {title}",
        "",
        f"**Type:** {results.suspension_type}",
        f"**Travel range:** {results.travels[0]*1000:.0f} … {results.travels[-1]*1000:.0f} mm",
        "",
        "## Roll centers",
        f"- Front RC height: **{results.roll_center_front*1000:.1f} mm**",
        f"- Rear RC height: **{results.roll_center_rear*1000:.1f} mm**",
        f"- Roll axis inclination: **{results.roll_axis_info.get('inclination_deg', 0):.2f} deg**",
        "",
        "## Anti-geometry",
        f"- Anti-dive (front): **{results.anti_dive_front:.1f} %**",
        f"- Anti-squat (rear): **{results.anti_squat_rear:.1f} %**",
        "",
        "## Ackermann",
        f"- Inside: **{np.degrees(results.ackermann.get('inside', 0)):.2f} deg**",
        f"- Outside: **{np.degrees(results.ackermann.get('outside', 0)):.2f} deg**",
        "",
    ]
    if results.camber_curve:
        name = next(iter(results.camber_curve))
        camb = results.camber_curve[name]
        toe = results.toe_curve[name]
        lines += [
            f"## Alignment curves ({name})",
            f"- Camber range: **{np.degrees(camb.min()):.2f} … {np.degrees(camb.max()):.2f} deg**",
            f"- Toe range: **{np.degrees(toe.min()):.3f} … {np.degrees(toe.max()):.3f} deg**",
        ]
        if name in results.bump_steer:
            g = results.bump_steer[name]["gradient_deg_per_mm"]
            lines.append(f"- Bump-steer gradient: **{g:.4f} deg/mm**")
    if results.packaging:
        lines += ["", "## Packaging"]
        for c in results.packaging:
            status = "OK" if c.ok else "FAIL"
            lines.append(f"- {c.pair}: {c.distance*1000:.1f} mm [{status}]")
    return "\n".join(lines)
