"""CAD assembly report."""
from __future__ import annotations

from typing import Any


def format_cad_report(assembly: Any, title: str = "Vehicle CAD Assembly Report") -> str:
    mp = assembly.mass_properties
    pkg = assembly.packaging()
    lines = [
        f"# {title}",
        "",
        f"**Wheelbase:** {assembly.config.wheelbase:.3f} m",
        f"**Track:** {assembly.config.track:.3f} m",
        f"**Ride height:** {assembly.config.ride_height:.3f} m",
        f"**Components:** {len(assembly.components)}",
        "",
        "## Mass properties",
        f"- Total mass: **{mp.total_mass:.1f} kg**",
        f"- CG: **[{mp.cg[0]:.3f}, {mp.cg[1]:.3f}, {mp.cg[2]:.3f}] m**",
        f"- Front axle load: **{mp.axle_load_front:.1f} kg**",
        f"- Rear axle load: **{mp.axle_load_rear:.1f} kg**",
        f"- Izz (approx): **{mp.Izz_approx:.0f} kg·m²**",
        "",
        "## Packaging",
        f"- Ground clearance (AABB): **{pkg.ground_clearance:.3f} m**",
        f"- Wheel–body gap proxy: **{pkg.wheel_to_body_min:.3f} m**",
        f"- Interference hits: **{len(pkg.interferences)}**",
        "",
        "## Component breakdown",
    ]
    for cat, m in sorted(mp.breakdown.items()):
        lines.append(f"- {cat}: {m:.1f} kg")
    return "\n".join(lines)
