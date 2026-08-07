"""Build a full body surface set from assembly parameters or defaults."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .panel import Panel
from .panel_library import (
    hood_panel, roof_panel, door_panel, fender_panel,
    floor_panel, undertray_panel, splitter_panel, diffuser_panel, wing_panel,
)
from .trimming import StitchedBody
from .blend_surface import BlendSurface
from .fillet import FilletSurface
from .panel import Panel as PanelCls


@dataclass
class SurfaceBuilder:
    wheelbase: float = 2.70
    width: float = 1.80
    height: float = 1.15
    panels: list[Panel] = field(default_factory=list)

    @classmethod
    def from_assembly(cls, assembly: Any) -> "SurfaceBuilder":
        cfg = getattr(assembly, "config", None)
        wb = float(getattr(cfg, "wheelbase", 2.70)) if cfg else 2.70
        width = float(getattr(cfg, "body_width", 1.80)) if cfg else 1.80
        height = float(getattr(cfg, "body_height", 1.15)) if cfg else 1.15
        return cls(wheelbase=wb, width=width, height=height)

    def generate_body(self) -> StitchedBody:
        wb, w, h = self.wheelbase, self.width, self.height
        panels = [
            hood_panel(wb, w * 0.9, h),
            roof_panel(wb, w * 0.75, h),
            door_panel(1.0, wb, h),
            door_panel(-1.0, wb, h),
            fender_panel(True, 1.0, wb),
            fender_panel(True, -1.0, wb),
            fender_panel(False, 1.0, wb),
            fender_panel(False, -1.0, wb),
            floor_panel(wb, w * 0.85),
            undertray_panel(wb, w * 0.85),
            splitter_panel(w),
            diffuser_panel(wb, w * 0.75),
            wing_panel(x=wb * 0.98, span=w * 0.85, z=h * 0.85),
        ]
        self.panels = panels
        body = StitchedBody()
        for p in panels:
            body.add(p)
        return body

    def blend_panels(self, a: Panel, b: Panel) -> Panel:
        blend = BlendSurface(a.surface, b.surface)
        return PanelCls(f"blend_{a.name}_{b.name}", "blend", blend)

    def fillet_panels(self, a: Panel, b: Panel, radius: float = 0.02) -> Panel:
        fil = FilletSurface(a.surface, b.surface, radius=radius)
        return PanelCls(f"fillet_{a.name}_{b.name}", "fillet", fil)
