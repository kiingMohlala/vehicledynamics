"""Phase 13.3 – Parametric Surface Modeling & Mesh Generation."""

from .panel import Panel
from .surface_builder import SurfaceBuilder
from .mesh_generator import MeshGenerator, GeneratedMesh
from .mesh_quality import MeshQuality, evaluate_mesh_quality
from .trimming import StitchedBody, TrimmedSurface, TrimDomain
from .loft_builder import MultiLoftSurface, loft_from_points
from .sweep_builder import SweepSurface
from .blend_surface import BlendSurface
from .fillet import FilletSurface
from .surfacing_report import format_surfacing_report
from .panel_library import hood_panel, roof_panel, door_panel, diffuser_panel

__all__ = [
    "Panel",
    "SurfaceBuilder",
    "MeshGenerator", "GeneratedMesh",
    "MeshQuality", "evaluate_mesh_quality",
    "StitchedBody", "TrimmedSurface", "TrimDomain",
    "MultiLoftSurface", "loft_from_points",
    "SweepSurface",
    "BlendSurface",
    "FilletSurface",
    "format_surfacing_report",
    "hood_panel", "roof_panel", "door_panel", "diffuser_panel",
]
