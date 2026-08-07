"""End-to-end preprocessing pipeline."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from vehicle_dynamics.surfacing.surface_builder import SurfaceBuilder
from .surface_mesher import SurfaceMesher, SurfaceMesh
from .volume_mesher import VolumeMesher, VolumeMesh
from .mesh_refinement import RefinementMap
from .boundary_conditions import default_cfd_bcs, default_fea_bcs, BoundaryCondition
from .material_regions import default_vehicle_materials, MaterialAssignment
from .mesh_validation import validate_surface_mesh, MeshValidationReport
from .cfd_export import export_openfoam, export_su2, export_stl, export_cgns_meta
from .fea_export import export_calculix, export_abaqus_inp, export_code_aster_mesh, export_vtk


@dataclass
class PipelineConfig:
    target: str = "cfd"  # cfd | fea | both
    mesh_size: float = 0.02
    boundary_layers: int = 5
    first_layer_height: float = 0.001
    growth_rate: float = 1.25
    speed: float = 40.0
    wheelbase: float = 2.70
    width: float = 1.80
    height: float = 1.15


@dataclass
class PipelineResult:
    surface: SurfaceMesh
    volume: VolumeMesh | None
    bcs: list[BoundaryCondition]
    materials: list[MaterialAssignment]
    validation: MeshValidationReport
    refinement: RefinementMap
    config: PipelineConfig
    exports: dict[str, Path] = field(default_factory=dict)

    def export_openfoam(self, case_dir: str) -> Path:
        p = export_openfoam(self.surface.vertices, self.surface.faces, case_dir, self.bcs)
        self.exports["openfoam"] = p
        return p

    def export_su2(self, path: str) -> Path:
        p = export_su2(self.surface.vertices, self.surface.faces, path)
        self.exports["su2"] = p
        return p

    def export_stl(self, path: str) -> Path:
        p = export_stl(self.surface.vertices, self.surface.faces, path)
        self.exports["stl"] = p
        return p

    def export_calculix(self, path: str) -> Path:
        p = export_calculix(self.surface.vertices, self.surface.faces, path)
        self.exports["calculix"] = p
        return p

    def export_abaqus(self, path: str) -> Path:
        p = export_abaqus_inp(self.surface.vertices, self.surface.faces, path)
        self.exports["abaqus"] = p
        return p


class PreprocessingPipeline:
    def __init__(self, config: PipelineConfig | None = None):
        self.config = config or PipelineConfig()

    def run(self, body: Any = None) -> PipelineResult:
        cfg = self.config
        if body is None:
            body = SurfaceBuilder(wheelbase=cfg.wheelbase, width=cfg.width, height=cfg.height).generate_body()
        surface = SurfaceMesher(mesh_size=cfg.mesh_size).mesh(body)
        volume = None
        if cfg.target in ("cfd", "both") and cfg.boundary_layers > 0:
            volume = VolumeMesher().mesh_from_surface(
                surface.vertices,
                surface.faces,
                surface.normals,
                n_layers=cfg.boundary_layers,
                first_height=cfg.first_layer_height,
                growth=cfg.growth_rate,
            )
        refinement = RefinementMap(default_size=cfg.mesh_size)
        # heuristic zones from body extent
        if len(surface.vertices):
            c = surface.vertices.mean(axis=0)
            refinement.add_leading_edge(c + np.array([-cfg.wheelbase * 0.4, 0, 0]))
            refinement.add_wheel(c + np.array([0, cfg.width * 0.4, 0]))
            refinement.add_diffuser(c + np.array([cfg.wheelbase * 0.4, 0, -0.1]))
        if cfg.target == "fea":
            bcs = default_fea_bcs()
        elif cfg.target == "both":
            bcs = default_cfd_bcs(speed=cfg.speed) + default_fea_bcs()
        else:
            bcs = default_cfd_bcs(speed=cfg.speed)
        materials = default_vehicle_materials()
        validation = validate_surface_mesh(surface.vertices, surface.faces, surface.normals)
        return PipelineResult(
            surface=surface,
            volume=volume,
            bcs=bcs,
            materials=materials,
            validation=validation,
            refinement=refinement,
            config=cfg,
        )
