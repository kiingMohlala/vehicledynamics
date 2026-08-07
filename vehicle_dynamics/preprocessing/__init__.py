"""Phase 13.4 – CFD/FEA Preprocessing & Automatic Mesh Pipeline."""

from .preprocessing_pipeline import PreprocessingPipeline, PipelineConfig, PipelineResult
from .surface_mesher import SurfaceMesher, SurfaceMesh
from .volume_mesher import VolumeMesher, VolumeMesh
from .boundary_conditions import BoundaryCondition, default_cfd_bcs, default_fea_bcs
from .material_regions import Material, MaterialAssignment, MATERIALS, default_vehicle_materials
from .mesh_refinement import RefinementMap, RefinementZone
from .geometry_cleanup import cleanup_mesh, CleanupResult
from .mesh_validation import validate_surface_mesh, MeshValidationReport
from .preprocessing_report import format_preprocessing_report
from .cfd_export import export_openfoam, export_su2, export_stl
from .fea_export import export_calculix, export_abaqus_inp, export_vtk

__all__ = [
    "PreprocessingPipeline", "PipelineConfig", "PipelineResult",
    "SurfaceMesher", "SurfaceMesh",
    "VolumeMesher", "VolumeMesh",
    "BoundaryCondition", "default_cfd_bcs", "default_fea_bcs",
    "Material", "MaterialAssignment", "MATERIALS", "default_vehicle_materials",
    "RefinementMap", "RefinementZone",
    "cleanup_mesh", "CleanupResult",
    "validate_surface_mesh", "MeshValidationReport",
    "format_preprocessing_report",
    "export_openfoam", "export_su2", "export_stl",
    "export_calculix", "export_abaqus_inp", "export_vtk",
]
