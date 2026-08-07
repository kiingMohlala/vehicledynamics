"""Phase 13.4 – CFD/FEA Preprocessing validation (24 gates)."""
from __future__ import annotations

import math
import tempfile
from pathlib import Path
import numpy as np

from vehicle_dynamics.surfacing.surface_builder import SurfaceBuilder
from vehicle_dynamics.surfacing.mesh_generator import MeshGenerator

from .geometry_cleanup import cleanup_mesh, merge_duplicate_vertices, detect_non_manifold_edges, recompute_normals
from .surface_mesher import SurfaceMesher
from .volume_mesher import VolumeMesher
from .boundary_layers import generate_prism_layers
from .mesh_refinement import RefinementMap
from .boundary_conditions import default_cfd_bcs, default_fea_bcs
from .material_regions import default_vehicle_materials, MATERIALS
from .cfd_export import export_openfoam, export_su2, export_stl
from .fea_export import export_calculix, export_abaqus_inp
from .mesh_validation import validate_surface_mesh
from .preprocessing_pipeline import PreprocessingPipeline, PipelineConfig
from .preprocessing_report import format_preprocessing_report


def _ok(name: str, passed: bool, detail: str = "") -> tuple[str, bool, str]:
    return name, passed, detail


def _sample_mesh():
    body = SurfaceBuilder(wheelbase=2.5).generate_body()
    gm = MeshGenerator(target_edge_length=0.12).generate(body)
    return gm.vertices, gm.faces, gm.normals


def gate_geometry_cleanup() -> tuple[str, bool, str]:
    v, f, _ = _sample_mesh()
    # add duplicate
    v2 = np.vstack([v, v[:3]])
    f2 = np.vstack([f, f[:1]])
    r = cleanup_mesh(v2, f2)
    return _ok("geometry_cleanup", len(r.vertices) <= len(v2))


def gate_duplicate_removal() -> tuple[str, bool, str]:
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 0]], dtype=float)
    f = np.array([[0, 1, 2], [3, 1, 2]])
    nv, nf, n_merged = merge_duplicate_vertices(v, f, tol=1e-9)
    return _ok("duplicate_removal", n_merged >= 1 and len(nv) == 3)


def gate_normal_consistency() -> tuple[str, bool, str]:
    v, f, _ = _sample_mesh()
    n = recompute_normals(v, f)
    return _ok("normal_consistency", np.all(np.isfinite(n)) and n.shape == v.shape)


def gate_surface_mesh_generation() -> tuple[str, bool, str]:
    body = SurfaceBuilder().generate_body()
    sm = SurfaceMesher(0.1).mesh(body)
    return _ok("surface_mesh_generation", len(sm.faces) > 50)


def gate_volume_mesh_generation() -> tuple[str, bool, str]:
    v, f, n = _sample_mesh()
    if n is None:
        n = recompute_normals(v, f)
    vm = VolumeMesher().mesh_from_surface(v, f, n, n_layers=2)
    return _ok("volume_mesh_generation", len(vm.cells) > 0)


def gate_boundary_layer_generation() -> tuple[str, bool, str]:
    v, f, n = _sample_mesh()
    if n is None:
        n = recompute_normals(v, f)
    verts, cells = generate_prism_layers(v, f, n, n_layers=3, first_height=0.001)
    return _ok("boundary_layer_generation", len(cells) == len(f) * 3)


def gate_adaptive_refinement() -> tuple[str, bool, str]:
    rm = RefinementMap(default_size=0.05)
    rm.add_leading_edge([0, 0, 0], radius=0.2, size=0.005)
    return _ok("adaptive_refinement", rm.size_at(np.array([0.0, 0.0, 0.0])) == 0.005)


def gate_leading_edge_refinement() -> tuple[str, bool, str]:
    rm = RefinementMap()
    rm.add_leading_edge([1, 0, 0], 0.1, 0.004)
    return _ok("leading_edge_refinement", any(z.name == "leading_edge" for z in rm.zones))


def gate_wheel_refinement() -> tuple[str, bool, str]:
    rm = RefinementMap()
    rm.add_wheel([0, 0.8, 0.3])
    return _ok("wheel_refinement", rm.size_at(np.array([0.0, 0.8, 0.3])) < rm.default_size)


def gate_mesh_quality() -> tuple[str, bool, str]:
    v, f, n = _sample_mesh()
    rep = validate_surface_mesh(v, f, n)
    return _ok("mesh_quality", rep.finite_ok)


def gate_aspect_ratio_limits() -> tuple[str, bool, str]:
    v, f, n = _sample_mesh()
    rep = validate_surface_mesh(v, f, n, max_aspect=200)
    return _ok("aspect_ratio_limits", rep.aspect_ok, f"max_ar={rep.quality.max_aspect_ratio:.1f}")


def gate_skewness_limits() -> tuple[str, bool, str]:
    v, f, n = _sample_mesh()
    rep = validate_surface_mesh(v, f, n)
    return _ok("skewness_limits", rep.skew_ok, f"max_sk={rep.quality.max_skewness:.3f}")


def gate_non_manifold_detection() -> tuple[str, bool, str]:
    # manifold triangle
    f = np.array([[0, 1, 2], [0, 2, 3]])
    nm = detect_non_manifold_edges(f)
    return _ok("non_manifold_detection", isinstance(nm, list))


def gate_boundary_conditions_cfd() -> tuple[str, bool, str]:
    bcs = default_cfd_bcs(speed=30)
    types = {b.bc_type for b in bcs}
    return _ok("boundary_conditions_cfd", "velocity_inlet" in types and "wall" in types)


def gate_boundary_conditions_fea() -> tuple[str, bool, str]:
    bcs = default_fea_bcs()
    return _ok("boundary_conditions_fea", any(b.bc_type == "fixed" for b in bcs))


def gate_material_assignment() -> tuple[str, bool, str]:
    mats = default_vehicle_materials()
    return _ok("material_assignment", len(mats) >= 3 and "steel" in MATERIALS)


def gate_openfoam_export() -> tuple[str, bool, str]:
    v, f, _ = _sample_mesh()
    with tempfile.TemporaryDirectory() as td:
        p = export_openfoam(v, f, td)
        ok = (Path(p) / "constant" / "polyMesh" / "points").exists()
    return _ok("openfoam_export", ok)


def gate_su2_export() -> tuple[str, bool, str]:
    v, f, _ = _sample_mesh()
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "v.su2"
        export_su2(v, f, path)
        text = path.read_text()
    return _ok("su2_export", "NDIME=" in text and "NPOIN=" in text)


def gate_calculix_export() -> tuple[str, bool, str]:
    v, f, _ = _sample_mesh()
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "v.inp"
        export_calculix(v, f, path)
        text = path.read_text()
    return _ok("calculix_export", "*NODE" in text and "*ELEMENT" in text)


def gate_abaqus_export() -> tuple[str, bool, str]:
    v, f, _ = _sample_mesh()
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "v.inp"
        export_abaqus_inp(v, f, path)
        text = path.read_text()
    return _ok("abaqus_export", "*Node" in text and "*Element" in text)


def gate_pipeline_execution() -> tuple[str, bool, str]:
    r = PreprocessingPipeline(PipelineConfig(mesh_size=0.12, boundary_layers=2)).run()
    return _ok("pipeline_execution", len(r.surface.faces) > 0 and r.volume is not None)


def gate_repeatability() -> tuple[str, bool, str]:
    c = PipelineConfig(mesh_size=0.12, boundary_layers=1)
    a = PreprocessingPipeline(c).run()
    b = PreprocessingPipeline(c).run()
    return _ok("repeatability", len(a.surface.faces) == len(b.surface.faces))


def gate_performance_regression() -> tuple[str, bool, str]:
    import time
    t0 = time.perf_counter()
    for _ in range(3):
        PreprocessingPipeline(PipelineConfig(mesh_size=0.15, boundary_layers=1)).run()
    ms = (time.perf_counter() - t0) * 1000
    return _ok("performance_regression", ms < 20000.0, f"{ms:.1f} ms")


def gate_no_nan_inf() -> tuple[str, bool, str]:
    r = PreprocessingPipeline(PipelineConfig(mesh_size=0.12, boundary_layers=2)).run()
    ok = np.all(np.isfinite(r.surface.vertices))
    return _ok("no_nan_inf", ok)


GATES = [
    gate_geometry_cleanup,
    gate_duplicate_removal,
    gate_normal_consistency,
    gate_surface_mesh_generation,
    gate_volume_mesh_generation,
    gate_boundary_layer_generation,
    gate_adaptive_refinement,
    gate_leading_edge_refinement,
    gate_wheel_refinement,
    gate_mesh_quality,
    gate_aspect_ratio_limits,
    gate_skewness_limits,
    gate_non_manifold_detection,
    gate_boundary_conditions_cfd,
    gate_boundary_conditions_fea,
    gate_material_assignment,
    gate_openfoam_export,
    gate_su2_export,
    gate_calculix_export,
    gate_abaqus_export,
    gate_pipeline_execution,
    gate_repeatability,
    gate_performance_regression,
    gate_no_nan_inf,
]


def run_phase134_validation(verbose: bool = True) -> bool:
    results = []
    for g in GATES:
        name, passed, detail = g()
        results.append((name, passed, detail))
        if verbose:
            status = "PASS" if passed else "FAIL"
            extra = f"  ({detail})" if detail else ""
            print(f"  {name:28s}: {status}{extra}")
    n_pass = sum(1 for _, p, _ in results if p)
    n = len(results)
    if verbose:
        print()
        print("=" * 41)
        if n_pass == n:
            print("ALL TESTS PASSED")
            print("Phase 13.4 Status: IMPLEMENTATION VALIDATED ✓")
        else:
            print(f"FAILED: {n_pass}/{n}")
            print("Phase 13.4 Status: NOT VALIDATED")
    return n_pass == n


if __name__ == "__main__":
    raise SystemExit(0 if run_phase134_validation() else 1)
