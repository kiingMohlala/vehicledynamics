"""Phase 13.0 – Vehicle Geometry & Class-A Surface Foundation validation (20 gates)."""
from __future__ import annotations

import math
import numpy as np

from .vector import normalize, cross, dot, distance, lerp
from .transforms import rot_z, quat_from_axis_angle, quat_to_matrix, transform_points
from .curves import Line, Arc, BezierCurve
from .splines import BSplineCurve
from .nurbs import NurbsCurve
from .surfaces import LoftSurface, BilinearSurface
from .continuity import ContinuityAnalyzer
from .curvature import curve_curvature, gaussian_curvature, mean_curvature
from .tessellation import tessellate_surface
from .mesh import Mesh
from .geometry_database import GeometryDatabase
from .geometry_report import format_geometry_report
from .class_a import analyze_class_a


def _ok(name: str, passed: bool, detail: str = "") -> tuple[str, bool, str]:
    return name, passed, detail


def gate_vector_math() -> tuple[str, bool, str]:
    a = np.array([1.0, 0.0, 0.0])
    b = np.array([0.0, 1.0, 0.0])
    c = cross(a, b)
    ok = abs(dot(c, a)) < 1e-12 and abs(distance(a, b) - math.sqrt(2)) < 1e-12
    return _ok("vector_math", ok)


def gate_transformations() -> tuple[str, bool, str]:
    R = rot_z(math.pi / 2)
    p = transform_points(np.array([1.0, 0.0, 0.0]), R, [0, 0, 0])
    q = quat_from_axis_angle([0, 0, 1], math.pi / 2)
    Rq = quat_to_matrix(q)
    ok = abs(p[1] - 1.0) < 1e-9 and abs(Rq[0, 1] + 1) < 1e-9
    return _ok("transformations", ok)


def gate_curve_evaluation() -> tuple[str, bool, str]:
    line = Line([0, 0, 0], [2, 0, 0])
    p = line.evaluate(0.5)
    return _ok("curve_evaluation", abs(p[0] - 1.0) < 1e-12)


def gate_curve_length() -> tuple[str, bool, str]:
    line = Line([0, 0, 0], [3, 4, 0])
    return _ok("curve_length", abs(line.length() - 5.0) < 1e-12)


def gate_bezier_continuity() -> tuple[str, bool, str]:
    # two cubic Beziers joined with G1 (shared endpoint + collinear handles)
    a = BezierCurve([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]])
    b = BezierCurve([[3, 0, 0], [4, 0, 0], [5, 0, 0], [6, 0, 0]])
    r = ContinuityAnalyzer().analyze_curves(a, b)
    return _ok("bezier_continuity", r.g0 and r.g1, f"g0={r.g0} g1={r.g1} g2={r.g2}")


def gate_bspline_generation() -> tuple[str, bool, str]:
    cps = np.array([[0, 0, 0], [1, 1, 0], [2, 1, 0], [3, 0, 0], [4, 0, 0]], dtype=float)
    c = BSplineCurve(cps, degree=3)
    p0, p1 = c.evaluate(0.0), c.evaluate(1.0)
    ok = np.allclose(p0, cps[0]) and np.allclose(p1, cps[-1])
    return _ok("bspline_generation", ok)


def gate_nurbs_generation() -> tuple[str, bool, str]:
    cps = np.array([[0, 0, 0], [1, 1, 0], [2, 0, 0], [3, 1, 0]], dtype=float)
    c = NurbsCurve(cps, weights=[1, 1, 1, 1], degree=3)
    pts = c.sample(20)
    return _ok("nurbs_generation", pts.shape == (20, 3) and np.all(np.isfinite(pts)))


def gate_surface_loft() -> tuple[str, bool, str]:
    a = BezierCurve([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]])
    b = BezierCurve([[0, 1, 0], [1, 1, 0], [2, 1, 0], [3, 1, 0]])
    s = LoftSurface(a, b)
    p = s.evaluate(0.5, 0.5)
    return _ok("surface_loft", abs(p[1] - 0.5) < 1e-9 and abs(p[0] - 1.5) < 0.2)


def gate_surface_normals() -> tuple[str, bool, str]:
    s = BilinearSurface(
        np.array([0, 0, 0.0]), np.array([1, 0, 0.0]),
        np.array([0, 1, 0.0]), np.array([1, 1, 0.0]),
    )
    n = s.normal(0.5, 0.5)
    return _ok("surface_normals", abs(abs(n[2]) - 1.0) < 1e-6, f"n={n}")


def gate_surface_tessellation() -> tuple[str, bool, str]:
    a = Line([0, 0, 0], [1, 0, 0])
    b = Line([0, 1, 0], [1, 1, 0])
    tess = tessellate_surface(LoftSurface(a, b), nu=5, nv=4)
    return _ok("surface_tessellation", tess.n_vertices == 20 and tess.n_faces > 0)


def gate_g0_continuity() -> tuple[str, bool, str]:
    a = Line([0, 0, 0], [1, 0, 0])
    b = Line([1, 0, 0], [2, 0, 0])
    r = ContinuityAnalyzer().analyze_curves(a, b)
    return _ok("g0_continuity", r.g0)


def gate_g1_continuity() -> tuple[str, bool, str]:
    a = Line([0, 0, 0], [1, 0, 0])
    b = Line([1, 0, 0], [2, 0, 0])
    r = ContinuityAnalyzer().analyze_curves(a, b)
    return _ok("g1_continuity", r.g1)


def gate_g2_continuity() -> tuple[str, bool, str]:
    # straight line has zero curvature both sides → G2
    a = Line([0, 0, 0], [1, 0, 0])
    b = Line([1, 0, 0], [2, 0, 0])
    r = ContinuityAnalyzer().analyze_curves(a, b)
    return _ok("g2_continuity", r.g2, f"err={r.g2_error:.4f}")


def gate_g3_continuity() -> tuple[str, bool, str]:
    a = Line([0, 0, 0], [1, 0, 0])
    b = Line([1, 0, 0], [2, 0, 0])
    r = ContinuityAnalyzer().analyze_curves(a, b)
    return _ok("g3_continuity", r.g3)


def gate_gaussian_curvature() -> tuple[str, bool, str]:
    # planar grid → Gaussian curvature ≈ 0
    u = np.linspace(0, 1, 12)
    v = np.linspace(0, 1, 12)
    grid = np.zeros((12, 12, 3))
    for i, ui in enumerate(u):
        for j, vj in enumerate(v):
            grid[i, j] = [ui, vj, 0.0]
    K = gaussian_curvature(grid)
    return _ok("gaussian_curvature", float(np.max(np.abs(K))) < 1e-6, f"max|K|={np.max(np.abs(K)):.2e}")


def gate_mesh_generation() -> tuple[str, bool, str]:
    s = BilinearSurface(
        np.array([0, 0, 0.0]), np.array([1, 0, 0.0]),
        np.array([0, 1, 0.0]), np.array([1, 1, 0.1]),
    )
    mesh = Mesh.from_surface(s, nu=8, nv=8, name="panel")
    return _ok("mesh_generation", mesh.n_vertices > 0 and mesh.n_faces > 0)


def gate_geometry_database() -> tuple[str, bool, str]:
    db = GeometryDatabase()
    db.add_hardpoint("LCA_front", [0.5, 0.7, -0.2])
    db.add_curve("roof", BezierCurve([[0, 0, 1], [1, 0, 1.2], [2, 0, 1.1], [3, 0, 1]]))
    return _ok("geometry_database", len(db) == 2 and "LCA_front" in db.hardpoints)


def gate_performance_regression() -> tuple[str, bool, str]:
    import time
    cps = np.random.default_rng(0).random((8, 3))
    c = NurbsCurve(cps, degree=3)
    t0 = time.perf_counter()
    for _ in range(200):
        c.sample(40)
    ms = (time.perf_counter() - t0) * 1000
    return _ok("performance_regression", ms < 5000.0, f"{ms:.1f} ms")


def gate_no_nan_inf() -> tuple[str, bool, str]:
    c = BezierCurve([[0, 0, 0], [1, 2, 0], [2, -1, 0], [3, 0, 0]])
    pts = c.sample(30)
    return _ok("no_nan_inf", np.all(np.isfinite(pts)))


def gate_regression_contract() -> tuple[str, bool, str]:
    try:
        from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
        sim = Simulation(SimulationConfig(dt=0.02))
        sim.load_scenario(ScenarioLibrary.straight_acceleration(0.3))
        r = sim.run(0.3)
        return _ok("regression_contract", len(r.telemetry.samples) > 0)
    except Exception as e:
        return _ok("regression_contract", False, str(e))


GATES = [
    gate_vector_math,
    gate_transformations,
    gate_curve_evaluation,
    gate_curve_length,
    gate_bezier_continuity,
    gate_bspline_generation,
    gate_nurbs_generation,
    gate_surface_loft,
    gate_surface_normals,
    gate_surface_tessellation,
    gate_g0_continuity,
    gate_g1_continuity,
    gate_g2_continuity,
    gate_g3_continuity,
    gate_gaussian_curvature,
    gate_mesh_generation,
    gate_geometry_database,
    gate_performance_regression,
    gate_no_nan_inf,
    gate_regression_contract,
]


def run_phase130_validation(verbose: bool = True) -> bool:
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
            print("Phase 13.0 Status: IMPLEMENTATION VALIDATED ✓")
        else:
            print(f"FAILED: {n_pass}/{n}")
            print("Phase 13.0 Status: NOT VALIDATED")
    return n_pass == n


if __name__ == "__main__":
    raise SystemExit(0 if run_phase130_validation() else 1)
