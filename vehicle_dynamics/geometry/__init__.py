"""Phase 13.0 – Vehicle Geometry, CAD & Class-A Surface Foundation."""

from .vector import normalize, cross, dot, distance, lerp
from .transforms import rot_x, rot_y, rot_z, euler_xyz, quat_from_axis_angle, quat_to_matrix, transform_points
from .coordinate_systems import Frame
from .curves import Line, Arc, BezierCurve
from .splines import BSplineCurve
from .nurbs import NurbsCurve
from .surfaces import LoftSurface, BilinearSurface
from .continuity import ContinuityAnalyzer, ContinuityResult
from .curvature import curve_curvature, gaussian_curvature, mean_curvature
from .tessellation import Tessellation, tessellate_surface
from .mesh import Mesh
from .class_a import ClassAReport, analyze_class_a
from .geometry_database import GeometryDatabase
from .geometry_report import format_geometry_report

__all__ = [
    "normalize", "cross", "dot", "distance", "lerp",
    "rot_x", "rot_y", "rot_z", "euler_xyz", "quat_from_axis_angle", "quat_to_matrix", "transform_points",
    "Frame",
    "Line", "Arc", "BezierCurve",
    "BSplineCurve",
    "NurbsCurve",
    "LoftSurface", "BilinearSurface",
    "ContinuityAnalyzer", "ContinuityResult",
    "curve_curvature", "gaussian_curvature", "mean_curvature",
    "Tessellation", "tessellate_surface",
    "Mesh",
    "ClassAReport", "analyze_class_a",
    "GeometryDatabase",
    "format_geometry_report",
]
