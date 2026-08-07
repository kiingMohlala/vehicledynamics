"""Phase 13.1 – Suspension Kinematics & Hardpoint Solver."""

from .hardpoints import HardpointSet, HardpointModel
from .suspension_types import SUSPENSION_TYPES
from .wheel_kinematics import CornerState, solve_double_wishbone_corner, solve_macpherson_corner
from .constraint_solver import solve_corner
from .instant_center import front_view_ic, side_view_ic
from .roll_center import roll_center_height, roll_axis
from .alignment import camber_from_upright, toe_from_heading, caster_from_kingpin, kpi_from_kingpin, scrub_radius, mechanical_trail
from .steering_geometry import ackermann_angles, ackermann_percentage, wheel_steer_from_rack
from .anti_geometry import anti_dive, anti_squat, anti_lift
from .bump_steer import bump_steer_curve, bump_steer_gradient
from .ackermann import parallel_steer, anti_ackermann
from .packaging import check_corner_packaging, ClearanceResult
from .kinematics_solver import KinematicsSolver, KinematicsResults
from .kinematics_report import format_kinematics_report

__all__ = [
    "HardpointSet", "HardpointModel", "SUSPENSION_TYPES",
    "CornerState", "solve_double_wishbone_corner", "solve_macpherson_corner", "solve_corner",
    "front_view_ic", "side_view_ic", "roll_center_height", "roll_axis",
    "camber_from_upright", "toe_from_heading", "caster_from_kingpin", "kpi_from_kingpin",
    "scrub_radius", "mechanical_trail",
    "ackermann_angles", "ackermann_percentage", "wheel_steer_from_rack",
    "anti_dive", "anti_squat", "anti_lift",
    "bump_steer_curve", "bump_steer_gradient",
    "parallel_steer", "anti_ackermann",
    "check_corner_packaging", "ClearanceResult",
    "KinematicsSolver", "KinematicsResults",
    "format_kinematics_report",
]
