"""High-level kinematics solver over travel and steering sweeps."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import numpy as np

from .hardpoints import HardpointModel
from .constraint_solver import solve_corner
from .roll_center import roll_axis
from .steering_geometry import ackermann_angles, wheel_steer_from_rack
from .anti_geometry import anti_dive, anti_squat
from .bump_steer import bump_steer_curve
from .packaging import check_corner_packaging


@dataclass
class KinematicsResults:
    suspension_type: str
    travels: np.ndarray
    corners: dict[str, list] = field(default_factory=dict)
    camber_curve: dict[str, np.ndarray] = field(default_factory=dict)
    toe_curve: dict[str, np.ndarray] = field(default_factory=dict)
    caster_curve: dict[str, np.ndarray] = field(default_factory=dict)
    roll_center_front: float = 0.0
    roll_center_rear: float = 0.0
    roll_axis_info: dict = field(default_factory=dict)
    ackermann: dict = field(default_factory=dict)
    anti_dive_front: float = 0.0
    anti_squat_rear: float = 0.0
    bump_steer: dict = field(default_factory=dict)
    packaging: list = field(default_factory=list)
    scrub_radius: dict = field(default_factory=dict)
    mechanical_trail: dict = field(default_factory=dict)
    meta: dict[str, Any] = field(default_factory=dict)


class KinematicsSolver:
    def __init__(self, model: HardpointModel, wheelbase: float = 2.7, track: float = 1.55, cg_height: float = 0.50):
        self.model = model
        self.wheelbase = wheelbase
        self.track = track
        self.cg_height = cg_height

    def solve(
        self,
        wheel_travel: tuple[float, float] = (-0.08, 0.08),
        n_points: int = 17,
        steering_angle: float = 0.0,
    ) -> KinematicsResults:
        travels = np.linspace(wheel_travel[0], wheel_travel[1], n_points)
        stype = self.model.suspension_type
        res = KinematicsResults(suspension_type=stype, travels=travels)

        for name, hp in self.model.corners.items():
            states = [solve_corner(hp, float(z), stype) for z in travels]
            res.corners[name] = states
            res.camber_curve[name] = np.array([s.camber for s in states])
            res.toe_curve[name] = np.array([s.toe for s in states])
            res.caster_curve[name] = np.array([s.caster for s in states])
            res.scrub_radius[name] = np.array([s.scrub for s in states])
            res.mechanical_trail[name] = np.array([s.trail for s in states])

        # design-position RC (travel≈0)
        i0 = int(np.argmin(np.abs(travels)))
        if "FL" in res.corners:
            res.roll_center_front = res.corners["FL"][i0].roll_center_z
            res.anti_dive_front = anti_dive(
                res.corners["FL"][i0].ic_side_view,
                res.corners["FL"][i0].contact[[0, 2]],
                self.cg_height, self.wheelbase,
            )
            res.bump_steer["FL"] = bump_steer_curve(travels, res.toe_curve["FL"])
            res.packaging = check_corner_packaging(self.model.corners["FL"].points)
        if "RL" in res.corners:
            res.roll_center_rear = res.corners["RL"][i0].roll_center_z
            res.anti_squat_rear = anti_squat(
                res.corners["RL"][i0].ic_side_view,
                res.corners["RL"][i0].contact[[0, 2]],
                self.cg_height, self.wheelbase,
            )
        res.roll_axis_info = roll_axis(res.roll_center_front, res.roll_center_rear, self.wheelbase)

        # steering / Ackermann at requested road-wheel angle (deg input)
        steer_rad = float(np.radians(steering_angle))
        res.ackermann = ackermann_angles(self.wheelbase, self.track, steer_rad)
        res.meta = {"n_points": n_points, "steering_angle_deg": steering_angle}
        return res
