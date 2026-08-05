"""Path follower combining steering + speed."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .reference_paths import ReferencePath
from .steering_controller import PurePursuit, StanleyController, SteeringPID
from .speed_controller import SpeedController
from .driver_state import DriverState


@dataclass
class PathFollower:
    path: ReferencePath
    mode: str = "pure_pursuit"  # pure_pursuit | stanley | pid
    pure_pursuit: PurePursuit = None  # type: ignore
    stanley: StanleyController = None  # type: ignore
    pid_steer: SteeringPID = None  # type: ignore
    speed: SpeedController = None  # type: ignore
    lookahead: float = 8.0

    def __post_init__(self) -> None:
        self.pure_pursuit = self.pure_pursuit or PurePursuit(lookahead=self.lookahead)
        self.stanley = self.stanley or StanleyController()
        self.pid_steer = self.pid_steer or SteeringPID()
        self.speed = self.speed or SpeedController()

    def step(
        self,
        x: float,
        y: float,
        psi: float,
        v: float,
        dt: float,
        s_hint: float = 0.0,
    ) -> DriverState:
        nearest, cte = self.path.nearest(x, y)
        # Advance along path by lookahead
        s_la = min(nearest.s + self.lookahead, self.path.length)
        target = self.path.sample(s_la)
        heading_err = float(np.arctan2(np.sin(nearest.psi - psi), np.cos(nearest.psi - psi)))
        # Prefer target heading for pursuit
        target_heading_err = float(
            np.arctan2(np.sin(target.psi - psi), np.cos(target.psi - psi))
        )

        if self.mode == "stanley":
            steer = self.stanley.step(heading_err, cte, v)
        elif self.mode == "pid":
            steer = self.pid_steer.step(heading_err, dt)
        else:
            steer = self.pure_pursuit.step(x, y, psi, target.x, target.y)

        v_ref = nearest.v_ref
        thr, br = self.speed.step(v, v_ref, dt)

        return DriverState(
            s_path=nearest.s,
            cross_track=cte,
            heading_error=heading_err,
            speed_error=v_ref - v,
            throttle=thr,
            brake=br,
            steer=steer,
            target_speed=v_ref,
            mode=self.mode,
        )
