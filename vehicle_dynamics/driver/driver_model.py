"""Configurable driver model."""

from __future__ import annotations

from dataclasses import dataclass

from .driver_inputs import OpenLoopProfile
from .driver_state import DriverState
from .path_follower import PathFollower
from .reference_paths import ReferencePath, make_straight
from .maneuver_library import Maneuver


@dataclass
class DriverConfig:
    enabled: bool = True
    mode: str = "pure_pursuit"   # open_loop | pure_pursuit | stanley | pid | maneuver
    lookahead: float = 8.0
    wheelbase: float = 2.7


class DriverModel:
    def __init__(
        self,
        config: DriverConfig | None = None,
        path: ReferencePath | None = None,
        profile: OpenLoopProfile | None = None,
    ):
        self.cfg = config or DriverConfig()
        self.path = path or make_straight()
        self.profile = profile
        self.follower = PathFollower(path=self.path, mode=self.cfg.mode, lookahead=self.cfg.lookahead)
        self.follower.pure_pursuit.wheelbase = self.cfg.wheelbase
        self.state = DriverState()
        self.time = 0.0

    def set_path(self, path: ReferencePath) -> None:
        self.path = path
        self.follower.path = path

    def set_maneuver(self, man: Maneuver) -> None:
        if man.path is not None:
            self.set_path(man.path)
            self.cfg.mode = "pure_pursuit"
            self.follower.mode = "pure_pursuit"
        if man.profile is not None:
            self.profile = man.profile
            self.cfg.mode = "open_loop"

    def step(
        self,
        x: float,
        y: float,
        psi: float,
        v: float,
        dt: float,
        *,
        external_throttle: float = 0.0,
        external_brake: float = 0.0,
        external_steer: float = 0.0,
    ) -> DriverState:
        if not self.cfg.enabled:
            self.state = DriverState(
                time=self.time,
                throttle=external_throttle,
                brake=external_brake,
                steer=external_steer,
                mode="pass_through",
            )
            self.time += dt
            return self.state

        if self.cfg.mode == "open_loop" and self.profile is not None:
            thr, br, st = self.profile.at(self.time)
            self.state = DriverState(
                time=self.time,
                throttle=thr,
                brake=br,
                steer=st,
                mode="open_loop",
            )
        else:
            mode = self.cfg.mode if self.cfg.mode in ("pure_pursuit", "stanley", "pid") else "pure_pursuit"
            self.follower.mode = mode
            self.state = self.follower.step(x, y, psi, v, dt)
            self.state.time = self.time
            self.state.mode = mode

        self.time += dt
        return self.state
