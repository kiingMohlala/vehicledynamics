"""Scenario definitions and library."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Any

from vehicle_dynamics.driver.maneuver_library import Maneuver, ManeuverLibrary
from vehicle_dynamics.driver.reference_paths import ReferencePath, make_straight, make_circle


@dataclass
class Scenario:
    name: str
    duration: float
    maneuver: Maneuver | None = None
    initial_vx: float = 0.0
    initial_gear: int = 1
    mu: float = 1.0
    events: list[tuple[float, str, Callable]] = field(default_factory=list)
    description: str = ""


class ScenarioLibrary:
    @staticmethod
    def straight_acceleration(duration: float = 10.0) -> Scenario:
        return Scenario(
            name="straight_acceleration",
            duration=duration,
            maneuver=ManeuverLibrary.launch_test(duration=duration),
            initial_vx=0.0,
            description="WOT launch",
        )

    @staticmethod
    def emergency_braking(v0: float = 27.8, duration: float = 5.0) -> Scenario:
        return Scenario(
            name="emergency_braking",
            duration=duration,
            maneuver=ManeuverLibrary.emergency_braking(duration=duration),
            initial_vx=v0,
            description="100 km/h panic stop",
        )

    @staticmethod
    def constant_radius(radius: float = 40.0, v: float = 15.0) -> Scenario:
        m = ManeuverLibrary.constant_speed_cornering(v=v, radius=radius)
        return Scenario(
            name="constant_radius",
            duration=m.duration,
            maneuver=m,
            initial_vx=v,
        )

    @staticmethod
    def double_lane_change(v: float = 20.0) -> Scenario:
        m = ManeuverLibrary.double_lane_change(v=v)
        return Scenario(name="double_lane_change", duration=m.duration, maneuver=m, initial_vx=v)

    @staticmethod
    def slalom(v: float = 15.0) -> Scenario:
        m = ManeuverLibrary.slalom(v=v)
        return Scenario(name="slalom", duration=max(m.duration, 12.0), maneuver=m, initial_vx=v)

    @staticmethod
    def figure_eight(v: float = 12.0) -> Scenario:
        m = ManeuverLibrary.figure_eight(v=v)
        return Scenario(name="figure_eight", duration=m.duration, maneuver=m, initial_vx=v)

    @staticmethod
    def moose_test(v: float = 18.0) -> Scenario:
        m = ManeuverLibrary.moose_test(v=v)
        return Scenario(name="moose_test", duration=m.duration, maneuver=m, initial_vx=v)

    @staticmethod
    def split_mu_braking(v0: float = 25.0) -> Scenario:
        sc = ScenarioLibrary.emergency_braking(v0=v0)
        sc.name = "split_mu_braking"
        sc.mu = 0.5
        return sc

    @staticmethod
    def wet_road(v: float = 15.0) -> Scenario:
        sc = ScenarioLibrary.straight_acceleration(8.0)
        sc.name = "wet_road"
        sc.mu = 0.6
        sc.initial_vx = v
        return sc

    @staticmethod
    def crosswind(duration: float = 8.0) -> Scenario:
        sc = Scenario(
            name="crosswind",
            duration=duration,
            maneuver=ManeuverLibrary.straight(length=150.0, v=25.0),
            initial_vx=25.0,
        )

        def gust(sim):
            sim.state.crosswind = 15.0  # m/s lateral wind proxy

        sc.events.append((2.0, "crosswind_gust", gust))
        return sc

    @staticmethod
    def drafting(duration: float = 10.0) -> Scenario:
        sc = Scenario(
            name="drafting",
            duration=duration,
            maneuver=ManeuverLibrary.straight(length=200.0, v=40.0),
            initial_vx=40.0,
        )

        def draft_on(sim):
            sim._draft_factor = 0.7  # drag reduction

        sc.events.append((1.0, "enter_draft", draft_on))
        return sc

    @staticmethod
    def hill_start() -> Scenario:
        sc = ScenarioLibrary.straight_acceleration(6.0)
        sc.name = "hill_start"
        return sc

    @staticmethod
    def launch_control() -> Scenario:
        sc = ScenarioLibrary.straight_acceleration(6.0)
        sc.name = "launch_control"
        return sc
