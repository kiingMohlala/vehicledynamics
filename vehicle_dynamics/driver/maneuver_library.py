"""Standard vehicle dynamics maneuvers (open-loop profiles)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .driver_inputs import OpenLoopProfile
from .reference_paths import (
    ReferencePath,
    make_straight,
    make_circle,
    make_slalom,
    make_figure_eight,
)


@dataclass
class Maneuver:
    name: str
    profile: OpenLoopProfile | None = None
    path: ReferencePath | None = None
    duration: float = 10.0
    description: str = ""


class ManeuverLibrary:
    @staticmethod
    def constant_speed_cornering(v: float = 15.0, radius: float = 40.0) -> Maneuver:
        return Maneuver(
            name="constant_speed_cornering",
            path=make_circle(radius=radius, v_ref=v),
            duration=2 * np.pi * radius / max(v, 0.1),
            description="Steady skidpad",
        )

    @staticmethod
    def step_steer(steer: float = 0.1, t_step: float = 1.0, duration: float = 6.0) -> Maneuver:
        t = np.array([0.0, t_step - 1e-3, t_step, duration])
        st = np.array([0.0, 0.0, steer, steer])
        return Maneuver(
            name="step_steer",
            profile=OpenLoopProfile(
                t=t,
                throttle=np.ones_like(t) * 0.2,
                brake=np.zeros_like(t),
                steer=st,
            ),
            duration=duration,
        )

    @staticmethod
    def ramp_steer(steer_rate: float = 0.05, duration: float = 5.0) -> Maneuver:
        t = np.linspace(0, duration, 50)
        st = np.clip(steer_rate * t, -0.5, 0.5)
        return Maneuver(
            name="ramp_steer",
            profile=OpenLoopProfile(t=t, throttle=np.ones_like(t) * 0.25, brake=np.zeros_like(t), steer=st),
            duration=duration,
        )

    @staticmethod
    def double_lane_change(v: float = 20.0) -> Maneuver:
        # ISO 3888-ish lateral offsets via path
        wps = [
            (0, 0), (50, 0), (70, 3.5), (100, 3.5), (120, 0), (150, 0),
            (170, -3.5), (200, -3.5), (220, 0), (260, 0),
        ]
        from .reference_paths import make_waypoints
        return Maneuver(
            name="double_lane_change",
            path=make_waypoints(wps, v_ref=v),
            duration=260.0 / max(v, 0.1),
            description="ISO 3888 style DLC",
        )

    @staticmethod
    def moose_test(v: float = 18.0) -> Maneuver:
        wps = [(0, 0), (40, 0), (55, 3.0), (75, 3.0), (90, 0), (130, 0)]
        from .reference_paths import make_waypoints
        return Maneuver(name="moose_test", path=make_waypoints(wps, v_ref=v), duration=130 / max(v, 0.1))

    @staticmethod
    def slalom(v: float = 15.0) -> Maneuver:
        return Maneuver(name="slalom", path=make_slalom(v_ref=v), duration=10.0)

    @staticmethod
    def sine_with_dwell(amp: float = 0.12, duration: float = 8.0) -> Maneuver:
        t = np.linspace(0, duration, 200)
        # 0.7 Hz sine with 0.5 s dwell at peak
        st = amp * np.sin(2 * np.pi * 0.7 * t)
        peak_i = np.argmax(st)
        dwell = int(0.5 / (duration / len(t)))
        st[peak_i:peak_i + dwell] = amp
        return Maneuver(
            name="sine_with_dwell",
            profile=OpenLoopProfile(t=t, throttle=np.ones_like(t) * 0.2, brake=np.zeros_like(t), steer=st),
            duration=duration,
        )

    @staticmethod
    def fishhook(duration: float = 7.0) -> Maneuver:
        t = np.array([0, 0.5, 1.0, 2.0, 2.5, 4.0, duration])
        st = np.array([0, 0, 0.25, 0.25, -0.25, -0.25, -0.25])
        return Maneuver(
            name="fishhook",
            profile=OpenLoopProfile(t=t, throttle=np.ones_like(t) * 0.15, brake=np.zeros_like(t), steer=st),
            duration=duration,
        )

    @staticmethod
    def emergency_braking(duration: float = 4.0) -> Maneuver:
        t = np.array([0.0, 0.5, 0.51, duration])
        br = np.array([0.0, 0.0, 1.0, 1.0])
        return Maneuver(
            name="emergency_braking",
            profile=OpenLoopProfile(t=t, throttle=np.zeros_like(t), brake=br, steer=np.zeros_like(t)),
            duration=duration,
        )

    @staticmethod
    def launch_test(duration: float = 5.0) -> Maneuver:
        t = np.array([0.0, 0.2, duration])
        thr = np.array([0.0, 1.0, 1.0])
        return Maneuver(
            name="launch_test",
            profile=OpenLoopProfile(t=t, throttle=thr, brake=np.zeros_like(t), steer=np.zeros_like(t)),
            duration=duration,
        )

    @staticmethod
    def coastdown(duration: float = 20.0) -> Maneuver:
        t = np.array([0.0, duration])
        return Maneuver(
            name="coastdown",
            profile=OpenLoopProfile(t=t, throttle=np.zeros(2), brake=np.zeros(2), steer=np.zeros(2)),
            duration=duration,
        )

    @staticmethod
    def skidpad(radius: float = 40.0, v: float = 12.0) -> Maneuver:
        return ManeuverLibrary.constant_speed_cornering(v=v, radius=radius)

    @staticmethod
    def figure_eight(v: float = 12.0) -> Maneuver:
        return Maneuver(name="figure_eight", path=make_figure_eight(v_ref=v), duration=30.0)

    @staticmethod
    def straight(length: float = 200.0, v: float = 20.0) -> Maneuver:
        return Maneuver(name="straight", path=make_straight(length=length, v_ref=v), duration=length / max(v, 0.1))
