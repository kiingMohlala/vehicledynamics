"""
Driver solver: produces DriverInputs for Phase 11.0 controls layer.
"""

from __future__ import annotations

from dataclasses import dataclass

from vehicle_dynamics.controls.driver_request import DriverInputs

from .driver_model import DriverConfig, DriverModel
from .maneuver_library import Maneuver
from .telemetry import TelemetryLogger, TelemetrySample
from .reference_paths import ReferencePath


@dataclass
class DriverSolver:
    config: DriverConfig | None = None

    def __post_init__(self) -> None:
        self.model = DriverModel(self.config or DriverConfig())
        self.telemetry = TelemetryLogger()

    def set_maneuver(self, man: Maneuver) -> None:
        self.model.set_maneuver(man)
        self.model.time = 0.0
        self.telemetry.clear()

    def set_path(self, path: ReferencePath) -> None:
        self.model.set_path(path)

    def step(
        self,
        vehicle_pose: dict,
        dt: float = 0.01,
        *,
        external: DriverInputs | None = None,
    ) -> DriverInputs:
        """
        vehicle_pose keys: x, y, psi, vx, (optional ax, ay, yaw_rate)
        Returns DriverInputs for ControlsSolver.
        """
        ext = external or DriverInputs()
        st = self.model.step(
            x=float(vehicle_pose.get("x", 0.0)),
            y=float(vehicle_pose.get("y", 0.0)),
            psi=float(vehicle_pose.get("psi", 0.0)),
            v=float(vehicle_pose.get("vx", 0.0)),
            dt=dt,
            external_throttle=ext.throttle,
            external_brake=ext.brake,
            external_steer=ext.steer,
        )
        self.telemetry.log(
            TelemetrySample(
                time=st.time,
                x=float(vehicle_pose.get("x", 0.0)),
                y=float(vehicle_pose.get("y", 0.0)),
                psi=float(vehicle_pose.get("psi", 0.0)),
                vx=float(vehicle_pose.get("vx", 0.0)),
                ax=float(vehicle_pose.get("ax", 0.0)),
                ay=float(vehicle_pose.get("ay", 0.0)),
                yaw_rate=float(vehicle_pose.get("yaw_rate", 0.0)),
                throttle=st.throttle,
                brake=st.brake,
                steer=st.steer,
                cross_track=st.cross_track,
                heading_error=st.heading_error,
                speed_error=st.speed_error,
                s_path=st.s_path,
                target_speed=st.target_speed,
            )
        )
        return DriverInputs(
            throttle=st.throttle,
            brake=st.brake,
            steer=st.steer,
            clutch=ext.clutch,
            gear_request=ext.gear_request,
        )
