"""
Integrated vehicle simulation engine.

Deterministic fixed-step loop:
  events → driver → controls → powertrain → differential → brakes → aero → vehicle → telemetry
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_dynamics.driver import DriverSolver, DriverConfig
from vehicle_dynamics.controls import ControlsSolver, ControlsConfig, DriverInputs
from vehicle_dynamics.powertrain import PowertrainSolver, EngineConfig
from vehicle_dynamics.powertrain.transmission import TransmissionSolver, TransmissionConfig
from vehicle_dynamics.powertrain.differential import DifferentialSolver, DifferentialConfig
from vehicle_dynamics.aerodynamics.coefficients import AeroConfig
from vehicle_dynamics.aerodynamics.aero_model import compute_aero_loads
from vehicle_dynamics.aerodynamics.ride_height import RideHeightState

from .simulation_state import SimulationState, VehicleState
from .timing import FixedTimestep
from .scheduler import UpdateScheduler
from .event_manager import EventManager
from .telemetry_recorder import TelemetryRecorder, SimSample
from .statistics import SimulationStatistics, compute_statistics
from .scenario_runner import Scenario, ScenarioLibrary
from .replay import ReplayBuffer


@dataclass
class SimulationConfig:
    dt: float = 0.01
    mass: float = 1400.0
    Iz: float = 2500.0
    wheelbase: float = 2.7
    track: float = 1.55
    wheel_radius: float = 0.32
    CdA: float = 0.65
    controls_enabled: bool = True
    powertrain_enabled: bool = True
    aero_enabled: bool = True
    seed: int = 0


@dataclass
class SimulationResults:
    state: SimulationState
    telemetry: TelemetryRecorder
    statistics: SimulationStatistics
    events_fired: list[str] = field(default_factory=list)

    def export_csv(self, path: str):
        return self.telemetry.export_csv(path)


class Simulation:
    def __init__(self, config: SimulationConfig | None = None):
        self.cfg = config or SimulationConfig()
        if self.cfg.dt <= 0 or self.cfg.dt > 0.1:
            raise ValueError("dt must be in (0, 0.1]")
        self.timer = FixedTimestep(dt=self.cfg.dt)
        self.scheduler = UpdateScheduler()
        self.events = EventManager()
        self.telemetry = TelemetryRecorder()
        self.state = SimulationState()
        self._draft_factor = 1.0
        self._scenario: Scenario | None = None
        self._events_fired: list[str] = []

        self.driver = DriverSolver(DriverConfig(enabled=True, mode="pure_pursuit"))
        self.controls = ControlsSolver(
            ControlsConfig(enabled=self.cfg.controls_enabled)
        )
        self.engine = PowertrainSolver(
            EngineConfig(enabled=self.cfg.powertrain_enabled)
        )
        self.trans = TransmissionSolver(
            TransmissionConfig(enabled=self.cfg.powertrain_enabled, initial_gear=1)
        )
        self.diff = DifferentialSolver(
            DifferentialConfig(enabled=True, diff_type="open")
        )
        self.aero_cfg = AeroConfig(enabled=self.cfg.aero_enabled)

    def reset(self, vx: float = 0.0, gear: int = 1) -> None:
        self.state = SimulationState(
            vehicle=VehicleState(
                vx=vx,
                engine_rpm=900.0 if vx < 1 else 2000.0,
                gear=gear,
                wheel_omega=np.ones(4) * (vx / self.cfg.wheel_radius),
            ),
            gear=gear,
        )
        self.telemetry.clear()
        self.events = EventManager()
        self._events_fired = []
        self._draft_factor = 1.0
        self.engine.reset()
        self.trans.reset()
        self.driver.model.time = 0.0
        self.driver.telemetry.clear()
        if gear != 0:
            self.trans.shift.state.current_gear = gear
            self.trans.gearbox.current_gear = gear

    def load_scenario(self, scenario: Scenario) -> None:
        self._scenario = scenario
        self.reset(vx=scenario.initial_vx, gear=scenario.initial_gear)
        self.state.mu_scale = scenario.mu
        if scenario.maneuver is not None:
            self.driver.set_maneuver(scenario.maneuver)
        for t, name, action in scenario.events:
            self.events.add(t, name, action)

    def _step_plant(self, thr: float, brk: float, steer: float, tlim: float, tv: float, dt: float) -> None:
        cfg = self.cfg
        st = self.state
        v = st.vehicle

        # --- Powertrain ---
        load = 50.0 + abs(v.vx) * 2.0
        eng = self.engine.step(throttle=thr * tlim, load_torque=load, dt=dt)
        omega_w = abs(v.vx) / cfg.wheel_radius
        tr = self.trans.step(
            eng.engine,
            clutch=1.0 if thr > 0.05 or v.vx > 1.0 else 0.3,
            requested_gear=0,
            dt=dt,
            omega_wheel=omega_w,
        )
        self.diff.set_tv_delta(tv)
        diff = self.diff.step(
            tr.wheel_torque,
            omega_left=omega_w,
            omega_right=omega_w,
            dt=dt,
            mu_left=st.mu_scale,
            mu_right=st.mu_scale,
        )
        T_drive = diff.driveline.torque_left + diff.driveline.torque_right

        # --- Brakes (simple) ---
        T_brake = brk * 2500.0 * 4  # total capacity
        T_net = T_drive - T_brake * np.sign(v.vx + 1e-6)

        # --- Aero ---
        drag = 0.0
        downforce = 0.0
        if cfg.aero_enabled and self.aero_cfg.enabled:
            ride = RideHeightState(h_front=v.ride_h_front, h_rear=v.ride_h_rear)
            aero = compute_aero_loads(max(abs(v.vx), 0.1), self.aero_cfg, ride=ride)
            drag = aero.drag * self._draft_factor
            downforce = aero.downforce_total
        # Crosswind lateral force proxy
        F_y_wind = st.crosswind * 40.0

        # --- Longitudinal dynamics ---
        Fx = T_net / cfg.wheel_radius - drag - 0.01 * cfg.mass * 9.81 * np.sign(v.vx + 1e-9)
        ax = Fx / cfg.mass
        # Lateral bicycle-ish
        vy_dot = -v.vx * v.yaw_rate + (F_y_wind + steer * 8000.0 * max(v.vx, 0.0) / 20.0) / cfg.mass
        r_dot = (steer * 4000.0 * max(v.vx, 1.0) / 20.0 + F_y_wind * 0.5) / cfg.Iz

        vx_new = v.vx + ax * dt
        # Prevent reverse unless braking from low speed
        if v.vx >= 0:
            vx_new = max(vx_new, 0.0)
        vy_new = v.vy + vy_dot * dt
        r_new = v.yaw_rate + r_dot * dt
        psi_new = v.psi + r_new * dt
        x_new = v.x + (vx_new * np.cos(psi_new) - vy_new * np.sin(psi_new)) * dt
        y_new = v.y + (vx_new * np.sin(psi_new) + vy_new * np.cos(psi_new)) * dt

        slip = 0.0
        if abs(vx_new) > 0.5 and T_drive > 100:
            slip = float(np.clip(0.05 + 0.001 * T_drive / max(st.mu_scale, 0.1), 0, 0.5))

        st.vehicle = VehicleState(
            x=float(x_new),
            y=float(y_new),
            psi=float(psi_new),
            vx=float(vx_new),
            vy=float(vy_new),
            yaw_rate=float(r_new),
            ax=float(ax),
            ay=float(vy_dot + vx_new * r_new),
            wheel_omega=np.ones(4) * (abs(vx_new) / cfg.wheel_radius),
            slip_ratio=np.array([slip * 0.5, slip * 0.5, slip, slip]),
            slip_angle=np.zeros(4),
            Fz=v.Fz + downforce / 4.0,
            engine_rpm=float(eng.engine.rpm),
            gear=int(tr.gear),
            fuel_g=float(eng.fuel.fuel_total_g),
            ride_h_front=v.ride_h_front,
            ride_h_rear=v.ride_h_rear,
            downforce=float(downforce),
            drag=float(drag),
        )
        st.throttle = thr
        st.brake = brk
        st.steer = steer
        st.engine_torque = float(eng.engine.torque_output)
        st.wheel_torque_L = float(diff.driveline.torque_left)
        st.wheel_torque_R = float(diff.driveline.torque_right)
        st.gear = int(tr.gear)
        st.time += dt

    def step(self) -> SimulationState:
        dt = self.cfg.dt
        st = self.state

        # Events
        fired = self.events.process(st.time, self)
        self._events_fired.extend(fired)

        # Driver
        drv = self.driver.step(st.vehicle.as_pose_dict(), dt)

        # Controls
        vs = st.vehicle.as_sensor_dict()
        vs["steer"] = drv.steer
        cmd = self.controls.step(vs, drv, dt)

        # Plant
        self._step_plant(
            thr=cmd.throttle,
            brk=float(np.mean(cmd.brake_pressures)),
            steer=cmd.steer if hasattr(cmd, "steer") else drv.steer,
            tlim=cmd.engine_torque_limit,
            tv=cmd.tv_request,
            dt=dt,
        )
        # Use driver steer (commands may not carry steer)
        self.state.steer = drv.steer

        # Telemetry
        v = self.state.vehicle
        self.telemetry.record(
            SimSample(
                time=self.state.time,
                x=v.x, y=v.y, psi=v.psi,
                vx=v.vx, vy=v.vy, ax=v.ax, ay=v.ay,
                yaw_rate=v.yaw_rate,
                throttle=self.state.throttle,
                brake=self.state.brake,
                steer=self.state.steer,
                engine_rpm=v.engine_rpm,
                engine_torque=self.state.engine_torque,
                gear=self.state.gear,
                torque_L=self.state.wheel_torque_L,
                torque_R=self.state.wheel_torque_R,
                downforce=v.downforce,
                drag=v.drag,
                fuel_g=v.fuel_g,
                mu_scale=self.state.mu_scale,
                slip_max=float(np.max(np.abs(v.slip_ratio))),
            )
        )
        return self.state

    def run(self, duration: float | None = None) -> SimulationResults:
        if duration is None:
            duration = self._scenario.duration if self._scenario else 10.0
        n = self.timer.n_steps(duration)
        for _ in range(n):
            self.step()
        stats = compute_statistics(self.telemetry)
        return SimulationResults(
            state=self.state,
            telemetry=self.telemetry,
            statistics=stats,
            events_fired=list(self._events_fired),
        )
