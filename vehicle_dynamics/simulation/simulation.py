"""
Integrated vehicle simulation engine.

Deterministic fixed-step loop:
  events → driver → strategy → controls → powertrain → differential → brakes → aero → vehicle → telemetry
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_dynamics.driver import DriverSolver, DriverConfig
from vehicle_dynamics.controls import ControlsSolver, ControlsConfig, DriverInputs
from vehicle_dynamics.powertrain import PowertrainSolver, EngineConfig
from vehicle_dynamics.powertrain.transmission import TransmissionSolver, TransmissionConfig
from vehicle_dynamics.powertrain.differential import DifferentialSolver, DifferentialConfig
from vehicle_dynamics.powertrain.strategy import (
    StrategySolver,
    StrategyConfig,
    DriverCommand as StrategyDriverCommand,
)
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
from .dual_track_plant import DualTrackPlant, DualTrackConfig


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
    strategy_enabled: bool = True
    drive_mode: str = "normal"
    aero_enabled: bool = True
    seed: int = 0
    # Powertrain binding (from VehicleDefinition / twin — no silent underpower)
    peak_torque_nm: float = 400.0
    peak_torque_rpm: float = 4500.0
    peak_power_kw: float = 0.0  # if >0, peak_torque derived if peak_torque still default
    redline_rpm: float = 7500.0
    idle_rpm: float = 900.0
    final_drive: float = 3.9
    mu_tire: float = 1.1  # longitudinal friction limit for Fx clamp
    use_dual_track: bool = True  # Phase 14.2C authoritative plant
    abs_enabled: bool = True
    drive_split_front: float = 0.35


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
        peak_tq = float(self.cfg.peak_torque_nm)
        if self.cfg.peak_power_kw and self.cfg.peak_power_kw > 0:
            # τ = P / ω at peak-torque rpm (engineering estimate if power specified)
            omega_pt = max(self.cfg.peak_torque_rpm, 1000.0) * 2.0 * np.pi / 60.0
            peak_from_power = (self.cfg.peak_power_kw * 1000.0) / omega_pt
            peak_tq = max(peak_tq, peak_from_power)
        self.engine = PowertrainSolver(
            EngineConfig(
                enabled=self.cfg.powertrain_enabled,
                peak_torque=peak_tq,
                peak_torque_rpm=self.cfg.peak_torque_rpm,
                redline_rpm=self.cfg.redline_rpm,
                idle_rpm=self.cfg.idle_rpm,
            )
        )
        self.trans = TransmissionSolver(
            TransmissionConfig(
                enabled=self.cfg.powertrain_enabled,
                initial_gear=1,
                final_drive=self.cfg.final_drive,
            )
        )
        self.diff = DifferentialSolver(
            DifferentialConfig(enabled=True, diff_type="open")
        )
        self.strategy = StrategySolver(
            StrategyConfig(
                enabled=self.cfg.strategy_enabled,
                drive_mode=self.cfg.drive_mode,
                transmission_mode="automatic",
            )
        )
        self.aero_cfg = AeroConfig(enabled=self.cfg.aero_enabled)
        self._last_strategy = None
        self.dual_track = None
        if self.cfg.use_dual_track:
            L = float(self.cfg.wheelbase)
            self.dual_track = DualTrackPlant(DualTrackConfig(
                mass=self.cfg.mass,
                Iz=self.cfg.Iz,
                a=0.45 * L,
                b=0.55 * L,
                track_f=self.cfg.track,
                track_r=self.cfg.track * 0.98,
                wheel_radius=self.cfg.wheel_radius,
                mu=self.cfg.mu_tire,
                abs_enabled=self.cfg.abs_enabled,
                drive_split_front=self.cfg.drive_split_front,
            ))

        self._last_clutch_torque = 0.0  # reaction load on crank from previous step
        self._trace: dict[str, float] = {}  # last plant trace for diagnostics
        self._shift_cooldown = 0.0  # seconds remaining before next auto upshift request

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
        self._last_clutch_torque = 0.0
        self._trace = {}
        self._shift_cooldown = 0.0
        if self.dual_track is not None:
            self.dual_track.reset(vx=vx)
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

    def _step_plant(
        self,
        thr: float,
        brk: float,
        steer: float,
        tlim: float,
        tv: float,
        dt: float,
        gear_request: int = 0,
        clutch_cmd: float | None = None,
    ) -> None:
        """
        Coupled plant.

        Phase 14.2B fix: do NOT apply an arbitrary crank load (50+vx*2) independent of
        the driveline. Road loads act at the wheels; the crank sees the clutch reaction
        torque from the previous step (and kinematic lock when the clutch is locked).
        """
        cfg = self.cfg
        st = self.state
        v = st.vehicle

        # --- Powertrain ---
        # Engine load: clutch reaction from previous step (road load path closes through tires).
        # Cap so launch does not instantly stall the map; dual-track still uses real wheel Fx.
        load = float(np.clip(self._last_clutch_torque, 0.0, 320.0))
        eng = self.engine.step(throttle=float(np.clip(thr * tlim, 0.0, 1.0)), load_torque=load, dt=dt)
        omega_w = abs(v.vx) / max(cfg.wheel_radius, 1e-6)

        # Clutch schedule: slip on launch, lock when on the map
        rpm = float(eng.engine.rpm)
        if thr > 0.05:
            if rpm < 2000.0 or v.vx < 4.0:
                clutch = float(np.clip(0.20 + 0.0005 * max(rpm - 900.0, 0.0) + 0.08 * v.vx, 0.15, 0.85))
            else:
                clutch = 1.0
        else:
            clutch = 0.25 if v.vx < 1.0 else 0.9

        # Plant-side sequential upshift with cooldown — only when post-shift RPM stays useful
        self._shift_cooldown = max(0.0, self._shift_cooldown - dt)
        req_gear = 0
        in_gear = int(getattr(self.trans.state, "gear", 1) or 1)
        if in_gear <= 0:
            in_gear = 1
        upshift_rpm = 0.82 * cfg.redline_rpm if thr > 0.5 else 0.72 * cfg.redline_rpm
        if (
            self._shift_cooldown <= 0.0
            and eng.engine.rpm > upshift_rpm
            and thr > 0.25
            and in_gear >= 1
            and in_gear < 6
            and not bool(getattr(self.trans.state, "shift_active", False))
            and v.vx > 2.0  # don't upshift while essentially stationary
        ):
            req_gear = 1  # sequential +1
            self._shift_cooldown = 1.4

        tr = self.trans.step(
            eng.engine,
            clutch=clutch,
            requested_gear=req_gear,
            dt=dt,
            omega_wheel=omega_w,
            launch=(thr > 0.7 and v.vx < 6.0),
        )
        self._last_clutch_torque = float(abs(tr.clutch_torque))

        # Kinematic lock when clutch is locked: engine tracks wheel * overall ratio
        if tr.locked and int(tr.gear) != 0 and omega_w > 0.05:
            try:
                ratio = float(self.trans.gearbox.ratios.overall(tr.gear))
            except Exception:
                try:
                    ratio = float(self.trans.gearbox.ratios.ratio(tr.gear)) * float(
                        self.trans.gearbox.ratios.final_drive
                    )
                except Exception:
                    ratio = 3.5 * cfg.final_drive
            omega_eng = omega_w * abs(ratio)
            # blend rather than hard snap to reduce jerk after upshift
            cur = float(self.engine.state.engine.omega)
            omega_eng = 0.7 * omega_eng + 0.3 * cur
            self.engine.state.engine.omega = float(omega_eng)
            self.engine.state.engine.rpm = float(omega_eng * 60.0 / (2.0 * np.pi))
            eng.engine.omega = float(omega_eng)
            eng.engine.rpm = float(omega_eng * 60.0 / (2.0 * np.pi))

        self.diff.set_tv_delta(tv)
        # Keep differential for diagnostics / 14.2B path
        Fz_wheel = float(max(np.sum(v.Fz), cfg.mass * 9.81)) / 4.0
        mu_eff = float(cfg.mu_tire * st.mu_scale)
        diff = self.diff.step(
            tr.wheel_torque,
            omega_left=omega_w,
            omega_right=omega_w,
            dt=dt,
            mu_left=mu_eff,
            mu_right=mu_eff,
            Fz_left=Fz_wheel,
            Fz_right=Fz_wheel,
        )

        # --- Aero ---
        drag = 0.0
        downforce = 0.0
        if cfg.aero_enabled and self.aero_cfg.enabled:
            ride = RideHeightState(h_front=v.ride_h_front, h_rear=v.ride_h_rear)
            aero = compute_aero_loads(max(abs(v.vx), 0.1), self.aero_cfg, ride=ride)
            drag = float(aero.drag * self._draft_factor)
            downforce = float(aero.downforce_total)
        F_y_wind = st.crosswind * 40.0

        if self.dual_track is not None and cfg.use_dual_track:
            # ===== Phase 14.2C authoritative dual-track + Dugoff + ABS =====
            dt_state = self.dual_track.step(
                vx=float(v.vx),
                vy=float(v.vy),
                yaw_rate=float(v.yaw_rate),
                steer=float(steer),
                drive_torque_total=float(tr.wheel_torque),
                brake_cmd=float(brk),
                dt=dt,
                downforce=downforce,
                mu_scale=float(st.mu_scale),
            )
            # Tire forces + aero drag / rolling
            rolling = 0.015 * cfg.mass * 9.81 * np.sign(v.vx + 1e-9)
            Fx = float(dt_state.ax * cfg.mass - drag - rolling)
            # Recompute ax consistent with residual drag
            ax = Fx / cfg.mass
            ay = float(dt_state.ay)
            r_dot = float(dt_state.yaw_acc)
            # Crosswind disturbance on lateral
            ay += F_y_wind / cfg.mass

            vx_new = v.vx + ax * dt
            if v.vx >= 0:
                vx_new = max(vx_new, 0.0)
            vy_new = v.vy + ay * dt
            r_new = v.yaw_rate + r_dot * dt
            psi_new = v.psi + r_new * dt
            x_new = v.x + (vx_new * np.cos(psi_new) - vy_new * np.sin(psi_new)) * dt
            y_new = v.y + (vx_new * np.sin(psi_new) + vy_new * np.cos(psi_new)) * dt

            arr = dt_state.as_arrays()
            st.vehicle = VehicleState(
                x=float(x_new),
                y=float(y_new),
                psi=float(psi_new),
                vx=float(vx_new),
                vy=float(vy_new),
                yaw_rate=float(r_new),
                ax=float(ax),
                ay=float(ay),
                wheel_omega=arr["omega"],
                slip_ratio=arr["kappa"],
                slip_angle=arr["alpha"],
                Fz=arr["Fz"],
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
            st.wheel_torque_L = float(self.dual_track.wheels[2].drive_torque + self.dual_track.wheels[0].drive_torque)
            st.wheel_torque_R = float(self.dual_track.wheels[3].drive_torque + self.dual_track.wheels[1].drive_torque)
            st.gear = int(tr.gear)
            st.time += dt

            self._trace = {
                "engine_torque_nm": float(eng.engine.torque_output),
                "engine_rpm": float(eng.engine.rpm),
                "clutch_torque_nm": float(tr.clutch_torque),
                "gearbox_wheel_torque_nm": float(tr.wheel_torque),
                "T_drive_nm": float(tr.wheel_torque),
                "Fx_tire_N": float(np.sum(arr["Fx"])),
                "Fy_tire_N": float(np.sum(arr["Fy"])),
                "ax": float(ax),
                "ay": float(ay),
                "gear": float(tr.gear),
                "tire_model": 1.0,  # flag: Dugoff active
                "abs_active_count": float(np.sum(dt_state.abs_active)),
                "Fz_sum": float(dt_state.Fz_sum),
                "residual_Fz": float(dt_state.residual_Fz),
            }
            self._dual_diag = self.dual_track.diagnostics()
            return

        # ===== Legacy 14.2B simplified plant (use_dual_track=False) =====
        T_drive = float(tr.wheel_torque)
        T_brake = brk * 2500.0 * 4
        T_net = T_drive - T_brake * np.sign(v.vx + 1e-6)
        Fx_cmd = T_net / max(cfg.wheel_radius, 1e-6)
        Fz_total = float(max(np.sum(v.Fz), cfg.mass * 9.81)) + downforce
        Fx_max = cfg.mu_tire * max(Fz_total, 1.0) * st.mu_scale
        Fx_trac = float(np.clip(Fx_cmd, -Fx_max, Fx_max))
        rolling = 0.01 * cfg.mass * 9.81 * np.sign(v.vx + 1e-9)
        Fx = Fx_trac - drag - rolling
        ax = Fx / cfg.mass
        util = abs(Fx_trac) / max(Fx_max, 1.0)
        slip = float(np.clip(0.02 + 0.25 * util, 0.0, 0.5)) if abs(Fx_cmd) > 50 else 0.0
        vy_dot = -v.vx * v.yaw_rate + (F_y_wind + steer * 8000.0 * max(v.vx, 0.0) / 20.0) / cfg.mass
        r_dot = (steer * 4000.0 * max(v.vx, 1.0) / 20.0 + F_y_wind * 0.5) / cfg.Iz
        vx_new = v.vx + ax * dt
        if v.vx >= 0:
            vx_new = max(vx_new, 0.0)
        vy_new = v.vy + vy_dot * dt
        r_new = v.yaw_rate + r_dot * dt
        psi_new = v.psi + r_new * dt
        x_new = v.x + (vx_new * np.cos(psi_new) - vy_new * np.sin(psi_new)) * dt
        y_new = v.y + (vx_new * np.sin(psi_new) + vy_new * np.cos(psi_new)) * dt
        st.vehicle = VehicleState(
            x=float(x_new), y=float(y_new), psi=float(psi_new),
            vx=float(vx_new), vy=float(vy_new), yaw_rate=float(r_new),
            ax=float(ax), ay=float(vy_dot + vx_new * r_new),
            wheel_omega=np.ones(4) * (abs(vx_new) / max(cfg.wheel_radius, 1e-6)),
            slip_ratio=np.array([slip * 0.5, slip * 0.5, slip, slip]),
            slip_angle=np.zeros(4),
            Fz=v.Fz + downforce / 4.0,
            engine_rpm=float(eng.engine.rpm), gear=int(tr.gear),
            fuel_g=float(eng.fuel.fuel_total_g),
            ride_h_front=v.ride_h_front, ride_h_rear=v.ride_h_rear,
            downforce=float(downforce), drag=float(drag),
        )
        st.throttle = thr
        st.brake = brk
        st.steer = steer
        st.engine_torque = float(eng.engine.torque_output)
        st.wheel_torque_L = float(diff.driveline.torque_left)
        st.wheel_torque_R = float(diff.driveline.torque_right)
        st.gear = int(tr.gear)
        st.time += dt

        self._trace = {
            "engine_torque_nm": float(eng.engine.torque_output),
            "engine_rpm": float(eng.engine.rpm),
            "clutch_torque_nm": float(tr.clutch_torque),
            "gearbox_wheel_torque_nm": float(tr.wheel_torque),
            "T_drive_nm": float(T_drive),
            "Fx_tire_N": float(Fx_trac),
            "ax": float(ax),
            "gear": float(tr.gear),
            "tire_model": 0.0,  # legacy μFz proxy path
        }

    def load_scenario(self, scenario: Scenario) -> None:
        self._scenario = scenario
        self.reset(vx=scenario.initial_vx, gear=scenario.initial_gear)
        self.state.mu_scale = scenario.mu
        if scenario.maneuver is not None:
            self.driver.set_maneuver(scenario.maneuver)
        for t, name, action in scenario.events:
            self.events.add(t, name, action)

    def step(self) -> SimulationState:
        dt = self.cfg.dt
        st = self.state

        # Events
        fired = self.events.process(st.time, self)
        self._events_fired.extend(fired)

        # Driver (path / maneuver intent)
        drv = self.driver.step(st.vehicle.as_pose_dict(), dt)

        # Strategy (Phase 10.5) — requests only
        strat_cmd = StrategyDriverCommand(
            throttle=float(getattr(drv, "throttle", 0.0)),
            brake=float(getattr(drv, "brake", 0.0)),
            steer=float(getattr(drv, "steer", 0.0)),
            launch=bool(getattr(drv, "launch_request", False)),
        )
        strat = self.strategy.step(
            strat_cmd,
            vehicle_speed=float(st.vehicle.vx),
            engine_rpm=float(st.vehicle.engine_rpm),
            dt=dt,
            current_gear=int(st.gear) if st.gear else 1,
        )
        self._last_strategy = strat

        # Map strategy requests onto driver inputs for controls
        # Controls remain the authority layer (ABS/TC/ESC).
        drv_for_controls = DriverInputs(
            throttle=float(strat.torque_factor) if self.cfg.strategy_enabled else float(drv.throttle),
            brake=float(max(getattr(drv, "brake", 0.0), strat.regen_request * 0.5)),
            steer=float(getattr(drv, "steer", 0.0)),
            clutch=float(strat.clutch_request),
            gear_request=int(strat.shift_request),
            launch_request=bool(strat.launch_active),
        )

        # Controls
        vs = st.vehicle.as_sensor_dict()
        vs["steer"] = drv_for_controls.steer
        cmd = self.controls.step(vs, drv_for_controls, dt)

        # Plant (pass strategy gear / clutch — previously dropped at the interface)
        self._step_plant(
            thr=cmd.throttle,
            brk=float(np.mean(cmd.brake_pressures)),
            steer=cmd.steer if hasattr(cmd, "steer") else drv_for_controls.steer,
            tlim=cmd.engine_torque_limit,
            gear_request=int(drv_for_controls.gear_request),
            clutch_cmd=float(drv_for_controls.clutch) if drv_for_controls.clutch > 0 else None,
            tv=cmd.tv_request,
            dt=dt,
        )
        # Use driver steer (commands may not carry steer)
        self.state.steer = drv_for_controls.steer

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
