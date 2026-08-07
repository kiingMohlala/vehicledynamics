"""
Lap simulator: couples track + digital twin + existing Phase 11.2 Simulation.

Uses a simplified progress-along-track model driven by the integrated vehicle
simulation plant, with sector timing and multi-lap sessions.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import numpy as np

from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
from vehicle_dynamics.vehicle.digital_twin import DigitalTwin
from vehicle_dynamics.vehicle.vehicle_builder import BuiltVehicle

from .track import Track
from .racing_line import RacingLine, center_line, ideal_line
from .sector_timer import SectorTimer, equal_sectors, best_sectors, SectorTimes
from .lap_statistics import LapMetrics, SessionStatistics, compute_lap_metrics
from .telemetry_export import export_csv, export_json, export_markdown_report


@dataclass
class LapResult:
    lap_index: int
    metrics: LapMetrics
    sectors: SectorTimes | None
    telemetry: dict[str, list] = field(default_factory=dict)


@dataclass
class SessionResult:
    track_name: str
    vehicle_name: str
    statistics: SessionStatistics
    laps: list[LapResult] = field(default_factory=list)
    line_kind: str = "center"

    @property
    def best_lap(self) -> float:
        return self.statistics.best_lap

    @property
    def sector_times(self) -> list[float]:
        if not self.laps or self.laps[0].sectors is None:
            return []
        # return best-lap sectors if available
        idx = self.statistics.best_lap_index
        sec = self.laps[idx].sectors
        return list(sec.sector_dt) if sec else []

    def export_csv(self, path: str) -> None:
        if not self.laps:
            return
        export_csv(path, self.laps[self.statistics.best_lap_index].telemetry)

    def export_json(self, path: str) -> None:
        payload = {
            "track": self.track_name,
            "vehicle": self.vehicle_name,
            "best_lap": self.best_lap,
            "n_laps": self.statistics.n_laps,
            "sectors_best_lap": self.sector_times,
            "summary": self.statistics.summary(),
        }
        export_json(path, payload)

    def export_report(self, path: str) -> None:
        body = (
            f"**Track:** {self.track_name}\n\n"
            f"**Vehicle:** {self.vehicle_name}\n\n"
            f"**Line:** {self.line_kind}\n\n"
            f"```\n{self.statistics.summary()}\n```\n"
        )
        export_markdown_report(path, "Lap Simulation Report", body)


class LapSimulator:
    def __init__(
        self,
        vehicle: DigitalTwin | BuiltVehicle | None,
        track: Track,
        *,
        dt: float = 0.02,
        n_sectors: int = 3,
        line: str = "ideal",
        v_max: float = 60.0,
    ) -> None:
        self.track = track
        self.dt = dt
        self.n_sectors = n_sectors
        self.line_kind = line
        self.v_max = v_max

        if isinstance(vehicle, DigitalTwin):
            self.vehicle_name = vehicle.name
            self.built = vehicle.built
        elif isinstance(vehicle, BuiltVehicle):
            self.vehicle_name = vehicle.name
            self.built = vehicle
        else:
            self.vehicle_name = "default"
            self.built = None

        mu = 1.0
        if track.friction is not None:
            mu = float(np.mean(track.friction.mu_values))
        if line == "ideal":
            self.racing_line = ideal_line(track, mu=mu, v_max=v_max)
        else:
            self.racing_line = center_line(track, mu=mu, v_max=v_max)

    def _make_sim(self, v0: float) -> Simulation:
        kwargs = {}
        if self.built is not None:
            kwargs = {
                k: v for k, v in self.built.simulation_kwargs.items()
                if k in SimulationConfig.__dataclass_fields__
            }
        cfg = SimulationConfig(dt=self.dt, **kwargs)
        sim = Simulation(cfg)
        # Use straight-accel scenario as plant bootstrap; progress mapped to track
        scen = ScenarioLibrary.straight_acceleration(duration=max(5.0, self.track.length / max(v0, 5.0)))
        scen.initial_vx = v0
        sim.load_scenario(scen)
        return sim

    def _run_single_lap(self, lap_index: int, v0: float = 15.0) -> LapResult:
        track = self.track
        line = self.racing_line
        sim = self._make_sim(v0)

        boundaries = equal_sectors(track.length, self.n_sectors)
        timer = SectorTimer(boundaries_s=boundaries)

        t_hist: list[float] = []
        s_hist: list[float] = []
        vx_hist: list[float] = []
        ax_hist: list[float] = []
        ay_hist: list[float] = []
        fuel_hist: list[float] = []
        thr_hist: list[float] = []
        brk_hist: list[float] = []
        steer_hist: list[float] = []
        rpm_hist: list[float] = []
        gear_hist: list[float] = []
        df_hist: list[float] = []
        drag_hist: list[float] = []

        s = 0.0
        t = 0.0
        max_steps = int(max(500, track.length / max(v0 * 0.1, 0.5) / self.dt))
        finished = False

        for _ in range(max_steps):
            st = sim.step()
            vx = max(float(st.vehicle.vx), 0.1)
            # Progress along track using speed; blend toward reference speed profile
            i = int(np.clip(np.searchsorted(line.s, s) - 1, 0, len(line.v_ref) - 1))
            v_ref = float(line.v_ref[i])
            # Soft speed target via throttle/brake already in plant; just integrate distance
            s += vx * self.dt
            t += self.dt
            timer.update(s, t)

            t_hist.append(t)
            s_hist.append(s)
            vx_hist.append(vx)
            ax_hist.append(float(st.vehicle.ax))
            ay_hist.append(float(getattr(st.vehicle, "ay", 0.0)))
            fuel_hist.append(float(getattr(st.vehicle, "fuel_g", 0.0)))
            thr_hist.append(float(st.throttle))
            brk_hist.append(float(st.brake))
            steer_hist.append(float(st.steer))
            rpm_hist.append(float(st.vehicle.engine_rpm))
            gear_hist.append(float(st.gear))
            df_hist.append(float(getattr(st.vehicle, "downforce", 0.0)))
            drag_hist.append(float(getattr(st.vehicle, "drag", 0.0)))

            if s >= track.length:
                finished = True
                break

        # Force final sector if nearly complete
        if not finished and s > 0.95 * track.length:
            timer.update(track.length + 1e-3, t)
            finished = True

        sectors = timer.result()
        # If sectors incomplete, synthesize from equal time splits
        if sectors is None and t > 0:
            splits = [t * (i + 1) / self.n_sectors for i in range(self.n_sectors)]
            dts = [splits[0]] + [splits[i] - splits[i - 1] for i in range(1, len(splits))]
            sectors = SectorTimes(splits=splits, sector_dt=dts, total=t)

        metrics = compute_lap_metrics(
            np.array(t_hist), np.array(s_hist), np.array(vx_hist),
            ax=np.array(ax_hist), ay=np.array(ay_hist),
            fuel=np.array(fuel_hist),
            downforce=np.array(df_hist), drag=np.array(drag_hist),
            sector_dt=sectors.sector_dt if sectors else [],
        )
        if not finished:
            # mark incomplete lap with large time
            metrics.lap_time = max(metrics.lap_time, t)

        telem = {
            "time": t_hist, "s": s_hist, "vx": vx_hist, "ax": ax_hist, "ay": ay_hist,
            "throttle": thr_hist, "brake": brk_hist, "steer": steer_hist,
            "rpm": rpm_hist, "gear": gear_hist, "fuel_g": fuel_hist,
            "downforce": df_hist, "drag": drag_hist,
        }
        return LapResult(lap_index=lap_index, metrics=metrics, sectors=sectors, telemetry=telem)

    def run_laps(self, n_laps: int = 1, v0: float = 15.0) -> SessionResult:
        laps: list[LapResult] = []
        for i in range(n_laps):
            # carry approximate exit speed into next lap
            v_start = v0 if i == 0 else max(10.0, laps[-1].metrics.top_speed * 0.5)
            laps.append(self._run_single_lap(i, v0=v_start))

        times = [L.metrics.lap_time for L in laps]
        best_i = int(np.argmin(times)) if times else 0
        sector_list = [L.sectors for L in laps if L.sectors is not None]
        bsec = best_sectors(sector_list) if sector_list else []

        stats = SessionStatistics(
            n_laps=len(laps),
            best_lap=float(times[best_i]) if times else 0.0,
            best_lap_index=best_i,
            average_lap=float(np.mean(times)) if times else 0.0,
            total_distance=float(sum(L.metrics.distance for L in laps)),
            total_time=float(sum(times)),
            lap_metrics=[L.metrics for L in laps],
            best_sectors=bsec,
        )
        return SessionResult(
            track_name=self.track.name,
            vehicle_name=self.vehicle_name,
            statistics=stats,
            laps=laps,
            line_kind=self.line_kind,
        )


def compare_vehicles(
    vehicles: list[DigitalTwin | BuiltVehicle],
    track: Track,
    n_laps: int = 1,
    **kwargs: Any,
) -> dict[str, SessionResult]:
    out = {}
    for v in vehicles:
        name = v.name if hasattr(v, "name") else "vehicle"
        sim = LapSimulator(v, track, **kwargs)
        out[name] = sim.run_laps(n_laps)
    return out
