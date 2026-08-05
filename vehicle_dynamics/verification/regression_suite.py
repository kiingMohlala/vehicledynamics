"""Full regression runner."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
from vehicle_dynamics.verification.regression_database import RegressionDatabase
from vehicle_dynamics.verification.proving_ground import ProvingGround
from vehicle_dynamics.verification.scenario_matrix import ScenarioMatrix
from vehicle_dynamics.verification.benchmark import BenchmarkRunner
from vehicle_dynamics.verification.numerical_monitor import NumericalMonitor
from vehicle_dynamics.verification.consistency_checks import ConsistencyChecker


@dataclass
class RegressionResult:
    name: str
    ok: bool
    details: dict = field(default_factory=dict)


class RegressionSuite:
    def __init__(self, baseline_root: str | Path | None = None, dt: float = 0.02):
        self.db = RegressionDatabase(baseline_root)
        self.dt = dt
        self.pg = ProvingGround(dt=dt)
        self.matrix = ScenarioMatrix()
        self.bench = BenchmarkRunner()
        self.monitor = NumericalMonitor()
        self.consistency = ConsistencyChecker()

    def capture_baselines(self) -> list[str]:
        """Create reference baselines for core scenarios."""
        lib = ScenarioLibrary
        specs = [
            ("reg_straight", lambda: lib.straight_acceleration(3.0), 3.0),
            ("reg_brake", lambda: lib.emergency_braking(v0=22.0, duration=3.0), 3.0),
            ("reg_dlc", lambda: lib.double_lane_change(v=15.0), 4.0),
        ]
        names = []
        for name, builder, dur in specs:
            sim = Simulation(SimulationConfig(dt=self.dt))
            sim.load_scenario(builder())
            res = sim.run(dur)
            self.db.capture(
                name,
                res.telemetry,
                res.statistics,
                final_vx=res.state.vehicle.vx,
                final_x=res.state.vehicle.x,
                final_y=res.state.vehicle.y,
            )
            names.append(name)
        return names

    def check_baselines(self) -> list[RegressionResult]:
        results = []
        for name in ("reg_straight", "reg_brake", "reg_dlc"):
            if self.db.load(name) is None:
                results.append(RegressionResult(name, False, {"error": "missing baseline"}))
                continue
            # Rebuild scenario
            lib = ScenarioLibrary
            builders = {
                "reg_straight": (lambda: lib.straight_acceleration(3.0), 3.0),
                "reg_brake": (lambda: lib.emergency_braking(v0=22.0, duration=3.0), 3.0),
                "reg_dlc": (lambda: lib.double_lane_change(v=15.0), 4.0),
            }
            builder, dur = builders[name]
            sim = Simulation(SimulationConfig(dt=self.dt))
            sim.load_scenario(builder())
            res = sim.run(dur)
            ok, det = self.db.compare(
                name, res.telemetry, res.statistics, final_vx=res.state.vehicle.vx
            )
            results.append(RegressionResult(name, ok, det))
        return results

    def run_full(self) -> dict:
        baseline_names = self.capture_baselines()
        checks = self.check_baselines()
        pg = self.pg.run_all()
        matrix = self.matrix.run_all(dt=self.dt)
        benches = self.bench.run_default()
        return {
            "baselines_captured": baseline_names,
            "baseline_checks": checks,
            "proving_ground": pg,
            "matrix": matrix,
            "benchmarks": benches,
            "all_pass": (
                all(c.ok for c in checks)
                and all(p.ok for p in pg)
                and all(m[1] for m in matrix)
                and all(b.ok for b in benches)
            ),
        }
