"""Large scenario combination matrix for CI."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
from vehicle_dynamics.simulation.scenario_runner import Scenario


@dataclass
class MatrixEntry:
    name: str
    builder: Callable[[], Scenario]
    duration: float
    dt: float = 0.02


class ScenarioMatrix:
    """Standardized set of short scenarios for regression sweeps."""

    def entries(self) -> list[MatrixEntry]:
        lib = ScenarioLibrary
        return [
            MatrixEntry("acc_straight", lambda: lib.straight_acceleration(3.0), 3.0),
            MatrixEntry("brake_dry", lambda: lib.emergency_braking(v0=20.0, duration=3.0), 3.0),
            MatrixEntry("brake_split", lambda: lib.split_mu_braking(v0=18.0), 3.0),
            MatrixEntry("dlc", lambda: lib.double_lane_change(v=15.0), 4.0),
            MatrixEntry("slalom", lambda: lib.slalom(v=12.0), 4.0),
            MatrixEntry("fig8", lambda: lib.figure_eight(v=10.0), 4.0),
            MatrixEntry("moose", lambda: lib.moose_test(v=14.0), 4.0),
            MatrixEntry("crosswind", lambda: lib.crosswind(5.0), 5.0),
            MatrixEntry("draft", lambda: lib.drafting(4.0), 4.0),
            MatrixEntry("wet", lambda: lib.wet_road(v=12.0), 4.0),
            MatrixEntry("const_radius", lambda: lib.constant_radius(radius=35.0, v=12.0), 5.0),
            MatrixEntry("launch", lambda: lib.launch_control(), 4.0),
        ]

    def run_all(self, dt: float = 0.02) -> list[tuple[str, bool, dict]]:
        results = []
        for e in self.entries():
            try:
                sim = Simulation(SimulationConfig(dt=dt))
                sim.load_scenario(e.builder())
                res = sim.run(e.duration)
                ok = len(res.telemetry.samples) > 5
                results.append((e.name, ok, {"n": len(res.telemetry.samples), "vx": res.state.vehicle.vx}))
            except Exception as ex:
                results.append((e.name, False, {"error": str(ex)}))
        return results
