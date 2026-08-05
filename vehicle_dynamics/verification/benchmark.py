"""Performance timing benchmarks."""

from __future__ import annotations

from dataclasses import dataclass
import time
import numpy as np

from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary


@dataclass
class BenchmarkResult:
    name: str
    n_steps: int
    total_s: float
    ms_per_step: float
    steps_per_s: float
    ok: bool = True
    message: str = ""


class BenchmarkRunner:
    def __init__(self, warn_ms: float = 50.0):
        self.warn_ms = warn_ms

    def run_scenario(self, name: str, duration: float = 3.0, dt: float = 0.02) -> BenchmarkResult:
        sim = Simulation(SimulationConfig(dt=dt))
        # Map name to scenario
        lib = ScenarioLibrary
        loaders = {
            "straight": lib.straight_acceleration,
            "brake": lambda: lib.emergency_braking(duration=duration),
            "dlc": lambda: lib.double_lane_change(v=15.0),
            "slalom": lambda: lib.slalom(v=12.0),
        }
        loader = loaders.get(name, lib.straight_acceleration)
        try:
            sc = loader() if name != "straight" else lib.straight_acceleration(duration)
            if name == "straight":
                sc = lib.straight_acceleration(duration)
            sim.load_scenario(sc)
        except Exception:
            sim.load_scenario(lib.straight_acceleration(duration))

        t0 = time.perf_counter()
        res = sim.run(duration)
        t1 = time.perf_counter()
        n = len(res.telemetry.samples)
        total = t1 - t0
        ms = (total / max(n, 1)) * 1000.0
        ok = ms < self.warn_ms
        return BenchmarkResult(
            name=name,
            n_steps=n,
            total_s=total,
            ms_per_step=ms,
            steps_per_s=n / max(total, 1e-9),
            ok=ok,
            message="" if ok else f"slow {ms:.2f} ms/step",
        )

    def run_default(self) -> list[BenchmarkResult]:
        return [
            self.run_scenario("straight", 2.0),
            self.run_scenario("brake", 2.0),
            self.run_scenario("dlc", 3.0),
        ]
