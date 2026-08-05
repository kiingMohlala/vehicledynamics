"""Standardized engineering proving-ground tests."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
from vehicle_dynamics.verification.numerical_monitor import NumericalMonitor
from vehicle_dynamics.verification.consistency_checks import ConsistencyChecker


@dataclass
class ProvingGroundResult:
    name: str
    category: str
    ok: bool
    diagnostics: dict = field(default_factory=dict)


class ProvingGround:
    def __init__(self, dt: float = 0.02):
        self.dt = dt
        self.monitor = NumericalMonitor()
        self.consistency = ConsistencyChecker()

    def _run(self, name: str, category: str, scenario_fn, duration: float) -> ProvingGroundResult:
        try:
            sim = Simulation(SimulationConfig(dt=self.dt))
            sc = scenario_fn()
            sim.load_scenario(sc)
            res = sim.run(duration)
            num = self.monitor.check(res.telemetry, self.dt)
            con = self.consistency.check(res.telemetry)
            ok = num.ok and con.ok and len(res.telemetry.samples) > 3
            return ProvingGroundResult(
                name=name,
                category=category,
                ok=ok,
                diagnostics={
                    "n": len(res.telemetry.samples),
                    "max_speed": res.statistics.max_speed,
                    "num_ok": num.ok,
                    "con_ok": con.ok,
                    "messages": num.messages + con.messages,
                },
            )
        except Exception as e:
            return ProvingGroundResult(name=name, category=category, ok=False, diagnostics={"error": str(e)})

    def run_braking(self) -> list[ProvingGroundResult]:
        lib = ScenarioLibrary
        return [
            self._run("dry_abs", "braking", lambda: lib.emergency_braking(v0=25.0, duration=4.0), 4.0),
            self._run("wet_abs", "braking", lambda: lib.wet_road(v=15.0), 3.0),
            self._run("split_mu", "braking", lambda: lib.split_mu_braking(v0=20.0), 3.5),
        ]

    def run_handling(self) -> list[ProvingGroundResult]:
        lib = ScenarioLibrary
        return [
            self._run("const_radius", "handling", lambda: lib.constant_radius(40.0, 12.0), 5.0),
            self._run("dlc", "handling", lambda: lib.double_lane_change(v=16.0), 5.0),
            self._run("moose", "handling", lambda: lib.moose_test(v=15.0), 4.0),
            self._run("slalom", "handling", lambda: lib.slalom(v=12.0), 4.0),
            self._run("fig8", "handling", lambda: lib.figure_eight(v=10.0), 4.0),
        ]

    def run_acceleration(self) -> list[ProvingGroundResult]:
        lib = ScenarioLibrary
        return [
            self._run("launch", "acceleration", lambda: lib.straight_acceleration(5.0), 5.0),
            self._run("launch_ctrl", "acceleration", lambda: lib.launch_control(), 4.0),
        ]

    def run_aero(self) -> list[ProvingGroundResult]:
        lib = ScenarioLibrary
        return [
            self._run("crosswind", "aero", lambda: lib.crosswind(6.0), 6.0),
            self._run("drafting", "aero", lambda: lib.drafting(5.0), 5.0),
        ]

    def run_powertrain(self) -> list[ProvingGroundResult]:
        lib = ScenarioLibrary
        return [
            self._run("wot", "powertrain", lambda: lib.straight_acceleration(4.0), 4.0),
            self._run("hill", "powertrain", lambda: lib.hill_start(), 4.0),
        ]

    def run_combined(self) -> list[ProvingGroundResult]:
        lib = ScenarioLibrary
        # Trail-braking proxy: DLC at higher speed; corner + brake via emergency after steer
        return [
            self._run("trail_proxy", "combined", lambda: lib.double_lane_change(v=18.0), 5.0),
            self._run("corner_exit", "combined", lambda: lib.constant_radius(30.0, 14.0), 5.0),
        ]

    def run_all(self) -> list[ProvingGroundResult]:
        out: list[ProvingGroundResult] = []
        out.extend(self.run_braking())
        out.extend(self.run_handling())
        out.extend(self.run_acceleration())
        out.extend(self.run_aero())
        out.extend(self.run_powertrain())
        out.extend(self.run_combined())
        return out
