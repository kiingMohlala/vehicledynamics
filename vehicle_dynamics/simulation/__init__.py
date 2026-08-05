"""Phase 11.2 – Integrated vehicle simulation & scenario engine."""

from .simulation_state import SimulationState, VehicleState
from .timing import FixedTimestep
from .scheduler import UpdateScheduler
from .event_manager import EventManager, SimulationEvent
from .telemetry_recorder import TelemetryRecorder, SimSample
from .statistics import SimulationStatistics, compute_statistics
from .scenario_runner import Scenario, ScenarioLibrary
from .replay import ReplayBuffer
from .simulation import SimulationConfig, Simulation, SimulationResults

__all__ = [
    "SimulationState",
    "VehicleState",
    "FixedTimestep",
    "UpdateScheduler",
    "EventManager",
    "SimulationEvent",
    "TelemetryRecorder",
    "SimSample",
    "SimulationStatistics",
    "compute_statistics",
    "Scenario",
    "ScenarioLibrary",
    "ReplayBuffer",
    "SimulationConfig",
    "Simulation",
    "SimulationResults",
]
