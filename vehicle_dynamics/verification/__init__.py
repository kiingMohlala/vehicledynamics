"""Phase 11.3 – System verification, regression & virtual proving ground."""

from .numerical_monitor import NumericalMonitor, NumericalReport
from .consistency_checks import ConsistencyChecker, ConsistencyReport
from .benchmark import BenchmarkRunner, BenchmarkResult
from .regression_database import RegressionDatabase, BaselineRecord
from .scenario_matrix import ScenarioMatrix
from .proving_ground import ProvingGround, ProvingGroundResult
from .regression_suite import RegressionSuite, RegressionResult
from .report import format_verification_report, write_text_report
from .validation_11_3 import run_phase113_validation

__all__ = [
    "NumericalMonitor",
    "NumericalReport",
    "ConsistencyChecker",
    "ConsistencyReport",
    "BenchmarkRunner",
    "BenchmarkResult",
    "RegressionDatabase",
    "BaselineRecord",
    "ScenarioMatrix",
    "ProvingGround",
    "ProvingGroundResult",
    "RegressionSuite",
    "RegressionResult",
    "format_verification_report",
    "write_text_report",
    "run_phase113_validation",
]
