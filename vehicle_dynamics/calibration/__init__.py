"""Phase 12.4 – Model Calibration, Validation & Parameter Identification."""

from .telemetry_loader import TelemetryData, load_telemetry, synthesize_telemetry
from .signal_processing import process_telemetry, butterworth_lowpass_simple, estimate_noise_std
from .synchronization import lag_by_correlation, align_signals
from .parameter_sets import CalibParameter, ParameterSet
from .validation_metrics import rmse, mae, summary_metrics, r2_score
from .optimizer import nelder_mead, differential_evolution, least_squares, grid_search, OptimizeResult
from .parameter_identification import coastdown_vx, tire_force_curve, suspension_step_response
from .uncertainty import bootstrap_uncertainty, UncertaintyResult
from .calibration_database import CalibrationDatabase, CalibrationRecord
from .calibration_runner import CalibrationRunner, CalibrationResult, calibrate_tire_curve, calibrate_suspension_step
from .calibration_report import format_calibration_report, export_calibration_report

__all__ = [
    "TelemetryData",
    "load_telemetry",
    "synthesize_telemetry",
    "process_telemetry",
    "butterworth_lowpass_simple",
    "estimate_noise_std",
    "lag_by_correlation",
    "align_signals",
    "CalibParameter",
    "ParameterSet",
    "rmse",
    "mae",
    "summary_metrics",
    "r2_score",
    "nelder_mead",
    "differential_evolution",
    "least_squares",
    "grid_search",
    "OptimizeResult",
    "coastdown_vx",
    "tire_force_curve",
    "suspension_step_response",
    "bootstrap_uncertainty",
    "UncertaintyResult",
    "CalibrationDatabase",
    "CalibrationRecord",
    "CalibrationRunner",
    "CalibrationResult",
    "calibrate_tire_curve",
    "calibrate_suspension_step",
    "format_calibration_report",
    "export_calibration_report",
]
