"""Phase 12.3 – Design Exploration, DOE & Batch Simulation."""

from .design_variables import DesignVariable
from .parameter_space import ParameterSpace
from .constraints import Constraint, bound_constraint, enforce
from .doe import (
    full_factorial, latin_hypercube, sobol_sampling, random_sampling,
    LatinHypercube, SobolSampling, FullFactorial,
)
from .batch_runner import BatchRunner, BatchResult, default_evaluator
from .parallel_runner import ParallelRunner
from .objective_functions import Objective, lap_time_objective, energy_objective, top_speed_objective
from .sensitivity import local_sensitivity, correlation_sensitivity, SensitivityResult
from .pareto_analysis import pareto_front, ParetoFront
from .surrogate_models import fit_polynomial, fit_idw, SurrogateModel, r2_score
from .experiment_database import ExperimentDatabase, ExperimentRecord
from .results_analysis import summarize, analyze
from .optimization_report import format_report, export_report

__all__ = [
    "DesignVariable",
    "ParameterSpace",
    "Constraint",
    "bound_constraint",
    "enforce",
    "full_factorial",
    "latin_hypercube",
    "sobol_sampling",
    "random_sampling",
    "LatinHypercube",
    "SobolSampling",
    "FullFactorial",
    "BatchRunner",
    "BatchResult",
    "default_evaluator",
    "ParallelRunner",
    "Objective",
    "lap_time_objective",
    "energy_objective",
    "top_speed_objective",
    "local_sensitivity",
    "correlation_sensitivity",
    "SensitivityResult",
    "pareto_front",
    "ParetoFront",
    "fit_polynomial",
    "fit_idw",
    "SurrogateModel",
    "r2_score",
    "ExperimentDatabase",
    "ExperimentRecord",
    "summarize",
    "analyze",
    "format_report",
    "export_report",
]
