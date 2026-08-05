"""Phase 9.5 – Aerodynamic optimization & AI design exploration."""

from .design_variables import DesignVector, DesignBounds, default_bounds
from .objective_functions import Objectives, evaluate_objectives
from .constraints import ConstraintSet, evaluate_constraints
from .pareto import pareto_front, dominates
from .nsga2 import NSGA2Config, nsga2_optimize
from .surrogate_model import SurrogateModel, train_surrogate
from .lap_time_optimizer import LapTimeModel, optimize_lap_time
from .ai_explorer import AIExplorer, ExplorationResult
from .optimization_report import format_optimization_report

__all__ = [
    "DesignVector",
    "DesignBounds",
    "default_bounds",
    "Objectives",
    "evaluate_objectives",
    "ConstraintSet",
    "evaluate_constraints",
    "pareto_front",
    "dominates",
    "NSGA2Config",
    "nsga2_optimize",
    "SurrogateModel",
    "train_surrogate",
    "LapTimeModel",
    "optimize_lap_time",
    "AIExplorer",
    "ExplorationResult",
    "format_optimization_report",
]
