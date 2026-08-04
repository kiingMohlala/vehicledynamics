"""Phase 7.2 – Handling metrics (analysis only)."""

from .metrics import SteadyStateMetrics, UtilizationMetrics, StabilityMetrics, DriverMetrics
from .steady_state import extract_steady_state
from .balance import utilization_metrics, classify_balance, BalanceResult
from .transient import extract_stability, extract_driver
from .report import HandlingReport


def analyze_run(
    time,
    vx,
    vy,
    r,
    delta,
    wheelbase: float = 2.7,
    utilization=None,
    X=None,
    Y=None,
    load_transfer=None,
    jacking=None,
    rc_migration=None,
) -> HandlingReport:
    """One-shot handling characterization from simulation arrays."""
    import numpy as np

    steady = extract_steady_state(time, vx, vy, r, delta, wheelbase)
    if utilization is None:
        utilization = np.zeros((len(time), 4))
    util = utilization_metrics(utilization)
    balance = classify_balance(steady.understeer_gradient_deg_per_g, util)
    stability = extract_stability(
        time, vx, vy, r, load_transfer, jacking, rc_migration
    )
    driver = extract_driver(time, vx, delta, X, Y)

    warnings = []
    if util.limiting_axle == "rear" and steady.understeer_gradient_deg_per_g < 0:
        warnings.append("Rear tires saturated with oversteer gradient.")
    if util.limiting_axle == "front" and steady.understeer_gradient_deg_per_g > 0:
        warnings.append("Front tires closer to limit (understeer).")
    if steady.max_ay_g > 1.2:
        warnings.append("Very high lateral acceleration — check tire μ realism.")
    if any(util.peak > 1.0 + 1e-6):
        warnings.append("Utilization exceeded 1.0 (numerical).")

    return HandlingReport(
        steady=steady,
        utilization=util,
        balance=balance,
        stability=stability,
        driver=driver,
        warnings=warnings,
    )


__all__ = [
    "SteadyStateMetrics",
    "UtilizationMetrics",
    "StabilityMetrics",
    "DriverMetrics",
    "BalanceResult",
    "HandlingReport",
    "extract_steady_state",
    "utilization_metrics",
    "classify_balance",
    "extract_stability",
    "extract_driver",
    "analyze_run",
]
