"""
High-level calibration runner: telemetry → optimize parameters → metrics.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable
import numpy as np

from .telemetry_loader import TelemetryData, synthesize_telemetry, load_telemetry
from .signal_processing import process_telemetry
from .parameter_sets import ParameterSet
from .parameter_identification import make_coastdown_model
from .optimizer import nelder_mead, differential_evolution, least_squares, grid_search, OptimizeResult
from .validation_metrics import summary_metrics, rmse
from .uncertainty import bootstrap_uncertainty
from .calibration_database import CalibrationDatabase


@dataclass
class CalibrationResult:
    best_parameters: dict[str, float]
    initial_parameters: dict[str, float]
    rmse: float
    r2: float
    metrics: dict[str, float]
    method: str
    nfev: int
    confidence: float = 0.0
    success: bool = True
    message: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "best_parameters": self.best_parameters,
            "initial_parameters": self.initial_parameters,
            "rmse": self.rmse,
            "r2": self.r2,
            "metrics": self.metrics,
            "method": self.method,
            "nfev": self.nfev,
            "confidence": self.confidence,
            "success": self.success,
            "message": self.message,
        }


@dataclass
class CalibrationRunner:
    parameter_set: ParameterSet | None = None
    method: str = "nelder-mead"  # nelder-mead | de | least_squares | grid
    database: CalibrationDatabase = field(default_factory=CalibrationDatabase)

    def __post_init__(self) -> None:
        if self.parameter_set is None:
            self.parameter_set = ParameterSet.default_vehicle()

    def calibrate(
        self,
        telemetry: TelemetryData,
        parameters: list[str] | None = None,
        signal: str = "vx",
        method: str | None = None,
    ) -> CalibrationResult:
        method = method or self.method
        data = process_telemetry(telemetry, dt=0.01)
        if signal not in data.channels:
            raise KeyError(f"Signal '{signal}' not in telemetry")

        # subset of parameters to calibrate
        full = self.parameter_set or ParameterSet.default_vehicle()
        names = parameters or ["Cd", "rolling_resistance", "mass"]
        subset = ParameterSet(params=[p for p in full.params if p.name in names])
        if not subset.params:
            subset = ParameterSet(params=[p for p in full.params if p.group in ("aero", "mass")][:3])

        t = data.time
        y_meas = data.channels[signal]
        v0 = float(y_meas[0])
        model = make_coastdown_model(t, v0)

        initial = subset.values()

        def cost_vec(x: np.ndarray) -> float:
            subset.set_vector(list(x))
            y_sim = model(subset.values())
            return rmse(y_meas, y_sim)

        def residual(x: np.ndarray) -> np.ndarray:
            subset.set_vector(list(x))
            y_sim = model(subset.values())
            n = min(len(y_meas), len(y_sim))
            return y_meas[:n] - y_sim[:n]

        x0 = np.array(subset.vector(), dtype=float)
        bounds = subset.bounds()

        if method in ("de", "differential_evolution"):
            opt = differential_evolution(cost_vec, bounds, maxiter=25, seed=0)
        elif method in ("least_squares", "ls"):
            opt = least_squares(residual, x0, bounds=bounds)
        elif method == "grid":
            opt = grid_search(cost_vec, bounds, levels=4)
        else:
            opt = nelder_mead(cost_vec, x0, maxiter=150)

        subset.set_vector(list(opt.x))
        y_sim = model(subset.values())
        metrics = summary_metrics(y_meas, y_sim, dt=float(t[1] - t[0]) if len(t) > 1 else 0.01)
        unc = bootstrap_uncertainty(cost_vec, opt.x, subset.names(), bounds, n_boot=15, seed=1)

        result = CalibrationResult(
            best_parameters=subset.values(),
            initial_parameters=initial,
            rmse=metrics["rmse"],
            r2=metrics["r2"],
            metrics=metrics,
            method=opt.method,
            nfev=opt.nfev,
            confidence=unc.confidence_score,
            success=opt.success,
            message=opt.message,
        )
        self.database.add(result.best_parameters, metrics, method=opt.method, nfev=opt.nfev)
        return result


def calibrate_tire_curve(
    slip: np.ndarray,
    force_meas: np.ndarray,
    Fz: float = 4000.0,
) -> dict[str, float]:
    from .parameter_identification import tire_force_curve
    from .optimizer import differential_evolution

    def cost(x: np.ndarray) -> float:
        mu, Cx = float(x[0]), float(x[1])
        pred = tire_force_curve(slip, mu, Cx, Fz=Fz)
        return rmse(force_meas, pred)

    opt = differential_evolution(cost, [(0.5, 1.8), (20000, 150000)], maxiter=20, seed=0)
    return {"tire_mu": float(opt.x[0]), "tire_Cx": float(opt.x[1]), "rmse": float(opt.fun), "nfev": opt.nfev}


def calibrate_suspension_step(
    t: np.ndarray,
    z_meas: np.ndarray,
    m: float = 300.0,
) -> dict[str, float]:
    from .parameter_identification import suspension_step_response
    from .optimizer import nelder_mead

    def cost(x: np.ndarray) -> float:
        k, c = float(x[0]), float(x[1])
        pred = suspension_step_response(t, k, c, m=m, z0=float(z_meas[0]))
        return rmse(z_meas, pred)

    x0 = np.array([30000.0, 2000.0])
    opt = nelder_mead(cost, x0, maxiter=120)
    return {"front_spring": float(opt.x[0]), "damping": float(opt.x[1]), "rmse": float(opt.fun), "nfev": opt.nfev}
