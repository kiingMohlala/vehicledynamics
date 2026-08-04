"""Handling balance classification."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from .metrics import UtilizationMetrics


@dataclass
class BalanceResult:
    classification: str
    understeer_gradient_deg_per_g: float
    front_utilization: float
    rear_utilization: float
    notes: str = ""


def utilization_metrics(
    utilization: np.ndarray,
    time: np.ndarray | None = None,
) -> UtilizationMetrics:
    """
    utilization: (n, 4) array
    """
    u = np.asarray(utilization, dtype=float)
    if u.ndim == 1:
        u = u.reshape(1, -1)
    peak = np.max(u, axis=0)
    mean = np.mean(u, axis=0)
    frac = np.mean(u > 0.90, axis=0)
    lim_w = int(np.argmax(peak))
    front_mean = float(0.5 * (mean[0] + mean[1]))
    rear_mean = float(0.5 * (mean[2] + mean[3]))
    if front_mean > rear_mean + 0.03:
        axle = "front"
    elif rear_mean > front_mean + 0.03:
        axle = "rear"
    else:
        axle = "balanced"
    return UtilizationMetrics(
        peak=peak,
        mean=mean,
        time_above_90=frac,
        limiting_wheel=lim_w,
        limiting_axle=axle,
        front_mean=front_mean,
        rear_mean=rear_mean,
    )


def classify_balance(
    K_deg_per_g: float,
    util: UtilizationMetrics,
) -> BalanceResult:
    """
    Combine understeer gradient with tire utilization.
    """
    front = util.front_mean
    rear = util.rear_mean
    notes = []

    if K_deg_per_g > 1.5:
        cls = "Understeer"
    elif K_deg_per_g > 0.3:
        cls = "Mild Understeer"
    elif K_deg_per_g > -0.3:
        cls = "Neutral steer"
    elif K_deg_per_g > -1.5:
        cls = "Mild oversteer"
    else:
        cls = "Strong oversteer"

    if util.limiting_axle == "front":
        notes.append("Front tires closer to saturation.")
    elif util.limiting_axle == "rear":
        notes.append("Rear tires closer to saturation.")
    if rear > front + 0.05 and K_deg_per_g < 0:
        notes.append("Rear utilization high with oversteer gradient.")
    if front > rear + 0.05 and K_deg_per_g > 0:
        notes.append("Front utilization high with understeer gradient.")

    return BalanceResult(
        classification=cls,
        understeer_gradient_deg_per_g=K_deg_per_g,
        front_utilization=front,
        rear_utilization=rear,
        notes=" ".join(notes),
    )
