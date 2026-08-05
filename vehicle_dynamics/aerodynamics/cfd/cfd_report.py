"""CFD / map calibration report."""

from __future__ import annotations

from .calibration import CalibrationResult
from .aero_database import AeroDatabase
from .uncertainty import UncertaintyEstimate


def format_cfd_report(
    db: AeroDatabase,
    cal: CalibrationResult | None = None,
    unc: UncertaintyEstimate | None = None,
    title: str = "CFD Aero Map Report",
) -> str:
    lines = [
        f"=== {title} ===",
        f"Mode:     {db.mode.value}",
        f"Samples:  {len(db.amap)}",
        f"Map name: {db.amap.name}",
    ]
    bounds = db.amap.bounds()
    if bounds:
        lines.append("Bounds:")
        for k, (lo, hi) in bounds.items():
            lines.append(f"  {k:12s}  [{lo:.4g}, {hi:.4g}]")
    if cal is not None:
        lines += [
            "",
            "Calibration",
            f"  success:    {cal.success}",
            f"  n:          {cal.n_samples}",
            f"  RMS Cd:     {cal.rms_Cd:.4f}",
            f"  RMS Cl_f:   {cal.rms_Cl_f:.4f}",
            f"  RMS Cl_r:   {cal.rms_Cl_r:.4f}",
            f"  bias Cd:    {cal.mean_bias_Cd:.4f}",
            f"  Cd calib:   {cal.config.coeffs.Cd:.4f}",
            f"  Cl_f calib: {cal.config.coeffs.Cl_front:.4f}",
            f"  Cl_r calib: {cal.config.coeffs.Cl_rear:.4f}",
        ]
    if unc is not None:
        lines += [
            "",
            "Uncertainty",
            f"  confidence: {unc.confidence:.2f}",
            f"  distance:   {unc.interp_distance:.3f}",
            f"  sigma_Cd:   {unc.sigma_Cd:.4f}",
        ]
    return "\n".join(lines)
