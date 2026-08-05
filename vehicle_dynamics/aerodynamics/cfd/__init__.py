"""Phase 9.3 – CFD calibration & aerodynamic map generation."""

from .cfd_map import AeroSample, AeroMapND
from .cfd_import import import_csv, import_openfoam_forces, import_su2_forces
from .interpolator import interpolate_sample
from .calibration import CalibrationResult, calibrate_against_samples
from .aero_database import AeroDatabase
from .map_generator import build_map_from_samples, export_map_csv
from .uncertainty import UncertaintyEstimate, estimate_uncertainty
from .cfd_report import format_cfd_report

__all__ = [
    "AeroSample",
    "AeroMapND",
    "import_csv",
    "import_openfoam_forces",
    "import_su2_forces",
    "interpolate_sample",
    "CalibrationResult",
    "calibrate_against_samples",
    "AeroDatabase",
    "build_map_from_samples",
    "export_map_csv",
    "UncertaintyEstimate",
    "estimate_uncertainty",
    "format_cfd_report",
]
