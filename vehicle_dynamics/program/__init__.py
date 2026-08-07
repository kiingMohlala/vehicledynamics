"""Phase 12.5 – Engineering Program Management & Requirements Traceability."""

from .requirements import Requirement, RequirementResult
from .requirement_database import RequirementDatabase
from .requirement_checker import check_requirements, CheckReport
from .verification_matrix import VerificationMatrix, VerificationLink
from .vehicle_revision import VehicleRevision, RevisionHistory
from .baseline_manager import Baseline, BaselineManager
from .change_tracking import ChangeRecord, ChangeLog
from .configuration_control import ConfigurationSnapshot, ConfigurationControl
from .engineering_signoff import SignOff, SignOffWorkflow, STAGES
from .evidence_database import Evidence, EvidenceDatabase
from .release_manager import ReleasePackage, ReleaseManager
from .compliance_report import format_compliance_report
from .engineering_program import EngineeringProgram

__all__ = [
    "Requirement",
    "RequirementResult",
    "RequirementDatabase",
    "check_requirements",
    "CheckReport",
    "VerificationMatrix",
    "VerificationLink",
    "VehicleRevision",
    "RevisionHistory",
    "Baseline",
    "BaselineManager",
    "ChangeRecord",
    "ChangeLog",
    "ConfigurationSnapshot",
    "ConfigurationControl",
    "SignOff",
    "SignOffWorkflow",
    "STAGES",
    "Evidence",
    "EvidenceDatabase",
    "ReleasePackage",
    "ReleaseManager",
    "format_compliance_report",
    "EngineeringProgram",
]
