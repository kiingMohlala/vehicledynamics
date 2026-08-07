"""Top-level engineering program facade."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .requirements import Requirement
from .requirement_database import RequirementDatabase
from .requirement_checker import check_requirements, CheckReport
from .verification_matrix import VerificationMatrix, VerificationLink
from .vehicle_revision import RevisionHistory, VehicleRevision
from .baseline_manager import BaselineManager
from .change_tracking import ChangeLog
from .configuration_control import ConfigurationControl, ConfigurationSnapshot
from .engineering_signoff import SignOffWorkflow
from .evidence_database import EvidenceDatabase, Evidence
from .release_manager import ReleaseManager, ReleasePackage
from .compliance_report import format_compliance_report


@dataclass
class EngineeringProgram:
    name: str
    requirements: RequirementDatabase = field(default_factory=RequirementDatabase)
    revisions: RevisionHistory = field(default_factory=RevisionHistory)
    baselines: BaselineManager = field(default_factory=BaselineManager)
    changes: ChangeLog = field(default_factory=ChangeLog)
    configurations: ConfigurationControl = field(default_factory=ConfigurationControl)
    signoff: SignOffWorkflow = field(default_factory=SignOffWorkflow)
    evidence: EvidenceDatabase = field(default_factory=EvidenceDatabase)
    verification: VerificationMatrix = field(default_factory=VerificationMatrix)
    last_check: CheckReport | None = None
    vehicle_definition: dict[str, Any] = field(default_factory=dict)

    def add_requirement(self, req: Requirement) -> None:
        self.requirements.add(req)

    def evaluate(self, metrics: dict[str, Any], scenario: str = "default", simulation_id: str = "sim0") -> CheckReport:
        report = check_requirements(self.requirements.active(), metrics)
        self.last_check = report
        for r in report.results:
            eid = f"ev_{r.req_id}_{simulation_id}"
            self.evidence.add(Evidence(
                evidence_id=eid,
                kind="validation",
                req_ids=[r.req_id],
                simulation_id=simulation_id,
                meta={"status": r.status, "value": r.value},
            ))
            self.verification.add(VerificationLink(
                req_id=r.req_id,
                scenario=scenario,
                simulation_id=simulation_id,
                evidence_id=eid,
                status=r.status,
                metrics={r.req_id: r.value},
            ))
        return report

    def verification_matrix(self) -> str:
        return self.verification.as_table()

    def add_revision(self, revision_id: str, label: str, parameters: dict[str, Any], notes: str = "", author: str = "") -> VehicleRevision:
        parent = self.revisions.latest()
        rev = VehicleRevision(
            revision_id=revision_id,
            label=label,
            parameters=dict(parameters),
            notes=notes,
            parent_id=parent.revision_id if parent else None,
            author=author,
        )
        if parent:
            self.changes.record(
                change_id=f"chg_{revision_id}",
                description=notes or f"Revision {label}",
                before=parent.parameters,
                after=parameters,
                author=author,
            )
        self.revisions.add(rev)
        self.vehicle_definition = dict(parameters)
        return rev

    def freeze_baseline(self, baseline_id: str, label: str, software_version: str = "", git_tag: str = "") -> Any:
        return self.baselines.freeze(
            baseline_id=baseline_id,
            label=label,
            vehicle_definition=self.vehicle_definition,
            requirements=[r.to_dict() for r in self.requirements.requirements],
            software_version=software_version,
            git_tag=git_tag,
        )

    def freeze_configuration(self, config_id: str, **kwargs: Any) -> str:
        snap = ConfigurationSnapshot(
            config_id=config_id,
            vehicle=kwargs.get("vehicle", self.vehicle_definition),
            track=kwargs.get("track", {}),
            weather=kwargs.get("weather", {}),
            controllers=kwargs.get("controllers", {}),
            tire_model=kwargs.get("tire_model", ""),
            solver=kwargs.get("solver", {}),
            software_version=kwargs.get("software_version", ""),
        )
        return self.configurations.freeze(snap)

    def compliance_report(self) -> str:
        table = self.last_check.as_table() if self.last_check else "(not evaluated)"
        return format_compliance_report({
            "program": self.name,
            "revision": self.revisions.latest().label if self.revisions.latest() else "",
            "baseline": self.baselines.list_ids()[-1] if self.baselines.list_ids() else "",
            "verification_table": table,
            "signoff": self.signoff.status(),
            "release_ready": self.signoff.release_ready(),
            "n_evidence": len(self.evidence),
            "n_revisions": len(self.revisions),
        })

    def create_release(self, root: str, release_id: str, version: str, **kwargs: Any) -> ReleasePackage:
        return ReleaseManager().create(
            root,
            release_id=release_id,
            program_name=self.name,
            version=version,
            vehicle=self.vehicle_definition,
            requirements=[r.to_dict() for r in self.requirements.requirements],
            verification_matrix={"links": [L.__dict__ for L in self.verification.links]},
            **kwargs,
        )
