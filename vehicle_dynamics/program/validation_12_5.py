"""Phase 12.5 – Engineering Program & Requirements Traceability validation (20 gates)."""
from __future__ import annotations

import math
import tempfile
from pathlib import Path

from .requirements import Requirement
from .requirement_database import RequirementDatabase
from .requirement_checker import check_requirements
from .verification_matrix import VerificationMatrix, VerificationLink
from .vehicle_revision import VehicleRevision, RevisionHistory
from .baseline_manager import BaselineManager
from .change_tracking import ChangeLog
from .configuration_control import ConfigurationControl, ConfigurationSnapshot
from .engineering_signoff import SignOffWorkflow
from .evidence_database import EvidenceDatabase, Evidence
from .release_manager import ReleaseManager
from .compliance_report import format_compliance_report
from .engineering_program import EngineeringProgram


def _ok(name: str, passed: bool, detail: str = "") -> tuple[str, bool, str]:
    return name, passed, detail


def gate_requirement_creation() -> tuple[str, bool, str]:
    r = Requirement("0-100", target=3.2, operator="<=", metric="zero_to_hundred_time")
    return _ok("requirement_creation", r.req_id == "0-100" and r.operator == "<=")


def gate_requirement_database() -> tuple[str, bool, str]:
    db = RequirementDatabase()
    db.add(Requirement("top_speed", 330, ">=", metric="top_speed_kmh"))
    db.add(Requirement("brake", 31, "<=", metric="brake_distance_m"))
    return _ok("requirement_database", len(db) == 2)


def gate_automatic_verification() -> tuple[str, bool, str]:
    reqs = [
        Requirement("0-100", 3.2, "<=", metric="zero_to_hundred_time"),
        Requirement("top_speed", 330, ">=", metric="top_speed_kmh"),
    ]
    report = check_requirements(reqs, {"zero_to_hundred_time": 3.0, "top_speed_kmh": 300})
    return _ok("automatic_verification", report.n_pass == 1 and report.n_fail == 1, report.as_table().split("\n")[-1])


def gate_verification_matrix() -> tuple[str, bool, str]:
    m = VerificationMatrix()
    m.add(VerificationLink("0-100", "accel", "sim1", "ev1", "PASS"))
    m.add(VerificationLink("brake", "brake_test", "sim2", "ev2", "FAIL"))
    s = m.summary()
    return _ok("verification_matrix", s["PASS"] == 1 and s["FAIL"] == 1)


def gate_vehicle_revision() -> tuple[str, bool, str]:
    h = RevisionHistory()
    h.add(VehicleRevision("A", "Rev A", {"Cd": 0.34}))
    h.add(VehicleRevision("B", "Rev B", {"Cd": 0.30}, parent_id="A"))
    d = h.get("B").delta_from(h.get("A"))
    return _ok("vehicle_revision", "Cd" in d and len(h) == 2)


def gate_baseline_management() -> tuple[str, bool, str]:
    bm = BaselineManager()
    bm.freeze("BL1", "Freeze 1", {"mass": 1400}, requirements=[{"req_id": "x"}])
    return _ok("baseline_management", bm.get("BL1").label == "Freeze 1")


def gate_configuration_control() -> tuple[str, bool, str]:
    cc = ConfigurationControl()
    h = cc.freeze(ConfigurationSnapshot("CFG1", vehicle={"name": "car"}, tire_model="pacejka"))
    return _ok("configuration_control", len(h) == 16 and cc.get("CFG1").tire_model == "pacejka")


def gate_change_tracking() -> tuple[str, bool, str]:
    log = ChangeLog()
    log.record("C1", "stiffen front", {"k": 1}, {"k": 2}, author="eng")
    return _ok("change_tracking", len(log) == 1)


def gate_evidence_storage() -> tuple[str, bool, str]:
    db = EvidenceDatabase()
    db.add(Evidence("E1", "telemetry", path="a.csv", req_ids=["0-100"]))
    return _ok("evidence_storage", len(db.for_requirement("0-100")) == 1)


def gate_report_generation() -> tuple[str, bool, str]:
    text = format_compliance_report({
        "program": "Test", "revision": "A", "baseline": "BL1",
        "verification_table": "ok", "signoff": {"suspension": "APPROVED"},
        "release_ready": False, "n_evidence": 1, "n_revisions": 1,
    })
    return _ok("report_generation", "Engineering Compliance" in text)


def gate_release_package() -> tuple[str, bool, str]:
    with tempfile.TemporaryDirectory() as td:
        pkg = ReleaseManager().create(
            Path(td) / "rel",
            release_id="R1",
            program_name="Hypercar",
            version="1.0.0",
            vehicle={"name": "H1"},
            requirements=[{"req_id": "0-100"}],
            reports={"summary.md": "# ok"},
        )
        ok = (pkg.path / "manifest.json").exists() and "vehicle/definition.json" in pkg.contents
    return _ok("release_package", ok)


def gate_signoff_workflow() -> tuple[str, bool, str]:
    wf = SignOffWorkflow()
    for st in ("suspension", "aero", "powertrain", "controls", "vehicle_release"):
        wf.approve(st, "lead")
    return _ok("signoff_workflow", wf.release_ready())


def gate_traceability() -> tuple[str, bool, str]:
    prog = EngineeringProgram("Hypercar Mk1")
    prog.add_requirement(Requirement("0-100", 3.2, "<=", metric="zero_to_hundred_time"))
    prog.evaluate({"zero_to_hundred_time": 3.0}, scenario="accel", simulation_id="s1")
    links = prog.verification.for_requirement("0-100")
    ev = prog.evidence.for_requirement("0-100")
    return _ok("traceability", len(links) == 1 and len(ev) == 1 and links[0].status == "PASS")


def gate_repeatability() -> tuple[str, bool, str]:
    prog = EngineeringProgram("P")
    prog.add_requirement(Requirement("brake", 31, "<=", metric="brake_distance_m"))
    r1 = prog.evaluate({"brake_distance_m": 30.0})
    r2 = prog.evaluate({"brake_distance_m": 30.0})
    return _ok("repeatability", r1.n_pass == r2.n_pass == 1)


def gate_parallel_projects() -> tuple[str, bool, str]:
    a = EngineeringProgram("A")
    b = EngineeringProgram("B")
    a.add_requirement(Requirement("top_speed", 200, ">=", metric="v"))
    b.add_requirement(Requirement("top_speed", 300, ">=", metric="v"))
    ra = a.evaluate({"v": 250})
    rb = b.evaluate({"v": 250})
    return _ok("parallel_projects", ra.n_pass == 1 and rb.n_fail == 1)


def gate_performance_regression() -> tuple[str, bool, str]:
    import time
    prog = EngineeringProgram("Perf")
    for i in range(50):
        prog.add_requirement(Requirement(f"r{i}", float(i), "<=", metric=f"m{i}"))
    metrics = {f"m{i}": float(i) - 0.1 for i in range(50)}
    t0 = time.perf_counter()
    prog.evaluate(metrics)
    ms = (time.perf_counter() - t0) * 1000
    return _ok("performance_regression", ms < 500.0, f"{ms:.2f} ms")


def gate_large_database() -> tuple[str, bool, str]:
    db = RequirementDatabase()
    for i in range(200):
        db.add(Requirement(f"req_{i}", float(i), "<=", metric=f"m{i}"))
    return _ok("large_database", len(db) == 200)


def gate_serialization() -> tuple[str, bool, str]:
    db = RequirementDatabase()
    db.add(Requirement("0-100", 3.2, "<=", metric="zero_to_hundred_time"))
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "req.json"
        db.save(p)
        db2 = RequirementDatabase().load(p)
    return _ok("serialization", len(db2) == 1 and db2.get("0-100").target == 3.2)


def gate_no_nan_inf() -> tuple[str, bool, str]:
    r = Requirement("x", 1.0, "<=", metric="x")
    res = r.evaluate({"x": 0.5})
    return _ok("no_nan_inf", res.value is not None and math.isfinite(res.value))


def gate_regression_contract() -> tuple[str, bool, str]:
    try:
        from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
        sim = Simulation(SimulationConfig(dt=0.02))
        sim.load_scenario(ScenarioLibrary.straight_acceleration(0.3))
        r = sim.run(0.3)
        return _ok("regression_contract", len(r.telemetry.samples) > 0)
    except Exception as e:
        return _ok("regression_contract", False, str(e))


GATES = [
    gate_requirement_creation,
    gate_requirement_database,
    gate_automatic_verification,
    gate_verification_matrix,
    gate_vehicle_revision,
    gate_baseline_management,
    gate_configuration_control,
    gate_change_tracking,
    gate_evidence_storage,
    gate_report_generation,
    gate_release_package,
    gate_signoff_workflow,
    gate_traceability,
    gate_repeatability,
    gate_parallel_projects,
    gate_performance_regression,
    gate_large_database,
    gate_serialization,
    gate_no_nan_inf,
    gate_regression_contract,
]


def run_phase125_validation(verbose: bool = True) -> bool:
    results = []
    for g in GATES:
        name, passed, detail = g()
        results.append((name, passed, detail))
        if verbose:
            status = "PASS" if passed else "FAIL"
            extra = f"  ({detail})" if detail else ""
            print(f"  {name:28s}: {status}{extra}")
    n_pass = sum(1 for _, p, _ in results if p)
    n = len(results)
    if verbose:
        print()
        print("=" * 41)
        if n_pass == n:
            print("ALL TESTS PASSED")
            print("Phase 12.5 Status: IMPLEMENTATION VALIDATED ✓")
        else:
            print(f"FAILED: {n_pass}/{n}")
            print("Phase 12.5 Status: NOT VALIDATED")
    return n_pass == n


if __name__ == "__main__":
    raise SystemExit(0 if run_phase125_validation() else 1)
