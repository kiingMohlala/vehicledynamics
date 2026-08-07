"""Design for Manufacturing checks."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import numpy as np

from .materials_database import get_material, MfgMaterial


@dataclass
class DFMIssue:
    severity: str  # info | warn | error
    code: str
    message: str
    part: str = ""


@dataclass
class DFMReport:
    score: float  # 0-100
    issues: list[DFMIssue] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not any(i.severity == "error" for i in self.issues)


def check_wall_thickness(thickness_mm: float, material: str, part: str = "") -> list[DFMIssue]:
    mat = get_material(material)
    issues = []
    if thickness_mm < mat.min_wall_mm:
        issues.append(DFMIssue("error", "WALL_THIN", f"Wall {thickness_mm:.2f} mm < min {mat.min_wall_mm} mm", part))
    elif thickness_mm < mat.min_wall_mm * 1.2:
        issues.append(DFMIssue("warn", "WALL_MARGINAL", f"Wall {thickness_mm:.2f} mm near minimum", part))
    return issues


def check_tool_access(depth_mm: float, diameter_mm: float, part: str = "") -> list[DFMIssue]:
    issues = []
    if diameter_mm < 1e-6:
        return [DFMIssue("error", "TOOL_ZERO", "Zero tool diameter", part)]
    ratio = depth_mm / diameter_mm
    if ratio > 8:
        issues.append(DFMIssue("error", "TOOL_DEEP", f"Aspect {ratio:.1f} > 8 (poor tool access)", part))
    elif ratio > 5:
        issues.append(DFMIssue("warn", "TOOL_ASPECT", f"Aspect {ratio:.1f} challenging", part))
    return issues


def check_hole_spacing(spacing_mm: float, hole_d_mm: float, part: str = "") -> list[DFMIssue]:
    issues = []
    if spacing_mm < 1.5 * hole_d_mm:
        issues.append(DFMIssue("warn", "HOLE_SPACE", f"Spacing {spacing_mm:.1f} < 1.5×D", part))
    return issues


def evaluate_dfm(
    parts: list[dict[str, Any]],
) -> DFMReport:
    """
    parts: dicts with keys name, material, thickness_mm?, depth_mm?, diameter_mm?, spacing_mm?
    """
    issues: list[DFMIssue] = []
    for p in parts:
        name = p.get("name", "")
        mat = p.get("material", "aluminum")
        if "thickness_mm" in p:
            issues.extend(check_wall_thickness(float(p["thickness_mm"]), mat, name))
        if "depth_mm" in p and "diameter_mm" in p:
            issues.extend(check_tool_access(float(p["depth_mm"]), float(p["diameter_mm"]), name))
        if "spacing_mm" in p and "diameter_mm" in p:
            issues.extend(check_hole_spacing(float(p["spacing_mm"]), float(p["diameter_mm"]), name))
    # score
    penalty = sum(15 if i.severity == "error" else 5 if i.severity == "warn" else 0 for i in issues)
    score = float(np.clip(100 - penalty, 0, 100))
    return DFMReport(score=score, issues=issues)
