"""Engineering report: stresses, utilization, critical members."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .assembler import Model
from .result import StaticResult
from .stiffness import local_stiffness
from .transform import transformation_matrix
from .mass_properties import compute_mass_properties, MassReport


@dataclass
class ElementStress:
    elem_id: int
    tag: str
    von_mises_Pa: float
    axial_Pa: float
    bending_Pa: float
    torsion_Pa: float
    utilization: float  # vs yield


def recover_element_stresses(model: Model, result: StaticResult) -> list[ElementStress]:
    out: list[ElementStress] = []
    for e in model.elements:
        T = transformation_matrix(e)
        k_loc = local_stiffness(e)
        dofs = np.concatenate([e.node_i.dof_indices(), e.node_j.dof_indices()])
        u_g = result.u[dofs]
        u_l = T @ u_g
        f_l = k_loc @ u_l

        sec = e.section
        od = sec.od if sec.od is not None else 0.04
        c = od / 2.0

        axial = abs(f_l[0]) / sec.A
        My = max(abs(f_l[4]), abs(f_l[10]))
        Mz = max(abs(f_l[5]), abs(f_l[11]))
        bending = My * c / sec.Iy + Mz * c / sec.Iz
        Mx = max(abs(f_l[3]), abs(f_l[9]))
        torsion = Mx * c / sec.J
        normal = axial + bending
        vm = float(np.sqrt(normal**2 + 3.0 * torsion**2))
        Sy = e.material.yield_strength
        util = vm / Sy if Sy > 0 else 0.0
        out.append(
            ElementStress(
                elem_id=e.id,
                tag=e.tag,
                von_mises_Pa=vm,
                axial_Pa=axial,
                bending_Pa=bending,
                torsion_Pa=torsion,
                utilization=util,
            )
        )
    out.sort(key=lambda s: -s.von_mises_Pa)
    result.element_stresses = {s.elem_id: s.von_mises_Pa for s in out}
    return out


def format_report(
    model: Model,
    result: StaticResult,
    case_name: str = "analysis",
    metrics: dict | None = None,
) -> str:
    stresses = recover_element_stresses(model, result)
    mass = compute_mass_properties(model)
    lines = [
        f"=== FEM Report: {case_name} ===",
        f"Success: {result.success} ({result.message})",
        f"Nodes: {mass.n_nodes}  Elements: {mass.n_elements}",
        f"Mass: {mass.total_mass_kg:.2f} kg  Length: {mass.total_length_m:.2f} m",
        f"CoM: ({mass.com[0]:.3f}, {mass.com[1]:.3f}, {mass.com[2]:.3f}) m",
        f"Max |u|: {result.max_displacement*1e3:.3f} mm",
    ]
    if result.torsional_stiffness_Nm_per_deg is not None:
        lines.append(
            f"Torsional stiffness: {result.torsional_stiffness_Nm_per_deg:.1f} Nm/deg"
        )
    if metrics:
        for k, v in metrics.items():
            lines.append(f"  {k}: {v}")

    lines.append("\nTop critical members (von Mises):")
    for s in stresses[:10]:
        lines.append(
            f"  id={s.elem_id:3d} tag={s.tag:16s} "
            f"σ={s.von_mises_Pa/1e6:7.1f} MPa  util={s.utilization:.3f}"
        )
    if stresses:
        lines.append(
            f"\nPeak utilization: {stresses[0].utilization:.3f} "
            f"(yield={model.elements[stresses[0].elem_id].material.yield_strength/1e6:.0f} MPa)"
        )
    return "\n".join(lines)
