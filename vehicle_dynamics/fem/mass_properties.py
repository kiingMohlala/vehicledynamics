"""Mass properties for tube-frame models."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from .assembler import Model


@dataclass
class MassReport:
    total_mass_kg: float
    total_length_m: float
    n_elements: int
    n_nodes: int
    com: tuple[float, float, float]
    mass_by_tag: dict[str, float]


def compute_mass_properties(model: Model) -> MassReport:
    total_m = 0.0
    total_L = 0.0
    mx = my = mz = 0.0
    by_tag: dict[str, float] = {}

    for e in model.elements:
        L = e.length()
        m = e.mass()
        total_m += m
        total_L += L
        mid = 0.5 * (e.node_i.coords() + e.node_j.coords())
        mx += m * mid[0]
        my += m * mid[1]
        mz += m * mid[2]
        tag = e.tag or "untagged"
        by_tag[tag] = by_tag.get(tag, 0.0) + m

    if total_m > 0:
        com = (mx / total_m, my / total_m, mz / total_m)
    else:
        com = (0.0, 0.0, 0.0)

    return MassReport(
        total_mass_kg=total_m,
        total_length_m=total_L,
        n_elements=len(model.elements),
        n_nodes=len(model.nodes),
        com=com,
        mass_by_tag=by_tag,
    )
