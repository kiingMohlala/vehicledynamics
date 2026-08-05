"""Lightweight engineering plots for tube-frame results."""

from __future__ import annotations

from pathlib import Path
import numpy as np

from .assembler import Model
from .result import StaticResult
from .report import recover_element_stresses


def plot_deformed(
    model: Model,
    result: StaticResult,
    scale: float = 50.0,
    path: str | Path | None = None,
    title: str = "Deformed shape",
) -> str:
    """
    Generate a 3D wireframe plot (undeformed grey, deformed coloured by stress).
    Returns path written (or 'memory' if matplotlib unavailable / no path).
    """
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        return "matplotlib_unavailable"

    stresses = recover_element_stresses(model, result)
    stress_map = {s.elem_id: s.von_mises_Pa for s in stresses}
    vmax = max(stress_map.values()) if stress_map else 1.0

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")

    for e in model.elements:
        pi = e.node_i.coords()
        pj = e.node_j.coords()
        ax.plot(
            [pi[0], pj[0]],
            [pi[1], pj[1]],
            [pi[2], pj[2]],
            color="0.7",
            linewidth=0.8,
        )

        ui = result.node_displacement(e.node_i.id)[:3]
        uj = result.node_displacement(e.node_j.id)[:3]
        qi = pi + scale * ui
        qj = pj + scale * uj
        cval = stress_map.get(e.id, 0.0) / vmax
        ax.plot(
            [qi[0], qj[0]],
            [qi[1], qj[1]],
            [qi[2], qj[2]],
            color=plt.cm.hot(0.2 + 0.8 * cval),
            linewidth=2.0,
        )

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(title)
    try:
        ax.set_box_aspect([1, 1, 0.6])
    except Exception:
        pass

    out = str(path) if path else "memory"
    if path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=120, bbox_inches="tight")
        plt.close(fig)
    else:
        plt.close(fig)
    return out
