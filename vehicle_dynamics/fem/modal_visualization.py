"""Modal shape visualization / simple animation frames."""

from __future__ import annotations

from pathlib import Path
import numpy as np

from .assembler import Model
from .modal_result import ModalResult, ModeShape


def plot_mode(
    model: Model,
    mode: ModeShape,
    scale: float = 0.15,
    path: str | Path | None = None,
    title: str | None = None,
) -> str:
    """
    Static 3D plot of a mode shape (undeformed + deformed).
    scale is relative to structure characteristic length.
    """
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        return "matplotlib_unavailable"

    # Characteristic length for auto-scaling
    coords = np.array([n.coords() for n in model.nodes])
    char_L = float(np.linalg.norm(coords.max(axis=0) - coords.min(axis=0))) + 1e-9
    # Peak translational component
    peak = 1e-12
    for n in model.nodes:
        b = 6 * n.id
        peak = max(peak, np.linalg.norm(mode.shape[b : b + 3]))
    amp = scale * char_L / peak

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    for e in model.elements:
        pi = e.node_i.coords()
        pj = e.node_j.coords()
        ax.plot([pi[0], pj[0]], [pi[1], pj[1]], [pi[2], pj[2]], color="0.7", lw=0.8)

        ui = mode.shape[e.node_i.dof_indices()[:3]]
        uj = mode.shape[e.node_j.dof_indices()[:3]]
        qi = pi + amp * ui
        qj = pj + amp * uj
        ax.plot([qi[0], qj[0]], [qi[1], qj[1]], [qi[2], qj[2]], color="C0", lw=2.0)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ttl = title or f"Mode {mode.index}: {mode.frequency_Hz:.2f} Hz ({mode.classification})"
    ax.set_title(ttl)
    try:
        ax.set_box_aspect([1, 1, 0.6])
    except Exception:
        pass

    out = str(path) if path else "memory"
    if path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return out


def animate_mode_frames(
    model: Model,
    mode: ModeShape,
    n_frames: int = 8,
    scale: float = 0.15,
) -> list[np.ndarray]:
    """
    Return list of node-coordinate arrays over one vibration cycle
    (for external animation). Does not require matplotlib.
    """
    coords0 = np.array([n.coords() for n in model.nodes])
    peak = 1e-12
    for n in model.nodes:
        b = 6 * n.id
        peak = max(peak, np.linalg.norm(mode.shape[b : b + 3]))
    char_L = float(np.linalg.norm(coords0.max(0) - coords0.min(0))) + 1e-9
    amp = scale * char_L / peak

    frames = []
    for k in range(n_frames):
        phase = np.sin(2 * np.pi * k / n_frames)
        pts = coords0.copy()
        for n in model.nodes:
            b = 6 * n.id
            pts[n.id] = coords0[n.id] + amp * phase * mode.shape[b : b + 3]
        frames.append(pts)
    return frames
