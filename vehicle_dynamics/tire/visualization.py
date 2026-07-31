"""
Phase 3.4.2 – Combined-Slip Dugoff Visualization

Generates the four primary surfaces for qualitative review:
  1. Fx(κ, α)
  2. Fy(κ, α)
  3. λ(κ, α)
  4. Utilization(κ, α)

These plots are intended for visual inspection of continuity,
coupling behaviour, and saturation characteristics before
integrating the model into BrakeSimulation.
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from .dugoff import DugoffTire, DugoffParams


def generate_surfaces(
    params: DugoffParams = None,
    Fz: float = 4000.0,
    kappa_range=(-1.0, 1.0),
    alpha_deg_range=(-15.0, 15.0),
    n_kappa: int = 81,
    n_alpha: int = 61,
    output_dir: str = "baseline/phase3/plots"
):
    if params is None:
        params = DugoffParams()

    tire = DugoffTire(params)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    kappas = np.linspace(kappa_range[0], kappa_range[1], n_kappa)
    alphas = np.deg2rad(np.linspace(alpha_deg_range[0], alpha_deg_range[1], n_alpha))

    Fx = np.zeros((n_kappa, n_alpha))
    Fy = np.zeros_like(Fx)
    lam = np.zeros_like(Fx)
    util = np.zeros_like(Fx)

    for i, k in enumerate(kappas):
        for j, a in enumerate(alphas):
            state = tire.longitudinal_lateral_force(k, a, Fz)
            Fx[i, j] = state.Fx
            Fy[i, j] = state.Fy
            lam[i, j] = state.lambda_
            util[i, j] = state.utilization

    K, A = np.meshgrid(kappas, np.rad2deg(alphas), indexing="ij")

    def _save_contour(data, title, filename, cmap="RdBu_r", levels=40):
        fig, ax = plt.subplots(figsize=(8, 6))
        cf = ax.contourf(K, A, data, levels=levels, cmap=cmap)
        ax.contour(K, A, data, levels=levels, colors="k", linewidths=0.3, alpha=0.4)
        fig.colorbar(cf, ax=ax, label=title)
        ax.set_xlabel("Longitudinal slip ratio κ")
        ax.set_ylabel("Slip angle α [deg]")
        ax.set_title(f"{title}\nFz = {Fz:.0f} N, μ = {params.mu}")
        ax.axhline(0, color="gray", lw=0.8)
        ax.axvline(0, color="gray", lw=0.8)
        fig.tight_layout()
        fig.savefig(out / filename, dpi=150)
        plt.close(fig)
        print(f"Saved: {out / filename}")

    _save_contour(Fx, "Longitudinal Force Fx [N]", "Fx_surface.png", cmap="RdBu_r")
    _save_contour(Fy, "Lateral Force Fy [N]", "Fy_surface.png", cmap="RdBu_r")
    _save_contour(lam, "Saturation Factor λ", "lambda_surface.png", cmap="viridis")
    _save_contour(util, "Friction Utilization", "utilization_surface.png", cmap="plasma")

    # Also save raw arrays for later analysis
    np.savez(
        out / "surfaces.npz",
        kappa=kappas,
        alpha_deg=np.rad2deg(alphas),
        Fx=Fx,
        Fy=Fy,
        lambda_=lam,
        utilization=util,
        Fz=Fz,
        mu=params.mu,
        Cx=params.Cx,
        Cy=params.Cy
    )
    print(f"Saved raw data: {out / 'surfaces.npz'}")

    return {
        "kappa": kappas,
        "alpha_deg": np.rad2deg(alphas),
        "Fx": Fx,
        "Fy": Fy,
        "lambda": lam,
        "utilization": util
    }


def quick_line_plots(params: DugoffParams = None, Fz: float = 4000.0, output_dir: str = "baseline/phase3/plots"):
    """Simple 1-D slices for quick inspection"""
    if params is None:
        params = DugoffParams()
    tire = DugoffTire(params)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # Pure longitudinal (alpha = 0)
    kappas = np.linspace(-1.0, 1.0, 201)
    Fx_long = [tire.longitudinal_lateral_force(k, 0.0, Fz).Fx for k in kappas]

    # Pure lateral (kappa = 0)
    alphas = np.deg2rad(np.linspace(-15, 15, 121))
    Fy_lat = [tire.longitudinal_lateral_force(0.0, a, Fz).Fy for a in alphas]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].plot(kappas, Fx_long, lw=2)
    axes[0].set_xlabel("Slip ratio κ")
    axes[0].set_ylabel("Fx [N]")
    axes[0].set_title("Pure Longitudinal (α = 0)")
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(0, color="k", lw=0.5)
    axes[0].axvline(0, color="k", lw=0.5)

    axes[1].plot(np.rad2deg(alphas), Fy_lat, lw=2, color="C1")
    axes[1].set_xlabel("Slip angle α [deg]")
    axes[1].set_ylabel("Fy [N]")
    axes[1].set_title("Pure Lateral (κ = 0)")
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(0, color="k", lw=0.5)
    axes[1].axvline(0, color="k", lw=0.5)

    fig.tight_layout()
    fig.savefig(out / "line_slices.png", dpi=150)
    plt.close(fig)
    print(f"Saved: {out / 'line_slices.png'}")


if __name__ == "__main__":
    print("Generating combined-slip surfaces...")
    generate_surfaces()
    quick_line_plots()
    print("Done.")
