import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
from dugoff import DugoffTire, DugoffParams

def generate_surfaces(params=None, Fz=4000.0, output_dir="/home/workdir/artifacts/baseline/phase3/plots"):
    if params is None:
        params = DugoffParams()
    tire = DugoffTire(params)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    kappas = np.linspace(-1.0, 1.0, 81)
    alphas = np.deg2rad(np.linspace(-15.0, 15.0, 61))

    Fx = np.zeros((len(kappas), len(alphas)))
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

    def save_contour(data, title, filename, cmap="RdBu_r", levels=40):
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

    save_contour(Fx, "Longitudinal Force Fx [N]", "Fx_surface.png", cmap="RdBu_r")
    save_contour(Fy, "Lateral Force Fy [N]", "Fy_surface.png", cmap="RdBu_r")
    save_contour(lam, "Saturation Factor λ", "lambda_surface.png", cmap="viridis")
    save_contour(util, "Friction Utilization", "utilization_surface.png", cmap="plasma")

    # Line slices
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    Fx_long = [tire.longitudinal_lateral_force(k, 0.0, Fz).Fx for k in kappas]
    alphas_deg = np.linspace(-15, 15, 121)
    Fy_lat = [tire.longitudinal_lateral_force(0.0, np.deg2rad(a), Fz).Fy for a in alphas_deg]

    axes[0].plot(kappas, Fx_long, lw=2)
    axes[0].set_xlabel("Slip ratio κ")
    axes[0].set_ylabel("Fx [N]")
    axes[0].set_title("Pure Longitudinal (α = 0)")
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(0, color="k", lw=0.5)
    axes[0].axvline(0, color="k", lw=0.5)

    axes[1].plot(alphas_deg, Fy_lat, lw=2, color="C1")
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

    np.savez(out / "surfaces.npz", kappa=kappas, alpha_deg=np.rad2deg(alphas),
             Fx=Fx, Fy=Fy, lambda_=lam, utilization=util, Fz=Fz, mu=params.mu)
    print(f"Saved: {out / 'surfaces.npz'}")
    print("Done.")

if __name__ == "__main__":
    generate_surfaces()
