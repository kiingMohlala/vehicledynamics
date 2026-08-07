"""Class-A continuity checks G0–G3 between curves / along surfaces."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import numpy as np

from .vector import distance, norm, normalize, dot


@dataclass
class ContinuityResult:
    g0: bool
    g1: bool
    g2: bool
    g3: bool
    g0_error: float = 0.0
    g1_error: float = 0.0
    g2_error: float = 0.0
    g3_error: float = 0.0

    @property
    def g2_pass(self) -> bool:
        return self.g2


class ContinuityAnalyzer:
    """
    Check continuity at a joint between two curves (end of a, start of b)
    or sample surface fairness proxies.
    """

    def __init__(self, a=None, b=None, tol_pos: float = 1e-4, tol_ang: float = 1e-3, tol_curv: float = 5e-2):
        self.a = a
        self.b = b
        self.tol_pos = tol_pos
        self.tol_ang = tol_ang
        self.tol_curv = tol_curv

    def analyze_curves(self, curve_a, curve_b) -> ContinuityResult:
        p_a = curve_a.evaluate(1.0)
        p_b = curve_b.evaluate(0.0)
        g0_err = distance(p_a, p_b)
        g0 = g0_err <= self.tol_pos

        t_a = normalize(curve_a.tangent(1.0))
        t_b = normalize(curve_b.tangent(0.0))
        g1_err = float(np.arccos(np.clip(abs(dot(t_a, t_b)), -1, 1)))
        g1 = g0 and g1_err <= self.tol_ang

        # curvature proxy via second finite difference (sample inward from joint)
        def curv_proxy(curve, t):
            eps = 1e-3
            # keep stencil fully inside [0,1]
            t = float(np.clip(t, eps, 1.0 - eps))
            p0 = curve.evaluate(t - eps)
            p1 = curve.evaluate(t)
            p2 = curve.evaluate(t + eps)
            return (p2 - 2 * p1 + p0) / (eps ** 2)

        c_a = curv_proxy(curve_a, 1.0 - 1e-3)
        c_b = curv_proxy(curve_b, 1e-3)
        ca_n, cb_n = norm(c_a), norm(c_b)
        if ca_n < 1e-8 and cb_n < 1e-8:
            g2_err = 0.0
            g2 = g1
        else:
            g2_err = norm(c_a - c_b) / (ca_n + cb_n + 1e-9)
            g2 = g1 and g2_err <= self.tol_curv

        # G3: third derivative proxy
        def jerk_proxy(curve, t):
            eps = 1e-3
            t = float(np.clip(t, 2 * eps, 1.0 - 2 * eps))
            return (curv_proxy(curve, t + eps) - curv_proxy(curve, t - eps)) / (2 * eps)

        j_a = jerk_proxy(curve_a, 1.0 - 2e-3)
        j_b = jerk_proxy(curve_b, 2e-3)
        ja_n, jb_n = norm(j_a), norm(j_b)
        if ja_n < 1e-6 and jb_n < 1e-6:
            g3_err = 0.0
            g3 = g2
        else:
            g3_err = norm(j_a - j_b) / (ja_n + jb_n + 1e-9)
            g3 = g2 and g3_err <= self.tol_curv * 2

        return ContinuityResult(g0, g1, g2, g3, g0_err, g1_err, g2_err, g3_err)

    def analyze_surface_fairness(self, surface, nu: int = 10, nv: int = 10) -> dict[str, float]:
        """Simple fairness: mean normal variation across grid."""
        grid = surface.sample_grid(nu, nv)
        normals = []
        for i in range(nu):
            for j in range(nv):
                u, v = i / max(nu - 1, 1), j / max(nv - 1, 1)
                normals.append(surface.normal(u, v))
        normals = np.array(normals)
        mean_n = normalize(np.mean(normals, axis=0))
        variation = float(np.mean([1 - abs(dot(n, mean_n)) for n in normals]))
        return {"normal_variation": variation, "fairness_score": float(max(0.0, 1.0 - variation))}
