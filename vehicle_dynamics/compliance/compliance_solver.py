"""
Static chassis compliance solver with full / reduced / disabled modes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_dynamics.fem.assembler import Model
from vehicle_dynamics.fem.constraints import apply_force, fix_node
from vehicle_dynamics.fem.solver import solve_static
from vehicle_dynamics.fem.result import StaticResult

from .pickup_mapper import PickupMap, default_cage_pickups
from .compliance_kinematics import GeometryDelta, compliance_geometry_update
from .reduced_model import ReducedComplianceModel, build_reduced_compliance


@dataclass
class ComplianceConfig:
    """
    compliance_mode:
      'disabled' – zero deformation (rigid chassis regression)
      'full'     – full FEM static solve each call
      'reduced'  – precomputed pickup compliance matrix
    """

    compliance_mode: str = "full"
    camber_gain: float = 1.0
    toe_gain: float = 1.0
    # Roles used as structural supports when applying wheel loads
    support_roles: tuple[str, ...] = ("susp_rl", "susp_rr")


@dataclass
class ComplianceState:
    u: np.ndarray
    geometry: GeometryDelta
    strain_energy: float = 0.0
    max_node_disp: float = 0.0
    mode: str = "full"
    success: bool = True
    message: str = "ok"
    fem_result: StaticResult | None = None


class ComplianceSolver:
    """
    Applies suspension pickup loads to the chassis FEM and returns
    geometry deltas for tire / suspension coupling.
    """

    def __init__(
        self,
        model: Model,
        pickup_map: PickupMap | None = None,
        config: ComplianceConfig | None = None,
        auto_support: bool = True,
    ):
        self.model = model
        self.pickups = pickup_map or default_cage_pickups(model)
        self.config = config or ComplianceConfig()
        self._reduced: ReducedComplianceModel | None = None
        self._supports_applied = False

        if auto_support and self.config.compliance_mode != "disabled":
            self._apply_default_supports()

        if self.config.compliance_mode == "reduced":
            self._reduced = build_reduced_compliance(self.model, self.pickups)

    def _apply_default_supports(self) -> None:
        """Fix support pickups (e.g. rear mounts) if not already constrained."""
        for role in self.config.support_roles:
            if role in self.pickups.nodes:
                nid = self.pickups.node_id(role)
                node = self.model.nodes[nid]
                if not np.any(node.fixed):
                    fix_node(node)
        self._supports_applied = True

    def ensure_reduced(self) -> ReducedComplianceModel:
        if self._reduced is None:
            self._reduced = build_reduced_compliance(self.model, self.pickups)
        return self._reduced

    def solve(
        self,
        loads: dict[str, tuple[float, float, float]],
    ) -> ComplianceState:
        """
        loads: pickup role → (Fx, Fy, Fz) [N]
        """
        mode = self.config.compliance_mode

        if mode == "disabled":
            u = np.zeros(self.model.ndof)
            geom = compliance_geometry_update(
                self.pickups, u,
                camber_gain=self.config.camber_gain,
                toe_gain=self.config.toe_gain,
            )
            return ComplianceState(
                u=u,
                geometry=geom,
                strain_energy=0.0,
                max_node_disp=0.0,
                mode=mode,
                success=True,
                message="disabled (rigid)",
            )

        if mode == "reduced":
            return self._solve_reduced(loads)

        return self._solve_full(loads)

    def _solve_full(
        self, loads: dict[str, tuple[float, float, float]]
    ) -> ComplianceState:
        F = np.zeros(self.model.ndof)
        for role, (fx, fy, fz) in loads.items():
            if role not in self.pickups.nodes:
                continue
            node = self.model.nodes[self.pickups.node_id(role)]
            apply_force(F, node, fx=fx, fy=fy, fz=fz)

        res = solve_static(self.model, F)
        if not res.success:
            return ComplianceState(
                u=np.zeros(self.model.ndof),
                geometry=GeometryDelta(),
                mode="full",
                success=False,
                message=res.message,
                fem_result=res,
            )

        u = res.u
        geom = compliance_geometry_update(
            self.pickups,
            u,
            camber_gain=self.config.camber_gain,
            toe_gain=self.config.toe_gain,
        )
        # Strain energy ½ uᵀ K u = ½ uᵀ F (for linear statics)
        energy = 0.5 * float(u @ F)
        max_disp = float(
            np.max(
                np.sqrt(u[0::6] ** 2 + u[1::6] ** 2 + u[2::6] ** 2)
            )
        )
        return ComplianceState(
            u=u,
            geometry=geom,
            strain_energy=energy,
            max_node_disp=max_disp,
            mode="full",
            success=True,
            message="ok",
            fem_result=res,
        )

    def _solve_reduced(
        self, loads: dict[str, tuple[float, float, float]]
    ) -> ComplianceState:
        red = self.ensure_reduced()
        disp = red.apply(loads)
        # Scatter into full u (translations only at pickups)
        u = np.zeros(self.model.ndof)
        for role, uv in disp.items():
            nid = self.pickups.node_id(role)
            base = 6 * nid
            u[base : base + 3] = uv

        geom = compliance_geometry_update(
            self.pickups,
            u,
            camber_gain=self.config.camber_gain,
            toe_gain=self.config.toe_gain,
        )
        # Approximate energy from reduced: ½ Fᵀ C F
        n = red.n_pickups
        Fv = np.zeros(3 * n)
        for i, role in enumerate(red.roles):
            fx, fy, fz = loads.get(role, (0.0, 0.0, 0.0))
            Fv[3 * i : 3 * i + 3] = [fx, fy, fz]
        energy = 0.5 * float(Fv @ (red.C @ Fv))
        max_disp = float(np.max(np.abs(u[0::6]**2 + u[1::6]**2 + u[2::6]**2))**0.5)

        return ComplianceState(
            u=u,
            geometry=geom,
            strain_energy=energy,
            max_node_disp=max_disp,
            mode="reduced",
            success=True,
            message="ok",
        )
