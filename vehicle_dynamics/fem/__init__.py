"""Phase 8 – 3D Euler-Bernoulli beam FEM (foundation + space-frame)."""

from .node import Node
from .beam import BeamElement
from .material import (
    Material,
    steel,
    aluminum,
    aluminium_6061,
    AISI_4130,
    stainless_304,
    custom_material,
)
from .section import Section, circular, rectangular, tube
from .tube_library import (
    tube_25x2,
    tube_32x2,
    tube_38x2,
    tube_45x2_5,
    tube_custom,
    mass_per_metre,
)
from .assembler import Model
from .constraints import fix_node, pin_node, apply_force
from .solver import solve_static
from .result import StaticResult
from .cage_builder import CageBuilder, CageParams, build_default_cage
from .load_cases import (
    torsional_rig,
    cornering,
    braking,
    acceleration,
    vertical_landing,
    harness_load,
)
from .mass_properties import compute_mass_properties
from .report import format_report, recover_element_stresses
from .visualization import plot_deformed
from .mass import assemble_mass
from .modal_solver import solve_modal
from .modal_result import ModalResult, ModeShape
from .modal_report import format_modal_report
from .modal_visualization import plot_mode, animate_mode_frames

__all__ = [
    "Node",
    "BeamElement",
    "Material",
    "steel",
    "aluminum",
    "aluminium_6061",
    "AISI_4130",
    "stainless_304",
    "custom_material",
    "Section",
    "circular",
    "rectangular",
    "tube",
    "tube_25x2",
    "tube_32x2",
    "tube_38x2",
    "tube_45x2_5",
    "tube_custom",
    "mass_per_metre",
    "Model",
    "fix_node",
    "pin_node",
    "apply_force",
    "solve_static",
    "StaticResult",
    "CageBuilder",
    "CageParams",
    "build_default_cage",
    "torsional_rig",
    "cornering",
    "braking",
    "acceleration",
    "vertical_landing",
    "harness_load",
    "compute_mass_properties",
    "format_report",
    "recover_element_stresses",
    "plot_deformed",
    "assemble_mass",
    "solve_modal",
    "ModalResult",
    "ModeShape",
    "format_modal_report",
    "plot_mode",
    "animate_mode_frames",
]
