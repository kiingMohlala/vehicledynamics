"""
Parametric space-frame / roll-cage topology builder (SI metres).

Coordinate system:
  X: longitudinal (rear → front)
  Y: lateral (+Y right)
  Z: vertical (ground up)
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .assembler import Model
from .material import Material, steel, AISI_4130
from .section import Section
from .tube_library import tube_38x2, tube_32x2


@dataclass
class CageParams:
    wheelbase: float = 2.50
    track: float = 1.65
    ground_clearance: float = 0.30
    roof_height: float = 1.70
    roof_taper: float = 0.85
    rear_hoop_frac: float = 0.75
    front_hoop_frac: float = 0.15
    material: Material = field(default_factory=AISI_4130)
    main_section: Section = field(default_factory=tube_38x2)
    secondary_section: Section = field(default_factory=tube_32x2)


class CageBuilder:
    """
    Builds a realistic tube-frame skeleton with tagged pickup nodes.
    """

    def __init__(self, params: CageParams | None = None):
        self.p = params or CageParams()
        self.model = Model()
        self._built = False

    def _n(self, x, y, z, tag="") -> object:
        return self.model.add_node(x, y, z, tag=tag)

    def _beam(self, a, b, section=None, tag="") -> object:
        sec = section or self.p.main_section
        return self.model.add_beam(a, b, self.p.material, sec, tag=tag)

    def build(self) -> Model:
        """Construct full cage topology and return the Model."""
        if self._built:
            return self.model

        p = self.p
        half = p.track / 2.0
        roof_half = half * p.roof_taper
        rear_x = p.rear_hoop_frac * p.wheelbase
        front_x = p.front_hoop_frac * p.wheelbase
        z0 = p.ground_clearance
        zh = p.roof_height

        # --- Primary hoop nodes ---
        rl_l = self._n(rear_x, -half, z0, "rear_lower_left")
        rl_r = self._n(rear_x, half, z0, "rear_lower_right")
        fl_l = self._n(front_x, -half, z0, "front_lower_left")
        fl_r = self._n(front_x, half, z0, "front_lower_right")

        ru_l = self._n(rear_x, -roof_half, zh, "rear_roof_left")
        ru_r = self._n(rear_x, roof_half, zh, "rear_roof_right")
        fu_l = self._n(front_x, -roof_half, zh, "front_roof_left")
        fu_r = self._n(front_x, roof_half, zh, "front_roof_right")

        # Main hoop
        self._beam(rl_l, ru_l, tag="main_hoop")
        self._beam(rl_r, ru_r, tag="main_hoop")
        self._beam(ru_l, ru_r, tag="main_hoop")

        # Front hoop
        self._beam(fl_l, fu_l, tag="front_hoop")
        self._beam(fl_r, fu_r, tag="front_hoop")
        self._beam(fu_l, fu_r, tag="front_hoop")

        # Roof rails + diagonal
        self._beam(ru_l, fu_l, tag="roof_rail")
        self._beam(ru_r, fu_r, tag="roof_rail")
        self._beam(ru_l, fu_r, p.secondary_section, tag="roof_diagonal")

        # Floor rails + cross members
        self._beam(rl_l, fl_l, tag="floor_rail")
        self._beam(rl_r, fl_r, tag="floor_rail")
        self._beam(rl_l, rl_r, tag="rear_cross")
        self._beam(fl_l, fl_r, tag="front_cross")

        # Side truss diagonals
        self._beam(ru_l, fl_l, p.secondary_section, tag="side_truss")
        self._beam(ru_r, fl_r, p.secondary_section, tag="side_truss")

        # Main-hoop diagonal brace
        self._beam(ru_l, rl_r, p.secondary_section, tag="main_brace")

        # A-pillars (front upper to a mid dash height forward of front hoop)
        dash_x = front_x + 0.35
        dash_z = z0 + 0.55
        dash_l = self._n(dash_x, -half * 0.9, dash_z, "dash_left")
        dash_r = self._n(dash_x, half * 0.9, dash_z, "dash_right")
        self._beam(fu_l, dash_l, tag="a_pillar")
        self._beam(fu_r, dash_r, tag="a_pillar")
        self._beam(dash_l, dash_r, p.secondary_section, tag="dash_bar")
        self._beam(fl_l, dash_l, p.secondary_section, tag="a_pillar_lower")
        self._beam(fl_r, dash_r, p.secondary_section, tag="a_pillar_lower")

        # Harness bar (across main hoop at ~ shoulder height)
        harness_z = z0 + 0.95
        hb_l = self._n(rear_x, -roof_half * 0.7, harness_z, "harness_left")
        hb_r = self._n(rear_x, roof_half * 0.7, harness_z, "harness_right")
        self._beam(ru_l, hb_l, p.secondary_section, tag="harness")
        self._beam(ru_r, hb_r, p.secondary_section, tag="harness")
        self._beam(hb_l, hb_r, tag="harness_bar")
        self._beam(rl_l, hb_l, p.secondary_section, tag="harness")
        self._beam(rl_r, hb_r, p.secondary_section, tag="harness")

        # Door bars
        door_mid_x = 0.5 * (rear_x + front_x)
        door_z = z0 + 0.40
        dl = self._n(door_mid_x, -half, door_z, "door_left")
        dr = self._n(door_mid_x, half, door_z, "door_right")
        self._beam(rl_l, dl, tag="door_bar")
        self._beam(dl, fl_l, tag="door_bar")
        self._beam(rl_r, dr, tag="door_bar")
        self._beam(dr, fl_r, tag="door_bar")

        # Rear braces
        rear_tail_x = min(p.wheelbase * 0.95, rear_x + 0.45)
        rt_l = self._n(rear_tail_x, -half * 0.85, z0 + 0.25, "rear_brace_left")
        rt_r = self._n(rear_tail_x, half * 0.85, z0 + 0.25, "rear_brace_right")
        self._beam(rl_l, rt_l, p.secondary_section, tag="rear_brace")
        self._beam(rl_r, rt_r, p.secondary_section, tag="rear_brace")
        self._beam(ru_l, rt_l, p.secondary_section, tag="rear_brace")
        self._beam(ru_r, rt_r, p.secondary_section, tag="rear_brace")
        self._beam(rt_l, rt_r, p.secondary_section, tag="rear_brace")

        # Seat rails
        seat_x0 = rear_x - 0.15
        seat_x1 = rear_x - 0.55
        sfl = self._n(seat_x1, -0.20, z0, "seat_front_left")
        sfr = self._n(seat_x1, 0.20, z0, "seat_front_right")
        srl = self._n(seat_x0, -0.20, z0, "seat_rear_left")
        srr = self._n(seat_x0, 0.20, z0, "seat_rear_right")
        for a, b in [(sfl, sfr), (srl, srr), (sfl, srl), (sfr, srr)]:
            self._beam(a, b, p.secondary_section, tag="seat_rail")
        self._beam(srl, rl_l, p.secondary_section, tag="seat_rail")
        self._beam(srr, rl_r, p.secondary_section, tag="seat_rail")

        # Engine cradle (between front and mid)
        eng_x = 0.45 * p.wheelbase
        e_l = self._n(eng_x, -0.25, z0, "engine_left")
        e_r = self._n(eng_x, 0.25, z0, "engine_right")
        e_lf = self._n(eng_x + 0.35, -0.25, z0, "engine_front_left")
        e_rf = self._n(eng_x + 0.35, 0.25, z0, "engine_front_right")
        for a, b in [(e_l, e_r), (e_lf, e_rf), (e_l, e_lf), (e_r, e_rf)]:
            self._beam(a, b, p.secondary_section, tag="engine_cradle")
        self._beam(e_l, fl_l, p.secondary_section, tag="engine_cradle")
        self._beam(e_r, fl_r, p.secondary_section, tag="engine_cradle")

        # Suspension pickup nodes (tagged only — loads applied via load_cases)
        self._n(front_x - 0.05, -half, z0 - 0.02, "susp_fl")
        self._n(front_x - 0.05, half, z0 - 0.02, "susp_fr")
        self._n(rear_x + 0.05, -half, z0 - 0.02, "susp_rl")
        self._n(rear_x + 0.05, half, z0 - 0.02, "susp_rr")
        # Connect suspension pickups lightly to nearby structure
        self._beam(
            self.model.get_node("susp_fl"), fl_l, p.secondary_section, tag="susp_link"
        )
        self._beam(
            self.model.get_node("susp_fr"), fl_r, p.secondary_section, tag="susp_link"
        )
        self._beam(
            self.model.get_node("susp_rl"), rl_l, p.secondary_section, tag="susp_link"
        )
        self._beam(
            self.model.get_node("susp_rr"), rl_r, p.secondary_section, tag="susp_link"
        )

        self._built = True
        return self.model


def build_default_cage() -> Model:
    return CageBuilder().build()
