"""
Map suspension hardpoints / load application points onto FEM nodes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable

from vehicle_dynamics.fem.assembler import Model
from vehicle_dynamics.fem.node import Node


class PickupRole(str, Enum):
    SUSP_FL = "susp_fl"
    SUSP_FR = "susp_fr"
    SUSP_RL = "susp_rl"
    SUSP_RR = "susp_rr"
    UPPER_FL = "upper_fl"
    UPPER_FR = "upper_fr"
    LOWER_FL = "lower_fl"
    LOWER_FR = "lower_fr"
    UPPER_RL = "upper_rl"
    UPPER_RR = "upper_rr"
    LOWER_RL = "lower_rl"
    LOWER_RR = "lower_rr"
    STEERING_RACK = "steering_rack"
    DAMPER_FL = "damper_fl"
    DAMPER_FR = "damper_fr"
    DAMPER_RL = "damper_rl"
    DAMPER_RR = "damper_rr"
    SPRING_FL = "spring_fl"
    SPRING_FR = "spring_fr"
    SPRING_RL = "spring_rl"
    SPRING_RR = "spring_rr"


@dataclass
class PickupMap:
    """
    role → FEM node id (and optional reference coordinates for kinematics).
    """

    nodes: dict[str, int] = field(default_factory=dict)
    refs: dict[str, tuple[float, float, float]] = field(default_factory=dict)

    def bind(self, role: str | PickupRole, node: Node) -> None:
        key = role.value if isinstance(role, PickupRole) else role
        self.nodes[key] = node.id
        self.refs[key] = (node.x, node.y, node.z)

    def node_id(self, role: str | PickupRole) -> int:
        key = role.value if isinstance(role, PickupRole) else role
        if key not in self.nodes:
            raise KeyError(f"Pickup role {key!r} not mapped")
        return self.nodes[key]

    def ref(self, role: str | PickupRole) -> tuple[float, float, float]:
        key = role.value if isinstance(role, PickupRole) else role
        return self.refs[key]

    def roles(self) -> list[str]:
        return list(self.nodes.keys())


def default_cage_pickups(model: Model) -> PickupMap:
    """
    Bind Phase 8.1 cage suspension tags (and approximate upper/lower
    from nearby structure) onto a PickupMap.
    """
    pm = PickupMap()
    # Primary wheel/load pickups from CageBuilder tags
    for role in (
        PickupRole.SUSP_FL,
        PickupRole.SUSP_FR,
        PickupRole.SUSP_RL,
        PickupRole.SUSP_RR,
    ):
        try:
            n = model.get_node(role.value)
            pm.bind(role, n)
        except KeyError:
            pass

    # Approximate upper/lower using roof / floor side nodes if present
    alias = {
        PickupRole.UPPER_FL: "front_roof_left",
        PickupRole.UPPER_FR: "front_roof_right",
        PickupRole.LOWER_FL: "front_lower_left",
        PickupRole.LOWER_FR: "front_lower_right",
        PickupRole.UPPER_RL: "rear_roof_left",
        PickupRole.UPPER_RR: "rear_roof_right",
        PickupRole.LOWER_RL: "rear_lower_left",
        PickupRole.LOWER_RR: "rear_lower_right",
        PickupRole.STEERING_RACK: "dash_left",  # placeholder attachment
    }
    for role, tag in alias.items():
        try:
            n = model.get_node(tag)
            pm.bind(role, n)
        except KeyError:
            continue

    # Damper / spring seats default to same as susp pickups if not separate
    for corner, susp in (
        (PickupRole.DAMPER_FL, PickupRole.SUSP_FL),
        (PickupRole.DAMPER_FR, PickupRole.SUSP_FR),
        (PickupRole.DAMPER_RL, PickupRole.SUSP_RL),
        (PickupRole.DAMPER_RR, PickupRole.SUSP_RR),
        (PickupRole.SPRING_FL, PickupRole.SUSP_FL),
        (PickupRole.SPRING_FR, PickupRole.SUSP_FR),
        (PickupRole.SPRING_RL, PickupRole.SUSP_RL),
        (PickupRole.SPRING_RR, PickupRole.SUSP_RR),
    ):
        if susp.value in pm.nodes and corner.value not in pm.nodes:
            # reuse node id / ref
            nid = pm.nodes[susp.value]
            pm.nodes[corner.value] = nid
            pm.refs[corner.value] = pm.refs[susp.value]

    return pm
