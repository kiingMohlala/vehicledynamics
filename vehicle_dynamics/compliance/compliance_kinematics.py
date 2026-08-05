"""
Convert pickup displacements into suspension geometry deltas
(camber, toe, track, roll-center proxy).
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .pickup_mapper import PickupMap


@dataclass
class GeometryDelta:
    """Per-corner and aggregate geometry change from chassis flex."""

    # radians
    d_camber_fl: float = 0.0
    d_camber_fr: float = 0.0
    d_camber_rl: float = 0.0
    d_camber_rr: float = 0.0
    d_toe_fl: float = 0.0
    d_toe_fr: float = 0.0
    d_toe_rl: float = 0.0
    d_toe_rr: float = 0.0
    # metres
    d_track_front: float = 0.0
    d_track_rear: float = 0.0
    d_rc_front: float = 0.0  # roll-center height change proxy
    d_rc_rear: float = 0.0
    max_pickup_disp: float = 0.0
    chassis_twist_rad: float = 0.0

    def as_dict(self) -> dict:
        return {k: getattr(self, k) for k in self.__dataclass_fields__}


def _disp(u: np.ndarray, node_id: int) -> np.ndarray:
    b = 6 * node_id
    return u[b : b + 3].copy()


def _rot(u: np.ndarray, node_id: int) -> np.ndarray:
    b = 6 * node_id
    return u[b + 3 : b + 6].copy()


def compliance_geometry_update(
    pickup_map: PickupMap,
    u: np.ndarray,
    *,
    camber_gain: float = 1.0,
    toe_gain: float = 1.0,
) -> GeometryDelta:
    """
    First-order mapping from FEM nodal displacements to alignment changes.

    Model (engineering approximation, not full hardpoint solver):
      - Vertical differential between upper/lower → camber
      - Longitudinal differential / lateral → toe
      - Lateral outward motion of L/R pickups → track change
      - Front vs rear vertical couple → chassis twist proxy
    """
    g = GeometryDelta()
    if u is None or u.size == 0:
        return g

    def safe_id(role: str) -> int | None:
        return pickup_map.nodes.get(role)

    # --- Camber from upper/lower relative vertical (and rotation ry) ---
    for corner, upper, lower, attr_c, attr_t in (
        ("fl", "upper_fl", "lower_fl", "d_camber_fl", "d_toe_fl"),
        ("fr", "upper_fr", "lower_fr", "d_camber_fr", "d_toe_fr"),
        ("rl", "upper_rl", "lower_rl", "d_camber_rl", "d_toe_rl"),
        ("rr", "upper_rr", "lower_rr", "d_camber_rr", "d_toe_rr"),
    ):
        uid, lid = safe_id(upper), safe_id(lower)
        if uid is None or lid is None:
            # fall back to susp pickup rotation
            susp = safe_id(f"susp_{corner}")
            if susp is not None:
                r = _rot(u, susp)
                # ry ≈ camber contribution, rz ≈ toe contribution
                setattr(g, attr_c, float(camber_gain * r[1]))
                setattr(g, attr_t, float(toe_gain * r[2]))
            continue

        du = _disp(u, uid)
        dl = _disp(u, lid)
        ref_u = np.array(pickup_map.ref(upper))
        ref_l = np.array(pickup_map.ref(lower))
        h = abs(ref_u[2] - ref_l[2]) + 1e-9
        # relative lateral motion of upper vs lower → camber
        d_camber = camber_gain * ((du[1] - dl[1]) / h)
        # relative longitudinal → toe
        d_toe = toe_gain * ((du[0] - dl[0]) / h)
        setattr(g, attr_c, float(d_camber))
        setattr(g, attr_t, float(d_toe))

    # --- Track width ---
    fl, fr = safe_id("susp_fl"), safe_id("susp_fr")
    rl, rr = safe_id("susp_rl"), safe_id("susp_rr")
    if fl is not None and fr is not None:
        g.d_track_front = float(_disp(u, fr)[1] - _disp(u, fl)[1])
    if rl is not None and rr is not None:
        g.d_track_rear = float(_disp(u, rr)[1] - _disp(u, rl)[1])

    # --- Roll-center height proxy: average vertical of lower pickups vs ref ---
    for side, la, lb, attr in (
        ("front", "lower_fl", "lower_fr", "d_rc_front"),
        ("rear", "lower_rl", "lower_rr", "d_rc_rear"),
    ):
        ia, ib = safe_id(la), safe_id(lb)
        if ia is not None and ib is not None:
            setattr(
                g,
                attr,
                float(0.5 * (_disp(u, ia)[2] + _disp(u, ib)[2])),
            )

    # --- Max pickup displacement ---
    max_d = 0.0
    for role, nid in pickup_map.nodes.items():
        max_d = max(max_d, float(np.linalg.norm(_disp(u, nid))))
    g.max_pickup_disp = max_d

    # --- Chassis twist: front L/R vertical differential vs rear ---
    if fl is not None and fr is not None and rl is not None and rr is not None:
        z_f = _disp(u, fl)[2] - _disp(u, fr)[2]
        z_r = _disp(u, rl)[2] - _disp(u, rr)[2]
        # track from refs
        y_f = abs(pickup_map.ref("susp_fr")[1] - pickup_map.ref("susp_fl")[1]) + 1e-9
        twist_f = z_f / y_f
        twist_r = z_r / (
            abs(pickup_map.ref("susp_rr")[1] - pickup_map.ref("susp_rl")[1]) + 1e-9
        )
        g.chassis_twist_rad = float(twist_f - twist_r)

    return g
