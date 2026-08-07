"""Load tracks from CSV or build from segment lists."""
from __future__ import annotations

from pathlib import Path
import numpy as np

from .track import Track
from .track_segments import TrackSegment, SurfaceProperties, straight, constant_radius, hairpin


def from_segments(name: str, segments: list[TrackSegment], ds: float = 1.0, closed: bool = True) -> Track:
    tr = Track(name=name, segments=segments, closed=closed)
    return tr.build(ds=ds)


def from_csv_centerline(path: str | Path, name: str | None = None, width: float = 12.0, mu: float = 1.0, ds: float | None = None) -> Track:
    """CSV columns: x,y[,z][,width][,mu]."""
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    x, y = data[:, 0], data[:, 1]
    z = data[:, 2] if data.shape[1] > 2 else np.zeros_like(x)
    w = data[:, 3] if data.shape[1] > 3 else np.full_like(x, width)
    # Build as dense single-segment track
    dx = np.diff(x, prepend=x[0])
    dy = np.diff(y, prepend=y[0])
    ds_arr = np.hypot(dx, dy)
    ds_arr[0] = 0.0
    s = np.cumsum(ds_arr)
    heading = np.arctan2(np.gradient(y), np.gradient(x) + 1e-12)
    tr = Track(name=name or Path(path).stem, segments=[])
    tr.s, tr.x, tr.y, tr.z = s, x, y, z
    tr.heading, tr.width = heading, w
    tr.banking_deg = np.zeros_like(s)
    from .curvature import curvature_from_xy
    from .friction_map import FrictionMap
    tr.curvature = curvature_from_xy(x, y)
    tr.friction = FrictionMap.uniform(float(s[-1]) if len(s) else 0.0, mu)
    tr.closed = True
    return tr


class TrackLibrary:
    @staticmethod
    def straight(length: float = 1000.0, width: float = 12.0, mu: float = 1.0, ds: float = 2.0) -> Track:
        seg = straight(length, width=width, surface=SurfaceProperties(mu=mu))
        return from_segments("straight", [seg], ds=ds, closed=False)

    @staticmethod
    def oval(radius: float = 100.0, straights: float = 200.0, width: float = 15.0, banking_deg: float = 8.0, ds: float = 2.0) -> Track:
        segs = [
            straight(straights, width=width, name="front_straight"),
            constant_radius(np.pi * radius, radius, banking_deg=banking_deg, width=width, name="turn1"),
            straight(straights, width=width, name="back_straight"),
            constant_radius(np.pi * radius, radius, banking_deg=banking_deg, width=width, name="turn2"),
        ]
        return from_segments("oval", segs, ds=ds, closed=True)

    @staticmethod
    def handling_course(ds: float = 1.5) -> Track:
        segs = [
            straight(120.0, name="start"),
            constant_radius(80.0, 40.0, name="t1"),
            straight(60.0, name="s2"),
            constant_radius(50.0, -25.0, name="t2"),
            straight(40.0, name="s3"),
            hairpin(radius=18.0, name="hairpin"),
            straight(80.0, name="s4"),
            constant_radius(70.0, 35.0, name="t3"),
            straight(100.0, name="finish"),
        ]
        return from_segments("handling_course", segs, ds=ds, closed=False)

    @staticmethod
    def figure_eight(radius: float = 40.0, ds: float = 1.5) -> Track:
        # Two opposing loops approximated as constant-radius arcs + short straights
        segs = [
            constant_radius(2 * np.pi * radius * 0.5, radius, name="loop_a"),
            straight(20.0, name="cross"),
            constant_radius(2 * np.pi * radius * 0.5, -radius, name="loop_b"),
            straight(20.0, name="cross2"),
        ]
        return from_segments("figure_eight", segs, ds=ds, closed=True)

    @staticmethod
    def slalom(n_gates: int = 6, spacing: float = 25.0, amplitude: float = 4.0, ds: float = 1.0) -> Track:
        segs = []
        for i in range(n_gates):
            r = 20.0 if i % 2 == 0 else -20.0
            segs.append(constant_radius(spacing, r, width=10.0, name=f"gate{i}"))
        segs.append(straight(40.0, name="exit"))
        return from_segments("slalom", segs, ds=ds, closed=False)

    @staticmethod
    def custom(segments: list[TrackSegment], name: str = "custom", ds: float = 1.5) -> Track:
        return from_segments(name, segments, ds=ds)
