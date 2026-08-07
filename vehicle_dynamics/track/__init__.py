"""Phase 12.1 – Virtual Test Track & Lap Simulation."""

from .track import Track
from .track_segments import TrackSegment, SurfaceProperties, straight, constant_radius, hairpin
from .track_loader import TrackLibrary, from_segments, from_csv_centerline
from .racing_line import RacingLine, center_line, ideal_line
from .sector_timer import SectorTimer, equal_sectors, best_sectors
from .lap_simulator import LapSimulator, LapResult, SessionResult, compare_vehicles
from .lap_statistics import LapMetrics, SessionStatistics
from .friction_map import FrictionMap
from .curvature import curvature_from_xy, reference_speed

__all__ = [
    "Track",
    "TrackSegment",
    "SurfaceProperties",
    "straight",
    "constant_radius",
    "hairpin",
    "TrackLibrary",
    "from_segments",
    "from_csv_centerline",
    "RacingLine",
    "center_line",
    "ideal_line",
    "SectorTimer",
    "equal_sectors",
    "best_sectors",
    "LapSimulator",
    "LapResult",
    "SessionResult",
    "compare_vehicles",
    "LapMetrics",
    "SessionStatistics",
    "FrictionMap",
    "curvature_from_xy",
    "reference_speed",
]
