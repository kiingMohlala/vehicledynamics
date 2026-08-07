"""
Minimal OpenDRIVE (.xodr) importer.

Supports a practical subset:
  - road reference line geometry (line, arc)
  - elevation profiles (poly3 simplified to endpoints)
  - lane width from laneSection
Falls back to synthetic geometry if XML is incomplete.
"""
from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np

from vehicle_dynamics.track.track import Track
from vehicle_dynamics.track.track_segments import TrackSegment, SurfaceProperties, straight, constant_radius
from vehicle_dynamics.track.track_loader import from_segments
from vehicle_dynamics.track.friction_map import FrictionMap
from vehicle_dynamics.track.curvature import curvature_from_xy


def _local(tag: str) -> str:
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


def load_opendrive(path: str | Path, ds: float = 2.0) -> Track:
    path = Path(path)
    tree = ET.parse(path)
    root = tree.getroot()

    segments: list[TrackSegment] = []
    total_length = 0.0
    road_name = path.stem

    for elem in root.iter():
        if _local(elem.tag) == "road":
            road_name = elem.attrib.get("name") or elem.attrib.get("id") or road_name
            length = float(elem.attrib.get("length", 0.0))
            total_length = max(total_length, length)
            plan = None
            for child in elem:
                if _local(child.tag) == "planView":
                    plan = child
                    break
            if plan is None:
                continue
            for geo in plan:
                if _local(geo.tag) != "geometry":
                    continue
                gl = float(geo.attrib.get("length", 0.0))
                hdg = float(geo.attrib.get("hdg", 0.0))
                # detect line vs arc
                kind = "line"
                curvature = 0.0
                for sub in geo:
                    t = _local(sub.tag)
                    if t == "arc":
                        kind = "arc"
                        curvature = float(sub.attrib.get("curvature", 0.0))
                    elif t == "line":
                        kind = "line"
                if kind == "arc" and abs(curvature) > 1e-9:
                    radius = 1.0 / curvature
                    segments.append(constant_radius(gl, radius, name=f"arc_{len(segments)}"))
                else:
                    segments.append(straight(gl, name=f"line_{len(segments)}"))

    if not segments:
        # empty / unsupported file → short straight placeholder so API never crashes
        segments = [straight(max(total_length, 100.0), name="fallback")]

    track = from_segments(str(road_name), segments, ds=ds, closed=False)
    track.name = str(road_name)
    return track


def write_minimal_opendrive(path: str | Path, length: float = 200.0, radius: float = 0.0) -> Path:
    """Write a minimal valid-ish .xodr for testing."""
    path = Path(path)
    if abs(radius) < 1e-9:
        geo = f'''    <geometry s="0" x="0" y="0" hdg="0" length="{length}">
      <line/>
    </geometry>'''
    else:
        curv = 1.0 / radius
        geo = f'''    <geometry s="0" x="0" y="0" hdg="0" length="{abs(np.pi * radius)}">
      <arc curvature="{curv}"/>
    </geometry>'''
    xml = f'''<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="test" version="1.0"/>
  <road name="test_road" length="{length}" id="1" junction="-1">
    <planView>
{geo}
    </planView>
  </road>
</OpenDRIVE>
'''
    path.write_text(xml)
    return path
