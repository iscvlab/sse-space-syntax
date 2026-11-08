"""
Dual Carriageway Transverse Cross-Connector (Rungs) Synthesizer
Detects parallel carriageway pairs on major urban arterials and synthesizes transverse connectors.
Eliminates the 'split-universe' artifact in dual-lane boulevards and connects side streets across the arterial.
"""

import math
from typing import List, Tuple
import shapely
from shapely.geometry import LineString


def get_segment_bearing(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculates bearing from p1 to p2 in radians [0, 2pi)."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    bearing = math.atan2(dx, dy)
    return bearing if bearing >= 0 else (bearing + 2 * math.pi)


def is_near_parallel(b1: float, b2: float, max_angle_deg: float = 25.0) -> bool:
    """Checks if two bearings are parallel or antiparallel within max_angle_deg."""
    diff = abs(b1 - b2) % math.pi
    if diff > math.pi / 2:
        diff = math.pi - diff
    return diff <= math.radians(max_angle_deg)


def synthesize_cross_rungs(
    lines: List[LineString],
    max_lateral_dist_m: float = 35.0,
    min_lateral_dist_m: float = 6.0,
    rung_spacing_m: float = 200.0,
    max_angle_deg: float = 25.0
) -> List[LineString]:
    """
    Synthesizes transverse connector lines ('rungs') between parallel arterial carriageways.
    All inputs must be in metric units (e.g. UTM Zone 47N).
    """
    if len(lines) < 2:
        return []

    # 1. Filter lines longer than 50m for arterial candidate search
    arterial_candidates = []
    for idx, line in enumerate(lines):
        if line.length >= 50.0:
            coords = list(line.coords)
            b = get_segment_bearing(coords[0], coords[-1])
            arterial_candidates.append((idx, line, b))

    if len(arterial_candidates) < 2:
        return []

    # 2. Build spatial index
    cand_lines = [c[1] for c in arterial_candidates]
    tree = shapely.STRtree(cand_lines)

    rungs: List[LineString] = []
    processed_pairs = set()

    for i, (_idx_a, line_a, bearing_a) in enumerate(arterial_candidates):
        # Query candidates within max lateral distance
        buf = line_a.buffer(max_lateral_dist_m)
        neighbors = tree.query(buf)

        for j in neighbors:
            if i >= j:
                continue
            pair_key = (min(i, j), max(i, j))
            if pair_key in processed_pairs:
                continue
            processed_pairs.add(pair_key)

            _idx_b, line_b, bearing_b = arterial_candidates[j]

            # Check parallel alignment
            if not is_near_parallel(bearing_a, bearing_b, max_angle_deg):
                continue

            # Sample points along line_a and project onto line_b
            dist_ab = line_a.distance(line_b)
            if dist_ab < min_lateral_dist_m or dist_ab > max_lateral_dist_m:
                continue

            length_a = line_a.length
            num_rungs = max(1, int(length_a // rung_spacing_m))
            step = length_a / (num_rungs + 1)

            for k in range(1, num_rungs + 1):
                pt_a = line_a.interpolate(k * step)
                proj_dist_b = line_b.project(pt_a)
                pt_b = line_b.interpolate(proj_dist_b)

                connector_len = pt_a.distance(pt_b)
                if min_lateral_dist_m <= connector_len <= max_lateral_dist_m:
                    rungs.append(LineString([(pt_a.x, pt_a.y), (pt_b.x, pt_b.y)]))

    return rungs
