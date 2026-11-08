"""
Watertight Endpoint Snapping & Gap Closing
Solves disconnected road segments and micro-gaps (0.5m - 3.0m) using R-Tree spatial indexing.
Ensures network connectivity without distorting street geometry.
"""

import math
from typing import List, Tuple, Dict, Set
import shapely
from shapely.geometry import LineString, Point


def snap_endpoints_rtree(lines: List[LineString], tolerance_m: float = 3.0) -> List[LineString]:
    """
    Snaps endpoints of polylines within tolerance_m to their cluster centroid.
    Assumes coordinates are in projected metric units (e.g. UTM Zone 47N in meters).
    """
    if not lines:
        return []

    # 1. Collect all endpoints
    endpoint_records = []  # list of (pt, line_idx, is_start, orig_coord)
    for idx, line in enumerate(lines):
        coords = list(line.coords)
        if len(coords) < 2:
            continue
        p_start = coords[0]
        p_end = coords[-1]
        endpoint_records.append((Point(p_start), idx, True, p_start))
        endpoint_records.append((Point(p_end), idx, False, p_end))

    # 2. Build STRtree on endpoints
    pts = [rec[0] for rec in endpoint_records]
    tree = shapely.STRtree(pts)

    # 3. Cluster endpoints within tolerance_m
    visited: Set[int] = set()
    clusters: List[List[int]] = []

    for i, pt in enumerate(pts):
        if i in visited:
            continue
        buf = pt.buffer(tolerance_m)
        candidate_indices = tree.query(buf)

        current_cluster = []
        for cand_idx in candidate_indices:
            if cand_idx not in visited:
                cand_pt = pts[cand_idx]
                dist = pt.distance(cand_pt)
                if dist <= tolerance_m:
                    visited.add(cand_idx)
                    current_cluster.append(cand_idx)

        if current_cluster:
            clusters.append(current_cluster)

    # 4. Compute centroid for each cluster and map snapped coordinates
    snapped_map: Dict[Tuple[int, bool], Tuple[float, float]] = {}
    for cluster in clusters:
        if len(cluster) == 1:
            continue

        sum_x = sum(pts[c].x for c in cluster)
        sum_y = sum(pts[c].y for c in cluster)
        cx = sum_x / len(cluster)
        cy = sum_y / len(cluster)

        for c in cluster:
            _, line_idx, is_start, _ = endpoint_records[c]
            snapped_map[(line_idx, is_start)] = (cx, cy)

    # 5. Reconstruct lines with snapped endpoints
    healed_lines = []
    for idx, line in enumerate(lines):
        coords = list(line.coords)
        if len(coords) < 2:
            continue

        start_pt = snapped_map.get((idx, True), coords[0])
        end_pt = snapped_map.get((idx, False), coords[-1])

        new_coords = [start_pt]
        if len(coords) > 2:
            new_coords.extend(coords[1:-1])
        new_coords.append(end_pt)

        dx = new_coords[-1][0] - new_coords[0][0]
        dy = new_coords[-1][1] - new_coords[0][1]
        if math.hypot(dx, dy) >= 0.1:
            healed_lines.append(LineString(new_coords))
        else:
            healed_lines.append(line)

    return healed_lines
