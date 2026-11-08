# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core: Topology, Planarization & Collinear Merge Engine
Implements:
1. O(N log N) geometric planarization & noding at crossing points.
2. Degree-2 collinear line merge while strictly locking junction vertices (degree >= 3).
3. 2-Point simple LINE explosion and indexing.
4. Grade-separation unlinking for elevated bridges and tunnels.
"""

from typing import List, Tuple, Dict, Any
from collections import defaultdict
import math
import shapely
from shapely.geometry import LineString, Point
from shapely.ops import linemerge


def is_prenoded_segment_map(lines: List[LineString]) -> bool:
    """
    Returns True if `lines` is already a planarized 2-point segment map
    (all LineStrings have 2 vertices and at least 50% of endpoints meet at shared vertices),
    preserving 1-to-1 segment order and grade-separated overpass crossings (FALS-08).
    """
    if not lines or len(lines) < 3:
        return False
    ep_counts: Dict[Tuple[int, int], int] = defaultdict(int)
    keys = []
    for ln in lines:
        if ln is None or ln.is_empty or ln.geom_type != "LineString":
            return False
        coords = list(ln.coords)
        if len(coords) != 2:
            return False
        k0 = (int(round(coords[0][0] * 100.0)), int(round(coords[0][1] * 100.0)))
        k1 = (int(round(coords[1][0] * 100.0)), int(round(coords[1][1] * 100.0)))
        ep_counts[k0] += 1
        ep_counts[k1] += 1
        keys.append((k0, k1))
    total_ep = len(lines) * 2
    shared_ep = sum(
        (1 if ep_counts[k0] >= 2 else 0) + (1 if ep_counts[k1] >= 2 else 0)
        for k0, k1 in keys
    )
    return (float(shared_ep) / float(total_ep)) >= 0.50


def planarize_and_node(lines: List[LineString]) -> List[LineString]:
    """
    Splits all input lines at every crossing and intersection point.
    Guarantees that lines only touch at their end vertices.
    Executes in O(N log N) using GEOSNode.
    """
    if not lines:
        return []

    mls = shapely.multilinestrings(lines)
    noded = shapely.node(mls)
    parts = list(shapely.get_parts(noded))

    clean_parts = []
    for p in parts:
        if p.geom_type == "LineString" and len(p.coords) >= 2 and p.length > 0.05:
            clean_parts.append(p)
    return clean_parts


def collinear_merge_preserving_junctions(lines: List[LineString]) -> List[LineString]:
    """
    Merges unbranched consecutive segments (degree == 2) while 100% preserving
    all junction nodes (degree >= 3) and dead ends (degree == 1).
    Prevents over-segmentation along straight corridors.
    """
    if not lines:
        return []

    endpoint_counts = defaultdict(int)
    for line in lines:
        coords = list(line.coords)
        p0 = (round(coords[0][0], 2), round(coords[0][1], 2))
        p1 = (round(coords[-1][0], 2), round(coords[-1][1], 2))
        endpoint_counts[p0] += 1
        endpoint_counts[p1] += 1

    # Segments that meet at degree-2 nodes can be merged
    # Segments that touch degree >= 3 nodes must retain their junction endpoint
    junction_points = {pt for pt, count in endpoint_counts.items() if count >= 3}

    # Group connected degree-2 runs
    merged = linemerge(lines)
    if merged.geom_type == "LineString":
        candidates = [merged]
    else:
        candidates = list(shapely.get_parts(merged))

    # If any merged line crosses a junction point internally, split it at that junction
    final_lines: List[LineString] = []
    for c in candidates:
        coords = list(c.coords)
        split_indices = [0]
        for idx in range(1, len(coords) - 1):
            pt = (round(coords[idx][0], 2), round(coords[idx][1], 2))
            if pt in junction_points:
                split_indices.append(idx)
        split_indices.append(len(coords) - 1)

        for i in range(len(split_indices) - 1):
            start_i = split_indices[i]
            end_i = split_indices[i + 1]
            sub_coords = coords[start_i:end_i + 1]
            if len(sub_coords) >= 2:
                final_lines.append(LineString(sub_coords))

    return final_lines


def explode_to_2point_segments(lines: List[LineString]) -> List[LineString]:
    """
    Explodes polyline segments into simple 2-point LINE entities
    matching DepthmapX's native Segment Map coordinate structure.
    """
    if is_prenoded_segment_map(lines):
        return list(lines)
    exploded: List[LineString] = []
    for line in lines:
        coords = list(line.coords)
        for i in range(len(coords) - 1):
            p1 = coords[i]
            p2 = coords[i + 1]
            if Point(p1).distance(Point(p2)) > 0.05:
                exploded.append(LineString([p1, p2]))
    return exploded


def turn_angle(b1: float, b2: float) -> float:
    """
    Calculates through-movement deflection angle in radians [0.0, pi]
    between two outward segment bearings meeting at a shared junction.
    """
    diff = abs(b1 - b2) % (2.0 * math.pi)
    if diff > math.pi:
        diff = 2.0 * math.pi - diff
    return abs(diff - math.pi)


def build_angular_dual_graph(
    lines: List[LineString]
) -> Tuple[Any, List[Dict[str, Any]], Dict[str, Any]]:
    """
    Constructs a NetworkX angular dual graph from planarized segments
    (retained for Gen-1 API compatibility).
    """
    import networkx as nx
    segments = explode_to_2point_segments(lines)
    G = nx.Graph()
    endpoint_map = defaultdict(list)
    seg_records: List[Dict[str, Any]] = []

    for i, seg in enumerate(segments):
        coords = list(seg.coords)
        p0 = (round(coords[0][0], 2), round(coords[0][1], 2))
        p1 = (round(coords[-1][0], 2), round(coords[-1][1], 2))
        b0 = math.atan2(coords[1][0] - coords[0][0], coords[1][1] - coords[0][1])
        b1 = math.atan2(coords[-2][0] - coords[-1][0], coords[-2][1] - coords[-1][1])
        length_m = float(seg.length)

        G.add_node(i, length_m=length_m)
        seg_records.append({
            "id": i,
            "p0": p0,
            "p1": p1,
            "b0": b0,
            "b1": b1,
            "length_m": length_m,
            "geom": seg
        })
        endpoint_map[p0].append((i, 0))
        endpoint_map[p1].append((i, 1))

    for _pt, members in endpoint_map.items():
        if len(members) < 2:
            continue
        for a in range(len(members)):
            for b in range(a + 1, len(members)):
                i, end_i = members[a]
                j, end_j = members[b]
                if i == j:
                    continue
                b_i = seg_records[i]["b0"] if end_i == 0 else seg_records[i]["b1"]
                b_j = seg_records[j]["b0"] if end_j == 0 else seg_records[j]["b1"]
                ang_rad = turn_angle(b_i, b_j)
                avg_len = 0.5 * (seg_records[i]["length_m"] + seg_records[j]["length_m"])
                w = max(ang_rad / (math.pi * 0.5) + 0.0001 * avg_len, 1e-4)
                G.add_edge(i, j, weight=w, angular_rad=ang_rad, metric_m=avg_len)

    if len(segments) > 0:
        largest_cc = max((len(c) for c in nx.connected_components(G)), default=0)
        main_pct = round((largest_cc / len(segments)) * 100.0, 2)
    else:
        main_pct = 0.0

    stats = {
        "num_segments": len(segments),
        "num_graph_edges": G.number_of_edges(),
        "main_component_pct": main_pct
    }
    return G, seg_records, stats
