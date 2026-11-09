# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core: Natural Roads (Axial Stroke Concatenation) Solver
Implements Jiang & Claramunt (2004) & Thomson (2003) Gestalt Good Continuity:
Pairs collinear segments across junctions (theta <= max_deflection_deg)
to reconstruct continuous movement corridors across dendritic urban superblocks.
"""

import math
from typing import List, Tuple, Dict
from collections import defaultdict
from shapely.geometry import LineString


def get_bearing(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculates directional angle of segment from p1 to p2."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.atan2(dx, dy)


def angular_deflection_rad(b1: float, b2: float) -> float:
    """Calculates angular deflection in radians between two vectors (0 = straight through, pi = sharp reversal)."""
    diff = abs(b1 - b2) % (2.0 * math.pi)
    if diff > math.pi:
        diff = 2.0 * math.pi - diff
    return abs(diff - math.pi)


def concatenate_natural_road_strokes(
    lines: List[LineString],
    max_deflection_deg: float = 20.0
) -> List[LineString]:
    """
    Concatenates contiguous line segments into perceptual 'natural road' strokes.
    Segments meeting at junctions with deflection <= max_deflection_deg are joined.
    """
    if not lines:
        return []

    max_deflection_rad = math.radians(max_deflection_deg)
    endpoint_map = defaultdict(list)
    seg_bearings = []

    for i, line in enumerate(lines):
        coords = list(line.coords)
        p_start = (round(coords[0][0], 2), round(coords[0][1], 2))
        p_end = (round(coords[-1][0], 2), round(coords[-1][1], 2))

        b_start = get_bearing(coords[0], coords[1])
        b_end = get_bearing(coords[-1], coords[-2])

        endpoint_map[p_start].append((i, True))
        endpoint_map[p_end].append((i, False))
        seg_bearings.append((b_start, b_end, coords))

    # Pair best collinear continuation across junctions
    connections: Dict[Tuple[int, bool], Tuple[int, bool]] = {}

    for pt, members in endpoint_map.items():
        if len(members) < 2:
            continue

        best_pair = None
        min_deflect = max_deflection_rad

        for a in range(len(members)):
            for b in range(a + 1, len(members)):
                i, is_start_i = members[a]
                j, is_start_j = members[b]
                if i == j:
                    continue

                b_i = seg_bearings[i][0] if is_start_i else seg_bearings[i][1]
                b_j = seg_bearings[j][0] if is_start_j else seg_bearings[j][1]

                defl = angular_deflection_rad(b_i, b_j)
                if defl < min_deflect:
                    min_deflect = defl
                    best_pair = (members[a], members[b])

        if best_pair is not None:
            m1, m2 = best_pair
            if m1 not in connections and m2 not in connections:
                connections[m1] = m2
                connections[m2] = m1

    # Trace continuous strokes
    visited_ends = set()
    strokes: List[LineString] = []

    for i in range(len(lines)):
        for is_start in [True, False]:
            end_key = (i, is_start)
            if end_key in visited_ends:
                continue

            # Skip if mid-stroke
            if end_key in connections and (i, not is_start) in connections:
                continue

            stroke_coords = []
            curr_i = i
            curr_start = is_start

            curr_coords = seg_bearings[curr_i][2]
            if curr_start:
                stroke_coords.extend(curr_coords)
                curr_exit_end = (curr_i, False)
            else:
                stroke_coords.extend(list(reversed(curr_coords)))
                curr_exit_end = (curr_i, True)

            visited_ends.add((curr_i, True))
            visited_ends.add((curr_i, False))

            while curr_exit_end in connections:
                next_end = connections[curr_exit_end]
                next_i, next_is_start = next_end

                if (next_i, True) in visited_ends and (next_i, False) in visited_ends:
                    break

                visited_ends.add((next_i, True))
                visited_ends.add((next_i, False))

                next_coords = seg_bearings[next_i][2]
                if next_is_start:
                    stroke_coords.extend(next_coords[1:])
                    curr_exit_end = (next_i, False)
                else:
                    stroke_coords.extend(list(reversed(next_coords))[1:])
                    curr_exit_end = (next_i, True)

            if len(stroke_coords) >= 2:
                strokes.append(LineString(stroke_coords))

    # Add remaining lines
    for i, line in enumerate(lines):
        if (i, True) not in visited_ends and (i, False) not in visited_ends:
            strokes.append(line)

    return strokes
