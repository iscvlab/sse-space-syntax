# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core: Geometric Cleaning & Watertight Snapping Engine
Implements:
1. Noise filtering (removes non-public parking aisles, service loops, construction).
2. Watertight endpoint snapping (STRtree 3.0m) to bridge OSM digitizing gaps.
3. Dual-carriageway transverse pedestrian cross-connectors (rungs) to unify divided corridors.
"""

import math
from typing import List, Tuple, Dict, Any, Set
from collections import defaultdict
import numpy as np
import shapely
from shapely.geometry import LineString, Point
from shapely.strtree import STRtree


DEFAULT_NOISE_HIGHWAYS: Set[str] = {
    'services', 'rest_area', 'construction', 'elevator', 'abandoned', 'proposed'
}

DEFAULT_MAJOR_HIGHWAYS: Set[str] = {
    'primary', 'secondary', 'tertiary', 'trunk', 'motorway',
    'primary_link', 'secondary_link', 'tertiary_link', 'trunk_link', 'motorway_link'
}


def filter_noise_features(
    features: List[Dict[str, Any]],
    noise_tags: Set[str] = None
) -> List[Dict[str, Any]]:
    """Filters out private access ways, service loops, and temporary construction stubs."""
    tags_to_drop = noise_tags or DEFAULT_NOISE_HIGHWAYS
    cleaned = []
    for f in features:
        props = f.get("properties", {})
        hw = str(props.get("highway", "")).lower()
        if hw not in tags_to_drop:
            cleaned.append(f)
    return cleaned


def extract_grade_level_from_props(
    props: Dict[str, Any],
    field_name: str = ""
) -> float:
    """
    Extracts a numeric elevation/grade level from a feature properties dictionary.
    Supports user-specified field_name as well as standard OSM tags ('layer',
    'z_order', 'level', 'bridge', 'tunnel'). Defaults to 0.0 (ground surface).
    """
    if not props:
        return 0.0

    def _parse_val(raw_val: Any, is_tunnel: bool = False) -> Tuple[bool, float]:
        if raw_val is None:
            return False, 0.0
        if isinstance(raw_val, bool):
            return True, ((-1.0 if is_tunnel else 1.0) if raw_val else 0.0)
        if isinstance(raw_val, (int, float)):
            fval = float(raw_val)
            if math.isnan(fval):
                return False, 0.0
            return True, fval
        s = str(raw_val).strip().lower()
        if not s or s in ("none", "null", "nan"):
            return False, 0.0
        if s in ("no", "false"):
            return True, 0.0
        if s in ("yes", "true", "viaduct", "bridge", "flyover", "overpass"):
            return True, (-1.0 if is_tunnel else 1.0)
        if s in ("tunnel", "underpass"):
            return True, -1.0
        if is_tunnel and s == "1":
            return True, -1.0
        try:
            fval = float(s)
            if not math.isnan(fval):
                return True, fval
        except ValueError:
            pass
        return False, 0.0

    if field_name:
        fname = str(field_name).strip()
        raw = props.get(fname)
        if raw is None:
            for k, v in props.items():
                if str(k).strip().lower() == fname.lower():
                    raw = v
                    break
        ok, val = _parse_val(raw, is_tunnel=(fname.lower() == "tunnel"))
        if ok:
            return val

    for k in ("layer", "z_order", "level"):
        raw = props.get(k)
        ok, val = _parse_val(raw, is_tunnel=False)
        if ok and val != 0.0:
            return val

    bridge_val = str(props.get("bridge", "") or "").strip().lower()
    if bridge_val in ("yes", "1", "true", "viaduct"):
        return 1.0

    tunnel_val = str(props.get("tunnel", "") or "").strip().lower()
    if tunnel_val in ("yes", "1", "true", "underpass"):
        return -1.0

    return 0.0


def snap_endpoints_watertight(
    lines: List[LineString],
    snap_distance_m: float = 3.0
) -> List[LineString]:
    """
    Projects dangling endpoints within snap_distance_m onto nearby intersecting streets.
    Resolves OpenStreetMap digitizing gaps where side sois terminate 0.5m - 2.5m away from main arterials.
    Uses O(N log N) STRtree spatial queries.
    """
    if not lines or snap_distance_m <= 0.0:
        return lines

    tree = STRtree(lines)
    snapped_lines: List[LineString] = []

    for idx, line in enumerate(lines):
        coords = list(line.coords)
        if len(coords) < 2:
            continue

        p_start = Point(coords[0])
        p_end = Point(coords[-1])

        # 1. Snap Start Point
        candidates_start = [lines[i] for i in tree.query(p_start.buffer(snap_distance_m)) if i != idx]
        new_start = coords[0]
        min_dist_start = snap_distance_m
        for c in candidates_start:
            d = c.distance(p_start)
            if 0.001 < d < min_dist_start:
                proj_pt = c.interpolate(c.project(p_start))
                new_start = (round(proj_pt.x, 3), round(proj_pt.y, 3))
                min_dist_start = d

        # 2. Snap End Point
        candidates_end = [lines[i] for i in tree.query(p_end.buffer(snap_distance_m)) if i != idx]
        new_end = coords[-1]
        min_dist_end = snap_distance_m
        for c in candidates_end:
            d = c.distance(p_end)
            if 0.001 < d < min_dist_end:
                proj_pt = c.interpolate(c.project(p_end))
                new_end = (round(proj_pt.x, 3), round(proj_pt.y, 3))
                min_dist_end = d

        new_coords = [new_start] + coords[1:-1] + [new_end]
        snapped_lines.append(LineString(new_coords))

    return snapped_lines


def synthesize_dual_carriageway_rungs(
    lines: List[LineString],
    line_types: List[str] = None,
    min_gap_m: float = 5.0,
    max_gap_m: float = 22.0,
    spacing_m: float = 60.0
) -> List[LineString]:
    """
    Generates transverse pedestrian cross-connections (rungs) between parallel dual-carriageway lanes.
    Solves the classic Space Syntax dual-carriageway problem where divided streets split into
    two parallel, disconnected graphs.
    """
    if not lines:
        return []

    # If highway types are provided, focus on major arterials; otherwise examine all lines >= 20m
    if line_types and len(line_types) == len(lines):
        candidate_indices = [
            i for i, hw in enumerate(line_types)
            if str(hw).lower() in DEFAULT_MAJOR_HIGHWAYS and lines[i].length >= 20.0
        ]
    else:
        candidate_indices = [i for i, ln in enumerate(lines) if ln.length >= 20.0]

    if not candidate_indices:
        return []

    candidate_lines = [lines[i] for i in candidate_indices]
    tree = STRtree(candidate_lines)
    rungs: List[LineString] = []

    for local_idx, line in enumerate(candidate_lines):
        length = line.length
        num_rungs = max(1, int(length // spacing_m))
        step_len = length / (num_rungs + 1)

        for step in range(1, num_rungs + 1):
            sample_pt = line.interpolate(step * step_len)
            nearby_indices = [j for j in tree.query(sample_pt.buffer(max_gap_m)) if j != local_idx]

            best_neighbor_pt = None
            min_dist = max_gap_m

            for j in nearby_indices:
                other_line = candidate_lines[j]
                other_proj = other_line.interpolate(other_line.project(sample_pt))
                dist = sample_pt.distance(other_proj)

                if min_gap_m <= dist < min_dist:
                    min_dist = dist
                    best_neighbor_pt = other_proj

            if best_neighbor_pt is not None:
                rungs.append(LineString([sample_pt, best_neighbor_pt]))

    return rungs


def angle_between_vectors(v1: Tuple[float, float], v2: Tuple[float, float]) -> float:
    """
    Computes the 2D angle in degrees [0.0, 180.0] between two vectors.
    Uses dot product and magnitude with numerical clamping.
    """
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    mag1 = math.hypot(v1[0], v1[1])
    mag2 = math.hypot(v2[0], v2[1])
    if mag1 == 0.0 or mag2 == 0.0:
        return 0.0
    cos_theta = max(-1.0, min(1.0, dot / (mag1 * mag2)))
    return math.degrees(math.acos(cos_theta))


def _pt_key(pt: Tuple[float, float]) -> Tuple[float, float]:
    """Canonical 1mm precision topological vertex key."""
    return (round(float(pt[0]), 3), round(float(pt[1]), 3))


def count_connected_components(lines: List[LineString]) -> int:
    """
    Counts the number of connected graph components in O(V + E) using BFS.
    """
    if not lines:
        return 0
    adj = defaultdict(list)
    for line in lines:
        coords = list(line.coords)
        if len(coords) < 2:
            continue
        p0 = _pt_key(coords[0])
        p1 = _pt_key(coords[-1])
        adj[p0].append(p1)
        adj[p1].append(p0)

    visited = set()
    components = 0
    for node in adj:
        if node not in visited:
            components += 1
            queue = [node]
            visited.add(node)
            while queue:
                curr = queue.pop()
                for neighbor in adj[curr]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
    return components


def count_graph_junctions(lines: List[LineString]) -> int:
    """
    Counts intersection nodes with degree >= 3.
    """
    if not lines:
        return 0
    endpoint_counts = defaultdict(int)
    for line in lines:
        coords = list(line.coords)
        if len(coords) < 2:
            continue
        p0 = _pt_key(coords[0])
        p1 = _pt_key(coords[-1])
        endpoint_counts[p0] += 1
        endpoint_counts[p1] += 1
    return sum(1 for deg in endpoint_counts.values() if deg >= 3)


def count_dangling_endpoints(lines: List[LineString]) -> int:
    """
    Counts true dead-end vertices with degree == 1.
    """
    if not lines:
        return 0
    endpoint_counts = defaultdict(int)
    for line in lines:
        coords = list(line.coords)
        if len(coords) < 2:
            continue
        endpoint_counts[_pt_key(coords[0])] += 1
        endpoint_counts[_pt_key(coords[-1])] += 1
    return sum(1 for deg in endpoint_counts.values() if deg == 1)


def collinear_join_with_turn_conservation(
    lines: List[LineString],
    max_turn_deflection: float = 15.0
) -> List[LineString]:
    """
    Merges unbranched consecutive lines (degree == 2) while:
    1. 100% preserving all junction nodes (degree >= 3) and dead ends (degree == 1).
    2. Strictly preserving sharp corners where deflection angle > max_turn_deflection (15.0°).
       Sharp 90° turns retain their boundary vertex to maintain Space Syntax angular step depth accuracy.
    """
    if not lines:
        return []

    adj = defaultdict(list)
    v_degree = defaultdict(int)
    for idx, ln in enumerate(lines):
        c = list(ln.coords)
        if len(c) < 2:
            continue
        p0 = _pt_key(c[0])
        p1 = _pt_key(c[-1])
        v_degree[p0] += 1
        v_degree[p1] += 1
        adj[p0].append((idx, 0, p1))
        adj[p1].append((idx, -1, p0))

    mergeable_nodes = set()
    for pt, deg in v_degree.items():
        if deg == 2:
            (idx1, end1, _), (idx2, end2, _) = adj[pt]
            if idx1 == idx2:
                continue
            c1, c2 = list(lines[idx1].coords), list(lines[idx2].coords)
            v1 = (pt[0] - c1[1][0], pt[1] - c1[1][1]) if end1 == 0 else (pt[0] - c1[-2][0], pt[1] - c1[-2][1])
            v2 = (c2[1][0] - pt[0], c2[1][1] - pt[1]) if end2 == 0 else (c2[-2][0] - pt[0], c2[-2][1] - pt[1])
            defl = angle_between_vectors(v1, v2)
            if defl <= max_turn_deflection:
                mergeable_nodes.add(pt)

    visited_edges = set()
    result_lines = []

    for pt in list(v_degree.keys()):
        if pt in mergeable_nodes:
            continue
        for edge_idx, end_pos, next_pt in adj[pt]:
            if edge_idx in visited_edges:
                continue
            visited_edges.add(edge_idx)
            cur_line = lines[edge_idx]
            cur_coords = list(cur_line.coords)
            if end_pos != 0:
                cur_coords = cur_coords[::-1]
            curr_pt = next_pt
            chain_coords = list(cur_coords)
            while curr_pt in mergeable_nodes:
                next_edges = [e for e in adj[curr_pt] if e[0] not in visited_edges]
                if not next_edges:
                    break
                next_e_idx, next_end_pos, n_next_pt = next_edges[0]
                visited_edges.add(next_e_idx)
                next_l_coords = list(lines[next_e_idx].coords)
                if next_end_pos != 0:
                    next_l_coords = next_l_coords[::-1]
                chain_coords.extend(next_l_coords[1:])
                curr_pt = n_next_pt
            result_lines.append(LineString(chain_coords))

    for idx, ln in enumerate(lines):
        if idx not in visited_edges:
            result_lines.append(ln)

    return result_lines


def remove_dead_end_stubs(
    lines: List[LineString],
    max_stub_length_m: float = 0.0
) -> List[LineString]:
    """
    Removes dead-end cul-de-sac / driveway stubs shorter than max_stub_length_m
    that hang off an intersection (one endpoint deg == 1, other endpoint deg >= 3).
    Preserves isolated single-segment components where both endpoints have deg == 1.
    """
    if not lines or max_stub_length_m <= 0.0:
        return list(lines)

    deg = defaultdict(int)
    for ln in lines:
        if ln is None or ln.is_empty or len(ln.coords) < 2:
            continue
        c = list(ln.coords)
        deg[_pt_key(c[0])] += 1
        deg[_pt_key(c[-1])] += 1

    filtered: List[LineString] = []
    for ln in lines:
        if ln is None or ln.is_empty or len(ln.coords) < 2:
            continue
        c = list(ln.coords)
        d0 = deg[_pt_key(c[0])]
        d1 = deg[_pt_key(c[-1])]
        is_hanging_stub = (
            ln.length < max_stub_length_m
            and ((d0 == 1 and d1 >= 3) or (d1 == 1 and d0 >= 3))
        )
        if not is_hanging_stub:
            filtered.append(ln)
    return filtered


def clean_network_adaptive_watertight(
    lines: List[LineString],
    mode: str = "auto",
    extend_distance: float = 15.0,
    angle_min: float = 15.0,
    angle_max: float = 165.0,
    min_stub_len: float = 0.5,
    max_turn_deflection: float = 15.0,
    lateral_fallback: bool = True,
    run_assertions: bool = True,
    dead_end_stub_len: float = 0.0,
    line_levels: List[float] = None
) -> Tuple[List[LineString], Dict[str, Any]]:
    """
    Enhanced 5-Step Watertight Network Cleaning Pipeline with Dual-Mode Interface Contract.
    Conforms strictly to SSE (Space Syntax Engine) Spatial Modeling Quality Standards (Invariants 1-10).

    Steps:
    0. Dangle Incidence Audit: Identify true dead ends (degree == 1); lock all junctions (degree >= 2).
    1. Explode to 2-point segments: Deconstruct polylines to unit approach tangent vectors v = (P_end - P_prev) / len.
    2. Adaptive Forward Raycasting:
       - Auto Mode: Adaptive search ceiling D_max = clamp(P90_gaps, 6.0m, 20.0m) via empirical nearest-neighbor gaps.
       - Manual Mode: User-defined numerical override via extend_distance parameter.
       - Deflection Angle Guard: 15° to 165° safe window prevents acute parallel frontage road false merges.
       - First-Hit Termination: Stops at earliest crossed edge (min_t) to prevent piercing barriers.
       - Slip-Road Obliquity Compensation: Evaluates perpendicular Right-of-Way gap d_perp = d * sin(theta).
       - Crossing Overshoot Assist (+0.05m): Guarantees exact float64 noding split in Step 3 before stub pruning.
       - Lateral Fallback: Connects perpendicular T-junction digitizing undershoots within 3.0m.
    3. Trim & Planarize:
       - Nodalize crossing lines in O(N log N) using shapely.node.
       - Micro-Stub Pruning: Automatically purge dangling overshoot stubs shorter than min_stub_len (0.5m).
    4. Collinear Join with Turn Conservation:
       - Merges degree-2 unbranched lines where angle deviation <= max_turn_deflection (15.0°).
       - Preserves sharp turns (> 15.0°) to maintain Space Syntax angular step depth fidelity.
    5. Post-Flight Topology Audit:
       - Validates zero invalid geometries, junction conservation, stub elimination, and length delta.

    Returns:
        (cleaned_lines, audit_report_dict)
    """
    if not lines:
        return [], {"error": "Empty input"}

    has_multi_levels = (
        line_levels is not None
        and len(line_levels) == len(lines)
        and len({float(lv) for lv in line_levels}) > 1
    )

    clean_input_lines = []
    clean_input_levels = []
    for idx_in, ln in enumerate(lines):
        if ln and not ln.is_empty and ln.geom_type == "LineString" and len(ln.coords) >= 2:
            clean_input_lines.append(ln)
            clean_input_levels.append(float(line_levels[idx_in]) if has_multi_levels else 0.0)
    if not clean_input_lines:
        return [], {"error": "No valid LineStrings in input"}

    # Step 0 & 1: Explode into 2-point segments & identify degree == 1 dangles
    segments: List[LineString] = []
    seg_levels: List[float] = []
    for line, lv in zip(clean_input_lines, clean_input_levels):
        coords = list(line.coords)
        for i in range(len(coords) - 1):
            seg = LineString([coords[i], coords[i + 1]])
            if seg.length > 0.001:
                segments.append(seg)
                seg_levels.append(lv)

    pt_degree = defaultdict(int)
    for seg in segments:
        c = list(seg.coords)
        pt_degree[_pt_key(c[0])] += 1
        pt_degree[_pt_key(c[-1])] += 1

    dangling_pts = {pt for pt, deg in pt_degree.items() if deg == 1}
    tree = STRtree(segments)

    # Identify active dangling endpoints (excluding pre-existing <0.5m overshoot stubs at degree>=3 junctions)
    dangle_records = []
    for s_idx, seg in enumerate(segments):
        c = list(seg.coords)
        p0_key = _pt_key(c[0])
        p1_key = _pt_key(c[-1])
        for is_end, p_key, other_key in [(True, p1_key, p0_key), (False, p0_key, p1_key)]:
            if p_key in dangling_pts:
                if min_stub_len > 0.0 and seg.length < min_stub_len and pt_degree[other_key] >= 3:
                    continue
                dangle_records.append((s_idx, seg, is_end, p_key))

    # Step 2a: Adaptive Distance Determination (Dual-Mode Contract)
    mode_clean = str(mode).strip().lower()
    if mode_clean == "auto":
        candidate_gaps = []
        for s_idx, seg, is_end, p_key in dangle_records:
            c = list(seg.coords)
            origin = c[-1] if is_end else c[0]
            p = Point(origin)
            indices = tree.query(p.buffer(30.0))
            dists = []
            for idx in indices:
                if idx != s_idx and not segments[idx].touches(seg):
                    d = segments[idx].distance(p)
                    if d > 0.05:
                        dists.append(d)
            if dists:
                candidate_gaps.append(min(dists))

        if len(candidate_gaps) >= 5:
            p90 = float(np.percentile(candidate_gaps, 90))
            effective_dist = max(6.0, min(20.0, p90))
        else:
            effective_dist = 18.39  # Empirical Bangkok OSM P90 baseline
    else:
        effective_dist = float(max(1.0, extend_distance))

    # Step 2b: Forward Raycasting with Deflection Angle Guard, Obliquity Compensation & Lateral Fallback
    ray_extensions: List[LineString] = []
    lateral_extensions: List[LineString] = []
    overshoot_eps = 0.05  # 5cm topological crossing overshoot so shapely.node splits target edges cleanly
    ray_search_max = effective_dist * 1.6  # Oblique slip-road slant envelope (e.g. d_perp / sin(35°))

    # Precompute contiguous (M, 2) unit direction vectors for vectorized cosine angle filtering
    M_segs = len(segments)
    seg_u = np.zeros((M_segs, 2), dtype=np.float64)
    for idx_m, s_m in enumerate(segments):
        cm = s_m.coords
        dx_m = cm[-1][0] - cm[0][0]
        dy_m = cm[-1][1] - cm[0][1]
        len_m = math.hypot(dx_m, dy_m)
        if len_m > 1e-9:
            seg_u[idx_m, 0] = dx_m / len_m
            seg_u[idx_m, 1] = dy_m / len_m
    seg_levels_arr = np.asarray(seg_levels, dtype=np.float64) if has_multi_levels else None
    cos_min_bound = math.cos(math.radians(min(angle_max, 180.0 - angle_min)))
    cos_max_bound = math.cos(math.radians(min(angle_min, 180.0 - angle_max)))
    cos_lat_bound = math.cos(math.radians(45.0))

    for s_idx, seg, is_end, p_key in dangle_records:
        c = list(seg.coords)
        if is_end:
            dx, dy = c[-1][0] - c[-2][0], c[-1][1] - c[-2][1]
            origin = c[-1]
        else:
            dx, dy = c[0][0] - c[1][0], c[0][1] - c[1][1]
            origin = c[0]
        mag = math.hypot(dx, dy)
        if mag <= 0.001:
            continue

        ux, uy = dx / mag, dy / mag
        target_pt = (origin[0] + ray_search_max * ux, origin[1] + ray_search_max * uy)
        ray = LineString([origin, target_pt])
        hits_raw = np.asarray(tree.query(ray), dtype=np.intp)
        best_hit = None
        min_hit_dist = ray_search_max

        if len(hits_raw) > 0:
            valid_mask = hits_raw != s_idx
            if has_multi_levels and seg_levels_arr is not None:
                valid_mask &= (seg_levels_arr[hits_raw] == seg_levels[s_idx])
            cand_indices = hits_raw[valid_mask]
            if len(cand_indices) > 0:
                abs_dots = np.abs(ux * seg_u[cand_indices, 0] + uy * seg_u[cand_indices, 1])
                ang_mask = (abs_dots >= cos_min_bound - 1e-9) & (abs_dots <= cos_max_bound + 1e-9)
                cand_indices = cand_indices[ang_mask]
                abs_dots = abs_dots[ang_mask]

                for h_idx, abs_dot in zip(cand_indices.tolist(), abs_dots.tolist()):
                    cand = segments[h_idx]
                    if cand.equals(seg):
                        continue
                    inter = ray.intersection(cand)
                    if inter.is_empty:
                        continue
                    pts = (
                        [inter] if inter.geom_type == "Point"
                        else (list(inter.geoms) if inter.geom_type == "MultiPoint" else [])
                    )
                    # sin(cross_ang) = sqrt(1 - cos^2(cross_ang))
                    sin_ang = max(0.5, math.sqrt(max(0.0, 1.0 - abs_dot * abs_dot)))

                    # Invariant 3: First-Hit Termination (minimum distance t along ray)
                    for ipt in pts:
                        d = math.hypot(ipt.x - origin[0], ipt.y - origin[1])
                        d_perp = d * sin_ang
                        if 0.02 < d < min_hit_dist and (d <= effective_dist or d_perp <= effective_dist):
                            min_hit_dist = d
                            best_hit = (ipt.x + overshoot_eps * ux, ipt.y + overshoot_eps * uy)

        if best_hit is not None:
            ray_extensions.append(LineString([origin, best_hit]))
        elif lateral_fallback:
            # Perpendicular Lateral Fallback for T-junction digitizing undershoots
            lat_tol = min(3.0, effective_dist * 0.25)
            p_orig = Point(origin)
            lat_hits = np.asarray(tree.query(p_orig.buffer(lat_tol)), dtype=np.intp)
            best_lat = None
            min_lat = lat_tol
            for idx in lat_hits.tolist():
                if idx == s_idx:
                    continue
                if has_multi_levels and seg_levels[idx] != seg_levels[s_idx]:
                    continue
                cand = segments[idx]
                if cand.touches(seg) or cand.equals(seg):
                    continue
                d = cand.distance(p_orig)
                if 0.02 < d < min_lat:
                    proj = cand.interpolate(cand.project(p_orig))
                    lat_dx, lat_dy = proj.x - origin[0], proj.y - origin[1]
                    lat_mag = math.hypot(lat_dx, lat_dy)
                    if lat_mag <= 1e-6:
                        continue
                    lux, luy = lat_dx / lat_mag, lat_dy / lat_mag
                    abs_dot_lat = abs(lux * float(seg_u[idx, 0]) + luy * float(seg_u[idx, 1]))
                    if abs_dot_lat <= cos_lat_bound + 1e-9:
                        min_lat = d
                        best_lat = (proj.x + overshoot_eps * lux, proj.y + overshoot_eps * luy)
            if best_lat is not None:
                lateral_extensions.append(LineString([origin, best_lat]))

    # Step 3: Trim & Planarize + Micro-Stub Pruning
    if has_multi_levels:
        level_elems = defaultdict(list)
        for seg, lv in zip(segments, seg_levels):
            level_elems[lv].append(seg)
        base_lv = seg_levels[0] if seg_levels else 0.0
        level_elems[base_lv].extend(ray_extensions + lateral_extensions)
        planar_lines: List[LineString] = []
        for lv_elems in level_elems.values():
            noded_lv = shapely.node(shapely.multilinestrings(lv_elems))
            planar_lines.extend(
                p for p in shapely.get_parts(noded_lv)
                if p.geom_type == "LineString" and p.length > 0.001
            )
    else:
        all_elements = segments + ray_extensions + lateral_extensions
        noded = shapely.node(shapely.multilinestrings(all_elements))
        planar_lines = [
            p for p in shapely.get_parts(noded)
            if p.geom_type == "LineString" and p.length > 0.001
        ]

    deg_planar = defaultdict(int)
    for pl in planar_lines:
        c = list(pl.coords)
        deg_planar[_pt_key(c[0])] += 1
        deg_planar[_pt_key(c[-1])] += 1

    # Always trim the 5cm synthetic crossing overshoot tails (< 0.06m), plus any micro-stubs < min_stub_len
    stub_cutoff = max(float(min_stub_len), overshoot_eps + 0.015)
    noding_overshoots_count = len(ray_extensions) + len(lateral_extensions)
    pruned_lines: List[LineString] = []
    total_trimmed_tails = 0
    for pl in planar_lines:
        c = list(pl.coords)
        p0 = _pt_key(c[0])
        p1 = _pt_key(c[-1])
        is_dangle = (deg_planar[p0] == 1 or deg_planar[p1] == 1)
        if is_dangle and pl.length < stub_cutoff:
            total_trimmed_tails += 1
            continue
        pruned_lines.append(pl)

    pruned_stubs_count = max(0, total_trimmed_tails - noding_overshoots_count) if min_stub_len > 0.0 else 0

    # Step 4: Collinear Join with Turn Conservation
    final_lines = collinear_join_with_turn_conservation(
        pruned_lines,
        max_turn_deflection=max_turn_deflection
    )

    dead_end_stubs_removed_count = 0
    if dead_end_stub_len > 0.0:
        before_stub_count = len(final_lines)
        final_lines = remove_dead_end_stubs(
            final_lines,
            max_stub_length_m=float(dead_end_stub_len)
        )
        dead_end_stubs_removed_count = before_stub_count - len(final_lines)

    # Step 5: Post-Flight Topology Audit Assertions
    invalid_count = sum(1 for ln in final_lines if not ln.is_valid)
    raw_junctions = count_graph_junctions(clean_input_lines)
    cleaned_junctions = count_graph_junctions(final_lines)
    raw_components = count_connected_components(clean_input_lines)
    cleaned_components = count_connected_components(final_lines)
    final_dangles = count_dangling_endpoints(final_lines)
    raw_total_len = sum(ln.length for ln in clean_input_lines)
    cleaned_total_len = sum(ln.length for ln in final_lines)
    len_delta_pct = ((cleaned_total_len - raw_total_len) / raw_total_len * 100.0) if raw_total_len > 0.0 else 0.0

    # Check remaining micro-stubs in final_lines
    deg_final = defaultdict(int)
    for fl in final_lines:
        c = list(fl.coords)
        deg_final[_pt_key(c[0])] += 1
        deg_final[_pt_key(c[-1])] += 1
    remaining_stubs = sum(
        1 for fl in final_lines
        if min_stub_len > 0.0 and fl.length < min_stub_len
        and (deg_final[_pt_key(fl.coords[0])] == 1 or deg_final[_pt_key(fl.coords[-1])] == 1)
    )

    assertions_passed = True
    assertion_errors = []
    if invalid_count > 0:
        assertions_passed = False
        assertion_errors.append(f"Found {invalid_count} invalid geometries after cleaning")
    if dead_end_stubs_removed_count == 0 and cleaned_junctions < raw_junctions:
        assertions_passed = False
        assertion_errors.append(f"Junction destruction detected: {cleaned_junctions} < {raw_junctions}")
    if remaining_stubs > 0:
        assertions_passed = False
        assertion_errors.append(f"Found {remaining_stubs} remaining micro-stubs < {min_stub_len}m")
    if cleaned_components > raw_components:
        assertions_passed = False
        assertion_errors.append(f"Network fragmentation detected: {cleaned_components} > {raw_components}")
    if (dead_end_stubs_removed_count == 0 and len_delta_pct < -0.5) or len_delta_pct > 5.0:
        assertions_passed = False
        assertion_errors.append(f"Network length abnormal expansion: {len_delta_pct:+.2f}%")

    if run_assertions and not assertions_passed:
        raise AssertionError(" | ".join(assertion_errors))

    report = {
        "mode": mode_clean,
        "effective_extend_distance": round(effective_dist, 2),
        "raw_lines_count": len(clean_input_lines),
        "exploded_segments_count": len(segments),
        "raw_dangles_count": len(dangling_pts),
        "cleaned_dangles_count": final_dangles,
        "ray_extensions_count": len(ray_extensions),
        "lateral_connectors_count": len(lateral_extensions),
        "pruned_micro_stubs_count": pruned_stubs_count,
        "remaining_micro_stubs_count": remaining_stubs,
        "dead_end_stubs_removed_count": dead_end_stubs_removed_count,
        "cleaned_lines_count": len(final_lines),
        "raw_junctions_count": raw_junctions,
        "cleaned_junctions_count": cleaned_junctions,
        "raw_components_count": raw_components,
        "cleaned_components_count": cleaned_components,
        "raw_total_length_m": round(raw_total_len, 2),
        "cleaned_total_length_m": round(cleaned_total_len, 2),
        "length_delta_pct": round(len_delta_pct, 2),
        "assertions_passed": assertions_passed
    }

    return final_lines, report
