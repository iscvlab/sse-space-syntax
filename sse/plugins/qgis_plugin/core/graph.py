# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core: Compressed Sparse Row (CSR) Dual Graph
Implements Turner's (2007) Angular Segment Analysis dual graph representation
stored as contiguous flat arrays for cache locality and minimal RAM footprint.
"""

import math
from typing import List, Tuple, Dict, Any, Set
from collections import defaultdict
import numpy as np
from shapely.geometry import LineString


def get_bearing(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculates bearing angle in radians from p1 to p2."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.atan2(dx, dy)


def calculate_turn_deflection(b1: float, b2: float, tulip_bins: int = 0) -> float:
    """
    Calculates turn deflection angle between two directions.
    0.0 = straight through (0° deflection, zero turn cost)
    1.0 = standard right-angle turn (90° deflection, cost 1.0)
    2.0 = complete 180° reversal (cost 2.0)
    When tulip_bins > 0 (e.g. 1024 in UCL depthmapX), quantizes angular deflection
    into discrete Tulip bins before scaling to [0.0, 2.0].
    """
    diff = abs(b1 - b2) % (2.0 * math.pi)
    if diff > math.pi:
        diff = 2.0 * math.pi - diff
    # Deflection from continuing straight through in [0.0, pi]
    deflection_rad = abs(diff - math.pi)
    if tulip_bins and tulip_bins > 0:
        bin_idx = round((deflection_rad / math.pi) * (tulip_bins * 0.5))
        return float(bin_idx) / (tulip_bins * 0.25)
    # Convert from radians [0..pi] to Turner turn cost [0..2]
    return deflection_rad / (math.pi * 0.5)


def detect_grade_unlinks(
    segments: List[LineString],
    segment_levels: List[float]
) -> Set[Tuple[int, int]]:
    """
    Automatically synthesizes grade-separation unlink pairs (min_id, max_id) at shared
    vertices where segments of different elevation/grade levels cross (e.g. bridge level=1
    crossing surface street level=0), while preserving 2-segment ramp transitions.
    """
    unlinks: Set[Tuple[int, int]] = set()
    if not segments or segment_levels is None or len(segment_levels) != len(segments):
        return unlinks

    endpoint_map = defaultdict(list)
    for i, line in enumerate(segments):
        if line is None or line.is_empty:
            continue
        coords = list(line.coords)
        if len(coords) < 2:
            continue
        p0 = (round(coords[0][0], 2), round(coords[0][1], 2))
        p1 = (round(coords[-1][0], 2), round(coords[-1][1], 2))
        endpoint_map[p0].append((i, 0))
        endpoint_map[p1].append((i, 1))

    for members in endpoint_map.values():
        if len(members) < 2:
            continue
        level_counts = defaultdict(int)
        for idx, _ in members:
            level_counts[float(segment_levels[idx])] += 1
        if len(level_counts) < 2:
            continue
        # Grade-separated crossing occurs when both levels pass through the junction
        # (>= 2 incident segments on both levels), preserving 1-segment bridge/ramp portals.
        for a in range(len(members)):
            for b in range(a + 1, len(members)):
                i = members[a][0]
                j = members[b][0]
                if i == j:
                    continue
                lv_i = float(segment_levels[i])
                lv_j = float(segment_levels[j])
                if lv_i != lv_j and level_counts[lv_i] >= 2 and level_counts[lv_j] >= 2:
                    unlinks.add((min(i, j), max(i, j)))

    return unlinks


class CSRDualGraph:
    """
    Memory-efficient Compressed Sparse Row (CSR) representation of Turner's Angular Dual Graph.
    - Graph Nodes: Road segments (0 to N-1).
    - Graph Edges: Physical connections at shared junctions.
    - Edge Weight: Angular turn deflection cost + metric impedance.
    - Edge Metric: Travel distance in meters.
    """
    def __init__(self, num_nodes: int):
        self.num_nodes = num_nodes
        self.row_offsets = np.zeros(num_nodes + 1, dtype=np.uint32)
        self.target_nodes = np.empty(0, dtype=np.uint32)
        self.angular_weights = np.empty(0, dtype=np.float32)
        self.metric_lengths = np.empty(0, dtype=np.float32)
        self.segment_lengths = np.zeros(num_nodes, dtype=np.float32)
        self.segment_weights = np.ones(num_nodes, dtype=np.float64)
        self.connectivity = np.zeros(num_nodes, dtype=np.int32)
        self.angular_connectivity = np.zeros(num_nodes, dtype=np.float64)
        self.segment_coords = np.zeros((num_nodes, 4), dtype=np.float64)
        self.segment_azimuths = np.zeros(num_nodes, dtype=np.float64)
        self.tulip_bins = 0
        self.fwd_offsets = np.zeros(num_nodes + 1, dtype=np.int32)
        self.fwd_targets = np.empty(0, dtype=np.int32)
        self.fwd_dirs = np.empty(0, dtype=np.int8)
        self.fwd_weights = np.empty(0, dtype=np.float32)
        self.bwd_offsets = np.zeros(num_nodes + 1, dtype=np.int32)
        self.bwd_targets = np.empty(0, dtype=np.int32)
        self.bwd_dirs = np.empty(0, dtype=np.int8)
        self.bwd_weights = np.empty(0, dtype=np.float32)

    @classmethod
    def from_segments(
        cls,
        segments: List[LineString],
        unlinks: Set[Tuple[int, int]] = None,
        angular_weight_ratio: float = 0.0001,
        tulip_bins: int = 0,
        segment_weights: List[float] = None,
        segment_levels: List[float] = None
    ) -> Tuple['CSRDualGraph', List[Dict[str, Any]]]:
        """
        Builds the CSR Dual Graph from planarized 2-point line segments.
        Respects grade-separation unlinks so flyovers/tunnels do not connect.
        """
        N = len(segments)
        graph = cls(N)
        graph.tulip_bins = int(tulip_bins or 0)
        unlinks_set: Set[Tuple[int, int]] = set(unlinks) if unlinks else set()
        if segment_levels is not None and len(segment_levels) == N:
            unlinks_set |= detect_grade_unlinks(segments, segment_levels)

        endpoint_map = defaultdict(list)
        seg_records = []

        for i, line in enumerate(segments):
            coords = list(line.coords)
            p0 = (round(coords[0][0], 2), round(coords[0][1], 2))
            p1 = (round(coords[-1][0], 2), round(coords[-1][1], 2))

            b0 = get_bearing(coords[0], coords[1])
            b1 = get_bearing(coords[-1], coords[-2])
            length_m = float(line.length)

            graph.segment_lengths[i] = length_m
            graph.segment_coords[i] = [
                float(coords[0][0]),
                float(coords[0][1]),
                float(coords[-1][0]),
                float(coords[-1][1])
            ]
            graph.segment_azimuths[i] = (math.degrees(b0) + 360.0) % 360.0

            rec = {
                "id": i,
                "p0": p0,
                "p1": p1,
                "b0": b0,
                "b1": b1,
                "length_m": length_m,
                "geom": line
            }
            seg_records.append(rec)

            endpoint_map[p0].append((i, 0))
            endpoint_map[p1].append((i, 1))

        if segment_weights is not None and len(segment_weights) == N:
            graph.segment_weights = np.asarray(segment_weights, dtype=np.float64)
        else:
            graph.segment_weights = graph.segment_lengths.astype(np.float64)

        # Build adjacency dictionary and directed endpoint connections (salalib/segmtulip.cpp)
        adj = defaultdict(dict)
        fwd_conns: List[Dict[Tuple[int, int], float]] = [dict() for _ in range(N)]
        bwd_conns: List[Dict[Tuple[int, int], float]] = [dict() for _ in range(N)]

        for pt, members in endpoint_map.items():
            if len(members) < 2:
                continue
            for a in range(len(members)):
                for b in range(a + 1, len(members)):
                    i, end_i = members[a]
                    j, end_j = members[b]
                    if i == j:
                        continue

                    # Check grade-separation unlinks
                    pair = (min(i, j), max(i, j))
                    if pair in unlinks_set:
                        continue

                    b_i = seg_records[i]["b0"] if end_i == 0 else seg_records[i]["b1"]
                    b_j = seg_records[j]["b0"] if end_j == 0 else seg_records[j]["b1"]

                    ang_cost = calculate_turn_deflection(b_i, b_j, tulip_bins=tulip_bins)
                    avg_len = (seg_records[i]["length_m"] + seg_records[j]["length_m"]) * 0.5

                    # Cost function: Angular deflection + small distance impedance
                    w = ang_cost + angular_weight_ratio * avg_len
                    w = max(w, 1e-4)

                    if j not in adj[i] or w < adj[i][j][0]:
                        adj[i][j] = (w, avg_len, ang_cost)
                    if i not in adj[j] or w < adj[j][i][0]:
                        adj[j][i] = (w, avg_len, ang_cost)

                    # Directed endpoint connections matching depthmapX makeSegmentConnections
                    c_i = graph.segment_coords[i]
                    c_j = graph.segment_coords[j]
                    li = max(seg_records[i]["length_m"], 1e-12)
                    lj = max(seg_records[j]["length_m"], 1e-12)
                    if end_i == 1:
                        dvi_x, dvi_y = (c_i[2] - c_i[0]) / li, (c_i[3] - c_i[1]) / li
                    else:
                        dvi_x, dvi_y = (c_i[0] - c_i[2]) / li, (c_i[1] - c_i[3]) / li
                    if end_j == 0:
                        dvj_x, dvj_y = (c_j[2] - c_j[0]) / lj, (c_j[3] - c_j[1]) / lj
                        dir_to_j = 1
                    else:
                        dvj_x, dvj_y = (c_j[0] - c_j[2]) / lj, (c_j[1] - c_j[3]) / lj
                        dir_to_j = -1
                    dir_to_i = 1 if end_i == 0 else -1
                    dot_ij = max(-1.0, min(1.0, dvi_x * dvj_x + dvi_y * dvj_y))
                    turn_w = float(np.float32(math.acos(dot_ij) / (math.pi * 0.5)))
                    if end_i == 1:
                        fwd_conns[i][(j, dir_to_j)] = turn_w
                    else:
                        bwd_conns[i][(j, dir_to_j)] = turn_w
                    if end_j == 1:
                        fwd_conns[j][(i, dir_to_i)] = turn_w
                    else:
                        bwd_conns[j][(i, dir_to_i)] = turn_w

        # Flatten adjacency into CSR arrays
        total_edges = sum(len(neighbors) for neighbors in adj.values())
        graph.target_nodes = np.empty(total_edges, dtype=np.uint32)
        graph.angular_weights = np.empty(total_edges, dtype=np.float32)
        graph.metric_lengths = np.empty(total_edges, dtype=np.float32)

        curr_offset = 0
        for u in range(N):
            graph.row_offsets[u] = curr_offset
            neighbors = adj.get(u, {})
            graph.connectivity[u] = len(neighbors)
            ang_conn_sum = 0.0
            for v, (weight, metric_m, ang_only) in neighbors.items():
                graph.target_nodes[curr_offset] = v
                graph.angular_weights[curr_offset] = weight
                graph.metric_lengths[curr_offset] = metric_m
                ang_conn_sum += float(ang_only)
                curr_offset += 1
            graph.angular_connectivity[u] = ang_conn_sum
        graph.row_offsets[N] = curr_offset

        total_fwd = sum(len(fc) for fc in fwd_conns)
        graph.fwd_targets = np.empty(total_fwd, dtype=np.int32)
        graph.fwd_dirs = np.empty(total_fwd, dtype=np.int8)
        graph.fwd_weights = np.empty(total_fwd, dtype=np.float32)
        f_off = 0
        for u in range(N):
            graph.fwd_offsets[u] = f_off
            for (t_seg, t_dir), tw in sorted(fwd_conns[u].items()):
                graph.fwd_targets[f_off] = t_seg
                graph.fwd_dirs[f_off] = t_dir
                graph.fwd_weights[f_off] = tw
                f_off += 1
        graph.fwd_offsets[N] = f_off

        total_bwd = sum(len(bc) for bc in bwd_conns)
        graph.bwd_targets = np.empty(total_bwd, dtype=np.int32)
        graph.bwd_dirs = np.empty(total_bwd, dtype=np.int8)
        graph.bwd_weights = np.empty(total_bwd, dtype=np.float32)
        b_off = 0
        for u in range(N):
            graph.bwd_offsets[u] = b_off
            for (t_seg, t_dir), tw in sorted(bwd_conns[u].items()):
                graph.bwd_targets[b_off] = t_seg
                graph.bwd_dirs[b_off] = t_dir
                graph.bwd_weights[b_off] = tw
                b_off += 1
        graph.bwd_offsets[N] = b_off

        return graph, seg_records

    def get_neighbors(self, u: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns (target_nodes, angular_weights, metric_lengths) for node u."""
        start = self.row_offsets[u]
        end = self.row_offsets[u + 1]
        return (
            self.target_nodes[start:end],
            self.angular_weights[start:end],
            self.metric_lengths[start:end]
        )

    def memory_footprint_bytes(self) -> int:
        """Returns exact in-memory byte size of the CSR graph arrays."""
        return (
            self.row_offsets.nbytes
            + self.target_nodes.nbytes
            + self.angular_weights.nbytes
            + self.metric_lengths.nbytes
            + self.segment_lengths.nbytes
        )
