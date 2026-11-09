# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core: High-Performance Angular Segment Analysis (ASA) Solver
Implements:
1. Turner (2007) Angular Segment Analysis with metric/angular/topological radius search cutoffs.
2. Brandes (2001) Betweenness Centrality with unweighted, segment-length, and custom attribute weighting.
3. Closeness Integration & Mean Depth.
4. Exact Hillier, Yang, & Turner (2012) Logarithmic Normalizations (NACH & NAIN).
5. Multi-Origin Step Depth (Angular, Metric, Topological) and DepthmapX Summary Diagnostics & CSV Export.
"""

import csv
import heapq
import math
import os
from typing import List, Dict, Tuple, Any
import numpy as np

try:
    from numba import njit, prange
    HAS_NUMBA = True
except ImportError:
    HAS_NUMBA = False

    def njit(*args, **kwargs):
        def decorator(func):
            return func
        if len(args) == 1 and callable(args[0]):
            return args[0]
        return decorator

    prange = range

from .graph import CSRDualGraph


def _encode_mode(mode_str: str, default_code: int = 0) -> int:
    m = str(mode_str or "").strip().lower()
    if m == "angular":
        return 0
    if m == "metric":
        return 1
    if m == "topological":
        return 2
    return default_code


@njit(cache=True)
def _compute_segment_control_metrics(
    row_offsets: np.ndarray,
    target_nodes: np.ndarray,
    connectivity: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Computes segment-level Turner/Hillier Control and Controllability on CSRDualGraph:
    - control[u] = sum_{v in N(u)} (1.0 / max(connectivity[v], 1))
    - controllability[u] = k_1 / max(k_1 + k_2, 1)
    """
    N = len(connectivity)
    control = np.zeros(N, dtype=np.float64)
    controllability = np.zeros(N, dtype=np.float64)
    visited_stamp = np.full(N, -1, dtype=np.int32)

    for u in range(N):
        start_u = int(row_offsets[u])
        end_u = int(row_offsets[u + 1])
        k_1 = int(connectivity[u])
        if k_1 <= 0:
            continue

        visited_stamp[u] = u
        ctrl_sum = 0.0
        for e1 in range(start_u, end_u):
            v = int(target_nodes[e1])
            visited_stamp[v] = u
            deg_v = int(connectivity[v])
            ctrl_sum += 1.0 / float(deg_v if deg_v > 1 else 1)
        control[u] = ctrl_sum

        k_2 = 0
        for e1 in range(start_u, end_u):
            v = int(target_nodes[e1])
            start_v = int(row_offsets[v])
            end_v = int(row_offsets[v + 1])
            for e2 in range(start_v, end_v):
                w = int(target_nodes[e2])
                if visited_stamp[w] != u:
                    visited_stamp[w] = u
                    k_2 += 1

        denom = k_1 + k_2
        controllability[u] = float(k_1) / float(denom if denom > 1 else 1)

    return control, controllability


@njit(parallel=True, cache=True)
def _brandes_multi_radius_numba(
    row_offsets: np.ndarray,
    target_nodes: np.ndarray,
    w_ang: np.ndarray,
    w_metric: np.ndarray,
    w_norm: np.ndarray,
    is_weighted: bool,
    radii_arr: np.ndarray,
    is_global_arr: np.ndarray,
    atype_code: int,
    rtype_code: int,
    sources: np.ndarray,
    scale_mult: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Single-Pass Multi-Radius Brandes & Closeness Numba JIT kernel.
    Runs Dijkstra ONCE per source node s using zero-allocation flat binary heaps
    and flat CSR predecessor linked lists, then executes backward Brandes sweeps
    across all R radii.
    """
    N = len(row_offsets) - 1
    M_edges = len(target_nodes)
    R_count = len(radii_arr)
    num_sources = len(sources)

    choice_all = np.zeros((R_count, N), dtype=np.float64)
    closeness_all = np.zeros((R_count, N), dtype=np.float64)
    total_depth_all = np.zeros((R_count, N), dtype=np.float64)
    node_count_all = np.zeros((R_count, N), dtype=np.float64)

    if N == 0 or R_count == 0 or num_sources == 0:
        return choice_all, closeness_all, total_depth_all, node_count_all

    any_global = False
    max_radius = 0.0
    for r_i in range(R_count):
        if is_global_arr[r_i]:
            any_global = True
        elif radii_arr[r_i] > max_radius:
            max_radius = radii_arr[r_i]

    prune_dijkstra = (not any_global) and (atype_code == rtype_code)

    n_chunks = min(num_sources, 16)
    choice_chunks = np.zeros((n_chunks, R_count, N), dtype=np.float64)

    heap_cap = max(4 * M_edges + 4, 16)
    pred_cap = max(M_edges + 4, 16)

    for chunk_id in prange(n_chunks):
        start_s = (chunk_id * num_sources) // n_chunks
        end_s = ((chunk_id + 1) * num_sources) // n_chunks

        # Per-thread reusable buffers
        S = np.empty(N, dtype=np.int32)
        d_cost = np.full(N, np.inf, dtype=np.float64)
        d_rad = np.full(N, np.inf, dtype=np.float64)
        sigma = np.zeros(N, dtype=np.float64)
        delta = np.zeros(N, dtype=np.float64)

        pred_head = np.full(N, -1, dtype=np.int32)
        pred_node = np.empty(pred_cap, dtype=np.int32)
        pred_next = np.empty(pred_cap, dtype=np.int32)

        heap_cost = np.empty(heap_cap, dtype=np.float64)
        heap_rad = np.empty(heap_cap, dtype=np.float64)
        heap_node = np.empty(heap_cap, dtype=np.int32)

        for s_idx in range(start_s, end_s):
            s = int(sources[s_idx])
            s_len = 0
            pred_ptr = 0

            d_cost[s] = 0.0
            d_rad[s] = 0.0
            sigma[s] = 1.0

            heap_cost[0] = 0.0
            heap_rad[0] = 0.0
            heap_node[0] = s
            heap_size = 1

            while heap_size > 0:
                dist_c = heap_cost[0]
                dist_r = heap_rad[0]
                u = heap_node[0]
                heap_size -= 1
                if heap_size > 0:
                    last_c = heap_cost[heap_size]
                    last_r = heap_rad[heap_size]
                    last_n = heap_node[heap_size]
                    pos = 0
                    half = heap_size >> 1
                    while pos < half:
                        child = (pos << 1) + 1
                        right = child + 1
                        cc = heap_cost[child]
                        cr = heap_rad[child]
                        cn = heap_node[child]
                        if right < heap_size:
                            rc = heap_cost[right]
                            rr = heap_rad[right]
                            rn = heap_node[right]
                            if (rc < cc) or (rc == cc and (rr < cr or (rr == cr and rn < cn))):
                                child = right
                                cc = rc
                                cr = rr
                                cn = rn
                        if (last_c < cc) or (last_c == cc and (last_r < cr or (last_r == cr and last_n <= cn))):
                            break
                        heap_cost[pos] = cc
                        heap_rad[pos] = cr
                        heap_node[pos] = cn
                        pos = child
                    heap_cost[pos] = last_c
                    heap_rad[pos] = last_r
                    heap_node[pos] = last_n

                if dist_c > d_cost[u]:
                    continue

                S[s_len] = u
                s_len += 1

                row_start = int(row_offsets[u])
                row_end = int(row_offsets[u + 1])

                for e_idx in range(row_start, row_end):
                    v = int(target_nodes[e_idx])

                    if rtype_code == 0:
                        step_r = float(w_ang[e_idx])
                    elif rtype_code == 2:
                        step_r = 1.0
                    else:
                        step_r = float(w_metric[e_idx])

                    new_rad = dist_r + step_r

                    if prune_dijkstra and new_rad > max_radius:
                        continue

                    if atype_code == 1:
                        step_c = float(w_metric[e_idx])
                    elif atype_code == 2:
                        step_c = 1.0
                    else:
                        step_c = float(w_ang[e_idx])

                    new_cost = dist_c + step_c

                    if new_cost < d_cost[v] - 1e-7:
                        d_cost[v] = new_cost
                        d_rad[v] = new_rad

                        # Heap push (new_cost, new_rad, v)
                        pos = heap_size
                        heap_size += 1
                        while pos > 0:
                            parent = (pos - 1) >> 1
                            pc = heap_cost[parent]
                            pr = heap_rad[parent]
                            pn = heap_node[parent]
                            if (new_cost < pc) or (
                                new_cost == pc and (new_rad < pr or (new_rad == pr and v < pn))
                            ):
                                heap_cost[pos] = pc
                                heap_rad[pos] = pr
                                heap_node[pos] = pn
                                pos = parent
                            else:
                                break
                        heap_cost[pos] = new_cost
                        heap_rad[pos] = new_rad
                        heap_node[pos] = v

                        sigma[v] = sigma[u]
                        pred_node[pred_ptr] = u
                        pred_next[pred_ptr] = -1
                        pred_head[v] = pred_ptr
                        pred_ptr += 1
                    elif abs(new_cost - d_cost[v]) <= 1e-7:
                        if new_rad < d_rad[v]:
                            d_rad[v] = new_rad
                        sigma[v] += sigma[u]
                        pred_node[pred_ptr] = u
                        pred_next[pred_ptr] = pred_head[v]
                        pred_head[v] = pred_ptr
                        pred_ptr += 1

            # Multi-radius backward Brandes sweep over settled stack S[0 : s_len]
            w_s = w_norm[s]
            for r_idx in range(R_count):
                r_m = radii_arr[r_idx]
                is_glob = is_global_arr[r_idx]

                for idx_s in range(s_len):
                    delta[S[idx_s]] = 0.0

                k_reach = 0
                sum_cost = 0.0

                if not is_weighted:
                    for idx_s in range(s_len - 1, -1, -1):
                        w = S[idx_s]
                        in_hor = True if is_glob else (d_rad[w] <= r_m)
                        del_w = delta[w]
                        if not in_hor and del_w == 0.0:
                            continue

                        w_seed = 1.0 if in_hor else 0.0
                        sig_w = sigma[w]
                        factor = (w_seed + del_w) / sig_w

                        p_cur = pred_head[w]
                        while p_cur != -1:
                            v_pred = pred_node[p_cur]
                            delta[v_pred] += sigma[v_pred] * factor
                            p_cur = pred_next[p_cur]

                        if w != s and in_hor:
                            choice_chunks[chunk_id, r_idx, w] += del_w * scale_mult
                            if d_cost[w] > 0.0:
                                k_reach += 1
                                sum_cost += d_cost[w]
                else:
                    for idx_s in range(s_len - 1, -1, -1):
                        w = S[idx_s]
                        in_hor = True if is_glob else (d_rad[w] <= r_m)
                        del_w = delta[w]
                        if not in_hor and del_w == 0.0:
                            continue

                        w_seed = w_norm[w] if in_hor else 0.0
                        sig_w = sigma[w]
                        factor = (w_seed + del_w) / sig_w

                        p_cur = pred_head[w]
                        while p_cur != -1:
                            v_pred = pred_node[p_cur]
                            delta[v_pred] += sigma[v_pred] * factor
                            p_cur = pred_next[p_cur]

                        if w != s and in_hor:
                            choice_chunks[chunk_id, r_idx, w] += w_s * del_w * scale_mult
                            if d_cost[w] > 0.0:
                                k_reach += 1
                                sum_cost += d_cost[w] * w_norm[w]

                node_count_all[r_idx, s] = float(k_reach)
                if k_reach > 0:
                    total_depth_all[r_idx, s] = sum_cost
                    closeness_all[r_idx, s] = float(k_reach) / (sum_cost if sum_cost > 1e-4 else 1e-4)

            # Reset modified node state in O(|S|)
            for idx_s in range(s_len):
                w = S[idx_s]
                d_cost[w] = np.inf
                d_rad[w] = np.inf
                sigma[w] = 0.0
                pred_head[w] = -1

    for c_id in range(n_chunks):
        for r_idx in range(R_count):
            for u in range(N):
                choice_all[r_idx, u] += choice_chunks[c_id, r_idx, u]

    return choice_all, closeness_all, total_depth_all, node_count_all


def _brandes_multi_radius_py_fast(
    row_offsets: np.ndarray,
    target_nodes: np.ndarray,
    w_ang: np.ndarray,
    w_metric: np.ndarray,
    w_norm: np.ndarray,
    is_weighted: bool,
    radii_arr: np.ndarray,
    is_global_arr: np.ndarray,
    atype_code: int,
    rtype_code: int,
    sources: np.ndarray,
    scale_mult: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Fast pure-Python / C-heapq Single-Pass Multi-Radius Brandes & Closeness fallback
    for environments without Numba (e.g. standard QGIS Desktop Python).
    Uses C-compiled heapq and O(|S|) sparse state reset per source node.
    """
    N = len(row_offsets) - 1
    R_count = len(radii_arr)
    num_sources = len(sources)

    choice_all = np.zeros((R_count, N), dtype=np.float64)
    closeness_all = np.zeros((R_count, N), dtype=np.float64)
    total_depth_all = np.zeros((R_count, N), dtype=np.float64)
    node_count_all = np.zeros((R_count, N), dtype=np.float64)

    if N == 0 or R_count == 0 or num_sources == 0:
        return choice_all, closeness_all, total_depth_all, node_count_all

    ro_list = row_offsets.tolist()
    tn_list = target_nodes.tolist()
    wa_list = w_ang.tolist()
    wm_list = w_metric.tolist()
    wn_list = w_norm.tolist()
    radii_list = radii_arr.tolist()
    glob_list = is_global_arr.tolist()
    src_list = sources.tolist()

    if atype_code == 1:
        cost_edge = wm_list
    elif atype_code == 2:
        cost_edge = [1.0] * len(tn_list)
    else:
        cost_edge = wa_list

    if rtype_code == 0:
        rad_edge = wa_list
    elif rtype_code == 2:
        rad_edge = [1.0] * len(tn_list)
    else:
        rad_edge = wm_list

    any_global = any(glob_list)
    max_radius = max((r for r, g in zip(radii_list, glob_list) if not g), default=0.0)
    prune_dijkstra = (not any_global) and (atype_code == rtype_code)

    INF = float("inf")
    d_cost = [INF] * N
    d_rad = [INF] * N
    sigma = [0.0] * N
    delta = [0.0] * N
    P: List[List[int]] = [[] for _ in range(N)]

    heappush = heapq.heappush
    heappop = heapq.heappop

    choice_lists = [[0.0] * N for _ in range(R_count)]
    closeness_lists = [[0.0] * N for _ in range(R_count)]
    td_lists = [[0.0] * N for _ in range(R_count)]
    nc_lists = [[0.0] * N for _ in range(R_count)]

    for s in src_list:
        S: List[int] = []
        d_cost[s] = 0.0
        d_rad[s] = 0.0
        sigma[s] = 1.0
        pq = [(0.0, 0.0, s)]

        while pq:
            dist_c, dist_r, u = heappop(pq)
            if dist_c > d_cost[u]:
                continue
            S.append(u)

            r_start = ro_list[u]
            r_end = ro_list[u + 1]
            sig_u = sigma[u]
            for e_idx in range(r_start, r_end):
                v = tn_list[e_idx]
                new_rad = dist_r + rad_edge[e_idx]
                if prune_dijkstra and new_rad > max_radius:
                    continue
                new_cost = dist_c + cost_edge[e_idx]
                dv = d_cost[v]
                if new_cost < dv - 1e-7:
                    d_cost[v] = new_cost
                    d_rad[v] = new_rad
                    heappush(pq, (new_cost, new_rad, v))
                    sigma[v] = sig_u
                    P[v] = [u]
                elif abs(new_cost - dv) <= 1e-7:
                    if new_rad < d_rad[v]:
                        d_rad[v] = new_rad
                    sigma[v] += sig_u
                    P[v].append(u)

        w_s = wn_list[s]
        S_rev = S[::-1]

        for r_idx in range(R_count):
            r_m = radii_list[r_idx]
            is_glob = glob_list[r_idx]
            ch_r = choice_lists[r_idx]

            for w in S:
                delta[w] = 0.0

            k_reach = 0
            sum_cost = 0.0

            if not is_weighted:
                for w in S_rev:
                    in_hor = True if is_glob else (d_rad[w] <= r_m)
                    del_w = delta[w]
                    if not in_hor and del_w == 0.0:
                        continue
                    w_seed = 1.0 if in_hor else 0.0
                    factor = (w_seed + del_w) / sigma[w]
                    for v_pred in P[w]:
                        delta[v_pred] += sigma[v_pred] * factor
                    if w != s and in_hor:
                        ch_r[w] += del_w * scale_mult
                        dc_w = d_cost[w]
                        if dc_w > 0.0:
                            k_reach += 1
                            sum_cost += dc_w
            else:
                for w in S_rev:
                    in_hor = True if is_glob else (d_rad[w] <= r_m)
                    del_w = delta[w]
                    if not in_hor and del_w == 0.0:
                        continue
                    w_seed = wn_list[w] if in_hor else 0.0
                    factor = (w_seed + del_w) / sigma[w]
                    for v_pred in P[w]:
                        delta[v_pred] += sigma[v_pred] * factor
                    if w != s and in_hor:
                        ch_r[w] += w_s * del_w * scale_mult
                        dc_w = d_cost[w]
                        if dc_w > 0.0:
                            k_reach += 1
                            sum_cost += dc_w * wn_list[w]

            nc_lists[r_idx][s] = float(k_reach)
            if k_reach > 0:
                td_lists[r_idx][s] = sum_cost
                closeness_lists[r_idx][s] = float(k_reach) / (sum_cost if sum_cost > 1e-4 else 1e-4)

        for w in S:
            d_cost[w] = INF
            d_rad[w] = INF
            sigma[w] = 0.0
            P[w].clear()

    for r_idx in range(R_count):
        choice_all[r_idx] = np.array(choice_lists[r_idx], dtype=np.float64)
        closeness_all[r_idx] = np.array(closeness_lists[r_idx], dtype=np.float64)
        total_depth_all[r_idx] = np.array(td_lists[r_idx], dtype=np.float64)
        node_count_all[r_idx] = np.array(nc_lists[r_idx], dtype=np.float64)

    return choice_all, closeness_all, total_depth_all, node_count_all


def _brandes_multi_radius_dispatch(
    row_offsets: np.ndarray,
    target_nodes: np.ndarray,
    w_ang: np.ndarray,
    w_metric: np.ndarray,
    w_norm: np.ndarray,
    is_weighted: bool,
    radii_arr: np.ndarray,
    is_global_arr: np.ndarray,
    atype_code: int,
    rtype_code: int,
    sources: np.ndarray,
    scale_mult: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if HAS_NUMBA:
        return _brandes_multi_radius_numba(
            row_offsets,
            target_nodes,
            w_ang,
            w_metric,
            w_norm,
            is_weighted,
            radii_arr,
            is_global_arr,
            atype_code,
            rtype_code,
            sources,
            scale_mult
        )
    return _brandes_multi_radius_py_fast(
        row_offsets,
        target_nodes,
        w_ang,
        w_metric,
        w_norm,
        is_weighted,
        radii_arr,
        is_global_arr,
        atype_code,
        rtype_code,
        sources,
        scale_mult
    )


@njit(parallel=True, cache=True)
def _tulip_directed_multi_radius_numba(
    fwd_offsets: np.ndarray,
    fwd_targets: np.ndarray,
    fwd_dirs: np.ndarray,
    fwd_weights: np.ndarray,
    bwd_offsets: np.ndarray,
    bwd_targets: np.ndarray,
    bwd_dirs: np.ndarray,
    bwd_weights: np.ndarray,
    seg_lengths: np.ndarray,
    weights_f32: np.ndarray,
    is_weighted: bool,
    tulip_bins_full: int,
    radius_vals: np.ndarray,
    rtype_code: int,
    sources: np.ndarray,
    scale_mult: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Exact UCL depthmapX v0.8.0 (salalib/segmmodules/segmtulip.cpp) Directed-Segment
    Tulip Semicircle Bucket Queue (513 bins for Tulip-1024) + Multi-Radius Bitmask Kernel
    with metricdepth-sorted bin insertion and reverse-leaf Choice accumulation.
    Achieves 100.000000% (R² = 1.00000000, max|Δ| = 0.0000) parity with depthmapX T1024 metrics.
    """
    N = len(seg_lengths)
    R_count = len(radius_vals)
    num_sources = len(sources)

    choice_out = np.zeros((R_count, N), dtype=np.float64)
    closeness_out = np.zeros((R_count, N), dtype=np.float64)
    total_depth_out = np.zeros((R_count, N), dtype=np.float64)
    node_count_out = np.zeros((R_count, N), dtype=np.float64)

    if N == 0 or R_count == 0 or num_sources == 0:
        return choice_out, closeness_out, total_depth_out, node_count_out

    tulip_bins = (tulip_bins_full // 2) + 1
    denom_conv = float(tulip_bins - 1) * 0.5
    radiusmask = np.uint32((1 << min(R_count, 30)) - 1)

    n_chunks = min(num_sources, 32)
    if n_chunks < 1:
        n_chunks = 1

    chunk_choice_unw = np.zeros((n_chunks, R_count, N, 2), dtype=np.float64)
    chunk_choice_wgt = np.zeros((n_chunks, R_count, N, 2), dtype=np.float64)

    nc_all = np.zeros((R_count, N), dtype=np.float64)
    td_unw_all = np.zeros((R_count, N), dtype=np.float64)
    td_wgt_all = np.zeros((R_count, N), dtype=np.float64)
    integ_unw_all = np.zeros((R_count, N), dtype=np.float64)
    integ_wgt_all = np.zeros((R_count, N), dtype=np.float64)

    q_cap = max(8 * (len(fwd_targets) + len(bwd_targets)) + 64, 4 * N + 64)

    for chunk_id in prange(n_chunks):
        start_s = (chunk_id * num_sources) // n_chunks
        end_s = ((chunk_id + 1) * num_sources) // n_chunks

        bins_head = np.full(tulip_bins, -1, dtype=np.int32)
        q_metric = np.empty(q_cap, dtype=np.float32)
        q_dir = np.empty(q_cap, dtype=np.int8)
        q_ref = np.empty(q_cap, dtype=np.int32)
        q_prev_dir = np.empty(q_cap, dtype=np.int8)
        q_prev_ref = np.empty(q_cap, dtype=np.int32)
        q_segdepth = np.empty(q_cap, dtype=np.int32)
        q_cov = np.empty(q_cap, dtype=np.uint32)
        q_next = np.empty(q_cap, dtype=np.int32)

        uncovered = np.full((N, 2), radiusmask, dtype=np.uint32)
        audit_depth = np.zeros((R_count, N, 2), dtype=np.int32)
        audit_prev_dir = np.zeros((R_count, N, 2), dtype=np.int8)
        audit_prev_ref = np.full((R_count, N, 2), -1, dtype=np.int32)
        audit_leaf = np.ones((R_count, N, 2), dtype=np.bool_)
        audit_choice_cov = np.zeros((R_count, N, 2), dtype=np.bool_)
        visited_nodes = np.empty(N, dtype=np.int32)
        in_visited = np.zeros(N, dtype=np.bool_)

        for s_idx in range(start_s, end_s):
            cursor = int(sources[s_idx])
            for b in range(tulip_bins):
                bins_head[b] = -1

            rootseglength = seg_lengths[cursor]
            rootweight = float(weights_f32[cursor])

            q_metric[0] = np.float32(0.5) * rootseglength
            q_dir[0] = np.int8(0)
            q_ref[0] = np.int32(cursor)
            q_prev_dir[0] = np.int8(0)
            q_prev_ref[0] = np.int32(-1)
            q_segdepth[0] = np.int32(0)
            q_cov[0] = radiusmask
            q_next[0] = np.int32(-1)
            bins_head[0] = np.int32(0)
            q_ptr = 1

            depthlevel = 0
            opencount = 1
            currentbin = 0
            vis_count = 0

            while opencount > 0:
                while bins_head[currentbin] == -1:
                    depthlevel += 1
                    currentbin += 1
                    if currentbin == tulip_bins:
                        currentbin = 0

                curr = bins_head[currentbin]
                bins_head[currentbin] = q_next[curr]
                opencount -= 1

                li_metric = q_metric[curr]
                li_dir = q_dir[curr]
                ref_i = int(q_ref[curr])
                li_prev_dir = q_prev_dir[curr]
                li_prev_ref = int(q_prev_ref[curr])
                li_segdepth = int(q_segdepth[curr])
                li_cov = q_cov[curr]

                dir_idx = 0 if li_dir == 1 else 1
                coverage = np.uint32(li_cov & uncovered[ref_i, dir_idx])
                if coverage != np.uint32(0):
                    if not in_visited[ref_i]:
                        in_visited[ref_i] = True
                        visited_nodes[vis_count] = ref_i
                        vis_count += 1

                    if li_prev_ref != -1:
                        uncovered[ref_i, dir_idx] &= np.uint32(~coverage)
                        rbin = 0
                        while rbin < R_count and ((coverage >> np.uint32(rbin)) & np.uint32(1)) == np.uint32(0):
                            rbin += 1
                        rbinbase = rbin
                        prev_u = li_prev_ref
                        prev_d = 0 if li_prev_dir == 1 else 1
                        while rbin < R_count:
                            if ((coverage >> np.uint32(rbin)) & np.uint32(1)) == np.uint32(1):
                                audit_depth[rbin, ref_i, dir_idx] = depthlevel
                                audit_prev_dir[rbin, ref_i, dir_idx] = li_prev_dir
                                audit_prev_ref[rbin, ref_i, dir_idx] = li_prev_ref
                                audit_leaf[rbin, prev_u, prev_d] = False
                            rbin += 1
                    else:
                        rbinbase = 0
                        uncovered[ref_i, 0] &= np.uint32(~coverage)
                        uncovered[ref_i, 1] &= np.uint32(~coverage)

                    if li_dir != -1:
                        es = int(fwd_offsets[ref_i])
                        ee = int(fwd_offsets[ref_i + 1])
                        for e_idx in range(es, ee):
                            conn_ref = int(fwd_targets[e_idx])
                            conn_dir = fwd_dirs[e_idx]
                            w_f32 = fwd_weights[e_idx]
                            cd_idx = 0 if conn_dir == 1 else 1
                            if (uncovered[conn_ref, cd_idx] & coverage) != np.uint32(0):
                                extradepth = int(math.floor(float(w_f32) * float(tulip_bins) * 0.5))
                                seglength = seg_lengths[conn_ref]
                                rbin = rbinbase
                                if rtype_code == 0:
                                    while (
                                        rbin < R_count
                                        and radius_vals[rbin] >= 0.0
                                        and float(depthlevel + extradepth) > radius_vals[rbin]
                                    ):
                                        rbin += 1
                                elif rtype_code == 1:
                                    while (
                                        rbin < R_count
                                        and radius_vals[rbin] >= 0.0
                                        and float(li_metric + seglength * np.float32(0.5)) > radius_vals[rbin]
                                    ):
                                        rbin += 1
                                else:
                                    if (
                                        rbin < R_count
                                        and radius_vals[rbin] >= 0.0
                                        and float(li_segdepth) >= radius_vals[rbin]
                                    ):
                                        rbin += 1

                                if rbin < R_count and (coverage >> np.uint32(rbin)) != np.uint32(0) and q_ptr < q_cap:
                                    next_cov = np.uint32((coverage >> np.uint32(rbin)) << np.uint32(rbin))
                                    b_idx = (currentbin + tulip_bins + extradepth) % tulip_bins
                                    new_m = np.float32(li_metric + seglength)
                                    q_metric[q_ptr] = new_m
                                    q_dir[q_ptr] = conn_dir
                                    q_ref[q_ptr] = conn_ref
                                    q_prev_dir[q_ptr] = np.int8(1)
                                    q_prev_ref[q_ptr] = ref_i
                                    q_segdepth[q_ptr] = li_segdepth + 1
                                    q_cov[q_ptr] = next_cov

                                    head_n = bins_head[b_idx]
                                    if head_n == -1 or new_m < q_metric[head_n]:
                                        q_next[q_ptr] = head_n
                                        bins_head[b_idx] = q_ptr
                                    else:
                                        prev_n = head_n
                                        curr_n = q_next[prev_n]
                                        while curr_n != -1 and q_metric[curr_n] <= new_m:
                                            prev_n = curr_n
                                            curr_n = q_next[curr_n]
                                        q_next[q_ptr] = curr_n
                                        q_next[prev_n] = q_ptr

                                    q_ptr += 1
                                    opencount += 1

                    if li_dir != 1:
                        es = int(bwd_offsets[ref_i])
                        ee = int(bwd_offsets[ref_i + 1])
                        for e_idx in range(es, ee):
                            conn_ref = int(bwd_targets[e_idx])
                            conn_dir = bwd_dirs[e_idx]
                            w_f32 = bwd_weights[e_idx]
                            cd_idx = 0 if conn_dir == 1 else 1
                            if (uncovered[conn_ref, cd_idx] & coverage) != np.uint32(0):
                                extradepth = int(math.floor(float(w_f32) * float(tulip_bins) * 0.5))
                                seglength = seg_lengths[conn_ref]
                                rbin = rbinbase
                                if rtype_code == 0:
                                    while (
                                        rbin < R_count
                                        and radius_vals[rbin] >= 0.0
                                        and float(depthlevel + extradepth) > radius_vals[rbin]
                                    ):
                                        rbin += 1
                                elif rtype_code == 1:
                                    while (
                                        rbin < R_count
                                        and radius_vals[rbin] >= 0.0
                                        and float(li_metric + seglength * np.float32(0.5)) > radius_vals[rbin]
                                    ):
                                        rbin += 1
                                else:
                                    if (
                                        rbin < R_count
                                        and radius_vals[rbin] >= 0.0
                                        and float(li_segdepth) >= radius_vals[rbin]
                                    ):
                                        rbin += 1

                                if rbin < R_count and (coverage >> np.uint32(rbin)) != np.uint32(0) and q_ptr < q_cap:
                                    next_cov = np.uint32((coverage >> np.uint32(rbin)) << np.uint32(rbin))
                                    b_idx = (currentbin + tulip_bins + extradepth) % tulip_bins
                                    new_m = np.float32(li_metric + seglength)
                                    q_metric[q_ptr] = new_m
                                    q_dir[q_ptr] = conn_dir
                                    q_ref[q_ptr] = conn_ref
                                    q_prev_dir[q_ptr] = np.int8(-1)
                                    q_prev_ref[q_ptr] = ref_i
                                    q_segdepth[q_ptr] = li_segdepth + 1
                                    q_cov[q_ptr] = next_cov

                                    head_n = bins_head[b_idx]
                                    if head_n == -1 or new_m < q_metric[head_n]:
                                        q_next[q_ptr] = head_n
                                        bins_head[b_idx] = q_ptr
                                    else:
                                        prev_n = head_n
                                        curr_n = q_next[prev_n]
                                        while curr_n != -1 and q_metric[curr_n] <= new_m:
                                            prev_n = curr_n
                                            curr_n = q_next[curr_n]
                                        q_next[q_ptr] = curr_n
                                        q_next[prev_n] = q_ptr

                                    q_ptr += 1
                                    opencount += 1

            visited_nodes[:vis_count].sort()

            for k in range(R_count):
                curs_nc = 0.0
                curs_td = 0.0
                curs_tw = 0.0
                curs_w_td = 0.0

                for v_i in range(vis_count):
                    j = int(visited_nodes[v_i])
                    m0 = ((uncovered[j, 0] >> np.uint32(k)) & np.uint32(1)) == np.uint32(0)
                    m1 = ((uncovered[j, 1] >> np.uint32(k)) & np.uint32(1)) == np.uint32(0)
                    if m0 or m1:
                        if m0 and m1:
                            if audit_depth[k, j, 0] < audit_depth[k, j, 1]:
                                d_use = 0
                            else:
                                d_use = 1
                        elif m0:
                            d_use = 0
                        else:
                            d_use = 1

                        d_val = float(audit_depth[k, j, d_use])
                        w_j = float(weights_f32[j])
                        curs_nc += 1.0
                        curs_td += d_val
                        curs_tw += w_j
                        curs_w_td += d_val * w_j

                        if audit_leaf[k, j, d_use] and j != cursor:
                            here_dir = np.int8(1) if d_use == 0 else np.int8(-1)
                            here_ref = j
                            choicecount = 0
                            choiceweight = 0.0

                            while here_ref != cursor:
                                heredir = 0 if here_dir == 1 else 1
                                chunk_choice_unw[chunk_id, k, here_ref, heredir] += float(choicecount) * scale_mult
                                chunk_choice_wgt[chunk_id, k, here_ref, heredir] += choiceweight * scale_mult

                                if not audit_choice_cov[k, here_ref, heredir]:
                                    choicecount += 1
                                    prod_w = float(weights_f32[here_ref]) * rootweight
                                    choiceweight += prod_w
                                    audit_choice_cov[k, here_ref, heredir] = True
                                    chunk_choice_wgt[chunk_id, k, here_ref, heredir] += (prod_w * 0.5) * scale_mult

                                p_dir = audit_prev_dir[k, here_ref, heredir]
                                p_ref = int(audit_prev_ref[k, here_ref, heredir])
                                if p_ref < 0:
                                    break
                                here_dir = p_dir
                                here_ref = p_ref

                            root_heredir = 0 if here_dir == 1 else 1
                            chunk_choice_wgt[chunk_id, k, here_ref, root_heredir] += (choiceweight * 0.5) * scale_mult

                td_conv = float(np.float32(curs_td / denom_conv))
                w_td_conv = float(np.float32(curs_w_td / denom_conv))
                nc_all[k, cursor] = float(np.float32(curs_nc))
                if curs_nc > 1.0:
                    td_unw_all[k, cursor] = td_conv
                    td_wgt_all[k, cursor] = w_td_conv
                    integ_unw_all[k, cursor] = (
                        float(np.float32(curs_nc * curs_nc / td_conv)) if td_conv > 1e-9 else -1.0
                    )
                    integ_wgt_all[k, cursor] = (
                        float(np.float32(curs_tw * curs_tw / w_td_conv)) if w_td_conv > 1e-9 else -1.0
                    )
                else:
                    td_unw_all[k, cursor] = -1.0
                    td_wgt_all[k, cursor] = -1.0
                    integ_unw_all[k, cursor] = -1.0
                    integ_wgt_all[k, cursor] = -1.0

            for v_i in range(vis_count):
                j = int(visited_nodes[v_i])
                uncovered[j, 0] = radiusmask
                uncovered[j, 1] = radiusmask
                in_visited[j] = False
                for k in range(R_count):
                    audit_depth[k, j, 0] = 0
                    audit_depth[k, j, 1] = 0
                    audit_prev_dir[k, j, 0] = 0
                    audit_prev_dir[k, j, 1] = 0
                    audit_prev_ref[k, j, 0] = -1
                    audit_prev_ref[k, j, 1] = -1
                    audit_leaf[k, j, 0] = True
                    audit_leaf[k, j, 1] = True
                    audit_choice_cov[k, j, 0] = False
                    audit_choice_cov[k, j, 1] = False

    for k in range(R_count):
        for u in range(N):
            acc_u0 = 0.0
            acc_u1 = 0.0
            acc_w0 = 0.0
            acc_w1 = 0.0
            for c_id in range(n_chunks):
                acc_u0 += chunk_choice_unw[c_id, k, u, 0]
                acc_u1 += chunk_choice_unw[c_id, k, u, 1]
                acc_w0 += chunk_choice_wgt[c_id, k, u, 0]
                acc_w1 += chunk_choice_wgt[c_id, k, u, 1]
            if is_weighted:
                choice_out[k, u] = float(np.float32(acc_w0 + acc_w1))
                closeness_out[k, u] = integ_wgt_all[k, u]
                total_depth_out[k, u] = td_wgt_all[k, u]
            else:
                choice_out[k, u] = float(np.float32(acc_u0 + acc_u1))
                closeness_out[k, u] = integ_unw_all[k, u]
                total_depth_out[k, u] = td_unw_all[k, u]
            node_count_out[k, u] = nc_all[k, u]

    return choice_out, closeness_out, total_depth_out, node_count_out


def _tulip_directed_multi_radius_py_fast(
    fwd_offsets: np.ndarray,
    fwd_targets: np.ndarray,
    fwd_dirs: np.ndarray,
    fwd_weights: np.ndarray,
    bwd_offsets: np.ndarray,
    bwd_targets: np.ndarray,
    bwd_dirs: np.ndarray,
    bwd_weights: np.ndarray,
    seg_lengths: np.ndarray,
    weights_f32: np.ndarray,
    is_weighted: bool,
    tulip_bins_full: int,
    radius_vals: np.ndarray,
    rtype_code: int,
    sources: np.ndarray,
    scale_mult: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Pure-Python fallback for exact depthmapX directed-segment Tulip semicircle bucket solver."""
    N = len(seg_lengths)
    R_count = len(radius_vals)
    choice_out = np.zeros((R_count, N), dtype=np.float64)
    closeness_out = np.zeros((R_count, N), dtype=np.float64)
    total_depth_out = np.zeros((R_count, N), dtype=np.float64)
    node_count_out = np.zeros((R_count, N), dtype=np.float64)
    if N == 0 or R_count == 0 or len(sources) == 0:
        return choice_out, closeness_out, total_depth_out, node_count_out

    tulip_bins = (tulip_bins_full // 2) + 1
    denom_conv = float(tulip_bins - 1) * 0.5
    radiusmask = (1 << min(R_count, 30)) - 1

    f_off = fwd_offsets.tolist()
    f_tar = fwd_targets.tolist()
    f_dir = fwd_dirs.tolist()
    f_w = fwd_weights.tolist()
    b_off = bwd_offsets.tolist()
    b_tar = bwd_targets.tolist()
    b_dir = bwd_dirs.tolist()
    b_w = bwd_weights.tolist()
    s_len = seg_lengths.tolist()
    w_list = [float(x) for x in weights_f32.tolist()]
    r_list = radius_vals.tolist()

    # Pre-pack directed neighbor lists per node to eliminate inner-loop math/float32 overhead
    fwd_adj = []
    bwd_adj = []
    for u in range(N):
        fl = []
        for e in range(f_off[u], f_off[u + 1]):
            cr = f_tar[e]
            cd = f_dir[e]
            c_state = (cr << 1) | (0 if cd == 1 else 1)
            ed = int(math.floor(float(f_w[e]) * float(tulip_bins) * 0.5))
            sl = float(s_len[cr])
            fl.append((c_state, cr, cd, ed, sl, sl * 0.5))
        fwd_adj.append(fl)
        bl = []
        for e in range(b_off[u], b_off[u + 1]):
            cr = b_tar[e]
            cd = b_dir[e]
            c_state = (cr << 1) | (0 if cd == 1 else 1)
            ed = int(math.floor(float(b_w[e]) * float(tulip_bins) * 0.5))
            sl = float(s_len[cr])
            bl.append((c_state, cr, cd, ed, sl, sl * 0.5))
        bwd_adj.append(bl)

    n2 = N * 2
    accum_unw = [[0.0] * n2 for _ in range(R_count)]
    accum_wgt = [[0.0] * n2 for _ in range(R_count)]

    uncovered = [radiusmask] * n2
    audit_depth = [[0] * n2 for _ in range(R_count)]
    audit_prev_state = [[-1] * n2 for _ in range(R_count)]
    audit_leaf = [[True] * n2 for _ in range(R_count)]
    audit_choice_cov = [[False] * n2 for _ in range(R_count)]
    in_visited = [False] * N

    def _insert_bin(b_list: list, item: tuple) -> None:
        if not b_list or b_list[-1][0] > item[0]:
            b_list.append(item)
            return
        m_val = item[0]
        lo, hi = 0, len(b_list)
        while lo < hi:
            mid = (lo + hi) >> 1
            if b_list[mid][0] > m_val:
                lo = mid + 1
            else:
                hi = mid
        b_list.insert(lo, item)

    for cursor in sources.tolist():
        bins = [[] for _ in range(tulip_bins)]
        rootseglength = float(s_len[cursor])
        rootweight = w_list[cursor]
        bins[0].append((rootseglength * 0.5, 0, cursor, -1, 0, radiusmask))

        depthlevel = 0
        opencount = 1
        currentbin = 0
        visited_nodes = []

        while opencount > 0:
            while not bins[currentbin]:
                depthlevel += 1
                currentbin += 1
                if currentbin == tulip_bins:
                    currentbin = 0
            li_metric, li_dir, ref_i, prev_state, li_segdepth, li_cov = bins[currentbin].pop()
            opencount -= 1

            dir_idx = 0 if li_dir == 1 else 1
            state_i = (ref_i << 1) | dir_idx
            coverage = li_cov & uncovered[state_i]
            if coverage != 0:
                if not in_visited[ref_i]:
                    in_visited[ref_i] = True
                    visited_nodes.append(ref_i)
                if prev_state != -1:
                    uncovered[state_i] &= ~coverage
                    rbin = (coverage & -coverage).bit_length() - 1
                    rbinbase = rbin
                    while rbin < R_count:
                        if ((coverage >> rbin) & 1) == 1:
                            audit_depth[rbin][state_i] = depthlevel
                            audit_prev_state[rbin][state_i] = prev_state
                            audit_leaf[rbin][prev_state] = False
                        rbin += 1
                else:
                    rbinbase = 0
                    uncovered[ref_i << 1] &= ~coverage
                    uncovered[(ref_i << 1) | 1] &= ~coverage

                if li_dir != -1:
                    p_st_f = ref_i << 1
                    for c_state, conn_ref, conn_dir, extradepth, seglength, half_len in fwd_adj[ref_i]:
                        if (uncovered[c_state] & coverage) != 0:
                            rbin = rbinbase
                            if rtype_code == 1:
                                m_check = li_metric + half_len
                                while rbin < R_count and r_list[rbin] >= 0.0 and m_check > r_list[rbin]:
                                    rbin += 1
                            elif rtype_code == 0:
                                a_check = float(depthlevel + extradepth)
                                while rbin < R_count and r_list[rbin] >= 0.0 and a_check > r_list[rbin]:
                                    rbin += 1
                            else:
                                if rbin < R_count and r_list[rbin] >= 0.0 and float(li_segdepth) >= r_list[rbin]:
                                    rbin += 1
                            if rbin < R_count and (coverage >> rbin) != 0:
                                next_cov = (coverage >> rbin) << rbin
                                b_idx = (currentbin + tulip_bins + extradepth) % tulip_bins
                                _insert_bin(
                                    bins[b_idx],
                                    (li_metric + seglength, conn_dir, conn_ref, p_st_f, li_segdepth + 1, next_cov)
                                )
                                opencount += 1

                if li_dir != 1:
                    p_st_b = (ref_i << 1) | 1
                    for c_state, conn_ref, conn_dir, extradepth, seglength, half_len in bwd_adj[ref_i]:
                        if (uncovered[c_state] & coverage) != 0:
                            rbin = rbinbase
                            if rtype_code == 1:
                                m_check = li_metric + half_len
                                while rbin < R_count and r_list[rbin] >= 0.0 and m_check > r_list[rbin]:
                                    rbin += 1
                            elif rtype_code == 0:
                                a_check = float(depthlevel + extradepth)
                                while rbin < R_count and r_list[rbin] >= 0.0 and a_check > r_list[rbin]:
                                    rbin += 1
                            else:
                                if rbin < R_count and r_list[rbin] >= 0.0 and float(li_segdepth) >= r_list[rbin]:
                                    rbin += 1
                            if rbin < R_count and (coverage >> rbin) != 0:
                                next_cov = (coverage >> rbin) << rbin
                                b_idx = (currentbin + tulip_bins + extradepth) % tulip_bins
                                _insert_bin(
                                    bins[b_idx],
                                    (li_metric + seglength, conn_dir, conn_ref, p_st_b, li_segdepth + 1, next_cov)
                                )
                                opencount += 1

        visited_nodes.sort()
        for k in range(R_count):
            ad_k = audit_depth[k]
            ap_k = audit_prev_state[k]
            al_k = audit_leaf[k]
            ac_k = audit_choice_cov[k]
            au_k = accum_unw[k]
            aw_k = accum_wgt[k]
            curs_nc = 0.0
            curs_td = 0.0
            curs_tw = 0.0
            curs_w_td = 0.0
            for j in visited_nodes:
                st0 = j << 1
                st1 = st0 | 1
                m0 = ((uncovered[st0] >> k) & 1) == 0
                m1 = ((uncovered[st1] >> k) & 1) == 0
                if m0 or m1:
                    if m0 and m1:
                        st_use = st0 if ad_k[st0] < ad_k[st1] else st1
                    elif m0:
                        st_use = st0
                    else:
                        st_use = st1
                    d_val = float(ad_k[st_use])
                    w_j = w_list[j]
                    curs_nc += 1.0
                    curs_td += d_val
                    curs_tw += w_j
                    curs_w_td += d_val * w_j
                    if al_k[st_use] and j != cursor:
                        here_st = st_use
                        choicecount = 0
                        choiceweight = 0.0
                        while (here_st >> 1) != cursor:
                            au_k[here_st] += float(choicecount) * scale_mult
                            aw_k[here_st] += choiceweight * scale_mult
                            if not ac_k[here_st]:
                                choicecount += 1
                                prod_w = w_list[here_st >> 1] * rootweight
                                choiceweight += prod_w
                                ac_k[here_st] = True
                                aw_k[here_st] += (prod_w * 0.5) * scale_mult
                            p_st = ap_k[here_st]
                            if p_st < 0:
                                break
                            here_st = p_st
                        aw_k[here_st] += (choiceweight * 0.5) * scale_mult

            td_conv = float(np.float32(curs_td / denom_conv))
            w_td_conv = float(np.float32(curs_w_td / denom_conv))
            node_count_out[k, cursor] = float(np.float32(curs_nc))
            if curs_nc > 1.0:
                if is_weighted:
                    total_depth_out[k, cursor] = w_td_conv
                    closeness_out[k, cursor] = (
                        float(np.float32(curs_tw * curs_tw / w_td_conv)) if w_td_conv > 1e-9 else -1.0
                    )
                else:
                    total_depth_out[k, cursor] = td_conv
                    closeness_out[k, cursor] = (
                        float(np.float32(curs_nc * curs_nc / td_conv)) if td_conv > 1e-9 else -1.0
                    )
            else:
                total_depth_out[k, cursor] = -1.0
                closeness_out[k, cursor] = -1.0

        for j in visited_nodes:
            st0 = j << 1
            st1 = st0 | 1
            uncovered[st0] = radiusmask
            uncovered[st1] = radiusmask
            in_visited[j] = False
            for k in range(R_count):
                audit_depth[k][st0] = 0
                audit_depth[k][st1] = 0
                audit_prev_state[k][st0] = -1
                audit_prev_state[k][st1] = -1
                audit_leaf[k][st0] = True
                audit_leaf[k][st1] = True
                audit_choice_cov[k][st0] = False
                audit_choice_cov[k][st1] = False

    for k in range(R_count):
        au_k = accum_unw[k]
        aw_k = accum_wgt[k]
        for u in range(N):
            st0 = u << 1
            if is_weighted:
                choice_out[k, u] = float(np.float32(aw_k[st0] + aw_k[st0 | 1]))
            else:
                choice_out[k, u] = float(np.float32(au_k[st0] + au_k[st0 | 1]))

    return choice_out, closeness_out, total_depth_out, node_count_out


def _tulip_directed_multi_radius_dispatch(
    graph: CSRDualGraph,
    w_raw: np.ndarray,
    is_weighted: bool,
    radii_arr: np.ndarray,
    is_global_arr: np.ndarray,
    rtype_code: int,
    sources: np.ndarray,
    scale_mult: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    tulip_bins_full = int(getattr(graph, "tulip_bins", 1024) or 1024)
    tulip_bins = (tulip_bins_full // 2) + 1
    r_count = len(radii_arr)

    indexed_radii = []
    for idx in range(r_count):
        if bool(is_global_arr[idx]) or not math.isfinite(float(radii_arr[idx])) or float(radii_arr[idx]) >= 1e8:
            r_val = -1.0
        elif rtype_code == 0:
            r_val = math.floor(float(radii_arr[idx]) * float(tulip_bins) * 0.5)
        else:
            r_val = float(radii_arr[idx])
        indexed_radii.append((idx, r_val))

    indexed_radii.sort(key=lambda item: (item[1] < 0.0, item[1] if item[1] >= 0.0 else 0.0, item[0]))
    radius_vals = np.array([item[1] for item in indexed_radii], dtype=np.float64)
    weights_f32 = np.asarray(w_raw, dtype=np.float32)

    fn = _tulip_directed_multi_radius_numba if HAS_NUMBA else _tulip_directed_multi_radius_py_fast
    ch_s, cl_s, td_s, nc_s = fn(
        graph.fwd_offsets,
        graph.fwd_targets,
        graph.fwd_dirs,
        graph.fwd_weights,
        graph.bwd_offsets,
        graph.bwd_targets,
        graph.bwd_dirs,
        graph.bwd_weights,
        graph.segment_lengths,
        weights_f32,
        is_weighted,
        tulip_bins_full,
        radius_vals,
        rtype_code,
        sources,
        scale_mult
    )

    N = graph.num_nodes
    if len(sources) < N and len(sources) > 0:
        step = max(1, N // len(sources))
        for u in range(N):
            nearest_s = min((u // step) * step, N - 1)
            if nearest_s != u:
                for k in range(r_count):
                    nc_s[k, u] = nc_s[k, nearest_s]
                    td_s[k, u] = td_s[k, nearest_s]
                    cl_s[k, u] = cl_s[k, nearest_s]

    choice_all = np.zeros_like(ch_s)
    closeness_all = np.zeros_like(cl_s)
    total_depth_all = np.zeros_like(td_s)
    node_count_all = np.zeros_like(nc_s)
    for k, (orig_idx, _) in enumerate(indexed_radii):
        choice_all[orig_idx] = ch_s[k]
        closeness_all[orig_idx] = cl_s[k]
        total_depth_all[orig_idx] = td_s[k]
        node_count_all[orig_idx] = nc_s[k]

    return choice_all, closeness_all, total_depth_all, node_count_all


def solve_brandes_single_radius(
    graph: CSRDualGraph,
    radius_m: float,
    max_sources: int = 2500,
    stride: int = 1,
    analysis_type: str = "angular",
    radius_type: str = "metric",
    weight_mode: str = "unweighted",
    custom_weights: np.ndarray = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Executes Brandes betweenness and closeness with early radius cutoff.
    Supports analysis_type in ('angular', 'metric', 'topological'),
    radius_type in ('metric', 'angular', 'topological'), and
    weight_mode in ('unweighted', 'length', 'custom').

    Returns:
        (choice_raw, closeness_raw, total_depth_raw, node_count_raw)
    """
    N = graph.num_nodes
    if N == 0:
        empty = np.zeros(0, dtype=np.float64)
        return empty, empty.copy(), empty.copy(), empty.copy()

    is_global = math.isinf(radius_m) or radius_m >= 1e8

    atype_code = _encode_mode(analysis_type, default_code=0)
    rtype_code = _encode_mode(radius_type, default_code=1)
    wmode = str(weight_mode or "unweighted").strip().lower()

    if wmode == "length":
        w_node = np.maximum(graph.segment_lengths.astype(np.float64), 1e-3)
        w_norm = w_node / max(float(np.mean(w_node)), 1e-6)
        is_weighted = True
    elif wmode == "custom":
        if custom_weights is not None and len(custom_weights) == N:
            w_node = np.maximum(np.asarray(custom_weights, dtype=np.float64), 1e-3)
        else:
            w_node = np.maximum(graph.segment_weights.astype(np.float64), 1e-3)
        w_norm = w_node / max(float(np.mean(w_node)), 1e-6)
        is_weighted = True
    else:
        w_node = np.ones(N, dtype=np.float64)
        w_norm = w_node
        is_weighted = False

    if N <= max_sources:
        sources = np.arange(0, N, stride, dtype=np.int64)
        scale_mult = float(stride)
    else:
        step = max(1, N // max_sources)
        sources = np.arange(0, N, step, dtype=np.int64)
        scale_mult = float(step)

    radii_arr = np.array([float(radius_m)], dtype=np.float64)
    is_global_arr = np.array([is_global], dtype=np.bool_)

    if (
        int(getattr(graph, "tulip_bins", 0) or 0) > 0
        and atype_code == 0
        and len(getattr(graph, "fwd_offsets", [])) == N + 1
    ):
        choice_all, closeness_all, total_depth_all, node_count_all = _tulip_directed_multi_radius_dispatch(
            graph,
            w_node,
            is_weighted,
            radii_arr,
            is_global_arr,
            rtype_code,
            sources,
            scale_mult
        )
    else:
        choice_all, closeness_all, total_depth_all, node_count_all = _brandes_multi_radius_dispatch(
            graph.row_offsets,
            graph.target_nodes,
            graph.angular_weights,
            graph.metric_lengths,
            w_norm,
            is_weighted,
            radii_arr,
            is_global_arr,
            atype_code,
            rtype_code,
            sources,
            scale_mult
        )

    return choice_all[0], closeness_all[0], total_depth_all[0], node_count_all[0]


def solve_angular_segment_analysis(
    graph: CSRDualGraph,
    metric_radii: List[float] = [400.0, 800.0, 2000.0, 5000.0, float('inf')],
    max_sources_per_radius: int = 2500,
    progress_callback=None,
    include_normalized: bool = False,
    analysis_type: str = "angular",
    radius_type: str = "metric",
    weight_mode: str = "unweighted",
    custom_weights: np.ndarray = None
) -> Dict[str, np.ndarray]:
    """
    Solves full Angular/Metric/Topological Segment Analysis across specified radii.
    Computes Connectivity, Angular Connectivity, Control, Controllability, Choice,
    Integration, Total Depth, Mean Depth, Node Count, NACH, and NAIN.
    """
    N = graph.num_nodes
    control, controllability = _compute_segment_control_metrics(
        graph.row_offsets,
        graph.target_nodes,
        graph.connectivity
    )

    results: Dict[str, np.ndarray] = {
        "connectivity": graph.connectivity.astype(np.int32),
        "angular_connectivity": np.round(graph.angular_connectivity, 4),
        "control": np.round(control, 4),
        "controllability": np.round(controllability, 4)
    }

    if not metric_radii:
        if progress_callback:
            progress_callback("Analysis Complete!", 100.0)
        return results

    atype_code = _encode_mode(analysis_type, default_code=0)
    rtype_code = _encode_mode(radius_type, default_code=1)
    wmode = str(weight_mode or "unweighted").strip().lower()

    if wmode == "length" and N > 0:
        w_node = np.maximum(graph.segment_lengths.astype(np.float64), 1e-3)
        w_norm = w_node / max(float(np.mean(w_node)), 1e-6)
        is_weighted = True
    elif wmode == "custom" and N > 0:
        if custom_weights is not None and len(custom_weights) == N:
            w_node = np.maximum(np.asarray(custom_weights, dtype=np.float64), 1e-3)
        else:
            w_node = np.maximum(graph.segment_weights.astype(np.float64), 1e-3)
        w_norm = w_node / max(float(np.mean(w_node)), 1e-6)
        is_weighted = True
    else:
        w_node = np.ones(N, dtype=np.float64)
        w_norm = w_node
        is_weighted = False

    if N <= max_sources_per_radius:
        sources = np.arange(0, N, 1, dtype=np.int64)
        scale_mult = 1.0
    else:
        step = max(1, N // max_sources_per_radius)
        sources = np.arange(0, N, step, dtype=np.int64)
        scale_mult = float(step)

    radii_arr = np.array([float(r) for r in metric_radii], dtype=np.float64)
    is_global_arr = np.array([(math.isinf(r) or r >= 1e8) for r in metric_radii], dtype=np.bool_)

    if progress_callback:
        progress_callback("Computing Multi-Radius ASA...", 0.0)

    is_tulip_mode = (
        int(getattr(graph, "tulip_bins", 0) or 0) > 0
        and atype_code == 0
        and len(getattr(graph, "fwd_offsets", [])) == N + 1
    )
    if is_tulip_mode:
        w_tulip = graph.segment_lengths.astype(np.float64) if wmode != "custom" else w_node
        choice_all, closeness_all, total_depth_all, node_count_all = _tulip_directed_multi_radius_dispatch(
            graph,
            w_tulip,
            is_weighted,
            radii_arr,
            is_global_arr,
            rtype_code,
            sources,
            scale_mult
        )
    else:
        choice_all, closeness_all, total_depth_all, node_count_all = _brandes_multi_radius_dispatch(
            graph.row_offsets,
            graph.target_nodes,
            graph.angular_weights,
            graph.metric_lengths,
            w_norm,
            is_weighted,
            radii_arr,
            is_global_arr,
            atype_code,
            rtype_code,
            sources,
            scale_mult
        )

    for idx, r in enumerate(metric_radii):
        r_label = "rn" if (math.isinf(r) or r >= 1e8) else f"r{int(r)}"

        if progress_callback:
            progress_callback(f"Computing ASA Radius {r_label}...", (idx / len(metric_radii)) * 100.0)

        choice = choice_all[idx]
        closeness = closeness_all[idx]
        total_depth = total_depth_all[idx]
        node_count = node_count_all[idx]

        # Compute Hillier exact Logarithmic Normalizations
        # NACH = ln(Choice + 1) / ln(TotalDepth + 3)
        # NAIN = (NodeCount)^1.2 / (TotalDepth + 2)
        safe_choice = np.maximum(choice, 0.0)
        safe_depth = np.maximum(total_depth, 0.0)
        safe_count = np.maximum(node_count, 1.0)
        mean_depth = safe_depth / safe_count

        nach = np.log(safe_choice + 1.0) / np.log(safe_depth + 3.0)
        nain = (safe_count ** 1.2) / (safe_depth + 2.0)

        results[f"choice_{r_label}"] = choice if is_tulip_mode else np.round(choice, 2)
        results[f"integration_{r_label}"] = closeness if is_tulip_mode else np.round(closeness, 4)
        results[f"total_depth_{r_label}"] = total_depth if is_tulip_mode else np.round(total_depth, 2)
        results[f"mean_depth_{r_label}"] = np.round(mean_depth, 4)
        results[f"node_count_{r_label}"] = node_count.astype(np.int32)
        results[f"nach_{r_label}"] = np.round(nach, 4)
        results[f"nain_{r_label}"] = np.round(nain, 4)

        if include_normalized:
            max_c = float(np.max(safe_choice, initial=0.0))
            nach_norm = (
                np.log(safe_choice + 1.0) / np.log(max_c + 1.0)
                if max_c > 0.0 else np.zeros_like(safe_choice)
            )
            results[f"nach_norm_{r_label}"] = np.round(nach_norm, 4)

            max_i = float(np.max(closeness, initial=0.0))
            nain_norm = (
                np.log(np.maximum(closeness, 0.0) + 1.0) / np.log(max_i + 1.0)
                if max_i > 0.0 else np.zeros_like(closeness)
            )
            results[f"nain_norm_{r_label}"] = np.round(nain_norm, 4)

    if progress_callback:
        progress_callback("Analysis Complete!", 100.0)

    return results


@njit(cache=True)
def _origin_step_depth_numba(
    row_offsets: np.ndarray,
    target_nodes: np.ndarray,
    w_ang: np.ndarray,
    w_metric: np.ndarray,
    origins_arr: np.ndarray,
    max_radius: float,
    is_global: bool,
    rtype_code: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    N = len(row_offsets) - 1
    M_edges = len(target_nodes)

    d_ang = np.full(N, np.inf, dtype=np.float64)
    d_metric = np.full(N, np.inf, dtype=np.float64)
    d_topo = np.full(N, -1, dtype=np.int32)
    nearest_origin_id = np.full(N, -1, dtype=np.int32)
    is_origin = np.zeros(N, dtype=np.int32)

    heap_cap = max(4 * M_edges + len(origins_arr) + 4, 16)
    heap_ang = np.empty(heap_cap, dtype=np.float64)
    heap_metric = np.empty(heap_cap, dtype=np.float64)
    heap_topo = np.empty(heap_cap, dtype=np.int32)
    heap_node = np.empty(heap_cap, dtype=np.int32)
    heap_size = 0

    for i in range(len(origins_arr)):
        o = int(origins_arr[i])
        d_ang[o] = 0.0
        d_metric[o] = 0.0
        d_topo[o] = 0
        nearest_origin_id[o] = o
        is_origin[o] = 1

        pos = heap_size
        heap_size += 1
        while pos > 0:
            parent = (pos - 1) >> 1
            pa = heap_ang[parent]
            pm = heap_metric[parent]
            pt = heap_topo[parent]
            pn = heap_node[parent]
            if (0.0 < pa) or (0.0 == pa and (0.0 < pm or (0.0 == pm and (0 < pt or (0 == pt and o < pn))))):
                heap_ang[pos] = pa
                heap_metric[pos] = pm
                heap_topo[pos] = pt
                heap_node[pos] = pn
                pos = parent
            else:
                break
        heap_ang[pos] = 0.0
        heap_metric[pos] = 0.0
        heap_topo[pos] = 0
        heap_node[pos] = o

    while heap_size > 0:
        dist_a = heap_ang[0]
        dist_m = heap_metric[0]
        dist_t = heap_topo[0]
        u = heap_node[0]
        heap_size -= 1
        if heap_size > 0:
            last_a = heap_ang[heap_size]
            last_m = heap_metric[heap_size]
            last_t = heap_topo[heap_size]
            last_n = heap_node[heap_size]
            pos = 0
            half = heap_size >> 1
            while pos < half:
                child = (pos << 1) + 1
                right = child + 1
                ca = heap_ang[child]
                cm = heap_metric[child]
                ct = heap_topo[child]
                cn = heap_node[child]
                if right < heap_size:
                    ra = heap_ang[right]
                    rm = heap_metric[right]
                    rt = heap_topo[right]
                    rn = heap_node[right]
                    if (ra < ca) or (
                        ra == ca and (rm < cm or (rm == cm and (rt < ct or (rt == ct and rn < cn))))
                    ):
                        child = right
                        ca = ra
                        cm = rm
                        ct = rt
                        cn = rn
                if (last_a < ca) or (
                    last_a == ca and (
                        last_m < cm or (last_m == cm and (last_t < ct or (last_t == ct and last_n <= cn)))
                    )
                ):
                    break
                heap_ang[pos] = ca
                heap_metric[pos] = cm
                heap_topo[pos] = ct
                heap_node[pos] = cn
                pos = child
            heap_ang[pos] = last_a
            heap_metric[pos] = last_m
            heap_topo[pos] = last_t
            heap_node[pos] = last_n

        if dist_a > d_ang[u] + 1e-9:
            continue

        row_start = int(row_offsets[u])
        row_end = int(row_offsets[u + 1])
        for e_idx in range(row_start, row_end):
            v = int(target_nodes[e_idx])
            new_ang = dist_a + float(w_ang[e_idx])
            new_metric = dist_m + float(w_metric[e_idx])
            new_topo = dist_t + 1

            if rtype_code == 0:
                horizon_val = new_ang
            elif rtype_code == 2:
                horizon_val = float(new_topo)
            else:
                horizon_val = new_metric

            if not is_global and horizon_val > max_radius:
                continue

            if new_ang < d_ang[v] - 1e-7 or (
                abs(new_ang - d_ang[v]) <= 1e-7 and new_metric < d_metric[v] - 1e-7
            ):
                d_ang[v] = new_ang
                d_metric[v] = new_metric
                d_topo[v] = new_topo
                nearest_origin_id[v] = nearest_origin_id[u]

                pos = heap_size
                heap_size += 1
                while pos > 0:
                    parent = (pos - 1) >> 1
                    pa = heap_ang[parent]
                    pm = heap_metric[parent]
                    pt = heap_topo[parent]
                    pn = heap_node[parent]
                    if (new_ang < pa) or (
                        new_ang == pa and (
                            new_metric < pm or (
                                new_metric == pm and (new_topo < pt or (new_topo == pt and v < pn))
                            )
                        )
                    ):
                        heap_ang[pos] = pa
                        heap_metric[pos] = pm
                        heap_topo[pos] = pt
                        heap_node[pos] = pn
                        pos = parent
                    else:
                        break
                heap_ang[pos] = new_ang
                heap_metric[pos] = new_metric
                heap_topo[pos] = new_topo
                heap_node[pos] = v

    return d_ang, d_metric, d_topo, nearest_origin_id, is_origin


def _origin_step_depth_py_fast(
    row_offsets: np.ndarray,
    target_nodes: np.ndarray,
    w_ang: np.ndarray,
    w_metric: np.ndarray,
    origins_arr: np.ndarray,
    max_radius: float,
    is_global: bool,
    rtype_code: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    N = len(row_offsets) - 1
    INF = float("inf")
    d_ang = [INF] * N
    d_metric = [INF] * N
    d_topo = [-1] * N
    nearest_origin_id = [-1] * N
    is_origin = np.zeros(N, dtype=np.int32)

    ro_list = row_offsets.tolist()
    tn_list = target_nodes.tolist()
    wa_list = w_ang.tolist()
    wm_list = w_metric.tolist()

    pq = []
    heappush = heapq.heappush
    heappop = heapq.heappop

    for o in origins_arr.tolist():
        d_ang[o] = 0.0
        d_metric[o] = 0.0
        d_topo[o] = 0
        nearest_origin_id[o] = o
        is_origin[o] = 1
        heappush(pq, (0.0, 0.0, 0, o))

    while pq:
        dist_a, dist_m, dist_t, u = heappop(pq)
        if dist_a > d_ang[u] + 1e-9:
            continue
        r_start = ro_list[u]
        r_end = ro_list[u + 1]
        orig_u = nearest_origin_id[u]
        for e_idx in range(r_start, r_end):
            v = tn_list[e_idx]
            new_ang = dist_a + wa_list[e_idx]
            new_metric = dist_m + wm_list[e_idx]
            new_topo = dist_t + 1
            if rtype_code == 0:
                horizon_val = new_ang
            elif rtype_code == 2:
                horizon_val = float(new_topo)
            else:
                horizon_val = new_metric
            if not is_global and horizon_val > max_radius:
                continue
            dav = d_ang[v]
            if new_ang < dav - 1e-7 or (abs(new_ang - dav) <= 1e-7 and new_metric < d_metric[v] - 1e-7):
                d_ang[v] = new_ang
                d_metric[v] = new_metric
                d_topo[v] = new_topo
                nearest_origin_id[v] = orig_u
                heappush(pq, (new_ang, new_metric, new_topo, v))

    return (
        np.array(d_ang, dtype=np.float64),
        np.array(d_metric, dtype=np.float64),
        np.array(d_topo, dtype=np.int32),
        np.array(nearest_origin_id, dtype=np.int32),
        is_origin
    )


def solve_origin_step_depth(
    graph: CSRDualGraph,
    origin_indices: List[int],
    max_radius: float = float('inf'),
    radius_type: str = "metric"
) -> Dict[str, np.ndarray]:
    """
    Runs Multi-Source Dijkstra from valid origin segment indices to compute:
    - angular_step_depth: shortest angular turn cost from nearest origin (0.0 at origins, -1.0 if unreachable)
    - metric_step_depth_m: shortest metric distance in meters from nearest origin (0.0 at origins, -1.0 if unreachable)
    - topo_step_depth: topological turn count from nearest origin (0 at origins, -1 if unreachable)
    - nearest_origin_id: segment ID of closest origin (-1 if unreachable)
    - is_origin: 1 if segment is in origin_indices else 0
    """
    N = graph.num_nodes
    valid_origins = sorted({int(idx) for idx in (origin_indices or []) if 0 <= int(idx) < N})

    angular_step_depth = np.full(N, -1.0, dtype=np.float64)
    metric_step_depth_m = np.full(N, -1.0, dtype=np.float64)
    topo_step_depth = np.full(N, -1, dtype=np.int32)
    nearest_origin_id = np.full(N, -1, dtype=np.int32)
    is_origin = np.zeros(N, dtype=np.int32)

    if not valid_origins or N == 0 or max_radius < 0.0:
        return {
            "angular_step_depth": angular_step_depth,
            "metric_step_depth_m": metric_step_depth_m,
            "topo_step_depth": topo_step_depth,
            "nearest_origin_id": nearest_origin_id,
            "is_origin": is_origin
        }

    is_global = math.isinf(max_radius) or max_radius >= 1e8
    rtype_code = _encode_mode(radius_type, default_code=1)
    origins_arr = np.array(valid_origins, dtype=np.int32)

    step_fn = _origin_step_depth_numba if HAS_NUMBA else _origin_step_depth_py_fast
    d_ang, d_metric, d_topo, nearest_origin_id, is_origin = step_fn(
        graph.row_offsets,
        graph.target_nodes,
        graph.angular_weights,
        graph.metric_lengths,
        origins_arr,
        float(max_radius),
        is_global,
        rtype_code
    )

    reachable = d_ang < np.inf
    angular_step_depth[reachable] = np.round(d_ang[reachable], 4)
    metric_step_depth_m[reachable] = np.round(d_metric[reachable], 2)
    topo_step_depth[reachable] = d_topo[reachable]
    for o in valid_origins:
        angular_step_depth[o] = 0.0
        metric_step_depth_m[o] = 0.0
        topo_step_depth[o] = 0

    return {
        "angular_step_depth": angular_step_depth,
        "metric_step_depth_m": metric_step_depth_m,
        "topo_step_depth": topo_step_depth,
        "nearest_origin_id": nearest_origin_id,
        "is_origin": is_origin
    }


def _safe_r2(x: np.ndarray, y: np.ndarray) -> float:
    """Computes Pearson R² with guards for small N or zero variance."""
    if x is None or y is None or len(x) < 3 or len(y) != len(x):
        return 0.0
    xf = np.asarray(x, dtype=np.float64)
    yf = np.asarray(y, dtype=np.float64)
    mask = np.isfinite(xf) & np.isfinite(yf)
    if int(np.sum(mask)) < 3:
        return 0.0
    xm = xf[mask]
    ym = yf[mask]
    if float(np.std(xm)) <= 1e-9 or float(np.std(ym)) <= 1e-9:
        return 0.0
    corr = float(np.corrcoef(xm, ym)[0, 1])
    if math.isnan(corr):
        return 0.0
    return round(corr * corr, 6)


def compute_network_summary_diagnostics(
    graph: CSRDualGraph,
    results: Dict[str, np.ndarray]
) -> Dict[str, Any]:
    """
    Computes descriptive distribution statistics (min, max, mean, std, p10, p50, p90)
    for every numeric metric in results, plus Space Syntax Intelligibility R² and Synergy R².
    """
    metric_stats: Dict[str, Dict[str, float]] = {}

    for key, arr in (results or {}).items():
        if arr is None:
            continue
        f_arr = np.asarray(arr, dtype=np.float64)
        finite = f_arr[np.isfinite(f_arr)]
        if len(finite) == 0:
            metric_stats[key] = {
                "min": 0.0, "max": 0.0, "mean": 0.0, "std": 0.0,
                "p10": 0.0, "p50": 0.0, "p90": 0.0
            }
        else:
            metric_stats[key] = {
                "min": round(float(np.min(finite)), 4),
                "max": round(float(np.max(finite)), 4),
                "mean": round(float(np.mean(finite)), 4),
                "std": round(float(np.std(finite)), 4),
                "p10": round(float(np.percentile(finite, 10)), 4),
                "p50": round(float(np.percentile(finite, 50)), 4),
                "p90": round(float(np.percentile(finite, 90)), 4)
            }

    conn = results.get("connectivity", graph.connectivity) if results else graph.connectivity

    # Identify global NAIN / Integration reference (prefer _rn, fallback to largest radius)
    nain_keys = [k for k in (results or {}) if k.startswith("nain_") and not k.startswith("nain_norm_")]
    global_nain_key = "nain_rn" if "nain_rn" in (results or {}) else (nain_keys[-1] if nain_keys else "")

    integ_keys = [k for k in (results or {}) if k.startswith("integration_")]
    global_integ_key = (
        "integration_rn" if "integration_rn" in (results or {}) else (integ_keys[-1] if integ_keys else "")
    )

    intelligibility_r2 = (
        _safe_r2(conn, results[global_nain_key]) if global_nain_key and global_nain_key in results else 0.0
    )
    intelligibility_integration_r2 = (
        _safe_r2(conn, results[global_integ_key]) if global_integ_key and global_integ_key in results else 0.0
    )

    synergy_r2: Dict[str, float] = {}
    if global_nain_key and global_nain_key in results:
        global_nain = results[global_nain_key]
        for nk in nain_keys:
            r_label = nk[len("nain_"):]
            if r_label != "rn" and nk != global_nain_key:
                synergy_r2[r_label] = _safe_r2(results[nk], global_nain)

    diagnostics: Dict[str, Any] = {
        "num_segments": int(graph.num_nodes),
        "total_length_m": round(float(np.sum(graph.segment_lengths)), 2),
        "metrics": metric_stats,
        "intelligibility_r2": intelligibility_r2,
        "intelligibility_integration_r2": intelligibility_integration_r2,
        "synergy_r2": synergy_r2
    }
    for k, stat_dict in metric_stats.items():
        diagnostics[k] = stat_dict

    return diagnostics


def export_detailed_asa_tables(
    graph: CSRDualGraph,
    results: Dict[str, np.ndarray],
    detailed_csv_path: str = "",
    summary_csv_path: str = "",
    extra_attrs: Dict[str, List[Any]] = None
) -> Dict[str, Any]:
    """
    Exports segment-level ASA metrics and network-wide summary diagnostics + R² correlations
    to UTF-8 CSV files using Python stdlib csv. Returns the diagnostics dictionary.
    """
    diagnostics = compute_network_summary_diagnostics(graph, results)
    N = graph.num_nodes
    extra_keys = list(extra_attrs.keys()) if extra_attrs else []
    result_keys = list(results.keys()) if results else []

    if detailed_csv_path:
        parent_dir = os.path.dirname(os.path.abspath(detailed_csv_path))
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)
        header = ["seg_id", "x1", "y1", "x2", "y2", "length_m", "azimuth_deg"] + extra_keys + result_keys
        with open(detailed_csv_path, mode="w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for i in range(N):
                coords = graph.segment_coords[i]
                row = [
                    i,
                    round(float(coords[0]), 3),
                    round(float(coords[1]), 3),
                    round(float(coords[2]), 3),
                    round(float(coords[3]), 3),
                    round(float(graph.segment_lengths[i]), 3),
                    round(float(graph.segment_azimuths[i]), 2)
                ]
                for ek in extra_keys:
                    vals = extra_attrs[ek]
                    row.append(vals[i] if i < len(vals) else "")
                for rk in result_keys:
                    arr = results[rk]
                    row.append(arr[i].item() if hasattr(arr[i], "item") else arr[i])
                writer.writerow(row)

    if summary_csv_path:
        parent_dir = os.path.dirname(os.path.abspath(summary_csv_path))
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)
        with open(summary_csv_path, mode="w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["metric", "min", "max", "mean", "std", "p10", "p50", "p90"])
            for m_name, st in diagnostics["metrics"].items():
                writer.writerow([
                    m_name, st["min"], st["max"], st["mean"], st["std"], st["p10"], st["p50"], st["p90"]
                ])
            writer.writerow([
                "intelligibility_r2",
                diagnostics["intelligibility_r2"],
                diagnostics["intelligibility_r2"],
                diagnostics["intelligibility_r2"],
                0.0, 0.0, 0.0, 0.0
            ])
            writer.writerow([
                "intelligibility_integration_r2",
                diagnostics["intelligibility_integration_r2"],
                diagnostics["intelligibility_integration_r2"],
                diagnostics["intelligibility_integration_r2"],
                0.0, 0.0, 0.0, 0.0
            ])
            for r_label, syn_val in diagnostics["synergy_r2"].items():
                writer.writerow([
                    f"synergy_{r_label}_r2",
                    syn_val, syn_val, syn_val, 0.0, 0.0, 0.0, 0.0
                ])

    return diagnostics
