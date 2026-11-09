# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core: Topological Axial Map Analysis Solver (Full depthmapX v0.8.0 Parity)
Implements Hillier & Hanson (1984), Teklenburg et al. (1993), Turner (2001, 2004), and Penn (2003):
1. Connectivity (C_i), Control, and Controllability
2. Node Count (NC), Total Depth (TD), Mean Depth (MD), and Harmonic Mean Depth (HMD)
3. Relative Asymmetry (RA), Real Relative Asymmetry (RRA via Hillier Diamond D_n), and RA [Penn]
4. Integration [HH], Integration [P-value], and Integration [Tekl]
5. Point Depth Entropy, Relativised Entropy (Poisson KL Divergence), and Intensity
6. Topological Shortest-Path Choice (Betweenness) and Normalized Choice (Choice [Norm])
7. Multi-radius evaluation (e.g., Global 'n', R3, R5) + Intelligibility (R²).
"""

import os
import math
from typing import List, Dict, Tuple, Optional, Union
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import breadth_first_order
from shapely.geometry import LineString
from shapely.strtree import STRtree

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


@njit(cache=True)
def _hillier_diamond_dn_numba(n: int) -> float:
    if n <= 2:
        return 1.0
    numerator = 2.0 * (n * (math.log2((n + 2.0) / 3.0) - 1.0) + 1.0)
    denominator = float((n - 1) * (n - 2))
    val = numerator / denominator
    return val if val > 1e-12 else 1e-12


def calculate_hillier_diamond_dn(n: int) -> float:
    """
    Computes Hillier's diamond benchmark D_n for an axial system of n lines:
    D_n = 2 * { n * [ log2((n + 2) / 3) - 1 ] + 1 } / [ (n - 1) * (n - 2) ]
    """
    if n <= 2:
        return 1.0
    numerator = 2.0 * (n * (math.log2((n + 2.0) / 3.0) - 1.0) + 1.0)
    denominator = float((n - 1) * (n - 2))
    return max(numerator / denominator, 1e-12)


def _radius_suffix(r: Optional[int]) -> str:
    return "" if r is None else f"_R{int(r)}"


@njit(parallel=True, cache=True)
def _solve_axial_numba(
    indptr: np.ndarray,
    indices: np.ndarray,
    N: int,
    radii_arr: np.ndarray,
    has_global: bool,
    include_choice: bool
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compiled Numba multi-threaded CSR BFS + Topological Choice kernel.
    Replaces ProcessPoolExecutor subprocesses with zero-allocation shared-memory threads.
    """
    R_count = len(radii_arr)
    metrics_out = np.full((R_count, 13, N), -1.0, dtype=np.float64)
    controllability = np.full(N, -1.0, dtype=np.float64)

    n_chunks = min(max(N, 1), 16)
    choice_chunks = (
        np.zeros((n_chunks, R_count, N), dtype=np.float64)
        if include_choice else np.zeros((1, 1, 1), dtype=np.float64)
    )

    for chunk_id in prange(n_chunks):
        start_s = (chunk_id * N) // n_chunks
        end_s = ((chunk_id + 1) * N) // n_chunks

        nodes = np.empty(N, dtype=np.int32)
        rank = np.full(N, -1, dtype=np.int32)
        pred_rank = np.empty(max(N, 1), dtype=np.int32)
        offsets = np.empty(N + 2, dtype=np.int32)
        counts_all = np.empty(N + 1, dtype=np.int64)
        delta_r = np.empty(N, dtype=np.float64)

        for s in range(start_s, end_s):
            nodes[0] = s
            rank[s] = 0
            head = 0
            tail = 1

            while head < tail:
                u = nodes[head]
                u_rk = head
                head += 1
                row_start = int(indptr[u])
                row_end = int(indptr[u + 1])
                for e_idx in range(row_start, row_end):
                    v = int(indices[e_idx])
                    if rank[v] == -1:
                        rank[v] = tail
                        nodes[tail] = v
                        pred_rank[tail - 1] = u_rk
                        tail += 1

            K = tail
            if K > 1:
                offsets[0] = 0
                offsets[1] = 1
                n_offsets = 2
                hi = 1
                search_pos = 0
                while hi < K:
                    while search_pos < K - 1 and pred_rank[search_pos] < hi:
                        search_pos += 1
                    hi = 1 + search_pos
                    offsets[n_offsets] = hi
                    n_offsets += 1
                n_counts = n_offsets - 1
                for d_i in range(n_counts):
                    counts_all[d_i] = offsets[d_i + 1] - offsets[d_i]
            else:
                offsets[0] = 0
                offsets[1] = 1
                n_counts = 1
                counts_all[0] = 1

            max_d_global = n_counts - 1

            if max_d_global >= 1:
                c1 = int(counts_all[1])
                c2 = int(counts_all[2]) if max_d_global >= 2 else 0
                if (c1 + c2) > 0:
                    controllability[s] = float(c1) / float(c1 + c2)

            for r_idx in range(R_count):
                r_lim = int(radii_arr[r_idx])
                if r_lim > 0 and max_d_global >= r_lim:
                    sub_len = r_lim
                    B_bins = r_lim + 1
                    max_wave = r_lim
                else:
                    sub_len = max_d_global
                    B_bins = max_d_global + 2
                    max_wave = max_d_global

                nc_sum = 0
                for d_i in range(1, sub_len + 1):
                    nc_sum += int(counts_all[d_i])
                nc = 1 + nc_sum
                metrics_out[r_idx, 0, s] = float(nc)

                if nc <= 1:
                    continue

                td = 0
                for d_i in range(1, sub_len + 1):
                    td += int(counts_all[d_i]) * d_i
                md = float(td) / float(nc - 1)
                metrics_out[r_idx, 2, s] = md

                ent = 0.0
                rel_ent = 0.0
                fact = 1.0
                h_denom = 0.0
                overflow = False
                exp_neg_md = math.exp(-md)

                for i_bin in range(sub_len):
                    cnt = int(counts_all[i_bin + 1])
                    fact *= float(i_bin + 1)
                    if cnt > 0:
                        prob = float(cnt) / float(nc)
                        ent -= prob * math.log2(prob)
                        if not overflow:
                            p_md = md ** i_bin
                            if math.isinf(p_md) or math.isnan(p_md) or math.isinf(fact) or fact <= 0.0:
                                overflow = True
                            else:
                                q = (p_md / fact) * exp_neg_md
                                if q > 0.0 and not math.isinf(q) and not math.isnan(q):
                                    rel_ent += prob * math.log2(prob / q)
                                else:
                                    overflow = True
                        h_denom += 1.0 / float(cnt)

                metrics_out[r_idx, 9, s] = ent
                metrics_out[r_idx, 10, s] = 0.0 if overflow else rel_ent
                metrics_out[r_idx, 11, s] = float(B_bins) / h_denom if h_denom > 0.0 else -1.0

                if td > nc:
                    metrics_out[r_idx, 12, s] = float(nc) * ent / float(td - nc)

                if nc > 2 and md > 1.0:
                    metrics_out[r_idx, 1, s] = float(td)
                    ra = 2.0 * (md - 1.0) / float(nc - 2)
                    dn = _hillier_diamond_dn_numba(nc)
                    rra = ra / dn
                    metrics_out[r_idx, 3, s] = ra
                    metrics_out[r_idx, 4, s] = rra

                    ihh = 1.0 / rra
                    metrics_out[r_idx, 6, s] = ihh
                    metrics_out[r_idx, 7, s] = ihh
                    if td > nc:
                        metrics_out[r_idx, 8, s] = math.log(0.5 * (nc - 2.0)) / math.log(float(td - nc + 1))

                    L = B_bins - 1
                    pal = 0.5 * L * (L + 1)
                    denom_p = (L - 1.0) * nc - pal + 1.0
                    if denom_p > 0.0:
                        metrics_out[r_idx, 5, s] = (L * nc - td - pal) / denom_p

                if include_choice and max_wave >= 2:
                    hi_limit = int(offsets[max_wave + 1])
                    for idx_c in range(hi_limit):
                        delta_r[idx_c] = 0.0
                    for d_step in range(max_wave, 0, -1):
                        lo_idx = int(offsets[d_step])
                        hi_idx = int(offsets[d_step + 1])
                        for idx_c in range(lo_idx, hi_idx):
                            p_rk = int(pred_rank[idx_c - 1])
                            delta_r[p_rk] += 1.0 + delta_r[idx_c]
                    for idx_c in range(1, hi_limit):
                        choice_chunks[chunk_id, r_idx, int(nodes[idx_c])] += delta_r[idx_c]

            for idx_k in range(K):
                rank[nodes[idx_k]] = -1

    choice_totals = np.zeros((R_count, N), dtype=np.float64)
    if include_choice:
        for c_id in range(n_chunks):
            for r_idx in range(R_count):
                for u in range(N):
                    choice_totals[r_idx, u] += choice_chunks[c_id, r_idx, u]

    return metrics_out, choice_totals, controllability


def _solve_axial_chunk(
    args: Tuple
) -> Tuple[int, int, Dict[str, np.ndarray], Dict[Optional[int], np.ndarray], np.ndarray]:
    """Worker function for multi-core Axial BFS & Choice evaluation."""
    indptr, indices, N, start_idx, end_idx, radii_clean, include_choice = args
    M = end_idx - start_idx
    adj_mat = csr_matrix((np.ones(len(indices), dtype=np.float32), indices, indptr), shape=(N, N))

    metric_names = []
    for r_lim in radii_clean:
        sfx = _radius_suffix(r_lim)
        metric_names.extend([
            f"Node_Count{sfx}",
            f"Total_Depth{sfx}",
            f"Mean_Depth{sfx}",
            f"RA{sfx}",
            f"RRA{sfx}",
            "RA_Penn_" if sfx == "" else f"RA_Penn{sfx}",
            "Integration_HH_" if sfx == "" else f"Integration_HH{sfx}",
            "Integration_P_value_" if sfx == "" else f"Integration_P_value{sfx}",
            "Integration_Tekl_" if sfx == "" else f"Integration_Tekl{sfx}",
            f"Entropy{sfx}",
            f"Relativised_Entropy{sfx}",
            f"Harmonic_Mean_Depth{sfx}",
            f"Intensity{sfx}",
        ])

    chunk_metrics = {k: np.full(M, -1.0, dtype=np.float64) for k in metric_names}
    choice_accum = {r_lim: np.zeros(N, dtype=np.float64) for r_lim in radii_clean} if include_choice else {}
    controllability_chunk = np.full(M, -1.0, dtype=np.float64)

    rank = np.empty(N, dtype=np.int32)
    log2_fact = np.cumsum(np.insert(np.log2(np.arange(1, N + 2, dtype=np.float64)), 0, 0.0))
    log2_e = math.log2(math.e)

    for local_i, s in enumerate(range(start_idx, end_idx)):
        nodes, pred = breadth_first_order(adj_mat, i_start=s, directed=False, return_predecessors=True)
        K = len(nodes)
        if K > 1:
            rank[nodes] = np.arange(K, dtype=np.int32)
            pred_rank = rank[pred[nodes[1:]]]
            offsets = [0, 1]
            hi = 1
            while hi < K:
                hi = 1 + int(np.searchsorted(pred_rank, hi, side="left"))
                offsets.append(hi)
            counts_all = np.diff(offsets)
        else:
            pred_rank = np.empty(0, dtype=np.int32)
            offsets = [0, 1]
            counts_all = np.array([1], dtype=np.int64)

        max_d_global = len(counts_all) - 1

        # Controllability: c1 / (c1 + c2)
        if max_d_global >= 1:
            c1 = int(counts_all[1])
            c2 = int(counts_all[2]) if max_d_global >= 2 else 0
            if (c1 + c2) > 0:
                controllability_chunk[local_i] = float(c1) / float(c1 + c2)

        for r_lim in radii_clean:
            sfx = _radius_suffix(r_lim)
            if r_lim is not None and max_d_global >= r_lim:
                c_sub = counts_all[1 : r_lim + 1]
                B_bins = r_lim + 1
                max_wave = r_lim
            else:
                c_sub = counts_all[1:]
                B_bins = max_d_global + 2
                max_wave = max_d_global

            nc = 1 + int(np.sum(c_sub))
            chunk_metrics[f"Node_Count{sfx}"][local_i] = float(nc)

            if nc <= 1:
                continue

            depths = np.arange(1, len(c_sub) + 1, dtype=np.int64)
            td = int(np.sum(c_sub * depths))
            md = float(td) / float(nc - 1)
            chunk_metrics[f"Mean_Depth{sfx}"][local_i] = md

            nz_mask = c_sub > 0
            if np.any(nz_mask):
                c_nz = c_sub[nz_mask].astype(np.float64)
                i_nz = np.nonzero(nz_mask)[0]
                probs = c_nz / float(nc)
                log2_probs = np.log2(probs)
                ent = float(-np.sum(probs * log2_probs))
                h_denom = float(np.sum(1.0 / c_nz))

                # log2(q_i) = i * log2(md) - log2((i+1)!) - md * log2(e)
                log2_md = math.log2(md) if md > 0.0 else -1e300
                log2_q = i_nz * log2_md - log2_fact[i_nz + 1] - md * log2_e
                if np.any(log2_q <= -1022.0) or np.any(~np.isfinite(log2_q)):
                    rel_ent = 0.0
                else:
                    rel_ent = float(np.sum(probs * (log2_probs - log2_q)))
            else:
                ent = 0.0
                rel_ent = 0.0
                h_denom = 0.0

            chunk_metrics[f"Entropy{sfx}"][local_i] = ent
            chunk_metrics[f"Relativised_Entropy{sfx}"][local_i] = rel_ent
            chunk_metrics[f"Harmonic_Mean_Depth{sfx}"][local_i] = float(B_bins) / h_denom if h_denom > 0.0 else -1.0

            if td > nc:
                chunk_metrics[f"Intensity{sfx}"][local_i] = float(nc) * ent / float(td - nc)

            if nc > 2 and md > 1.0:
                chunk_metrics[f"Total_Depth{sfx}"][local_i] = float(td)
                ra = 2.0 * (md - 1.0) / float(nc - 2)
                dn = calculate_hillier_diamond_dn(nc)
                rra = ra / dn
                chunk_metrics[f"RA{sfx}"][local_i] = ra
                chunk_metrics[f"RRA{sfx}"][local_i] = rra

                ihh = 1.0 / rra
                col_hh = "Integration_HH_" if sfx == "" else f"Integration_HH{sfx}"
                col_pv = "Integration_P_value_" if sfx == "" else f"Integration_P_value{sfx}"
                col_tk = "Integration_Tekl_" if sfx == "" else f"Integration_Tekl{sfx}"
                col_rap = "RA_Penn_" if sfx == "" else f"RA_Penn{sfx}"

                chunk_metrics[col_hh][local_i] = ihh
                chunk_metrics[col_pv][local_i] = ihh
                if td > nc:
                    chunk_metrics[col_tk][local_i] = math.log(0.5 * (nc - 2.0)) / math.log(float(td - nc + 1))

                L = B_bins - 1
                pal = 0.5 * L * (L + 1)
                denom_p = (L - 1.0) * nc - pal + 1.0
                if denom_p > 0.0:
                    chunk_metrics[col_rap][local_i] = (L * nc - td - pal) / denom_p

            # Choice (Topological Betweenness Centrality)
            if include_choice and max_wave >= 2:
                hi_limit = offsets[max_wave + 1]
                delta_r = np.zeros(hi_limit, dtype=np.float64)
                for d_step in range(max_wave, 0, -1):
                    lo_idx = offsets[d_step]
                    hi_idx = offsets[d_step + 1]
                    np.add.at(delta_r, pred_rank[lo_idx - 1 : hi_idx - 1], 1.0 + delta_r[lo_idx : hi_idx])
                choice_accum[r_lim][nodes[1:hi_limit]] += delta_r[1:hi_limit]

    return start_idx, end_idx, chunk_metrics, choice_accum, controllability_chunk


def solve_axial_analysis(
    axial_lines: List[LineString],
    radii: Optional[List[Optional[Union[int, float, str]]]] = None,
    include_choice: bool = True,
    max_workers: Optional[int] = None,
    progress_callback=None
) -> Tuple[Dict[str, np.ndarray], float]:
    """
    Solves full depthmapX-parity Topological Axial Analysis across multiple step-depth radii:
    - Connectivity, Control, Controllability, Line_Length
    - Node Count, Total Depth, Mean Depth, Harmonic Mean Depth
    - RA, RRA, RA [Penn]
    - Integration [HH], Integration [P-value], Integration [Tekl]
    - Entropy, Relativised Entropy, Intensity
    - Choice, Choice [Norm]
    - Intelligibility R² correlation
    """
    N = len(axial_lines)
    if N == 0:
        return {}, 0.0

    if radii is None:
        radii_clean: List[Optional[int]] = [None, 3, 5]
    else:
        radii_clean = []
        for r in radii:
            is_inf_str = isinstance(r, str) and r.strip().lower() in ("n", "inf", "global", "all")
            if r is None or is_inf_str or (isinstance(r, float) and math.isinf(r)):
                if None not in radii_clean:
                    radii_clean.append(None)
            else:
                r_int = int(round(float(r)))
                if r_int > 0 and r_int not in radii_clean:
                    radii_clean.append(r_int)
        if not radii_clean:
            radii_clean = [None, 3, 5]

    # 1. Build axial intersection CSR graph in O(N log N) via C-vectorized STRtree
    tree = STRtree(axial_lines)
    pairs = tree.query(axial_lines, predicate="intersects")
    mask = pairs[0] != pairs[1]
    u_arr = pairs[0][mask].astype(np.int32)
    v_arr = pairs[1][mask].astype(np.int32)

    # Sort edges for canonical deterministic BFS order matching depthmapX
    order = np.lexsort((v_arr, u_arr))
    u_arr = u_arr[order]
    v_arr = v_arr[order]

    adj_mat = csr_matrix((np.ones(len(u_arr), dtype=np.float32), (u_arr, v_arr)), shape=(N, N))
    connectivity = np.diff(adj_mat.indptr).astype(np.int32)
    line_lengths = np.array([float(line.length) for line in axial_lines], dtype=np.float64)

    # 2. Vectorized Control calculation: sum_{j in N(i)} (1 / deg(j))
    control = np.full(N, -1.0, dtype=np.float64)
    inv_deg = np.where(connectivity > 0, 1.0 / np.maximum(connectivity, 1), 0.0)
    if len(u_arr) > 0:
        ctrl_sum = np.bincount(u_arr, weights=inv_deg[v_arr], minlength=N)
        control = np.where(connectivity > 0, ctrl_sum, -1.0)

    # 3. Execute multi-threaded Numba CSR BFS + Choice kernel (or fallback to _solve_axial_chunk)
    indptr = adj_mat.indptr.astype(np.int32)
    indices = adj_mat.indices.astype(np.int32)

    if HAS_NUMBA:
        radii_arr = np.array([-1 if r is None else int(r) for r in radii_clean], dtype=np.int64)
        has_global = any(r is None for r in radii_clean)
        metrics_out, choice_arr, controllability = _solve_axial_numba(
            indptr,
            indices,
            N,
            radii_arr,
            has_global,
            bool(include_choice)
        )
        all_metrics: Dict[str, np.ndarray] = {}
        choice_totals = {}
        for r_idx, r_lim in enumerate(radii_clean):
            sfx = _radius_suffix(r_lim)
            col_rap = "RA_Penn_" if sfx == "" else f"RA_Penn{sfx}"
            col_hh = "Integration_HH_" if sfx == "" else f"Integration_HH{sfx}"
            col_pv = "Integration_P_value_" if sfx == "" else f"Integration_P_value{sfx}"
            col_tk = "Integration_Tekl_" if sfx == "" else f"Integration_Tekl{sfx}"

            all_metrics[f"Node_Count{sfx}"] = metrics_out[r_idx, 0]
            all_metrics[f"Total_Depth{sfx}"] = metrics_out[r_idx, 1]
            all_metrics[f"Mean_Depth{sfx}"] = metrics_out[r_idx, 2]
            all_metrics[f"RA{sfx}"] = metrics_out[r_idx, 3]
            all_metrics[f"RRA{sfx}"] = metrics_out[r_idx, 4]
            all_metrics[col_rap] = metrics_out[r_idx, 5]
            all_metrics[col_hh] = metrics_out[r_idx, 6]
            all_metrics[col_pv] = metrics_out[r_idx, 7]
            all_metrics[col_tk] = metrics_out[r_idx, 8]
            all_metrics[f"Entropy{sfx}"] = metrics_out[r_idx, 9]
            all_metrics[f"Relativised_Entropy{sfx}"] = metrics_out[r_idx, 10]
            all_metrics[f"Harmonic_Mean_Depth{sfx}"] = metrics_out[r_idx, 11]
            all_metrics[f"Intensity{sfx}"] = metrics_out[r_idx, 12]
            if include_choice:
                choice_totals[r_lim] = choice_arr[r_idx]
    else:
        if max_workers is None:
            # Avoid Windows subprocess spawn overhead (and spawning qgis-bin.exe inside QGIS)
            if os.name == "nt" or N < 5000:
                workers = 1
            else:
                cpu_cnt = os.cpu_count() or 4
                workers = min(max(1, cpu_cnt - 2), 12)
        else:
            workers = max(1, int(max_workers))

        if workers == 1 or N < 1500:
            _, _, all_metrics, choice_totals, controllability = _solve_axial_chunk(
                (indptr, indices, N, 0, N, radii_clean, include_choice)
            )
        else:
            chunk_size = math.ceil(N / workers)
            tasks = []
            for start_i in range(0, N, chunk_size):
                end_i = min(N, start_i + chunk_size)
                tasks.append((indptr, indices, N, start_i, end_i, radii_clean, include_choice))

            all_metrics = {}
            choice_totals = {r_lim: np.zeros(N, dtype=np.float64) for r_lim in radii_clean} if include_choice else {}
            controllability = np.full(N, -1.0, dtype=np.float64)

            with ProcessPoolExecutor(max_workers=workers) as executor:
                for s_idx, e_idx, c_metrics, c_choice, c_ctrl in executor.map(_solve_axial_chunk, tasks):
                    if not all_metrics:
                        for k in c_metrics.keys():
                            all_metrics[k] = np.full(N, -1.0, dtype=np.float64)
                    for k, arr in c_metrics.items():
                        all_metrics[k][s_idx:e_idx] = arr
                    if include_choice:
                        for r_lim, ch_arr in c_choice.items():
                            choice_totals[r_lim] += ch_arr
                    controllability[s_idx:e_idx] = c_ctrl

    # 4. Assemble results in official depthmapX schema + backwards-compatible keys
    results: Dict[str, np.ndarray] = {
        "Depthmap_Ref": np.arange(N, dtype=np.int32),
        "Connectivity": connectivity.astype(np.float64),
        "Line_Length": line_lengths,
        "Control": control,
        "Controllability": controllability,
    }

    if include_choice:
        for r_lim in radii_clean:
            sfx = _radius_suffix(r_lim)
            nc_arr = all_metrics[f"Node_Count{sfx}"]
            ch_raw = choice_totals[r_lim]
            ch_final = np.where(nc_arr > 2.0, ch_raw, -1.0)
            denom_ch = (nc_arr - 1.0) * (nc_arr - 2.0)
            ch_norm = np.where(nc_arr > 2.0, 2.0 * ch_raw / np.maximum(denom_ch, 1.0), -1.0)
            results[f"Choice{sfx}"] = ch_final
            results["Choice_Norm_" if sfx == "" else f"Choice_Norm{sfx}"] = ch_norm

    for k, arr in all_metrics.items():
        results[k] = arr

    # Intelligibility: Pearson R² between Connectivity and Global Integration [HH]
    global_int_col = (
        "Integration_HH_" if "Integration_HH_" in results
        else next((k for k in results if k.startswith("Integration_HH")), None)
    )
    intelligibility_r2 = 0.0
    if global_int_col is not None:
        int_rn = results[global_int_col]
        valid_mask = (connectivity > 0) & (int_rn > 0)
        if np.sum(valid_mask) > 2:
            corr_matrix = np.corrcoef(connectivity[valid_mask], int_rn[valid_mask])
            r_val = corr_matrix[0, 1] if not np.isnan(corr_matrix[0, 1]) else 0.0
            intelligibility_r2 = float(r_val ** 2)

    # Legacy alias keys for backward compatibility with existing OpenDepthmap scripts & tests
    results["ax_connectivity"] = connectivity
    results["ax_control"] = np.round(control, 4)
    results["ax_controllability"] = np.round(controllability, 4)
    if "Total_Depth" in results:
        results["ax_total_depth"] = np.where(results["Total_Depth"] >= 0, np.round(results["Total_Depth"], 2), 0.0)
    if "Mean_Depth" in results:
        results["ax_mean_depth"] = np.where(results["Mean_Depth"] >= 0, np.round(results["Mean_Depth"], 3), 0.0)
    if "RA" in results:
        results["ax_ra"] = np.where(results["RA"] >= 0, np.round(results["RA"], 4), 0.0)
    if "RRA" in results:
        results["ax_rra"] = np.where(results["RRA"] >= 0, np.round(results["RRA"], 4), 0.0)
    if "Integration_HH_" in results:
        results["ax_integration_rn"] = np.where(
            results["Integration_HH_"] >= 0, np.round(results["Integration_HH_"], 3), 0.0
        )
    if "Integration_HH_R3" in results:
        results["ax_integration_r3"] = np.where(
            results["Integration_HH_R3"] >= 0, np.round(results["Integration_HH_R3"], 3), 0.0
        )

    return results, intelligibility_r2
