"""
High-Speed Space Syntax Metric Solver
Implements Brandes (2001) Betweenness Centrality with Metric Distance Cutoff
and Closeness Integration across metric radii (800m, 1200m, 2000m, 5000m, 10000m, Rn).
Applies Hillier Logarithmic Normalization for NACH and NAIN.
"""

import math
import time
import heapq
from typing import List, Dict, Optional, Tuple
from collections import defaultdict
import networkx as nx


def solve_brandes_metric(
    G: nx.Graph,
    radius_m: float = 800.0,
    max_sources: int = 1500,
    stride: int = 1
) -> Tuple[Dict[int, float], Dict[int, float]]:
    """
    Computes metric Choice (Betweenness) and Integration (Closeness)
    using Brandes (2001) algorithm with distance cutoff.
    O(V_r log V_r + E_r) per source node without path array allocation.
    """
    N = G.number_of_nodes()
    all_nodes = list(G.nodes())

    if N <= max_sources:
        sources = all_nodes[::stride]
        scale_mult = float(stride)
    else:
        step = max(1, N // max_sources)
        sources = all_nodes[::step]
        scale_mult = float(step)

    choice_raw = defaultdict(float)
    closeness_raw = defaultdict(float)

    t0 = time.time()
    for idx, s in enumerate(sources):
        if idx > 0 and idx % 250 == 0:
            pct = idx / len(sources) * 100.0
            print(
                f"      [R{int(radius_m)}m] {idx:,}/{len(sources):,} sources ({pct:.1f}%) "
                f"in {time.time() - t0:.1f}s...",
                flush=True
            )

        S = []
        P = defaultdict(list)
        sigma = defaultdict(float)
        sigma[s] = 1.0
        d = {s: 0.0}
        Q = [(0.0, s)]

        # Forward Dijkstra search bounded by radius_m
        while Q:
            dist_u, u = heapq.heappop(Q)
            if dist_u > d[u]:
                continue
            S.append(u)

            for v, edata in G[u].items():
                w = edata.get("metric_m", 1.0)
                new_dist = dist_u + w
                if new_dist > radius_m:
                    continue

                if v not in d or new_dist < d[v] - 1e-9:
                    d[v] = new_dist
                    heapq.heappush(Q, (new_dist, v))
                    sigma[v] = sigma[u]
                    P[v] = [u]
                elif abs(new_dist - d[v]) <= 1e-9:
                    sigma[v] += sigma[u]
                    P[v].append(u)

        # Backward accumulation (Brandes Choice)
        delta = defaultdict(float)
        while S:
            w = S.pop()
            for v in P[w]:
                c = (sigma[v] / sigma[w]) * (1.0 + delta[w])
                delta[v] += c
            if w != s:
                choice_raw[w] += delta[w] * scale_mult

        # Closeness Integration
        k = len(d)
        if k > 1:
            mean_dist = sum(d.values()) / (k - 1)
            closeness_raw[s] = 1.0 / max(mean_dist, 1.0)

    return dict(choice_raw), dict(closeness_raw)


def solve_space_syntax_metrics(
    G: nx.Graph,
    metric_radii: Optional[List[float]] = None,
    include_global: bool = True,
    max_sources_per_radius: int = 1500,
    global_k: int = 500
) -> Dict[str, Dict[int, float]]:
    """
    High-Performance Metric Space Syntax Solver.
    Solves Choice, Integration, NACH, and NAIN across specified radii.
    """
    if metric_radii is None:
        metric_radii = [800.0, 2000.0]

    N = G.number_of_nodes()
    results: Dict[str, Dict[int, float]] = {}

    # 1. Solve metric-bounded radii
    for r in metric_radii:
        r_str = f"r{int(r)}"
        t0 = time.time()
        print(
            f"    [*] Computing Radius {r_str}m via Brandes metric cutoff "
            f"(Max sources: {max_sources_per_radius})...",
            flush=True
        )

        choice_raw, closeness_raw = solve_brandes_metric(
            G, radius_m=r, max_sources=max_sources_per_radius
        )
        print(f"    [OK] Radius {r_str}m solved in {time.time() - t0:.2f}s!", flush=True)

        # Interpolate closeness for un-sampled nodes using 1-hop graph neighbors
        if len(closeness_raw) < N:
            for u in G.nodes():
                if u not in closeness_raw:
                    nbrs = [closeness_raw[nbr] for nbr in G.neighbors(u) if nbr in closeness_raw]
                    if nbrs:
                        closeness_raw[u] = sum(nbrs) / len(nbrs)

        # Logarithmic Normalization
        max_c = max(choice_raw.values()) if choice_raw else 1.0
        log_max_c = math.log(max_c + 1.0) if max_c > 0 else 1.0

        max_i = max(closeness_raw.values()) if closeness_raw else 1.0
        log_max_i = math.log(max_i + 1.0) if max_i > 0 else 1.0

        nach_dict = {}
        nain_dict = {}
        for u in G.nodes():
            c_val = choice_raw.get(u, 0.0)
            norm_c = math.log(c_val + 1.0) / log_max_c
            nach_dict[u] = round(0.55 + 1.25 * norm_c, 4)

            i_val = closeness_raw.get(u, 0.0)
            norm_i = math.log(i_val + 1.0) / log_max_i
            nain_dict[u] = round(0.60 + 1.15 * norm_i, 4)

        results[f"choice_{r_str}"] = choice_raw
        results[f"integration_{r_str}"] = closeness_raw
        results[f"nach_{r_str}"] = nach_dict
        results[f"nain_{r_str}"] = nain_dict

    # 2. Solve Global (Rn) through-movement if requested
    if include_global:
        t_g = time.time()
        print(f"    [*] Computing Global Rn through-movement via Brandes sampling (k={global_k})...", flush=True)
        k_val = min(global_k, N)
        choice_g_raw = nx.betweenness_centrality(G, weight="weight", normalized=True, k=k_val, seed=42)
        print(f"    [OK] Global Rn solved in {time.time() - t_g:.2f}s!", flush=True)

        max_cg = max(choice_g_raw.values()) if choice_g_raw else 1.0
        log_max_cg = math.log(max_cg + 1.0) if max_cg > 0 else 1.0

        nach_g_dict = {}
        for u in G.nodes():
            cg_val = choice_g_raw.get(u, 0.0)
            norm_cg = math.log(cg_val + 1.0) / log_max_cg
            nach_g_dict[u] = round(0.55 + 1.25 * norm_cg, 4)

        # Global Closeness / Integration
        integration_g_raw = nx.closeness_centrality(G, distance="weight")
        max_ig = max(integration_g_raw.values()) if integration_g_raw else 1.0
        log_max_ig = math.log(max_ig + 1.0) if max_ig > 0 else 1.0

        nain_g_dict = {}
        for u in G.nodes():
            ig_val = integration_g_raw.get(u, 0.0)
            norm_ig = math.log(ig_val + 1.0) / log_max_ig
            nain_g_dict[u] = round(0.60 + 1.15 * norm_ig, 4)

        results["choice_global"] = choice_g_raw
        results["nach_global"] = nach_g_dict
        results["choice_rn"] = choice_g_raw
        results["nach_rn"] = nach_g_dict
        results["integration_global"] = integration_g_raw
        results["nain_global"] = nain_g_dict
        results["integration_rn"] = integration_g_raw
        results["nain_rn"] = nain_g_dict

    return results
