# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core: Visibility Graph Analysis (VGA) & Isovist Engine (Full depthmapX v0.8.0 Parity)
Implements Benedikt (1979) & Turner et al. (2001, 2004, 2007):
1. Watertight Grid Discretization (PointMap column-major ordering & Depthmap_Ref encoding)
2. C-Vectorized & Numba JIT Line-of-Sight (LOS) Adjacency Graph Construction
3. Analytical Corner-Ray 2D Isovist Solver (Single-Point & 3D Batch-Vectorized):
   - Isovist Area, Isovist Perimeter, Isovist Compactness, Isovist Occlusivity
   - Isovist Min Radial, Isovist Max Radial, Isovist Drift Magnitude, Isovist Drift Angle
4. Full VGA Local & Global Measures:
   - Connectivity, Point First Moment, Point Second Moment, Through vision
   - Visual Clustering Coefficient, Visual Control, Visual Controllability
   - Visual Integration [HH], Visual Integration [P-value], Visual Integration [Tekl]
   - Visual Mean Depth, Visual Node Count, Visual Entropy, Visual Relativised Entropy
"""

import math
from typing import List, Tuple, Dict, Any, Optional
import numpy as np
import shapely
from scipy.sparse import csr_matrix, _sparsetools
from scipy.sparse.csgraph import shortest_path
from shapely.geometry import Polygon, Point

from .solvers_axial import calculate_hillier_diamond_dn  # noqa: F401

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


def _iter_linear_components(geoms: Optional[List[Any]]):
    """Yields LineString/LinearRing components from lines, polygons, or multi-geometries."""
    if not geoms:
        return
    for g in geoms:
        if g is None or g.is_empty:
            continue
        gtype = getattr(g, "geom_type", "")
        if gtype in ("LineString", "LinearRing"):
            yield g
        elif gtype == "Polygon":
            if g.exterior is not None and not g.exterior.is_empty:
                yield g.exterior
            for ring in g.interiors:
                if ring is not None and not ring.is_empty:
                    yield ring
        elif hasattr(g, "geoms"):
            yield from _iter_linear_components(list(g.geoms))


def generate_vga_grid(
    boundary_geom: Any,
    cell_size_m: float = 1.0,
    obstacles: Optional[List[Any]] = None,
    depthmapx_align: bool = False
) -> List[Point]:
    """
    Generates uniform walkable grid points inside boundary_geom while excluding obstacles
    (supporting both LineString walls and Polygon/MultiPolygon building footprints).
    When depthmapx_align=True, aligns grid points to exact integer step multiples from
    (minx, miny) in column-major order matching depthmapX PointMap.
    """
    minx, miny, maxx, maxy = boundary_geom.bounds
    if depthmapx_align:
        x_coords = np.arange(minx + cell_size_m, maxx - 1e-9, cell_size_m)
        y_coords = np.arange(miny + cell_size_m, maxy - 1e-9, cell_size_m)
    else:
        x_coords = np.arange(minx + cell_size_m * 0.5, maxx, cell_size_m)
        y_coords = np.arange(miny + cell_size_m * 0.5, maxy, cell_size_m)

    obs_valid = [g for g in (obstacles or []) if g is not None and not g.is_empty]
    obs_union = shapely.union_all(obs_valid) if obs_valid else None
    shapely.prepare(boundary_geom)
    if obs_union is not None:
        shapely.prepare(obs_union)

    grid_points: List[Point] = []
    for x in x_coords:
        for y in y_coords:
            pt = Point(float(x), float(y))
            if boundary_geom.contains(pt):
                if obs_union is not None:
                    if obs_union.intersects(pt.buffer(1e-6)):
                        continue
                grid_points.append(pt)

    return grid_points


def _extract_wall_segments_array(obstacle_lines: Optional[Any]) -> np.ndarray:
    """
    Converts LineString / Polygon / MultiPolygon list into an (M, 2, 2) float64 array
    of 2-point wall segments. If already an (M, 2, 2) ndarray, returns it directly.
    """
    if isinstance(obstacle_lines, np.ndarray):
        if obstacle_lines.ndim == 3 and obstacle_lines.shape[1:] == (2, 2):
            return obstacle_lines
        if obstacle_lines.size == 0:
            return np.empty((0, 2, 2), dtype=np.float64)

    seg_list = []
    for line in _iter_linear_components(obstacle_lines):
        coords = list(line.coords)
        for k in range(len(coords) - 1):
            p0, p1 = coords[k], coords[k + 1]
            if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) > 1e-9:
                seg_list.append([[float(p0[0]), float(p0[1])], [float(p1[0]), float(p1[1])]])
    if not seg_list:
        return np.empty((0, 2, 2), dtype=np.float64)
    return np.asarray(seg_list, dtype=np.float64)


def compute_isovist_at_point(
    origin: Tuple[float, float],
    obstacle_lines: List[Any],
    max_radius_m: float = 100.0,
    ray_count: int = 360,
    segs: Optional[np.ndarray] = None
) -> Dict[str, Any]:
    """
    Computes exact analytical 2D Isovist polygon and Benedikt (1979) / depthmapX indicators
    at a single origin point:
    Area, Perimeter, Compactness, Occlusivity, Min/Max Radial, Drift Magnitude & Angle.
    """
    ox, oy = float(origin[0]), float(origin[1])
    if segs is None:
        segs = _extract_wall_segments_array(obstacle_lines)

    # Add bounding horizon box at max_radius_m so open spaces remain bounded
    R = float(max_radius_m)
    horizon_box = np.array([
        [[ox - R, oy - R], [ox + R, oy - R]],
        [[ox + R, oy - R], [ox + R, oy + R]],
        [[ox + R, oy + R], [ox - R, oy + R]],
        [[ox - R, oy + R], [ox - R, oy - R]],
    ], dtype=np.float64)
    all_segs = np.vstack([segs, horizon_box]) if len(segs) > 0 else horizon_box

    rel = all_segs - np.array([ox, oy], dtype=np.float64)
    P = rel[:, 0, :]
    D = rel[:, 1, :] - rel[:, 0, :]

    # Filter out edge-on collinear walls (zero angular span from origin)
    non_col = np.abs(P[:, 0] * D[:, 1] - P[:, 1] * D[:, 0]) > 1e-9
    rel = rel[non_col]
    P = P[non_col]
    D = D[non_col]

    angs = np.mod(np.arctan2(rel[:, :, 1], rel[:, :, 0]), 2.0 * math.pi)
    u_angs = np.unique(np.concatenate([angs.ravel(), [0.0, 2.0 * math.pi]]))
    u_angs.sort()
    keep = np.concatenate([[True], np.diff(u_angs) > 1e-11])
    u_angs = u_angs[keep]

    if len(u_angs) < 2:
        return {
            "area": 0.0, "perimeter": 0.0, "compactness": 0.0,
            "occlusivity": 0.0, "min_radial": 0.0, "max_radial": 0.0,
            "drift_dist": 0.0, "drift_angle": 0.0
        }

    mid_angs = 0.5 * (u_angs[:-1] + u_angs[1:])
    vm = np.stack([np.cos(mid_angs), np.sin(mid_angs)], axis=1)

    denom = D[:, 0:1] * vm[:, 1] - D[:, 1:2] * vm[:, 0]
    valid = np.abs(denom) > 1e-12
    safe_denom = np.where(valid, denom, 1.0)

    t = np.where(valid, (P[:, 1:2] * D[:, 0:1] - P[:, 0:1] * D[:, 1:2]) / safe_denom, 1e18)
    u_seg = np.where(valid, (P[:, 1:2] * vm[:, 0] - P[:, 0:1] * vm[:, 1]) / safe_denom, -1.0)

    mask = (t > 1e-9) & (u_seg >= -1e-9) & (u_seg <= 1.0 + 1e-9)
    win = np.argmin(np.where(mask, t, 1e18), axis=0)

    v0 = np.stack([np.cos(u_angs[:-1]), np.sin(u_angs[:-1])], axis=1)
    v1 = np.stack([np.cos(u_angs[1:]), np.sin(u_angs[1:])], axis=1)
    Dw = D[win]
    Pw = P[win]
    nw = Pw[:, 1] * Dw[:, 0] - Pw[:, 0] * Dw[:, 1]

    r0 = nw / (Dw[:, 0] * v0[:, 1] - Dw[:, 1] * v0[:, 0])
    r1 = nw / (Dw[:, 0] * v1[:, 1] - Dw[:, 1] * v1[:, 0])

    p0 = r0[:, None] * v0
    p1 = r1[:, None] * v1

    tri_area = 0.5 * np.abs(p0[:, 0] * p1[:, 1] - p0[:, 1] * p1[:, 0])
    area = float(np.sum(tri_area))
    occ = float(np.sum(np.abs(r1 - np.roll(r0, -1))))
    wall_perim = float(np.sum(np.linalg.norm(p1 - p0, axis=1)))
    perim = wall_perim + occ
    compactness = (4.0 * math.pi * area) / max(perim ** 2, 1e-12)

    cx = float(np.sum(tri_area * (p0[:, 0] + p1[:, 0]) / 3.0)) / max(area, 1e-12)
    cy = float(np.sum(tri_area * (p0[:, 1] + p1[:, 1]) / 3.0)) / max(area, 1e-12)
    drift_mag = math.hypot(cx, cy)
    drift_ang = math.degrees(math.atan2(cy, cx)) % 360.0

    max_radial = float(np.max(np.maximum(r0, r1)))
    seg_d = p1 - p0
    l2 = np.sum(seg_d ** 2, axis=1)
    proj = np.clip(-np.sum(p0 * seg_d, axis=1) / np.maximum(l2, 1e-12), 0.0, 1.0)
    closest = p0 + proj[:, None] * seg_d
    min_radial = float(np.min(np.linalg.norm(closest, axis=1)))

    # Check collinear walls whose endpoints may be closer than non-collinear segments
    col_segs = (all_segs - np.array([ox, oy], dtype=np.float64))[~non_col]
    for c_seg in col_segs:
        for end_pt in c_seg:
            d_end = float(math.hypot(end_pt[0], end_pt[1]))
            if 1e-9 < d_end < min_radial:
                ang_e = math.atan2(end_pt[1], end_pt[0]) % (2.0 * math.pi)
                sec_idx = min(max(int(np.searchsorted(u_angs, ang_e, side="right")) - 1, 0), len(r0) - 1)
                if d_end <= max(float(r0[sec_idx]), float(r1[sec_idx])) + 1e-6:
                    min_radial = d_end

    # Construct polygon coordinates in world space
    poly_coords = []
    for k in range(len(p0)):
        if k > 0 and np.linalg.norm(p1[k - 1] - p0[k]) > 1e-6:
            poly_coords.append((ox + float(p0[k, 0]), oy + float(p0[k, 1])))
        poly_coords.append((ox + float(p1[k, 0]), oy + float(p1[k, 1])))
    if len(poly_coords) >= 3:
        iso_poly = Polygon(poly_coords)
    else:
        iso_poly = Point(ox, oy).buffer(0.1)

    return {
        "polygon": iso_poly,
        "Isovist_Area": area,
        "Isovist_Perimeter": perim,
        "Isovist_Compactness": compactness,
        "Isovist_Occlusivity": occ,
        "Isovist_Min_Radial": min_radial,
        "Isovist_Max_Radial": max_radial,
        "Isovist_Drift_Magnitude": drift_mag,
        "Isovist_Drift_Angle": drift_ang,
        # Legacy aliases
        "area": round(area, 3),
        "perimeter": round(perim, 3),
        "compactness": round(compactness, 4),
        "occlusivity": round(occ, 3),
        "drift_x": round(cx, 3),
        "drift_y": round(cy, 3),
        "drift_dist": round(drift_mag, 3),
    }


def _compute_isovists_batch(
    coords: np.ndarray,
    wall_segs: np.ndarray,
    max_radius_m: float = 100.0,
    chunk_size: int = 512
) -> Dict[str, np.ndarray]:
    """
    3D Batch-Vectorized Analytical Isovist Solver across all N grid origins simultaneously.
    Preserves 100% IEEE-754 parity with compute_isovist_at_point.
    """
    N = len(coords)
    out = {
        "Isovist_Area": np.zeros(N, dtype=np.float64),
        "Isovist_Compactness": np.zeros(N, dtype=np.float64),
        "Isovist_Drift_Angle": np.zeros(N, dtype=np.float64),
        "Isovist_Drift_Magnitude": np.zeros(N, dtype=np.float64),
        "Isovist_Max_Radial": np.zeros(N, dtype=np.float64),
        "Isovist_Min_Radial": np.zeros(N, dtype=np.float64),
        "Isovist_Occlusivity": np.zeros(N, dtype=np.float64),
        "Isovist_Perimeter": np.zeros(N, dtype=np.float64),
    }
    if N == 0:
        return out

    R = float(max_radius_m)
    rel_hbox = np.array([
        [[-R, -R], [R, -R]],
        [[R, -R], [R, R]],
        [[R, R], [-R, R]],
        [[-R, R], [-R, -R]],
    ], dtype=np.float64)

    M = len(wall_segs)
    for start in range(0, N, chunk_size):
        end = min(N, start + chunk_size)
        B = end - start
        c_xy = coords[start:end]
        if M > 0:
            rel_w = wall_segs[None, :, :, :] - c_xy[:, None, None, :]
            rel_h = np.broadcast_to(rel_hbox[None, :, :, :], (B, 4, 2, 2))
            rel = np.concatenate([rel_w, rel_h], axis=1)
        else:
            rel = np.broadcast_to(rel_hbox[None, :, :, :], (B, 4, 2, 2)).copy()

        S = rel.shape[1]
        P = rel[:, :, 0, :]
        D = rel[:, :, 1, :] - P

        cross_pd = P[:, :, 0] * D[:, :, 1] - P[:, :, 1] * D[:, :, 0]
        non_col = np.abs(cross_pd) > 1e-9

        angs = np.mod(np.arctan2(rel[:, :, :, 1], rel[:, :, :, 0]), 2.0 * math.pi)
        angs = np.where(non_col[:, :, None], angs, 0.0)
        flat_angs = angs.reshape(B, 2 * S)
        zeros_2pi = np.broadcast_to(np.array([[0.0, 2.0 * math.pi]], dtype=np.float64), (B, 2))
        all_angs = np.concatenate([flat_angs, zeros_2pi], axis=1)
        all_angs.sort(axis=1)

        keep = np.ones_like(all_angs, dtype=bool)
        keep[:, 1:] = np.diff(all_angs, axis=1) > 1e-11
        n_uniq = np.sum(keep, axis=1)
        n_sec = n_uniq - 1

        comp_idx = np.argsort(~keep, axis=1, kind="mergesort")
        u_angs = np.take_along_axis(all_angs, comp_idx, axis=1)
        max_sec = int(np.max(n_sec))
        if max_sec <= 0:
            continue

        u0 = u_angs[:, :max_sec]
        u1 = u_angs[:, 1:max_sec + 1]
        sec_idx_grid = np.arange(max_sec)[None, :]
        valid_sec = sec_idx_grid < n_sec[:, None]
        u0 = np.where(valid_sec, u0, 0.0)
        u1 = np.where(valid_sec, u1, 1e-3)

        mid_angs = 0.5 * (u0 + u1)
        vm_x = np.cos(mid_angs)
        vm_y = np.sin(mid_angs)

        Dx = D[:, :, 0:1]
        Dy = D[:, :, 1:2]
        Px = P[:, :, 0:1]
        Py = P[:, :, 1:2]
        denom = Dx * vm_y[:, None, :] - Dy * vm_x[:, None, :]
        valid = (np.abs(denom) > 1e-12) & non_col[:, :, None]
        safe_denom = np.where(valid, denom, 1.0)

        num_t = Py * Dx - Px * Dy
        t = np.where(valid, num_t / safe_denom, 1e18)
        u_seg = np.where(valid, (Py * vm_x[:, None, :] - Px * vm_y[:, None, :]) / safe_denom, -1.0)

        mask = valid & (t > 1e-9) & (u_seg >= -1e-9) & (u_seg <= 1.0 + 1e-9)
        win = np.argmin(np.where(mask, t, 1e18), axis=1)

        v0_x = np.cos(u0)
        v0_y = np.sin(u0)
        v1_x = np.cos(u1)
        v1_y = np.sin(u1)

        Dw_x = np.take_along_axis(D[:, :, 0], win, axis=1)
        Dw_y = np.take_along_axis(D[:, :, 1], win, axis=1)
        Pw_x = np.take_along_axis(P[:, :, 0], win, axis=1)
        Pw_y = np.take_along_axis(P[:, :, 1], win, axis=1)
        nw = Pw_y * Dw_x - Pw_x * Dw_y

        r0 = nw / (Dw_x * v0_y - Dw_y * v0_x)
        r1 = nw / (Dw_x * v1_y - Dw_y * v1_x)
        r0 = np.where(valid_sec, r0, 0.0)
        r1 = np.where(valid_sec, r1, 0.0)

        p0_x = r0 * v0_x
        p0_y = r0 * v0_y
        p1_x = r1 * v1_x
        p1_y = r1 * v1_y

        tri_area = np.where(valid_sec, 0.5 * np.abs(p0_x * p1_y - p0_y * p1_x), 0.0)
        area = np.sum(tri_area, axis=1)

        r0_next = np.zeros_like(r0)
        r0_next[:, :-1] = r0[:, 1:]
        last_sec_idx = np.maximum(n_sec - 1, 0)[:, None]
        np.put_along_axis(r0_next, last_sec_idx, r0[:, 0:1], axis=1)
        occ = np.sum(np.where(valid_sec, np.abs(r1 - r0_next), 0.0), axis=1)

        seg_dx = p1_x - p0_x
        seg_dy = p1_y - p0_y
        l2 = seg_dx * seg_dx + seg_dy * seg_dy
        wall_perim = np.sum(np.where(valid_sec, np.sqrt(l2), 0.0), axis=1)
        perim = wall_perim + occ
        compactness = (4.0 * math.pi * area) / np.maximum(perim * perim, 1e-12)

        cx = np.sum(tri_area * (p0_x + p1_x) / 3.0, axis=1) / np.maximum(area, 1e-12)
        cy = np.sum(tri_area * (p0_y + p1_y) / 3.0, axis=1) / np.maximum(area, 1e-12)
        drift_mag = np.hypot(cx, cy)
        drift_ang = np.mod(np.degrees(np.arctan2(cy, cx)), 360.0)

        max_radial = np.max(np.where(valid_sec, np.maximum(r0, r1), 0.0), axis=1)
        proj = np.clip(-(p0_x * seg_dx + p0_y * seg_dy) / np.maximum(l2, 1e-12), 0.0, 1.0)
        cl_x = p0_x + proj * seg_dx
        cl_y = p0_y + proj * seg_dy
        min_radial = np.min(np.where(valid_sec, np.hypot(cl_x, cl_y), 1e18), axis=1)

        has_col = np.flatnonzero(np.any(~non_col, axis=1))
        for b in has_col:
            col_segs = rel[b, ~non_col[b]]
            ns_b = int(n_sec[b])
            u_b = u_angs[b, :ns_b + 1]
            for c_seg in col_segs:
                for end_pt in c_seg:
                    d_end = float(math.hypot(end_pt[0], end_pt[1]))
                    if 1e-9 < d_end < min_radial[b]:
                        ang_e = math.atan2(end_pt[1], end_pt[0]) % (2.0 * math.pi)
                        sec_i = min(max(int(np.searchsorted(u_b, ang_e, side="right")) - 1, 0), ns_b - 1)
                        if d_end <= max(float(r0[b, sec_i]), float(r1[b, sec_i])) + 1e-6:
                            min_radial[b] = d_end

        out["Isovist_Area"][start:end] = area
        out["Isovist_Compactness"][start:end] = compactness
        out["Isovist_Drift_Angle"][start:end] = drift_ang
        out["Isovist_Drift_Magnitude"][start:end] = drift_mag
        out["Isovist_Max_Radial"][start:end] = max_radial
        out["Isovist_Min_Radial"][start:end] = min_radial
        out["Isovist_Occlusivity"][start:end] = occ
        out["Isovist_Perimeter"][start:end] = perim

    return out


@njit(parallel=True, cache=True)
def _build_los_edges_numba(
    coords: np.ndarray,
    wall_segs: np.ndarray,
    max_vision_m: float
) -> np.ndarray:
    """
    Numba @njit(parallel=True) Line-of-Sight symmetric visibility matrix builder.
    Tests all pairs (i, j) against M wall segments using exact 1D-ratio and collinear intervals.
    """
    N = coords.shape[0]
    M = wall_segs.shape[0]
    vis = np.zeros((N, N), dtype=np.bool_)
    use_max_vis = max_vision_m > 0.0
    max_vis_sq = max_vision_m * max_vision_m
    eps = 1e-11

    for i in prange(N):
        xi = coords[i, 0]
        yi = coords[i, 1]
        for j in range(i + 1, N):
            xj = coords[j, 0]
            yj = coords[j, 1]
            if use_max_vis:
                dx = xi - xj
                dy = yi - yj
                if dx * dx + dy * dy > max_vis_sq:
                    continue
            blocked = False
            for m in range(M):
                wx1 = wall_segs[m, 0, 0]
                wy1 = wall_segs[m, 0, 1]
                wx2 = wall_segs[m, 1, 0]
                wy2 = wall_segs[m, 1, 1]
                vx = wx2 - wx1
                vy = wy2 - wy1
                dx1_i = xi - wx1
                dy1_i = yi - wy1
                dx1_j = xj - wx1
                dy1_j = yj - wy1
                si = vx * dy1_i - vy * dx1_i
                sj = vx * dy1_j - vy * dx1_j
                if abs(vx) >= abs(vy):
                    ui = dx1_i / vx
                    uj = dx1_j / vx
                else:
                    ui = dy1_i / vy
                    uj = dy1_j / vy

                if si > eps and sj < -eps:
                    ap = (ui + 1e-9) / si
                    an = (uj + 1e-9) / sj
                    bp = (ui - 1.0 - 1e-9) / si
                    bn = (uj - 1.0 - 1e-9) / sj
                    if ap >= an and bp <= bn:
                        blocked = True
                        break
                elif si < -eps and sj > eps:
                    ap = (uj + 1e-9) / sj
                    an = (ui + 1e-9) / si
                    bp = (uj - 1.0 - 1e-9) / sj
                    bn = (ui - 1.0 - 1e-9) / si
                    if ap >= an and bp <= bn:
                        blocked = True
                        break
                else:
                    if abs(si) <= eps and -1e-9 <= ui <= 1.0 + 1e-9:
                        blocked = True
                        break
                    if abs(sj) <= eps and -1e-9 <= uj <= 1.0 + 1e-9:
                        blocked = True
                        break
                    if abs(si) <= eps and abs(sj) <= eps:
                        if max(ui, uj) >= -1e-9 and min(ui, uj) <= 1.0 + 1e-9:
                            blocked = True
                            break
            if not blocked:
                vis[i, j] = True
                vis[j, i] = True
    return vis


def _build_los_edges_vec(
    coords: np.ndarray,
    wall_segs: np.ndarray,
    max_vision_m: Optional[float] = None
) -> np.ndarray:
    """
    C-Vectorized 1D-Ratio Line-of-Sight symmetric visibility matrix builder.
    Eliminates N*(N-1)/2 Shapely LineString heap allocations by slicing positive/negative
    half-spaces per wall segment.
    """
    N = len(coords)
    x = coords[:, 0]
    y = coords[:, 1]
    vis = np.ones((N, N), dtype=bool)
    if max_vision_m is not None and max_vision_m > 0:
        dx = x[:, None] - x[None, :]
        dy = y[:, None] - y[None, :]
        vis &= (dx * dx + dy * dy) <= (float(max_vision_m) ** 2)

    eps = 1e-11
    for m in range(len(wall_segs)):
        wx1, wy1 = wall_segs[m, 0, 0], wall_segs[m, 0, 1]
        wx2, wy2 = wall_segs[m, 1, 0], wall_segs[m, 1, 1]
        vx = wx2 - wx1
        vy = wy2 - wy1
        dx1 = x - wx1
        dy1 = y - wy1
        s = vx * dy1 - vy * dx1
        u = (dx1 / vx) if abs(vx) >= abs(vy) else (dy1 / vy)
        pos = np.flatnonzero(s > eps)
        neg = np.flatnonzero(s < -eps)
        if len(pos) > 0 and len(neg) > 0:
            sp = s[pos]
            sn = s[neg]
            up = u[pos]
            un = u[neg]
            ap = (up + 1e-9) / sp
            an = (un + 1e-9) / sn
            bp = (up - 1.0 - 1e-9) / sp
            bn = (un - 1.0 - 1e-9) / sn
            hit = (ap[:, None] >= an[None, :]) & (bp[:, None] <= bn[None, :])
            r_h, c_h = np.nonzero(hit)
            vis[pos[r_h], neg[c_h]] = False
        zero = np.flatnonzero(np.abs(s) <= eps)
        if len(zero) > 0:
            uz = u[zero]
            on_seg = zero[(uz >= -1e-9) & (uz <= 1.0 + 1e-9)]
            if len(on_seg) > 0:
                vis[on_seg, :] = False
            if len(zero) > 1:
                z_hit = (
                    (np.maximum(uz[:, None], uz[None, :]) >= -1e-9)
                    & (np.minimum(uz[:, None], uz[None, :]) <= 1.0 + 1e-9)
                )
                rz, cz = np.nonzero(z_hit)
                vis[zero[rz], zero[cz]] = False

    vis &= vis.T
    np.fill_diagonal(vis, False)
    return vis


@njit(parallel=True, cache=True)
def _solve_vga_bfs_metrics_numba(
    indptr: np.ndarray,
    indices: np.ndarray,
    N: int
) -> Tuple[
    np.ndarray, np.ndarray, np.ndarray, np.ndarray,
    np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray
]:
    """
    Numba @njit(parallel=True) unweighted CSR BFS kernel over all N grid points.
    Computes visual_controllability, node_count, total_depth, mean_depth,
    visual_integration_hh, visual_integration_pv, visual_integration_tk,
    visual_entropy, and visual_rel_entropy in parallel C loops.
    """
    visual_controllability = np.full(N, -1.0, dtype=np.float64)
    node_count = np.ones(N, dtype=np.float64)
    total_depth = np.zeros(N, dtype=np.float64)
    mean_depth = np.full(N, -1.0, dtype=np.float64)
    visual_integration_hh = np.full(N, -1.0, dtype=np.float64)
    visual_integration_pv = np.full(N, -1.0, dtype=np.float64)
    visual_integration_tk = np.full(N, -1.0, dtype=np.float64)
    visual_entropy = np.full(N, -1.0, dtype=np.float64)
    visual_rel_entropy = np.full(N, -1.0, dtype=np.float64)

    for i in prange(N):
        dist = np.full(N, -1, dtype=np.int32)
        queue = np.empty(N, dtype=np.int32)
        depth_counts = np.zeros(N, dtype=np.int32)

        dist[i] = 0
        queue[0] = i
        head = 0
        tail = 1
        max_d = 0
        td = 0

        while head < tail:
            u = queue[head]
            head += 1
            du_next = dist[u] + 1
            for idx_e in range(indptr[u], indptr[u + 1]):
                v = indices[idx_e]
                if dist[v] == -1:
                    dist[v] = du_next
                    queue[tail] = v
                    tail += 1
                    depth_counts[du_next] += 1
                    td += du_next
                    if du_next > max_d:
                        max_d = du_next

        reach = tail - 1
        nc = float(1 + reach)
        node_count[i] = nc
        total_depth[i] = float(td)

        c1 = float(depth_counts[1]) if max_d >= 1 else 0.0
        c2 = float(depth_counts[2]) if max_d >= 2 else 0.0
        if (c1 + c2) > 0.0:
            visual_controllability[i] = c1 / (1.0 + c1 + c2)

        if reach > 0:
            md = float(td) / float(reach)
            mean_depth[i] = md
            if nc > 2.0 and md > 1.0:
                ra = 2.0 * (md - 1.0) / (nc - 2.0)
                num_dn = 2.0 * (nc * (math.log2((nc + 2.0) / 3.0) - 1.0) + 1.0)
                den_dn = (nc - 1.0) * (nc - 2.0)
                dn = num_dn / den_dn
                if dn < 1e-12:
                    dn = 1e-12
                ihh = dn / ra
                visual_integration_hh[i] = ihh
                visual_integration_pv[i] = ihh
                if float(td) > nc:
                    visual_integration_tk[i] = math.log(0.5 * (nc - 2.0)) / math.log(float(td) - nc + 1.0)

                ent = 0.0
                rel_ent = 0.0
                fact = 1.0
                for d_step in range(1, max_d + 1):
                    idx_b = d_step - 1
                    fact *= float(d_step)
                    cnt = depth_counts[d_step]
                    if cnt > 0:
                        prob = float(cnt) / nc
                        ent -= prob * math.log2(prob)
                        q = ((md ** idx_b) / fact) * math.exp(-md)
                        if q > 0.0:
                            rel_ent += prob * math.log2(prob / q)
                visual_entropy[i] = ent
                visual_rel_entropy[i] = rel_ent

    return (
        visual_controllability,
        node_count,
        total_depth,
        mean_depth,
        visual_integration_hh,
        visual_integration_pv,
        visual_integration_tk,
        visual_entropy,
        visual_rel_entropy,
    )


def _solve_vga_bfs_metrics_vec(
    vis_sym: np.ndarray,
    u_arr: np.ndarray,
    v_arr: np.ndarray
) -> Tuple[
    np.ndarray, np.ndarray, np.ndarray, np.ndarray,
    np.ndarray, np.ndarray, np.ndarray, np.ndarray,
    np.ndarray, np.ndarray, np.ndarray, np.ndarray
]:
    """
    BLAS SGEMM Frontier BFS + Vectorized VGA Global Metrics & Clustering Solver.
    Uses a single float32 BLAS matrix product M2 = M @ M to simultaneously obtain
    both Visual_Clustering_Coefficient triangles and Depth-2 reachability, then
    propagates remaining frontier rows in C/BLAS without Python node loops.
    """
    N = len(vis_sym)
    M = vis_sym.astype(np.float32)
    connectivity = np.sum(vis_sym, axis=1).astype(np.float64)
    inv_k = np.where(connectivity > 0, 1.0 / np.maximum(connectivity, 1.0), 0.0)
    visual_control = vis_sym @ inv_k

    # 1st BLAS SGEMM gives both triangle counts and Depth-2 shortest paths
    M2 = M @ M
    triangles_twice = np.einsum("ij,ij->i", M2, M, dtype=np.float64)
    denom_cc = connectivity * (connectivity - 1.0)
    visual_clustering = np.where(connectivity > 1.0, triangles_twice / np.maximum(denom_cc, 1.0), -1.0)

    reached = vis_sym.copy()
    np.fill_diagonal(reached, True)
    front = (M2 > 0.5) & (~reached)
    counts_list = [connectivity.astype(np.int32)]

    d = 2
    while np.any(front):
        counts_list.append(np.sum(front, axis=1, dtype=np.int32))
        reached |= front
        if np.all(reached):
            break
        active_rows = np.flatnonzero(np.any(front, axis=1) & (~np.all(reached, axis=1)))
        if len(active_rows) == 0:
            break
        if d >= 14:
            # High-diameter maze fallback: finish remaining rows via C shortest_path
            row_idx = np.concatenate([u_arr, v_arr])
            col_idx = np.concatenate([v_arr, u_arr])
            A = csr_matrix((np.ones(len(row_idx), dtype=np.float64), (row_idx, col_idx)), shape=(N, N))
            D_mat = shortest_path(A, directed=False, unweighted=True)
            finite_mask = np.isfinite(D_mat) & (D_mat > 0)
            max_d_full = int(np.max(D_mat[finite_mask])) if np.any(finite_mask) else 1
            counts_list = [np.sum(D_mat == step_d, axis=1, dtype=np.int32) for step_d in range(1, max_d_full + 1)]
            break
        next_front = np.zeros((N, N), dtype=bool)
        next_front[active_rows] = ((front[active_rows].astype(np.float32) @ M) > 0.5) & (~reached[active_rows])
        front = next_front
        d += 1

    counts_2d = np.column_stack(counts_list) if len(counts_list) > 1 else counts_list[0][:, None]
    c1 = counts_2d[:, 0].astype(np.float64)
    c2 = counts_2d[:, 1].astype(np.float64) if counts_2d.shape[1] >= 2 else np.zeros(N, dtype=np.float64)
    visual_controllability = np.where((c1 + c2) > 0, c1 / (1.0 + c1 + c2), -1.0)

    reach_count = np.sum(counts_2d, axis=1)
    node_count = (1 + reach_count).astype(np.float64)
    depths_vec = np.arange(1, counts_2d.shape[1] + 1, dtype=np.float64)
    total_depth = counts_2d.astype(np.float64) @ depths_vec
    mean_depth = np.where(reach_count > 0, total_depth / np.maximum(reach_count, 1), -1.0)

    valid = (node_count > 2.0) & (mean_depth > 1.0)
    nc_s = np.where(valid, node_count, 3.0)
    md_s = np.where(valid, mean_depth, 2.0)
    ra = 2.0 * (md_s - 1.0) / (nc_s - 2.0)
    dn = 2.0 * (nc_s * (np.log2((nc_s + 2.0) / 3.0) - 1.0) + 1.0) / ((nc_s - 1.0) * (nc_s - 2.0))
    dn = np.maximum(dn, 1e-12)
    ihh = np.where(valid, dn / ra, -1.0)
    visual_integration_hh = ihh
    visual_integration_pv = ihh.copy()

    valid_tk = valid & (total_depth > node_count)
    td_s = np.where(valid_tk, total_depth, nc_s + 2.0)
    visual_integration_tk = np.where(
        valid_tk,
        np.log(0.5 * (nc_s - 2.0)) / np.log(td_s - nc_s + 1.0),
        -1.0
    )

    max_d = counts_2d.shape[1]
    prob = counts_2d.astype(np.float64) / nc_s[:, None]
    pos_p = counts_2d > 0
    safe_p = np.where(pos_p, prob, 1.0)
    ent = -np.sum(np.where(pos_p, prob * np.log2(safe_p), 0.0), axis=1)

    idx_b = np.arange(max_d, dtype=np.float64)[None, :]
    fact = np.cumprod(np.arange(1, max_d + 1, dtype=np.float64))[None, :]
    q = ((md_s[:, None] ** idx_b) / fact) * np.exp(-md_s[:, None])
    pos_q = pos_p & (q > 0.0)
    safe_q = np.where(pos_q, q, 1.0)
    rel_ent = np.sum(np.where(pos_q, prob * np.log2(safe_p / safe_q), 0.0), axis=1)

    visual_entropy = np.where(valid, ent, -1.0)
    visual_rel_entropy = np.where(valid, rel_ent, -1.0)

    return (
        connectivity,
        visual_control,
        visual_clustering,
        visual_controllability,
        node_count,
        total_depth,
        mean_depth,
        visual_integration_hh,
        visual_integration_pv,
        visual_integration_tk,
        visual_entropy,
        visual_rel_entropy,
    )


@njit(cache=True)
def _compute_through_vision_numba(
    u_arr: np.ndarray,
    v_arr: np.ndarray,
    ix: np.ndarray,
    iy: np.ndarray,
    off_x: int,
    off_y: int,
    grid_w: int,
    grid_h: int
) -> np.ndarray:
    """
    Numba @njit Through Vision kernel implementing exact depthmapX
    PixelBase::quickPixelateLine IEEE-754 parity.
    """
    N = len(ix)
    tv_grid = np.zeros((grid_w, grid_h), dtype=np.int64)
    M = len(u_arr)

    for e_idx in range(M):
        u = int(u_arr[e_idx])
        v = int(v_arr[e_idx])
        for dir_flag in range(2):
            if dir_flag == 0:
                px, py, qx, qy = int(ix[u]), int(iy[u]), int(ix[v]), int(iy[v])
            else:
                px, py, qx, qy = int(ix[v]), int(iy[v]), int(ix[u]), int(iy[u])
            dx_l = float(qx - px)
            dy_l = float(qy - py)
            if abs(dx_l) == abs(dy_l):
                continue
            elif abs(dx_l) > abs(dy_l):
                t_steps = int(abs(dx_l))
                pol = 1
            else:
                t_steps = int(abs(dy_l))
                pol = 2
            dx_l /= float(t_steps)
            dy_l /= float(t_steps)
            ppx = float(px) + 0.5 + dx_l
            ppy = float(py) + 0.5 + dy_l
            for _ in range(1, t_steps):
                if pol == 1 and abs(math.floor(ppy) - ppy) < 1e-9:
                    tv_grid[int(math.floor(ppx)) + off_x, int(math.floor(ppy + 0.5)) + off_y] += 1
                    tv_grid[int(math.floor(ppx)) + off_x, int(math.floor(ppy - 0.5)) + off_y] += 1
                elif pol == 2 and abs(math.floor(ppx) - ppx) < 1e-9:
                    tv_grid[int(math.floor(ppx + 0.5)) + off_x, int(math.floor(ppy)) + off_y] += 1
                    tv_grid[int(math.floor(ppx - 0.5)) + off_x, int(math.floor(ppy)) + off_y] += 1
                else:
                    tv_grid[int(math.floor(ppx)) + off_x, int(math.floor(ppy)) + off_y] += 1
                ppx += dx_l
                ppy += dy_l

    out = np.empty(N, dtype=np.float64)
    for i in range(N):
        out[i] = float(tv_grid[int(ix[i]) + off_x, int(iy[i]) + off_y])
    return out


def _accumulate_axis_tv_vec(
    u_maj: np.ndarray,
    u_min: np.ndarray,
    v_maj: np.ndarray,
    v_min: np.ndarray,
    W_maj: int,
    H_min: int,
    off_min: int
) -> np.ndarray:
    """
    Key-compressed C-vectorized helper for Through Vision along one dominant axis.
    Compresses O(N^2) edges into O(W * H^2) unique (t_steps, y1, y2) triples and
    accumulates both forward and reverse traces via shifted 1D histograms.
    Guarantees 100.0% exact IEEE-754 parity with PixelBase::quickPixelateLine.
    """
    out_T = np.zeros((H_min, W_maj), dtype=np.int64)
    if len(u_maj) == 0:
        return out_T.T

    dmaj = v_maj - u_maj
    swap = dmaj < 0
    x1 = np.where(swap, v_maj, u_maj)
    y1 = np.where(swap, v_min, u_min)
    x2 = np.where(swap, u_maj, v_maj)
    y2 = np.where(swap, u_min, v_min)
    t = x2 - x1
    max_t = int(np.max(t)) + 1

    n_keys = max_t * H_min * H_min
    key = (t * H_min + y1) * H_min + y2
    active_keys = np.flatnonzero(np.bincount(key, minlength=n_keys) > 0)
    K = len(active_keys)
    if K == 0:
        return out_T.T

    remap = np.empty(n_keys, dtype=np.int32)
    remap[active_keys] = np.arange(K, dtype=np.int32)
    X = np.bincount(remap[key] * W_maj + x1, minlength=K * W_maj).reshape((K, W_maj))

    qy_k = active_keys % H_min
    rem = active_keys // H_min
    py_k = rem % H_min
    kt = rem // H_min
    order = np.argsort(-kt, kind="mergesort")
    kt = kt[order]
    py_k = py_k[order]
    qy_k = qy_k[order]
    X = np.ascontiguousarray(X[order], dtype=np.int64)
    X_dbl = np.ascontiguousarray(X * 2, dtype=np.int64)

    max_step = int(kt[0])
    py_orig = (py_k - off_min).astype(np.float64)
    qy_orig = (qy_k - off_min).astype(np.float64)
    kt_flt = kt.astype(np.float64)
    dy_fwd = (qy_orig - py_orig) / kt_flt
    dy_rev = (py_orig - qy_orig) / kt_flt

    iy_fwd = np.zeros((max_step, K), dtype=np.int32)
    tie_fwd = np.zeros((max_step, K), dtype=bool)
    iy_rev_al = np.zeros((max_step, K), dtype=np.int32)
    tie_rev_al = np.zeros((max_step, K), dtype=bool)

    ppy_f = py_orig + 0.5 + dy_fwd
    ppy_r = qy_orig + 0.5 + dy_rev
    neg_kt = -kt
    lims = np.empty(max_step, dtype=np.int32)
    row_arange = np.arange(K, dtype=np.int32)
    for s in range(1, max_step):
        lim = int(np.searchsorted(neg_kt, -s, side="left"))
        lims[s] = lim
        if lim <= 0:
            break
        cf = ppy_f[:lim]
        ff = np.floor(cf)
        iy_fwd[s, :lim] = ff.astype(np.int32) + off_min
        tie_fwd[s, :lim] = (cf - ff) < 1e-9

        cr = ppy_r[:lim]
        fr = np.floor(cr)
        s_al = kt[:lim] - s
        r_sub = row_arange[:lim]
        iy_rev_al[s_al, r_sub] = fr.astype(np.int32) + off_min
        tie_rev_al[s_al, r_sub] = (cr - fr) < 1e-9

        ppy_f[:lim] += dy_fwd[:lim]
        ppy_r[:lim] += dy_rev[:lim]

    indptr = np.arange(K + 1, dtype=np.int32)
    ones = np.ones(K, dtype=np.int64)
    Y_buf = np.zeros((H_min, W_maj), dtype=np.int64)
    for s in range(1, max_step):
        lim = int(lims[s])
        if lim <= 0:
            break
        y_f = iy_fwd[s, :lim]
        y_r = iy_rev_al[s, :lim]
        Y_buf.fill(0)
        _sparsetools.csc_matvecs(
            H_min, lim, W_maj, indptr[:lim + 1], y_f, ones[:lim], X_dbl[:lim], Y_buf
        )
        diff_mask = y_f != y_r
        if np.any(diff_mask):
            d_idx = np.flatnonzero(diff_mask)
            np.add.at(Y_buf, y_f[d_idx], -X[d_idx])
            np.add.at(Y_buf, y_r[d_idx], X[d_idx])
        tf = tie_fwd[s, :lim]
        if np.any(tf):
            idx1 = np.flatnonzero(tf)
            np.add.at(Y_buf, y_f[idx1] - 1, X[idx1])
        tr = tie_rev_al[s, :lim]
        if np.any(tr):
            idx2 = np.flatnonzero(tr)
            np.add.at(Y_buf, y_r[idx2] - 1, X[idx2])
        out_T[:, s:] += Y_buf[:, :W_maj - s]

    return out_T.T


def _compute_through_vision_vec(
    u_arr: np.ndarray,
    v_arr: np.ndarray,
    ix: np.ndarray,
    iy: np.ndarray,
    off_x: int,
    off_y: int,
    grid_w: int,
    grid_h: int
) -> np.ndarray:
    """C-Vectorized Through Vision solver with 100% IEEE-754 parity."""
    N = len(ix)
    if len(u_arr) == 0:
        return np.zeros(N, dtype=np.float64)

    gx = ix + off_x
    gy = iy + off_y
    ux = gx[u_arr]
    uy = gy[u_arr]
    vx = gx[v_arr]
    vy = gy[v_arr]
    adx = np.abs(vx - ux)
    ady = np.abs(vy - uy)

    m1 = (adx > ady) & (adx >= 2)
    m2 = (ady > adx) & (ady >= 2)

    grid1 = _accumulate_axis_tv_vec(ux[m1], uy[m1], vx[m1], vy[m1], grid_w, grid_h, off_y)
    grid2 = _accumulate_axis_tv_vec(uy[m2], ux[m2], vy[m2], vx[m2], grid_h, grid_w, off_x).T
    tv_grid = grid1 + grid2
    return tv_grid[gx, gy].astype(np.float64)


def solve_vga_visibility_graph(
    grid_points: List[Point],
    obstacle_lines: List[Any],
    max_vision_m: Optional[float] = None,
    include_isovist: bool = True,
    include_through_vision: bool = True,
    cell_size_m: float = 1.0,
    origin_xy: Optional[Tuple[float, float]] = None
) -> Dict[str, Any]:
    """
    Builds Line-of-Sight visibility graph and solves full
    depthmapX v0.8.0 VGA + Isovist schema:
    - Depthmap_Ref, Connectivity, Point_First_Moment, Point_Second_Moment
    - Through_vision
    - Visual_Clustering_Coefficient, Visual_Control, Visual_Controllability
    - Visual_Integration_HH_, Visual_Integration_P_value_, Visual_Integration_Tekl_
    - Visual_Mean_Depth, Visual_Node_Count, Visual_Entropy, Visual_Relativised_Entropy
    - Isovist_Area, Isovist_Compactness, Isovist_Drift_Angle, Isovist_Drift_Magnitude,
      Isovist_Max_Radial, Isovist_Min_Radial, Isovist_Occlusivity, Isovist_Perimeter
    """
    N = len(grid_points)
    if N == 0:
        return {}

    coords = np.array([[float(pt.x), float(pt.y)] for pt in grid_points], dtype=np.float64)

    if origin_xy is not None:
        x0, y0 = float(origin_xy[0]), float(origin_xy[1])
    else:
        x0 = math.floor(float(np.min(coords[:, 0])) / cell_size_m) * cell_size_m
        y0 = math.floor(float(np.min(coords[:, 1])) / cell_size_m) * cell_size_m

    ix = np.round((coords[:, 0] - x0) / cell_size_m).astype(np.int32)
    iy = np.round((coords[:, 1] - y0) / cell_size_m).astype(np.int32)
    depthmap_ref = (ix << 16) + iy

    # Pre-extract 2-point wall segments once (supports LineString, Polygon, MultiPolygon)
    wall_segs = _extract_wall_segments_array(obstacle_lines)

    # 1. C-Vectorized / Numba Line-of-Sight Visibility Graph Construction
    if HAS_NUMBA and len(wall_segs) > 0:
        max_vis_val = float(max_vision_m) if (max_vision_m is not None and max_vision_m > 0) else -1.0
        vis_sym = _build_los_edges_numba(coords, wall_segs, max_vis_val)
    else:
        vis_sym = _build_los_edges_vec(coords, wall_segs, max_vision_m=max_vision_m)

    u_arr, v_arr = np.nonzero(np.triu(vis_sym, k=1))
    u_arr = u_arr.astype(np.int32)
    v_arr = v_arr.astype(np.int32)
    vis_dists = np.hypot(coords[u_arr, 0] - coords[v_arr, 0], coords[u_arr, 1] - coords[v_arr, 1])

    # 2. Point First & Second Moments
    first_moment = (
        np.bincount(u_arr, weights=vis_dists, minlength=N)
        + np.bincount(v_arr, weights=vis_dists, minlength=N)
    )
    vis_dists_sq = vis_dists ** 2
    second_moment = (
        np.bincount(u_arr, weights=vis_dists_sq, minlength=N)
        + np.bincount(v_arr, weights=vis_dists_sq, minlength=N)
    )

    # 3 & 4. Visual Control, Clustering, All-Pairs BFS, Integration & Entropy
    if HAS_NUMBA:
        row_idx = np.concatenate([u_arr, v_arr])
        col_idx = np.concatenate([v_arr, u_arr])
        A = csr_matrix((np.ones(len(row_idx), dtype=np.float64), (row_idx, col_idx)), shape=(N, N))
        connectivity = np.array(A.sum(axis=1)).ravel().astype(np.float64)
        inv_k = np.where(connectivity > 0, 1.0 / np.maximum(connectivity, 1.0), 0.0)
        visual_control = A.dot(inv_k)
        M_f32 = vis_sym.astype(np.float32)
        M2_f32 = M_f32 @ M_f32
        triangles_twice = np.einsum("ij,ij->i", M2_f32, M_f32, dtype=np.float64)
        denom_cc = connectivity * (connectivity - 1.0)
        visual_clustering = np.where(connectivity > 1.0, triangles_twice / np.maximum(denom_cc, 1.0), -1.0)
        (
            visual_controllability,
            node_count,
            total_depth,
            mean_depth,
            visual_integration_hh,
            visual_integration_pv,
            visual_integration_tk,
            visual_entropy,
            visual_rel_entropy,
        ) = _solve_vga_bfs_metrics_numba(A.indptr.astype(np.int32), A.indices.astype(np.int32), N)
    else:
        (
            connectivity,
            visual_control,
            visual_clustering,
            visual_controllability,
            node_count,
            total_depth,
            mean_depth,
            visual_integration_hh,
            visual_integration_pv,
            visual_integration_tk,
            visual_entropy,
            visual_rel_entropy,
        ) = _solve_vga_bfs_metrics_vec(vis_sym, u_arr, v_arr)

    # 5. Through Vision (exact depthmapX PixelBase::quickPixelateLine IEEE-754 parity)
    through_vision = np.zeros(N, dtype=np.float64)
    if include_through_vision and len(u_arr) > 0:
        max_ix = int(np.max(ix)) + 3
        max_iy = int(np.max(iy)) + 3
        min_ix = min(0, int(np.min(ix)))
        min_iy = min(0, int(np.min(iy)))
        off_x = -min_ix
        off_y = -min_iy
        grid_w = max_ix + off_x + 2
        grid_h = max_iy + off_y + 2
        if HAS_NUMBA:
            through_vision = _compute_through_vision_numba(
                u_arr, v_arr, ix, iy, off_x, off_y, grid_w, grid_h
            )
        else:
            through_vision = _compute_through_vision_vec(
                u_arr, v_arr, ix, iy, off_x, off_y, grid_w, grid_h
            )

    # 6. Analytical Isovist Properties across all grid points
    if include_isovist and len(wall_segs) > 0:
        iso_cols = _compute_isovists_batch(coords, wall_segs)
    else:
        iso_cols = {
            "Isovist_Area": np.zeros(N, dtype=np.float64),
            "Isovist_Compactness": np.zeros(N, dtype=np.float64),
            "Isovist_Drift_Angle": np.zeros(N, dtype=np.float64),
            "Isovist_Drift_Magnitude": np.zeros(N, dtype=np.float64),
            "Isovist_Max_Radial": np.zeros(N, dtype=np.float64),
            "Isovist_Min_Radial": np.zeros(N, dtype=np.float64),
            "Isovist_Occlusivity": np.zeros(N, dtype=np.float64),
            "Isovist_Perimeter": np.zeros(N, dtype=np.float64),
        }

    results: Dict[str, Any] = {
        "Depthmap_Ref": depthmap_ref,
        "Connectivity": connectivity,
        "Isovist_Area": iso_cols["Isovist_Area"],
        "Isovist_Compactness": iso_cols["Isovist_Compactness"],
        "Isovist_Drift_Angle": iso_cols["Isovist_Drift_Angle"],
        "Isovist_Drift_Magnitude": iso_cols["Isovist_Drift_Magnitude"],
        "Isovist_Max_Radial": iso_cols["Isovist_Max_Radial"],
        "Isovist_Min_Radial": iso_cols["Isovist_Min_Radial"],
        "Isovist_Occlusivity": iso_cols["Isovist_Occlusivity"],
        "Isovist_Perimeter": iso_cols["Isovist_Perimeter"],
        "Point_First_Moment": first_moment,
        "Point_Second_Moment": second_moment,
        "Through_vision": through_vision,
        "Visual_Clustering_Coefficient": visual_clustering,
        "Visual_Control": visual_control,
        "Visual_Controllability": visual_controllability,
        "Visual_Integration_HH_": visual_integration_hh,
        "Visual_Integration_P_value_": visual_integration_pv,
        "Visual_Integration_Tekl_": visual_integration_tk,
        "Visual_Mean_Depth": mean_depth,
        "Visual_Node_Count": node_count,
        "Visual_Entropy": visual_entropy,
        "Visual_Relativised_Entropy": visual_rel_entropy,
        # Legacy aliases for backwards compatibility with existing scripts/tests
        "vga_connectivity": connectivity.astype(np.int32),
        "vga_total_depth": np.round(total_depth, 2),
        "vga_mean_depth": np.round(mean_depth, 3),
        "vga_integration_hh": np.round(visual_integration_hh, 3),
        "vga_control": np.round(visual_control, 4),
        "vga_clustering_coef": np.round(visual_clustering, 4),
    }
    return results
