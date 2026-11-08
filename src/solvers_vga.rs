//! Visibility Graph Analysis (`VGA`) & Analytical 2D Benedikt (1979) Isovist Solver.
//!
//! Full mathematical parity with UCL `depthmapX v0.8.0` (`salalib/vgamodules/`, `salalib/isovist.cpp`,
//! `salalib/pixelbase.cpp`, `salalib/pointdata.h`).

use crate::types::{IsovistResult, Point2D, Polyline, VgaGridPoint, VgaResults};
use rayon::prelude::*;
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::f64::consts::PI;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Instant;

/// Maximum interactive VGA grid cell count so accidental ultra-fine slider values (e.g. `0.05 m`
/// on a large building) automatically scale effective grid spacing instead of truncating the
/// floorplan mid-room or triggering billion-pair `O(N²)` stalls.
pub const MAX_INTERACTIVE_VGA_CELLS: usize = 8_500;

/// Exact UCL `depthmapX` (`salalib/pointdata.h` / `pafagent.cpp`) 32-bin octant direction quantizer (`0..31`).
#[inline]
pub fn whichbin(dx: f64, dy: f64) -> u8 {
    let mut b: i32 = if dy.abs() > dx.abs() { 1 } else { 0 };
    let ratio = if b == 0 {
        if dx.abs() < 1e-15 {
            return 0;
        }
        let r = dy.abs() / dx.abs();
        if dx > 0.0 {
            b = if dy >= 0.0 { 0 } else { -32 };
        } else {
            b = if dy >= 0.0 { -16 } else { 16 };
        }
        r
    } else {
        let r = dx.abs() / dy.abs();
        if dy > 0.0 {
            b = if dx >= 0.0 { -8 } else { 8 };
        } else {
            b = if dx >= 0.0 { 24 } else { -24 };
        }
        r
    };

    if ratio < 1e-12 {
        // +0
    } else if ratio < 0.2679491924311227 {
        // < tan(15 deg)
        b += 1;
    } else if ratio < 0.5773502691896257 {
        // < tan(30 deg)
        b += 2;
    } else if ratio < 1.0 - 1e-12 {
        // < tan(45 deg)
        b += 3;
    } else {
        b += 4;
    }

    if b < 0 {
        b = -b;
    }
    (b.rem_euclid(32)) as u8
}

#[derive(Clone, Copy)]
struct WallSeg {
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

fn extract_wall_segments(obstacles: &[Polyline]) -> Vec<WallSeg> {
    let mut segs = Vec::new();
    for poly in obstacles {
        for w in poly.coords.windows(2) {
            let p0 = w[0];
            let p1 = w[1];
            if p0.dist(p1) > 1e-9 {
                segs.push(WallSeg {
                    x1: p0.x,
                    y1: p0.y,
                    x2: p1.x,
                    y2: p1.y,
                    min_x: p0.x.min(p1.x),
                    min_y: p0.y.min(p1.y),
                    max_x: p0.x.max(p1.x),
                    max_y: p0.y.max(p1.y),
                });
            }
        }
    }
    // Sort by min_x ascending so all ray/segment queries can break early once w.min_x > query_max_x
    segs.sort_by(|a, b| {
        a.min_x
            .partial_cmp(&b.min_x)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    segs
}

#[inline]
fn point_to_segment_dist(p: Point2D, a: Point2D, b: Point2D) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let l2 = dx * dx + dy * dy;
    if l2 <= 1e-18 {
        return p.dist(a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / l2).clamp(0.0, 1.0);
    let proj = Point2D::new(a.x + t * dx, a.y + t * dy);
    p.dist(proj)
}

#[inline]
fn point_in_ring(pt: Point2D, ring: &[Point2D]) -> bool {
    if ring.len() < 3 {
        return false;
    }
    let mut inside = false;
    let n = ring.len();
    let mut j = n - 1;
    for i in 0..n {
        let pi = ring[i];
        let pj = ring[j];
        let intersects = ((pi.y > pt.y) != (pj.y > pt.y))
            && (pt.x < (pj.x - pi.x) * (pt.y - pi.y) / (pj.y - pi.y) + pi.x);
        if intersects {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[inline]
fn ring_bbox_area(ring: &[Point2D]) -> (f64, f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for p in ring {
        min_x = min_x.min(p.x);
        min_y = min_y.min(p.y);
        max_x = max_x.max(p.x);
        max_y = max_y.max(p.y);
    }
    let area = (max_x - min_x).max(0.0) * (max_y - min_y).max(0.0);
    (min_x, min_y, max_x, max_y, area)
}

#[inline]
fn segments_intersect_inclusive(
    a0: Point2D,
    a1: Point2D,
    b0: Point2D,
    b1: Point2D,
    eps: f64,
) -> bool {
    let min_ax = a0.x.min(a1.x) - eps;
    let max_ax = a0.x.max(a1.x) + eps;
    let min_ay = a0.y.min(a1.y) - eps;
    let max_ay = a0.y.max(a1.y) + eps;

    let min_bx = b0.x.min(b1.x);
    let max_bx = b0.x.max(b1.x);
    let min_by = b0.y.min(b1.y);
    let max_by = b0.y.max(b1.y);

    if max_bx < min_ax || min_bx > max_ax || max_by < min_ay || min_by > max_ay {
        return false;
    }

    let dxa = a1.x - a0.x;
    let dya = a1.y - a0.y;
    let dxb = b1.x - b0.x;
    let dyb = b1.y - b0.y;
    let denom = dxa * dyb - dya * dxb;

    if denom.abs() > 1e-12 {
        let t = ((b0.x - a0.x) * dyb - (b0.y - a0.y) * dxb) / denom;
        let u = ((b0.x - a0.x) * dya - (b0.y - a0.y) * dxa) / denom;
        t >= -eps && t <= 1.0 + eps && u >= -eps && u <= 1.0 + eps
    } else {
        point_to_segment_dist(a0, b0, b1) <= eps
            || point_to_segment_dist(a1, b0, b1) <= eps
            || point_to_segment_dist(b0, a0, a1) <= eps
            || point_to_segment_dist(b1, a0, a1) <= eps
    }
}

/// Exact UCL `depthmapX` 4-connected wall-blocking flood-fill VGA grid generator (`PointMap::fillGrid`).
///
/// Starting from `seed`, performs a 4-connected cardinal BFS on grid coordinates `(ix * step, iy * step)`
/// blocked by any wall segment in `obstacles`, then sorts cells in column-major `(ix, iy)` order.
/// If the requested `cell_size_m` would exceed `MAX_INTERACTIVE_VGA_CELLS` (`8,500` cells) before
/// filling the connected floorplan, `step` is automatically widened so the entire floorplan is
/// covered without room truncation or `O(N²)` solver stalls.
pub fn generate_vga_grid_flood_fill(
    obstacles: &[Polyline],
    seed: Point2D,
    cell_size_m: f64,
) -> Vec<VgaGridPoint> {
    let mut step = cell_size_m.max(1e-3);
    let wall_segs = extract_wall_segments(obstacles);
    if wall_segs.is_empty() {
        return Vec::new();
    }

    let mut raw_min_x = f64::INFINITY;
    let mut raw_min_y = f64::INFINITY;
    let mut raw_max_x = f64::NEG_INFINITY;
    let mut raw_max_y = f64::NEG_INFINITY;
    for w in &wall_segs {
        raw_min_x = raw_min_x.min(w.min_x);
        raw_min_y = raw_min_y.min(w.min_y);
        raw_max_x = raw_max_x.max(w.max_x);
        raw_max_y = raw_max_y.max(w.max_y);
    }
    if !raw_min_x.is_finite() {
        return Vec::new();
    }

    const DIRS: [(i32, i32); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];
    let mut final_visited: HashSet<(i32, i32)> = HashSet::new();

    for _attempt in 0..12 {
        let min_x = raw_min_x - step * 2.0;
        let min_y = raw_min_y - step * 2.0;
        let max_x = raw_max_x + step * 2.0;
        let max_y = raw_max_y + step * 2.0;

        let ix0 = (seed.x / step).round() as i32;
        let iy0 = (seed.y / step).round() as i32;

        let mut visited: HashSet<(i32, i32)> =
            HashSet::with_capacity(MAX_INTERACTIVE_VGA_CELLS.min(4096));
        let mut queue: VecDeque<(i32, i32)> = VecDeque::new();
        visited.insert((ix0, iy0));
        queue.push_back((ix0, iy0));
        let mut overflowed = false;

        while let Some((ix, iy)) = queue.pop_front() {
            if visited.len() >= MAX_INTERACTIVE_VGA_CELLS {
                overflowed = true;
                break;
            }
            let x0 = (ix as f64) * step;
            let y0 = (iy as f64) * step;
            let p0 = Point2D::new(x0, y0);

            for &(dx, dy) in &DIRS {
                let nx = ix + dx;
                let ny = iy + dy;
                if visited.contains(&(nx, ny)) {
                    continue;
                }
                let x1 = (nx as f64) * step;
                let y1 = (ny as f64) * step;
                if x1 < min_x || x1 > max_x || y1 < min_y || y1 > max_y {
                    continue;
                }
                let p1 = Point2D::new(x1, y1);
                let seg_min_x = x0.min(x1) - 1e-4;
                let seg_max_x = x0.max(x1) + 1e-4;
                let seg_min_y = y0.min(y1) - 1e-4;
                let seg_max_y = y0.max(y1) + 1e-4;

                let mut blocked = false;
                for w in &wall_segs {
                    if w.min_x > seg_max_x {
                        break;
                    }
                    if w.max_x < seg_min_x || w.max_y < seg_min_y || w.min_y > seg_max_y {
                        continue;
                    }
                    let w0 = Point2D::new(w.x1, w.y1);
                    let w1 = Point2D::new(w.x2, w.y2);
                    if segments_intersect_inclusive(p0, p1, w0, w1, 1e-9)
                        || point_to_segment_dist(p1, w0, w1) < 1e-5
                    {
                        blocked = true;
                        break;
                    }
                }

                if !blocked {
                    visited.insert((nx, ny));
                    queue.push_back((nx, ny));
                }
            }
        }

        final_visited = visited;
        if !overflowed {
            break;
        }
        // Automatically step up grid spacing so the full connected floorplan fits within MAX_INTERACTIVE_VGA_CELLS
        step = ((step * 1.30) * 100.0).ceil() / 100.0;
    }

    let mut coords: Vec<(i32, i32)> = final_visited.into_iter().collect();
    coords.sort_unstable();

    let min_ix = coords.iter().map(|c| c.0).min().unwrap_or(0);
    let min_iy = coords.iter().map(|c| c.1).min().unwrap_or(0);

    coords
        .into_iter()
        .enumerate()
        .map(|(id, (ix, iy))| VgaGridPoint {
            id,
            x: (ix as f64) * step,
            y: (iy as f64) * step,
            grid_i: ix - min_ix,
            grid_j: iy - min_iy,
            depthmap_ref: (ix << 16) | (iy & 0xFFFF),
        })
        .collect()
}

/// Returns the actual effective grid cell spacing in meters inferred from `points` (or `fallback_step`).
pub fn effective_vga_grid_spacing(points: &[VgaGridPoint], fallback_step: f64) -> f64 {
    for w in points.windows(2) {
        let di = (w[1].grid_i - w[0].grid_i).abs();
        let dj = (w[1].grid_j - w[0].grid_j).abs();
        if di + dj == 1 {
            let d = Point2D::new(w[0].x, w[0].y).dist(Point2D::new(w[1].x, w[1].y));
            if d > 1e-5 {
                return d;
            }
        }
    }
    fallback_step.max(1e-3)
}

/// Discretizes walkable grid points inside `boundary_rings[0]` (or via 4-connected flood-fill
/// when `boundary_rings` specifies a single seed point `[[seed]]`) in column-major order
/// matching UCL `depthmapX` `PointMap`.
pub fn generate_vga_grid(
    boundary_rings: &[Vec<Point2D>],
    obstacles: &[Polyline],
    cell_size_m: f64,
    depthmapx_align: bool,
) -> Vec<VgaGridPoint> {
    let step = cell_size_m.max(1e-3);

    if boundary_rings.len() == 1 && boundary_rings[0].len() == 1 {
        return generate_vga_grid_flood_fill(obstacles, boundary_rings[0][0], step);
    }

    if boundary_rings.is_empty() && !obstacles.is_empty() {
        let mut b_min_x = f64::INFINITY;
        let mut b_min_y = f64::INFINITY;
        let mut b_max_x = f64::NEG_INFINITY;
        let mut b_max_y = f64::NEG_INFINITY;
        for obs in obstacles {
            for p in &obs.coords {
                b_min_x = b_min_x.min(p.x);
                b_min_y = b_min_y.min(p.y);
                b_max_x = b_max_x.max(p.x);
                b_max_y = b_max_y.max(p.y);
            }
        }
        if b_min_x.is_finite() && b_max_x > b_min_x && b_max_y > b_min_y {
            let center = Point2D::new(0.5 * (b_min_x + b_max_x), 0.5 * (b_min_y + b_max_y));
            let ff = generate_vga_grid_flood_fill(obstacles, center, step);
            if ff.len() >= 4 {
                return ff;
            }
        }
    }

    // Derive exterior boundary ring if boundary_rings is empty
    let derived_exterior: Vec<Point2D>;
    let exterior_ring: &[Point2D] = if !boundary_rings.is_empty() && boundary_rings[0].len() >= 3 {
        &boundary_rings[0]
    } else {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for obs in obstacles {
            for p in &obs.coords {
                min_x = min_x.min(p.x);
                min_y = min_y.min(p.y);
                max_x = max_x.max(p.x);
                max_y = max_y.max(p.y);
            }
        }
        if !min_x.is_finite() || (max_x - min_x) < 1e-6 || (max_y - min_y) < 1e-6 {
            min_x = 0.0;
            min_y = 0.0;
            max_x = 20.0;
            max_y = 20.0;
        }
        derived_exterior = vec![
            Point2D::new(min_x, min_y),
            Point2D::new(max_x, min_y),
            Point2D::new(max_x, max_y),
            Point2D::new(min_x, max_y),
            Point2D::new(min_x, min_y),
        ];
        &derived_exterior
    };

    let (min_x, min_y, max_x, max_y, ext_area) = ring_bbox_area(exterior_ring);
    if !min_x.is_finite() || max_x <= min_x || max_y <= min_y {
        return Vec::new();
    }

    let hole_rings: &[Vec<Point2D>] = if boundary_rings.len() > 1 {
        &boundary_rings[1..]
    } else {
        &[]
    };

    // Identify closed interior obstacle polygons (e.g. pillars/courtyards strictly smaller than exterior)
    let mut closed_obstacle_rings: Vec<&[Point2D]> = Vec::new();
    for obs in obstacles {
        if obs.coords.len() >= 4 {
            let first = obs.coords[0];
            let last = *obs.coords.last().unwrap();
            if first.dist(last) <= 1e-3 {
                let (_, _, _, _, obs_area) = ring_bbox_area(&obs.coords);
                if obs_area < ext_area * 0.90 {
                    closed_obstacle_rings.push(&obs.coords);
                }
            }
        }
    }

    let wall_segs = extract_wall_segments(obstacles);
    let mut eff_step = step;
    while ((max_x - min_x) / eff_step) * ((max_y - min_y) / eff_step)
        > (MAX_INTERACTIVE_VGA_CELLS * 2) as f64
    {
        eff_step = ((eff_step * 1.30) * 100.0).ceil() / 100.0;
    }

    loop {
        let mut xs = Vec::new();
        let mut ys = Vec::new();
        if depthmapx_align {
            let mut x = min_x + eff_step;
            while x < max_x - 1e-9 {
                xs.push(x);
                x += eff_step;
            }
            let mut y = min_y + eff_step;
            while y < max_y - 1e-9 {
                ys.push(y);
                y += eff_step;
            }
        } else {
            let mut x = min_x + eff_step * 0.5;
            while x < max_x - 1e-9 {
                xs.push(x);
                x += eff_step;
            }
            let mut y = min_y + eff_step * 0.5;
            while y < max_y - 1e-9 {
                ys.push(y);
                y += eff_step;
            }
        }

        if xs.is_empty() || ys.is_empty() {
            return Vec::new();
        }

        let x0 = (xs[0] / eff_step).floor() * eff_step;
        let y0 = (ys[0] / eff_step).floor() * eff_step;

        let mut points = Vec::new();
        let mut exceeded_cap = false;
        // Column-major order (outer X, inner Y) matching depthmapX PointMap
        'col_loop: for &x in &xs {
            let gi = ((x - x0) / eff_step).round() as i32;
            for &y in &ys {
                if points.len() >= MAX_INTERACTIVE_VGA_CELLS {
                    exceeded_cap = true;
                    break 'col_loop;
                }
                let pt = Point2D::new(x, y);
                if !point_in_ring(pt, exterior_ring) {
                    continue;
                }
                if hole_rings.iter().any(|h| point_in_ring(pt, h)) {
                    continue;
                }
                if closed_obstacle_rings.iter().any(|r| point_in_ring(pt, r)) {
                    continue;
                }
                // Exclude points within 1e-4m of any exterior boundary edge or obstacle wall segment
                let mut near_wall = false;
                for w in exterior_ring.windows(2) {
                    if point_to_segment_dist(pt, w[0], w[1]) < 1e-4 {
                        near_wall = true;
                        break;
                    }
                }
                if near_wall {
                    continue;
                }
                for s in &wall_segs {
                    if s.min_x > pt.x + 1e-3 {
                        break;
                    }
                    if pt.x >= s.min_x - 1e-3
                        && pt.x <= s.max_x + 1e-3
                        && pt.y >= s.min_y - 1e-3
                        && pt.y <= s.max_y + 1e-3
                        && point_to_segment_dist(
                            pt,
                            Point2D::new(s.x1, s.y1),
                            Point2D::new(s.x2, s.y2),
                        ) < 1e-4
                    {
                        near_wall = true;
                        break;
                    }
                }
                if near_wall {
                    continue;
                }

                let gj = ((y - y0) / eff_step).round() as i32;
                let depthmap_ref = (gi << 16) | (gj & 0xFFFF);
                let id = points.len();
                points.push(VgaGridPoint {
                    id,
                    x,
                    y,
                    grid_i: gi,
                    grid_j: gj,
                    depthmap_ref,
                });
            }
        }

        if !exceeded_cap || eff_step >= 5.0 {
            return points;
        }
        eff_step = ((eff_step * 1.30) * 100.0).ceil() / 100.0;
    }
}

/// Computes the analytical 2D Benedikt (1979) corner-ray Isovist polygon and indicators from `origin`.
pub fn compute_isovist_at_point(
    origin: Point2D,
    obstacles: &[Polyline],
    max_radius_m: f64,
) -> IsovistResult {
    let ox = origin.x;
    let oy = origin.y;
    let r_bound = if max_radius_m.is_finite() && max_radius_m > 0.1 {
        max_radius_m
    } else {
        100.0
    };

    let mut all_segs: Vec<([f64; 2], [f64; 2])> = Vec::new();
    for obs in obstacles {
        for w in obs.coords.windows(2) {
            if w[0].dist(w[1]) > 1e-9 {
                all_segs.push(([w[0].x - ox, w[0].y - oy], [w[1].x - ox, w[1].y - oy]));
            }
        }
    }
    // Add bounding horizon box at r_bound
    all_segs.push(([-r_bound, -r_bound], [r_bound, -r_bound]));
    all_segs.push(([r_bound, -r_bound], [r_bound, r_bound]));
    all_segs.push(([r_bound, r_bound], [-r_bound, r_bound]));
    all_segs.push(([-r_bound, r_bound], [-r_bound, -r_bound]));

    let mut non_col_segs: Vec<([f64; 2], [f64; 2])> = Vec::with_capacity(all_segs.len());
    let mut col_segs: Vec<([f64; 2], [f64; 2])> = Vec::new();
    let mut raw_angs: Vec<f64> = Vec::with_capacity(all_segs.len() * 2 + 2);
    raw_angs.push(0.0);
    raw_angs.push(2.0 * PI);

    for &(p0, p1) in &all_segs {
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let cross = p0[0] * dy - p0[1] * dx;
        if cross.abs() > 1e-9 {
            non_col_segs.push((p0, [dx, dy]));
            let a0 = p0[1].atan2(p0[0]).rem_euclid(2.0 * PI);
            let a1 = p1[1].atan2(p1[0]).rem_euclid(2.0 * PI);
            raw_angs.push(a0);
            raw_angs.push(a1);
        } else {
            col_segs.push((p0, p1));
        }
    }

    raw_angs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mut u_angs: Vec<f64> = Vec::with_capacity(raw_angs.len());
    for a in raw_angs {
        if u_angs.last().map_or(true, |&prev| (a - prev) > 1e-11) {
            u_angs.push(a);
        }
    }

    if u_angs.len() < 2 || non_col_segs.is_empty() {
        return IsovistResult {
            origin,
            polygon: Vec::new(),
            area: 0.0,
            perimeter: 0.0,
            compactness: 0.0,
            occlusivity: 0.0,
            min_radial: 0.0,
            max_radial: 0.0,
            drift_magnitude: 0.0,
            drift_angle_deg: 0.0,
            drift_target: origin,
        };
    }

    let n_sec = u_angs.len() - 1;
    let mut r0_vec = Vec::with_capacity(n_sec);
    let mut r1_vec = Vec::with_capacity(n_sec);
    let mut p0_vec: Vec<[f64; 2]> = Vec::with_capacity(n_sec);
    let mut p1_vec: Vec<[f64; 2]> = Vec::with_capacity(n_sec);

    for k in 0..n_sec {
        let a0 = u_angs[k];
        let a1 = u_angs[k + 1];
        let mid = 0.5 * (a0 + a1);
        let vmx = mid.cos();
        let vmy = mid.sin();

        let mut best_t = 1e18;
        let mut best_idx = 0usize;

        for (idx, &(p, d)) in non_col_segs.iter().enumerate() {
            let denom = d[0] * vmy - d[1] * vmx;
            if denom.abs() > 1e-12 {
                let t = (p[1] * d[0] - p[0] * d[1]) / denom;
                let u_seg = (p[1] * vmx - p[0] * vmy) / denom;
                if t > 1e-9 && (-1e-9..=1.0 + 1e-9).contains(&u_seg) && t < best_t {
                    best_t = t;
                    best_idx = idx;
                }
            }
        }

        let (pw, dw) = non_col_segs[best_idx];
        let nw = pw[1] * dw[0] - pw[0] * dw[1];
        let v0 = [a0.cos(), a0.sin()];
        let v1 = [a1.cos(), a1.sin()];
        let den0 = dw[0] * v0[1] - dw[1] * v0[0];
        let den1 = dw[0] * v1[1] - dw[1] * v1[0];
        let r0 = if den0.abs() > 1e-15 { (nw / den0).max(0.0) } else { best_t };
        let r1 = if den1.abs() > 1e-15 { (nw / den1).max(0.0) } else { best_t };

        r0_vec.push(r0);
        r1_vec.push(r1);
        p0_vec.push([r0 * v0[0], r0 * v0[1]]);
        p1_vec.push([r1 * v1[0], r1 * v1[1]]);
    }

    let mut area = 0.0;
    let mut occlusivity = 0.0;
    let mut wall_perim = 0.0;
    let mut cx_num = 0.0;
    let mut cy_num = 0.0;
    let mut max_radial = 0.0_f64;
    let mut min_radial = f64::INFINITY;

    for k in 0..n_sec {
        let p0 = p0_vec[k];
        let p1 = p1_vec[k];
        let r0 = r0_vec[k];
        let r1 = r1_vec[k];
        let r0_next = r0_vec[(k + 1) % n_sec];

        let tri_area = 0.5 * (p0[0] * p1[1] - p0[1] * p1[0]).abs();
        area += tri_area;
        occlusivity += (r1 - r0_next).abs();

        let sdx = p1[0] - p0[0];
        let sdy = p1[1] - p0[1];
        let l2 = sdx * sdx + sdy * sdy;
        wall_perim += l2.sqrt();

        cx_num += tri_area * (p0[0] + p1[0]) / 3.0;
        cy_num += tri_area * (p0[1] + p1[1]) / 3.0;

        max_radial = max_radial.max(r0.max(r1));
        let proj = (-(p0[0] * sdx + p0[1] * sdy) / l2.max(1e-12)).clamp(0.0, 1.0);
        let clx = p0[0] + proj * sdx;
        let cly = p0[1] + proj * sdy;
        min_radial = min_radial.min(clx.hypot(cly));
    }

    // Check collinear wall endpoints
    for &(c0, c1) in &col_segs {
        for end_pt in [c0, c1] {
            let d_end = end_pt[0].hypot(end_pt[1]);
            if d_end > 1e-9 && d_end < min_radial {
                let ang_e = end_pt[1].atan2(end_pt[0]).rem_euclid(2.0 * PI);
                let pos = u_angs.partition_point(|&u| u <= ang_e);
                let sec_idx = pos.saturating_sub(1).min(n_sec - 1);
                if d_end <= r0_vec[sec_idx].max(r1_vec[sec_idx]) + 1e-6 {
                    min_radial = d_end;
                }
            }
        }
    }
    if !min_radial.is_finite() {
        min_radial = 0.0;
    }

    let perimeter = wall_perim + occlusivity;
    let compactness = ((4.0 * PI * area) / (perimeter * perimeter).max(1e-12)).clamp(0.0, 1.0);
    let cx = cx_num / area.max(1e-12);
    let cy = cy_num / area.max(1e-12);
    let drift_magnitude = cx.hypot(cy);
    let drift_angle_deg = cy.atan2(cx).to_degrees().rem_euclid(360.0);
    let drift_target = Point2D::new(ox + cx, oy + cy);

    let mut polygon = Vec::with_capacity(n_sec * 2);
    for k in 0..n_sec {
        if k > 0 {
            let prev = p1_vec[k - 1];
            let cur = p0_vec[k];
            if (prev[0] - cur[0]).hypot(prev[1] - cur[1]) > 1e-6 {
                polygon.push(Point2D::new(ox + cur[0], oy + cur[1]));
            }
        } else {
            polygon.push(Point2D::new(ox + p0_vec[0][0], oy + p0_vec[0][1]));
        }
        polygon.push(Point2D::new(ox + p1_vec[k][0], oy + p1_vec[k][1]));
    }

    IsovistResult {
        origin,
        polygon,
        area,
        perimeter,
        compactness,
        occlusivity,
        min_radial,
        max_radial,
        drift_magnitude,
        drift_angle_deg,
        drift_target,
    }
}

#[inline]
fn los_visible(xi: f64, yi: f64, xj: f64, yj: f64, walls: &[WallSeg]) -> bool {
    let ray_min_x = xi.min(xj) - 1e-9;
    let ray_min_y = yi.min(yj) - 1e-9;
    let ray_max_x = xi.max(xj) + 1e-9;
    let ray_max_y = yi.max(yj) + 1e-9;
    let eps = 1e-11;

    for w in walls {
        if w.min_x > ray_max_x {
            break;
        }
        if w.max_x < ray_min_x || w.max_y < ray_min_y || w.min_y > ray_max_y {
            continue;
        }
        let vx = w.x2 - w.x1;
        let vy = w.y2 - w.y1;
        let dx1_i = xi - w.x1;
        let dy1_i = yi - w.y1;
        let dx1_j = xj - w.x1;
        let dy1_j = yj - w.y1;
        let si = vx * dy1_i - vy * dx1_i;
        let sj = vx * dy1_j - vy * dx1_j;
        let (ui, uj) = if vx.abs() >= vy.abs() {
            (dx1_i / vx, dx1_j / vx)
        } else {
            (dy1_i / vy, dy1_j / vy)
        };

        if si > eps && sj < -eps {
            let ap = (ui + 1e-9) / si;
            let an = (uj + 1e-9) / sj;
            let bp = (ui - 1.0 - 1e-9) / si;
            let bn = (uj - 1.0 - 1e-9) / sj;
            if ap >= an && bp <= bn {
                return false;
            }
        } else if si < -eps && sj > eps {
            let ap = (uj + 1e-9) / sj;
            let an = (ui + 1e-9) / si;
            let bp = (uj - 1.0 - 1e-9) / sj;
            let bn = (ui - 1.0 - 1e-9) / si;
            if ap >= an && bp <= bn {
                return false;
            }
        } else {
            if si.abs() <= eps && (-1e-9..=1.0 + 1e-9).contains(&ui) {
                return false;
            }
            if sj.abs() <= eps && (-1e-9..=1.0 + 1e-9).contains(&uj) {
                return false;
            }
            if si.abs() <= eps && sj.abs() <= eps && ui.max(uj) >= -1e-9 && ui.min(uj) <= 1.0 + 1e-9
            {
                return false;
            }
        }
    }
    true
}

/// Builds an instant (`< 1.5 ms`) 8-connected walkable Moore grid lattice on `points` before running
/// the full all-pairs VGA visibility graph (matching UCL `depthmapX` Step 1: Set Grid + Fill Grid,
/// allowing immediate manual agent placement on the created grid).
pub fn build_walkable_vga_grid_lattice(
    points: &[VgaGridPoint],
    obstacles: &[Polyline],
    cell_size_m: f64,
) -> VgaResults {
    let t0 = Instant::now();
    let n = points.len();
    let eff_cell_m = effective_vga_grid_spacing(points, cell_size_m);
    if n == 0 {
        return VgaResults {
            points: Vec::new(),
            cell_size_m: eff_cell_m,
            row_offsets: vec![0],
            col_indices: Vec::new(),
            bin_indices: Vec::new(),
            attributes: BTreeMap::new(),
            elapsed_ms: 0.0,
        };
    }

    let walls = extract_wall_segments(obstacles);
    let mut coord_to_idx: HashMap<(i32, i32), usize> = HashMap::with_capacity(n * 2);
    for (idx, p) in points.iter().enumerate() {
        coord_to_idx.insert((p.grid_i, p.grid_j), idx);
    }

    let mut row_offsets = Vec::with_capacity(n + 1);
    row_offsets.push(0u32);
    let mut col_indices = Vec::with_capacity(n * 8);
    let mut bin_indices = Vec::with_capacity(n * 8);
    let mut connectivity = vec![0.0_f64; n];

    const MOORE_DIRS: [(i32, i32); 8] = [
        (1, 0),
        (1, 1),
        (0, 1),
        (-1, 1),
        (-1, 0),
        (-1, -1),
        (0, -1),
        (1, -1),
    ];

    for (i, pi) in points.iter().enumerate() {
        let mut nbrs: Vec<(u32, u8)> = Vec::with_capacity(8);
        for &(di, dj) in &MOORE_DIRS {
            if let Some(&j) = coord_to_idx.get(&(pi.grid_i + di, pi.grid_j + dj)) {
                let pj = points[j];
                if los_visible(pi.x, pi.y, pj.x, pj.y, &walls) {
                    let bin = whichbin(di as f64, dj as f64);
                    nbrs.push((j as u32, bin));
                }
            }
        }
        nbrs.sort_unstable_by_key(|k| k.0);
        connectivity[i] = nbrs.len() as f64;
        for (v, b) in nbrs {
            col_indices.push(v);
            bin_indices.push(b);
        }
        row_offsets.push(col_indices.len() as u32);
    }

    let mut attributes = BTreeMap::new();
    attributes.insert(
        "Depthmap_Ref".to_string(),
        points.iter().map(|p| p.depthmap_ref as f64).collect(),
    );
    attributes.insert("Connectivity".to_string(), connectivity);
    attributes.insert("Gate_Counts".to_string(), vec![0.0; n]);

    VgaResults {
        points: points.to_vec(),
        cell_size_m: eff_cell_m,
        row_offsets,
        col_indices,
        bin_indices,
        attributes,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
    }
}

fn compute_through_vision(points: &[VgaGridPoint], row_offsets: &[u32], col_indices: &[u32]) -> Vec<f64> {
    let n = points.len();
    if n == 0 || col_indices.is_empty() {
        return vec![0.0; n];
    }

    let mut min_ix = 0i32;
    let mut min_iy = 0i32;
    let mut max_ix = 0i32;
    let mut max_iy = 0i32;
    for p in points {
        min_ix = min_ix.min(p.grid_i);
        min_iy = min_iy.min(p.grid_j);
        max_ix = max_ix.max(p.grid_i);
        max_iy = max_iy.max(p.grid_j);
    }
    let off_x = -min_ix + 2;
    let off_y = -min_iy + 2;
    let grid_w = (max_ix + off_x + 4) as usize;
    let grid_h = (max_iy + off_y + 4) as usize;

    let mut tv_grid = vec![0i64; grid_w * grid_h];

    for u in 0..n {
        let start = row_offsets[u] as usize;
        let end = row_offsets[u + 1] as usize;
        let px = points[u].grid_i;
        let py = points[u].grid_j;
        for &v_u32 in &col_indices[start..end] {
            let v = v_u32 as usize;
            let qx = points[v].grid_i;
            let qy = points[v].grid_j;
            let dx_i = qx - px;
            let dy_i = qy - py;
            if dx_i.abs() == dy_i.abs() {
                continue;
            }
            let (t_steps, pol) = if dx_i.abs() > dy_i.abs() {
                (dx_i.abs(), 1)
            } else {
                (dy_i.abs(), 2)
            };
            if t_steps <= 1 {
                continue;
            }
            let dx_l = (dx_i as f64) / (t_steps as f64);
            let dy_l = (dy_i as f64) / (t_steps as f64);
            let mut ppx = (px as f64) + 0.5 + dx_l;
            let mut ppy = (py as f64) + 0.5 + dy_l;
            for _ in 1..t_steps {
                if pol == 1 && (ppy.floor() - ppy).abs() < 1e-9 {
                    let gx = (ppx.floor() as i32 + off_x) as usize;
                    let gy1 = ((ppy + 0.5).floor() as i32 + off_y) as usize;
                    let gy2 = ((ppy - 0.5).floor() as i32 + off_y) as usize;
                    if gx < grid_w && gy1 < grid_h {
                        tv_grid[gx * grid_h + gy1] += 1;
                    }
                    if gx < grid_w && gy2 < grid_h {
                        tv_grid[gx * grid_h + gy2] += 1;
                    }
                } else if pol == 2 && (ppx.floor() - ppx).abs() < 1e-9 {
                    let gx1 = ((ppx + 0.5).floor() as i32 + off_x) as usize;
                    let gx2 = ((ppx - 0.5).floor() as i32 + off_x) as usize;
                    let gy = (ppy.floor() as i32 + off_y) as usize;
                    if gx1 < grid_w && gy < grid_h {
                        tv_grid[gx1 * grid_h + gy] += 1;
                    }
                    if gx2 < grid_w && gy < grid_h {
                        tv_grid[gx2 * grid_h + gy] += 1;
                    }
                } else {
                    let gx = (ppx.floor() as i32 + off_x) as usize;
                    let gy = (ppy.floor() as i32 + off_y) as usize;
                    if gx < grid_w && gy < grid_h {
                        tv_grid[gx * grid_h + gy] += 1;
                    }
                }
                ppx += dx_l;
                ppy += dy_l;
            }
        }
    }

    points
        .iter()
        .map(|p| {
            let gx = (p.grid_i + off_x) as usize;
            let gy = (p.grid_j + off_y) as usize;
            tv_grid[gx * grid_h + gy] as f64
        })
        .collect()
}

#[inline]
fn sorted_slice_intersection_count(a: &[u32], b: &[u32]) -> usize {
    let mut i = 0usize;
    let mut j = 0usize;
    let mut count = 0usize;
    while i < a.len() && j < b.len() {
        let va = a[i];
        let vb = b[j];
        if va == vb {
            count += 1;
            i += 1;
            j += 1;
        } else if va < vb {
            i += 1;
        } else {
            j += 1;
        }
    }
    count
}

/// Solves Visibility Graph Analysis (`VGA`) and optional Global BFS & Analytical Isovist indicators.
pub fn solve_vga(
    points: &[VgaGridPoint],
    obstacles: &[Polyline],
    cell_size_m: f64,
    max_visibility_m: f64,
    compute_global: bool,
    compute_isovist_metrics: bool,
) -> VgaResults {
    solve_vga_with_progress(
        points,
        obstacles,
        cell_size_m,
        max_visibility_m,
        compute_global,
        compute_isovist_metrics,
        None,
        None,
    )
    .unwrap_or_else(|| VgaResults {
        points: Vec::new(),
        cell_size_m,
        row_offsets: vec![0],
        col_indices: Vec::new(),
        bin_indices: Vec::new(),
        attributes: BTreeMap::new(),
        elapsed_ms: 0.0,
    })
}

/// Non-blocking, progress-reporting & cancellable Visibility Graph Analysis (`VGA`) solver.
///
/// Exploits upper-triangle Line-of-Sight symmetry (`j in (i + 1)..n`), sorted wall early-exit,
/// bounded clustering sampling on ultra-high-degree cells, and log-space Poisson relative entropy
/// (`UL-03` factorial overflow guard). Returns `None` if `cancel_flag` is triggered.
pub fn solve_vga_with_progress(
    points: &[VgaGridPoint],
    obstacles: &[Polyline],
    cell_size_m: f64,
    max_visibility_m: f64,
    compute_global: bool,
    compute_isovist_metrics: bool,
    progress_pct: Option<&AtomicUsize>,
    cancel_flag: Option<&AtomicBool>,
) -> Option<VgaResults> {
    let t0 = Instant::now();
    let n = points.len();
    let eff_cell_m = effective_vga_grid_spacing(points, cell_size_m);
    if n == 0 {
        if let Some(p) = progress_pct {
            p.store(100, Ordering::Relaxed);
        }
        return Some(VgaResults {
            points: Vec::new(),
            cell_size_m: eff_cell_m,
            row_offsets: vec![0],
            col_indices: Vec::new(),
            bin_indices: Vec::new(),
            attributes: BTreeMap::new(),
            elapsed_ms: 0.0,
        });
    }

    let is_cancelled = || cancel_flag.map_or(false, |c| c.load(Ordering::Relaxed));
    let set_prog = |val: usize| {
        if let Some(p) = progress_pct {
            p.store(val.min(100), Ordering::Relaxed);
        }
    };
    set_prog(2);

    let walls = extract_wall_segments(obstacles);
    let use_max_vis = max_visibility_m.is_finite() && max_visibility_m > 0.0;
    let max_vis_sq = if use_max_vis {
        max_visibility_m * max_visibility_m
    } else {
        f64::INFINITY
    };

    // Stage 1 (2% -> 52%): Parallel Upper-Triangle Line-of-Sight (j in (i+1)..n)
    // Cuts ray-wall intersection evaluations in half via LOS symmetry
    let completed_los = AtomicUsize::new(0);
    let upper_rows: Vec<Vec<(u32, u8, u8, f64)>> = (0..n)
        .into_par_iter()
        .map(|i| {
            if is_cancelled() {
                return Vec::new();
            }
            let pi = points[i];
            let mut row = Vec::new();
            for j in (i + 1)..n {
                let pj = points[j];
                let dx_m = pj.x - pi.x;
                let dy_m = pj.y - pi.y;
                let d2 = dx_m * dx_m + dy_m * dy_m;
                if d2 > max_vis_sq {
                    continue;
                }
                if los_visible(pi.x, pi.y, pj.x, pj.y, &walls) {
                    let gdx = (pj.grid_i - pi.grid_i) as f64;
                    let gdy = (pj.grid_j - pi.grid_j) as f64;
                    let (bin_ij, bin_ji) = if gdx.abs() > 0.0 || gdy.abs() > 0.0 {
                        (whichbin(gdx, gdy), whichbin(-gdx, -gdy))
                    } else {
                        (whichbin(dx_m, dy_m), whichbin(-dx_m, -dy_m))
                    };
                    row.push((j as u32, bin_ij, bin_ji, d2.sqrt()));
                }
            }
            let done = completed_los.fetch_add(1, Ordering::Relaxed) + 1;
            if (done & 63) == 0 || done == n {
                set_prog(2 + (done * 50) / n);
            }
            row
        })
        .collect();

    if is_cancelled() {
        return None;
    }

    // Assemble symmetric sorted adjacency rows
    let mut deg_counts = vec![0usize; n];
    for (i, urow) in upper_rows.iter().enumerate() {
        deg_counts[i] += urow.len();
        for &(j_u32, _, _, _) in urow {
            deg_counts[j_u32 as usize] += 1;
        }
    }

    let mut adj_rows: Vec<Vec<(u32, u8, f64)>> = deg_counts
        .iter()
        .map(|&d| Vec::with_capacity(d))
        .collect();

    for (i, urow) in upper_rows.into_iter().enumerate() {
        let i_u32 = i as u32;
        for (j_u32, bin_ij, bin_ji, dist) in urow {
            adj_rows[i].push((j_u32, bin_ij, dist));
            adj_rows[j_u32 as usize].push((i_u32, bin_ji, dist));
        }
    }

    let mut row_offsets = Vec::with_capacity(n + 1);
    row_offsets.push(0u32);
    let total_edges: usize = adj_rows.iter().map(|r| r.len()).sum();
    let mut col_indices = Vec::with_capacity(total_edges);
    let mut bin_indices = Vec::with_capacity(total_edges);

    let mut connectivity = vec![0.0_f64; n];
    let mut first_moment = vec![0.0_f64; n];
    let mut second_moment = vec![0.0_f64; n];

    for (i, row) in adj_rows.iter().enumerate() {
        connectivity[i] = row.len() as f64;
        let mut m1 = 0.0;
        let mut m2 = 0.0;
        for &(v, bin, dist) in row {
            col_indices.push(v);
            bin_indices.push(bin);
            m1 += dist;
            m2 += dist * dist;
        }
        first_moment[i] = m1;
        second_moment[i] = m2;
        row_offsets.push(col_indices.len() as u32);
    }

    if is_cancelled() {
        return None;
    }
    set_prog(56);

    // Stage 2 (56% -> 68%): Visual Control & Visual Clustering Coefficient
    let inv_k: Vec<f64> = connectivity
        .iter()
        .map(|&k| if k > 0.0 { 1.0 / k } else { 0.0 })
        .collect();

    let (visual_control, visual_clustering): (Vec<f64>, Vec<f64>) = (0..n)
        .into_par_iter()
        .map(|i| {
            if is_cancelled() {
                return (0.0, -1.0);
            }
            let s = row_offsets[i] as usize;
            let e = row_offsets[i + 1] as usize;
            let nbrs = &col_indices[s..e];
            let k = nbrs.len();
            if k == 0 {
                return (0.0, -1.0);
            }
            let mut ctrl = 0.0;
            for &u_u32 in nbrs {
                ctrl += inv_k[u_u32 as usize];
            }
            let cc = if k > 1 {
                let stride = (k / 240).max(1);
                let mut tri_acc = 0usize;
                let mut sampled_u = 0usize;
                for idx_n in (0..k).step_by(stride) {
                    let u = nbrs[idx_n] as usize;
                    let us = row_offsets[u] as usize;
                    let ue = row_offsets[u + 1] as usize;
                    tri_acc += sorted_slice_intersection_count(nbrs, &col_indices[us..ue]);
                    sampled_u += 1;
                }
                let scale = (k as f64) / (sampled_u.max(1) as f64);
                ((tri_acc as f64 * scale) / ((k * (k - 1)) as f64)).clamp(0.0, 1.0)
            } else {
                -1.0
            };
            (ctrl, cc)
        })
        .unzip();

    if is_cancelled() {
        return None;
    }
    set_prog(68);

    let through_vision = compute_through_vision(points, &row_offsets, &col_indices);
    set_prog(72);

    // Stage 3 (72% -> 92%): Global BFS & Controllability
    let mut visual_controllability = vec![-1.0_f64; n];
    let mut node_count = vec![1.0_f64; n];
    let mut total_depth = vec![0.0_f64; n];
    let mut mean_depth = vec![-1.0_f64; n];
    let mut visual_integration_hh = vec![-1.0_f64; n];
    let mut visual_integration_pval = vec![-1.0_f64; n];
    let mut visual_integration_tekl = vec![-1.0_f64; n];
    let mut visual_entropy = vec![-1.0_f64; n];
    let mut visual_rel_entropy = vec![-1.0_f64; n];

    if compute_global {
        let completed_bfs = AtomicUsize::new(0);
        let bfs_metrics: Vec<[f64; 9]> = (0..n)
            .into_par_iter()
            .map(|i| {
                if is_cancelled() {
                    return [-1.0, 1.0, 0.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0];
                }
                let mut dist = vec![-1i32; n];
                let mut queue = Vec::with_capacity(n);
                let mut depth_counts = vec![0u32; n + 1];

                dist[i] = 0;
                queue.push(i as u32);
                let mut head = 0usize;
                let mut max_d = 0usize;
                let mut td = 0u64;

                while head < queue.len() {
                    let u = queue[head] as usize;
                    head += 1;
                    let du_next = (dist[u] + 1) as usize;
                    let s = row_offsets[u] as usize;
                    let e = row_offsets[u + 1] as usize;
                    for &v_u32 in &col_indices[s..e] {
                        let v = v_u32 as usize;
                        if dist[v] == -1 {
                            dist[v] = du_next as i32;
                            queue.push(v_u32);
                            depth_counts[du_next] += 1;
                            td += du_next as u64;
                            if du_next > max_d {
                                max_d = du_next;
                            }
                        }
                    }
                }

                let reach = queue.len() - 1;
                let nc = (1 + reach) as f64;
                let td_f = td as f64;

                let c1 = if max_d >= 1 { depth_counts[1] as f64 } else { 0.0 };
                let c2 = if max_d >= 2 { depth_counts[2] as f64 } else { 0.0 };
                let controllability = if (c1 + c2) > 0.0 {
                    c1 / (1.0 + c1 + c2)
                } else {
                    -1.0
                };

                let mut md = -1.0;
                let mut ihh = -1.0;
                let mut ipv = -1.0;
                let mut itk = -1.0;
                let mut ent = -1.0;
                let mut rel_ent = -1.0;

                if reach > 0 {
                    md = td_f / (reach as f64);
                    if nc > 2.0 && md > 1.0 {
                        let ra = 2.0 * (md - 1.0) / (nc - 2.0);
                        let num_dn = 2.0 * (nc * (((nc + 2.0) / 3.0).log2() - 1.0) + 1.0);
                        let den_dn = (nc - 1.0) * (nc - 2.0);
                        let dn = (num_dn / den_dn).max(1e-12);
                        ihh = dn / ra;
                        ipv = ihh;
                        if td_f > nc {
                            itk = (0.5 * (nc - 2.0)).ln() / (td_f - nc + 1.0).ln();
                        }

                        // Log-space Poisson probability q(d) to prevent factorial overflow > 170! (UL-03)
                        let mut e_acc = 0.0;
                        let mut re_acc = 0.0;
                        let mut ln_fact = 0.0_f64;
                        let ln_md = md.ln();
                        for d_step in 1..=max_d {
                            let idx_b = (d_step - 1) as f64;
                            ln_fact += (d_step as f64).ln();
                            let cnt = depth_counts[d_step];
                            if cnt > 0 {
                                let prob = (cnt as f64) / nc;
                                e_acc -= prob * prob.log2();
                                let ln_q = idx_b * ln_md - md - ln_fact;
                                let q = ln_q.exp();
                                if q > 0.0 && q.is_finite() {
                                    re_acc += prob * (prob / q).log2();
                                }
                            }
                        }
                        ent = e_acc;
                        rel_ent = re_acc;
                    }
                }

                let done = completed_bfs.fetch_add(1, Ordering::Relaxed) + 1;
                if (done & 63) == 0 || done == n {
                    set_prog(72 + (done * 20) / n);
                }

                [controllability, nc, td_f, md, ihh, ipv, itk, ent, rel_ent]
            })
            .collect();

        if is_cancelled() {
            return None;
        }

        for (i, m) in bfs_metrics.iter().enumerate() {
            visual_controllability[i] = m[0];
            node_count[i] = m[1];
            total_depth[i] = m[2];
            mean_depth[i] = m[3];
            visual_integration_hh[i] = m[4];
            visual_integration_pval[i] = m[5];
            visual_integration_tekl[i] = m[6];
            visual_entropy[i] = m[7];
            visual_rel_entropy[i] = m[8];
        }
    } else {
        // Compute Visual_Controllability via local 2-hop neighborhood
        visual_controllability = (0..n)
            .into_par_iter()
            .map(|i| {
                let s = row_offsets[i] as usize;
                let e = row_offsets[i + 1] as usize;
                let c1 = e - s;
                if c1 == 0 {
                    return -1.0;
                }
                let mut seen = vec![false; n];
                seen[i] = true;
                for &u in &col_indices[s..e] {
                    seen[u as usize] = true;
                }
                let mut c2 = 0usize;
                for &u in &col_indices[s..e] {
                    let us = row_offsets[u as usize] as usize;
                    let ue = row_offsets[u as usize + 1] as usize;
                    for &w in &col_indices[us..ue] {
                        let wi = w as usize;
                        if !seen[wi] {
                            seen[wi] = true;
                            c2 += 1;
                        }
                    }
                }
                (c1 as f64) / (1.0 + (c1 as f64) + (c2 as f64))
            })
            .collect();
    }

    let mut attributes = BTreeMap::new();
    attributes.insert(
        "Depthmap_Ref".to_string(),
        points.iter().map(|p| p.depthmap_ref as f64).collect(),
    );
    attributes.insert("Connectivity".to_string(), connectivity);
    attributes.insert("Point_First_Moment".to_string(), first_moment);
    attributes.insert("Point_Second_Moment".to_string(), second_moment);
    attributes.insert("Visual_Control".to_string(), visual_control);
    attributes.insert("Visual_Controllability".to_string(), visual_controllability);
    attributes.insert(
        "Visual_Clustering_Coefficient".to_string(),
        visual_clustering,
    );
    attributes.insert("Through_Vision".to_string(), through_vision);

    if compute_global {
        attributes.insert("Visual_Node_Count".to_string(), node_count);
        attributes.insert("Visual_Total_Depth".to_string(), total_depth);
        attributes.insert("Visual_Mean_Depth".to_string(), mean_depth);
        attributes.insert("Visual_Integration_HH".to_string(), visual_integration_hh);
        attributes.insert(
            "Visual_Integration_Pval".to_string(),
            visual_integration_pval,
        );
        attributes.insert(
            "Visual_Integration_Tekl".to_string(),
            visual_integration_tekl,
        );
        attributes.insert("Visual_Entropy".to_string(), visual_entropy);
        attributes.insert(
            "Visual_Relativised_Entropy".to_string(),
            visual_rel_entropy,
        );
    }

    // Stage 4 (92% -> 100%): Per-Cell Analytical Isovist Columns (when requested and N <= 3,500)
    if compute_isovist_metrics && n <= 3_500 {
        let iso_horizon = if use_max_vis { max_visibility_m } else { 100.0 };
        let completed_iso = AtomicUsize::new(0);
        let iso_vec: Vec<IsovistResult> = points
            .par_iter()
            .map(|p| {
                if is_cancelled() {
                    return IsovistResult {
                        origin: Point2D::new(p.x, p.y),
                        polygon: Vec::new(),
                        area: 0.0,
                        perimeter: 0.0,
                        compactness: 0.0,
                        occlusivity: 0.0,
                        min_radial: 0.0,
                        max_radial: 0.0,
                        drift_magnitude: 0.0,
                        drift_angle_deg: 0.0,
                        drift_target: Point2D::new(p.x, p.y),
                    };
                }
                let res = compute_isovist_at_point(Point2D::new(p.x, p.y), obstacles, iso_horizon);
                let done = completed_iso.fetch_add(1, Ordering::Relaxed) + 1;
                if (done & 63) == 0 || done == n {
                    set_prog(92 + (done * 8) / n);
                }
                res
            })
            .collect();

        if is_cancelled() {
            return None;
        }

        attributes.insert(
            "Isovist_Area".to_string(),
            iso_vec.iter().map(|r| r.area).collect(),
        );
        attributes.insert(
            "Isovist_Perimeter".to_string(),
            iso_vec.iter().map(|r| r.perimeter).collect(),
        );
        attributes.insert(
            "Isovist_Compactness".to_string(),
            iso_vec.iter().map(|r| r.compactness).collect(),
        );
        attributes.insert(
            "Isovist_Occlusivity".to_string(),
            iso_vec.iter().map(|r| r.occlusivity).collect(),
        );
        attributes.insert(
            "Isovist_Min_Radial".to_string(),
            iso_vec.iter().map(|r| r.min_radial).collect(),
        );
        attributes.insert(
            "Isovist_Max_Radial".to_string(),
            iso_vec.iter().map(|r| r.max_radial).collect(),
        );
        attributes.insert(
            "Isovist_Drift_Magnitude".to_string(),
            iso_vec.iter().map(|r| r.drift_magnitude).collect(),
        );
        attributes.insert(
            "Isovist_Drift_Angle".to_string(),
            iso_vec.iter().map(|r| r.drift_angle_deg).collect(),
        );
    }

    attributes.insert("Gate_Counts".to_string(), vec![0.0; n]);
    set_prog(100);

    Some(VgaResults {
        points: points.to_vec(),
        cell_size_m: eff_cell_m,
        row_offsets,
        col_indices,
        bin_indices,
        attributes,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_whichbin_all_octants() {
        // Test cardinal and diagonal directions across all 8 octants
        let angles_deg: [f64; 19] = [
            0.0, 10.0, 20.0, 40.0, 50.0, 70.0, 80.0, 90.0, 100.0, 135.0, 170.0, 180.0, 200.0,
            225.0, 260.0, 270.0, 300.0, 315.0, 350.0,
        ];
        for &deg in &angles_deg {
            let rad = deg.to_radians();
            let b = whichbin(rad.cos(), rad.sin());
            assert!(b < 32, "bin out of range for angle {}: {}", deg, b);
        }
        assert_eq!(whichbin(1.0, 0.0), 0);
        assert_eq!(whichbin(-1.0, 0.0), 16);
        assert_eq!(whichbin(0.0, 1.0), 8);
        assert_eq!(whichbin(0.0, -1.0), 24);
    }

    #[test]
    fn test_isovist_unobstructed_square_box() {
        let box_wall = Polyline::new(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(10.0, 0.0),
            Point2D::new(10.0, 10.0),
            Point2D::new(0.0, 10.0),
            Point2D::new(0.0, 0.0),
        ]);
        let iso = compute_isovist_at_point(Point2D::new(5.0, 5.0), &[box_wall], 100.0);
        assert!(iso.area > 99.9 && iso.area < 100.1, "area={}", iso.area);
        assert!(
            iso.perimeter > 39.9 && iso.perimeter < 40.1,
            "perim={}",
            iso.perimeter
        );
        assert!(
            iso.compactness > 0.78 && iso.compactness <= 1.0,
            "compactness={}",
            iso.compactness
        );
        assert!(iso.occlusivity < 1e-6, "occ={}", iso.occlusivity);
        assert!(iso.drift_magnitude < 1e-6, "drift={}", iso.drift_magnitude);
    }

    #[test]
    fn test_vga_005m_adaptive_grid_and_walkable_lattice() {
        use crate::benchmarks::{load_benchmark, BenchmarkPreset};
        let ds = load_benchmark(BenchmarkPreset::ArchitecturalGalleryVga);
        let pts = generate_vga_grid(&ds.boundary_rings, &ds.obstacles, 0.05, true);
        let eff_step = effective_vga_grid_spacing(&pts, 0.05);
        assert!(eff_step >= 0.05 && eff_step < 0.25);
        assert!(
            pts.len() > 1800 && pts.len() <= MAX_INTERACTIVE_VGA_CELLS,
            "unexpected cell count at 0.05m adaptive scaling: {}",
            pts.len()
        );

        let lattice = build_walkable_vga_grid_lattice(&pts, &ds.obstacles, eff_step);
        assert_eq!(lattice.points.len(), pts.len());
        assert!(lattice.col_indices.len() > pts.len() * 2);

        // Verify solve_vga_with_progress produces finite values across all columns (including log-space Relativised Entropy)
        let pts_30 = generate_vga_grid(&ds.boundary_rings, &ds.obstacles, 0.30, true);
        let vga = solve_vga_with_progress(&pts_30, &ds.obstacles, 0.30, 50.0, true, false, None, None)
            .expect("vga should succeed");
        for (col_name, vals) in &vga.attributes {
            for &v in vals {
                assert!(v.is_finite(), "Non-finite value in column {}", col_name);
            }
        }
    }
}
