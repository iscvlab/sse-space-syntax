//! Geometric Cleaning, Planarization & 5-Step P90 Adaptive Watertight Network Cleaner.
//!
//! Implements:
//! 1. Exact 2D segment-segment intersection planarization & noding (`planarize_and_node`).
//! 2. 2-point segment explosion with bearing & azimuth computation (`explode_to_2point_segments`).
//! 3. Watertight endpoint snapping (`snap_endpoints_watertight`) and dual-carriageway rungs (`synthesize_dual_carriageway_rungs`).
//! 4. Degree-2 collinear join with turn conservation (`collinear_join_with_turn_conservation`).
//! 5. 5-Step P90 Adaptive Watertight Cleaner (`clean_network_adaptive_watertight`).

use crate::types::{
    compute_network_circuity_avg, CleanerConfig, CleanerReport, Point2D, Polyline, SegmentRecord,
};
use std::collections::{HashMap, HashSet};
use std::time::Instant;

const MAJOR_HIGHWAYS: &[&str] = &[
    "primary",
    "secondary",
    "tertiary",
    "trunk",
    "motorway",
    "primary_link",
    "secondary_link",
    "tertiary_link",
    "trunk_link",
    "motorway_link",
];

/// Computes the 2D angle in degrees `[0.0, 180.0]` between two vectors.
#[inline]
pub fn angle_between_vectors(v1: (f64, f64), v2: (f64, f64)) -> f64 {
    let dot = v1.0 * v2.0 + v1.1 * v2.1;
    let mag1 = v1.0.hypot(v1.1);
    let mag2 = v2.0.hypot(v2.1);
    if mag1 <= 1e-12 || mag2 <= 1e-12 {
        return 0.0;
    }
    let cos_theta = (dot / (mag1 * mag2)).clamp(-1.0, 1.0);
    cos_theta.acos().to_degrees()
}

/// Projects point `p` onto 2-point segment `(a, b)`, returning `(closest_point, distance, t_clamped)`.
#[inline]
fn project_point_to_segment(p: Point2D, a: Point2D, b: Point2D) -> (Point2D, f64, f64) {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len2 = dx * dx + dy * dy;
    if len2 <= 1e-18 {
        return (a, p.dist(a), 0.0);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
    let proj = Point2D::new(a.x + t * dx, a.y + t * dy);
    (proj, p.dist(proj), t)
}

/// Projects point `p` onto a multi-vertex `Polyline`, returning `(closest_point, distance)`.
fn project_point_to_polyline(p: Point2D, line: &Polyline) -> (Point2D, f64) {
    if line.coords.is_empty() {
        return (p, f64::INFINITY);
    }
    if line.coords.len() == 1 {
        return (line.coords[0], p.dist(line.coords[0]));
    }
    let mut best_pt = line.coords[0];
    let mut best_dist = f64::INFINITY;
    for w in line.coords.windows(2) {
        let (proj, d, _) = project_point_to_segment(p, w[0], w[1]);
        if d < best_dist {
            best_dist = d;
            best_pt = proj;
        }
    }
    (best_pt, best_dist)
}

/// Interpolates a point at distance `dist_m` along a `Polyline`.
fn interpolate_polyline(line: &Polyline, dist_m: f64) -> Point2D {
    if line.coords.is_empty() {
        return Point2D::new(0.0, 0.0);
    }
    if line.coords.len() == 1 || dist_m <= 0.0 {
        return line.coords[0];
    }
    let mut accum = 0.0;
    for w in line.coords.windows(2) {
        let seg_len = w[0].dist(w[1]);
        if accum + seg_len >= dist_m {
            let rem = dist_m - accum;
            let t = if seg_len > 1e-12 {
                (rem / seg_len).clamp(0.0, 1.0)
            } else {
                0.0
            };
            return Point2D::new(
                w[0].x + t * (w[1].x - w[0].x),
                w[0].y + t * (w[1].y - w[0].y),
            );
        }
        accum += seg_len;
    }
    *line.coords.last().unwrap()
}

/// Computes intersection parameter `(t, u)` between segment `A(t) = a0 + t*(a1 - a0)`
/// and segment `B(u) = b0 + u*(b1 - b0)` for `t, u in [0, 1]`.
#[inline]
fn segment_intersection_params(
    a0: Point2D,
    a1: Point2D,
    b0: Point2D,
    b1: Point2D,
) -> Option<(f64, f64, Point2D)> {
    let dax = a1.x - a0.x;
    let day = a1.y - a0.y;
    let dbx = b1.x - b0.x;
    let dby = b1.y - b0.y;
    let denom = dax * dby - day * dbx;
    if denom.abs() <= 1e-11 {
        return None;
    }
    let wx = b0.x - a0.x;
    let wy = b0.y - a0.y;
    let t = (wx * dby - wy * dbx) / denom;
    let u = (wx * day - wy * dax) / denom;
    let eps = 1e-7;
    if t >= -eps && t <= 1.0 + eps && u >= -eps && u <= 1.0 + eps {
        let tc = t.clamp(0.0, 1.0);
        let uc = u.clamp(0.0, 1.0);
        let pt = Point2D::new(a0.x + tc * dax, a0.y + tc * day);
        Some((tc, uc, pt))
    } else {
        None
    }
}

/// Splits all input polylines at every proper and endpoint intersection (`t in (0.0, 1.0)`),
/// returning non-degenerate 2-point `Polyline`s (`length > 0.005`) while preserving `level`, `highway`, `name`.
/// When multi-level polylines exist, only intersects segments on the same `level`.
pub fn planarize_and_node(lines: &[Polyline]) -> Vec<Polyline> {
    if lines.is_empty() {
        return Vec::new();
    }

    #[derive(Clone)]
    struct RawSeg {
        p0: Point2D,
        p1: Point2D,
        min_x: f64,
        max_x: f64,
        min_y: f64,
        max_y: f64,
        level: f64,
        level_key: i64,
        highway: String,
        name: String,
    }

    let mut raw_segs: Vec<RawSeg> = Vec::new();
    let mut distinct_levels: HashSet<i64> = HashSet::new();

    for line in lines {
        if line.coords.len() < 2 {
            continue;
        }
        let lv_key = (line.level * 1000.0).round() as i64;
        distinct_levels.insert(lv_key);
        for w in line.coords.windows(2) {
            let p0 = w[0];
            let p1 = w[1];
            if p0.dist(p1) <= 1e-5 {
                continue;
            }
            raw_segs.push(RawSeg {
                p0,
                p1,
                min_x: p0.x.min(p1.x) - 1e-4,
                max_x: p0.x.max(p1.x) + 1e-4,
                min_y: p0.y.min(p1.y) - 1e-4,
                max_y: p0.y.max(p1.y) + 1e-4,
                level: line.level,
                level_key: lv_key,
                highway: line.highway.clone(),
                name: line.name.clone(),
            });
        }
    }

    let n = raw_segs.len();
    if n == 0 {
        return Vec::new();
    }

    let multi_level = distinct_levels.len() > 1;
    let mut splits: Vec<Vec<f64>> = vec![vec![0.0, 1.0]; n];

    // Sort indices by min_x for sweep-line bounding-box pruning
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| {
        raw_segs[a]
            .min_x
            .partial_cmp(&raw_segs[b].min_x)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    for idx_a in 0..n {
        let i = order[idx_a];
        let sa = &raw_segs[i];
        for idx_b in (idx_a + 1)..n {
            let j = order[idx_b];
            let sb = &raw_segs[j];
            if sb.min_x > sa.max_x {
                break;
            }
            if sb.max_y < sa.min_y || sb.min_y > sa.max_y {
                continue;
            }
            if multi_level && sa.level_key != sb.level_key {
                continue;
            }

            if let Some((tc, uc, _)) = segment_intersection_params(sa.p0, sa.p1, sb.p0, sb.p1) {
                if tc > 1e-7 && tc < 1.0 - 1e-7 {
                    splits[i].push(tc);
                }
                if uc > 1e-7 && uc < 1.0 - 1e-7 {
                    splits[j].push(uc);
                }
            }

            // Also check T-junction / collinear endpoint projections within 0.5mm
            for bp in [sb.p0, sb.p1] {
                let (_, d, t_proj) = project_point_to_segment(bp, sa.p0, sa.p1);
                if d <= 5e-4 && t_proj > 1e-7 && t_proj < 1.0 - 1e-7 {
                    splits[i].push(t_proj);
                }
            }
            for ap in [sa.p0, sa.p1] {
                let (_, d, u_proj) = project_point_to_segment(ap, sb.p0, sb.p1);
                if d <= 5e-4 && u_proj > 1e-7 && u_proj < 1.0 - 1e-7 {
                    splits[j].push(u_proj);
                }
            }
        }
    }

    let mut out: Vec<Polyline> = Vec::with_capacity(n * 2);
    let mut seen_edges: HashSet<((i64, i64), (i64, i64), i64)> = HashSet::with_capacity(n * 2);

    for (i, seg) in raw_segs.into_iter().enumerate() {
        let ts = &mut splits[i];
        ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mut dedup: Vec<f64> = Vec::with_capacity(ts.len());
        for &t in ts.iter() {
            if dedup.last().map_or(true, |&last| (t - last).abs() > 1e-7) {
                dedup.push(t);
            }
        }

        let dx = seg.p1.x - seg.p0.x;
        let dy = seg.p1.y - seg.p0.y;
        for w in dedup.windows(2) {
            let t0 = w[0];
            let t1 = w[1];
            let a = Point2D::new(seg.p0.x + t0 * dx, seg.p0.y + t0 * dy);
            let b = Point2D::new(seg.p0.x + t1 * dx, seg.p0.y + t1 * dy);
            if a.dist(b) > 0.005 {
                let ka = a.key_mm();
                let kb = b.key_mm();
                if ka == kb {
                    continue;
                }
                let canonical = if ka <= kb {
                    (ka, kb, seg.level_key)
                } else {
                    (kb, ka, seg.level_key)
                };
                if seen_edges.insert(canonical) {
                    out.push(Polyline::with_meta(
                        Polyline::new(vec![a, b]),
                        seg.level,
                        seg.highway.clone(),
                        seg.name.clone(),
                    ));
                }
            }
        }
    }

    out
}

/// Trims axial overshoot tails on raw axial lines (`depthmapX` `stubremoval` rule, typically `0.25..0.40`).
///
/// For any line that intersects other lines in its interior (`0 < t_1 <= ... <= t_k < 1`):
/// - If the start tip `t = 0` has no endpoint contact and `t_1 <= stub_removal_ratio`, trims start to `t_1`.
/// - If the end tip `t = 1` has no endpoint contact and `(1.0 - t_k) <= stub_removal_ratio`, trims end to `t_k`.
pub fn trim_axial_overshoot_whiskers(lines: &[Polyline], stub_removal_ratio: f64) -> Vec<Polyline> {
    if lines.is_empty() || stub_removal_ratio <= 0.0 {
        return lines.to_vec();
    }

    #[derive(Clone)]
    struct SegItem {
        p0: Point2D,
        p1: Point2D,
        min_x: f64,
        max_x: f64,
        min_y: f64,
        max_y: f64,
        level: f64,
        level_key: i64,
        highway: String,
        name: String,
    }

    let mut segs: Vec<SegItem> = Vec::new();
    let mut distinct_levels: HashSet<i64> = HashSet::new();

    for line in lines {
        if line.coords.len() < 2 {
            continue;
        }
        let lv_key = (line.level * 1000.0).round() as i64;
        distinct_levels.insert(lv_key);
        for w in line.coords.windows(2) {
            let p0 = w[0];
            let p1 = w[1];
            if p0.dist(p1) <= 1e-5 {
                continue;
            }
            segs.push(SegItem {
                p0,
                p1,
                min_x: p0.x.min(p1.x) - 1e-3,
                max_x: p0.x.max(p1.x) + 1e-3,
                min_y: p0.y.min(p1.y) - 1e-3,
                max_y: p0.y.max(p1.y) + 1e-3,
                level: line.level,
                level_key: lv_key,
                highway: line.highway.clone(),
                name: line.name.clone(),
            });
        }
    }

    let n = segs.len();
    if n == 0 {
        return Vec::new();
    }

    let multi_level = distinct_levels.len() > 1;
    let mut interior_hits: Vec<Vec<f64>> = vec![Vec::new(); n];
    let mut start_contact: Vec<bool> = vec![false; n];
    let mut end_contact: Vec<bool> = vec![false; n];

    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| {
        segs[a]
            .min_x
            .partial_cmp(&segs[b].min_x)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    for idx_a in 0..n {
        let i = order[idx_a];
        let sa = &segs[i];
        for idx_b in (idx_a + 1)..n {
            let j = order[idx_b];
            let sb = &segs[j];
            if sb.min_x > sa.max_x {
                break;
            }
            if sb.max_y < sa.min_y || sb.min_y > sa.max_y {
                continue;
            }
            if multi_level && sa.level_key != sb.level_key {
                continue;
            }

            if let Some((tc, uc, _)) = segment_intersection_params(sa.p0, sa.p1, sb.p0, sb.p1) {
                if tc <= 1e-4 {
                    start_contact[i] = true;
                } else if tc >= 1.0 - 1e-4 {
                    end_contact[i] = true;
                } else {
                    interior_hits[i].push(tc);
                }

                if uc <= 1e-4 {
                    start_contact[j] = true;
                } else if uc >= 1.0 - 1e-4 {
                    end_contact[j] = true;
                } else {
                    interior_hits[j].push(uc);
                }
            }
        }
    }

    let mut out = Vec::with_capacity(n);
    for (i, s) in segs.into_iter().enumerate() {
        let hits = &mut interior_hits[i];
        hits.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mut dedup: Vec<f64> = Vec::with_capacity(hits.len());
        for &t in hits.iter() {
            if dedup.last().map_or(true, |&last| (t - last).abs() > 1e-4) {
                dedup.push(t);
            }
        }

        let mut t0 = 0.0_f64;
        let mut t1 = 1.0_f64;

        if dedup.len() >= 2 {
            let first_t = dedup[0];
            let last_t = *dedup.last().unwrap();
            if !start_contact[i] && first_t <= stub_removal_ratio {
                t0 = first_t;
            }
            if !end_contact[i] && (1.0 - last_t) <= stub_removal_ratio {
                t1 = last_t;
            }
        } else if dedup.len() == 1 {
            let hit_t = dedup[0];
            if !start_contact[i] && hit_t <= stub_removal_ratio && (1.0 - hit_t) > stub_removal_ratio {
                t0 = hit_t;
            } else if !end_contact[i]
                && (1.0 - hit_t) <= stub_removal_ratio
                && hit_t > stub_removal_ratio
            {
                t1 = hit_t;
            }
        }

        if t1 - t0 > 1e-4 {
            let dx = s.p1.x - s.p0.x;
            let dy = s.p1.y - s.p0.y;
            let a = Point2D::new(s.p0.x + t0 * dx, s.p0.y + t0 * dy);
            let b = Point2D::new(s.p0.x + t1 * dx, s.p0.y + t1 * dy);
            if a.dist(b) > 0.01 {
                out.push(
                    Polyline::new(vec![a, b]).with_meta(s.level, s.highway, s.name),
                );
            }
        }
    }

    out
}

/// Prunes short degree-1 collinear corner overshoot whiskers (`length <= max_whisker_m`)
/// on already-noded segment networks where an axial line was split across a cross-street junction.
pub fn prune_noded_corner_whiskers(
    noded_lines: &[Polyline],
    max_whisker_m: f64,
    stub_removal_ratio: f64,
) -> Vec<Polyline> {
    if noded_lines.is_empty() || max_whisker_m <= 0.0 || stub_removal_ratio <= 0.0 {
        return noded_lines.to_vec();
    }

    let mut current: Vec<Polyline> = noded_lines.to_vec();

    for _pass in 0..2 {
        let mut adj: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        for (idx, pl) in current.iter().enumerate() {
            if pl.coords.len() >= 2 {
                adj.entry(pl.coords[0].key_mm()).or_default().push(idx);
                adj.entry(pl.coords.last().unwrap().key_mm())
                    .or_default()
                    .push(idx);
            }
        }

        let mut keep = vec![true; current.len()];
        let mut removed_any = false;

        for (idx, pl) in current.iter().enumerate() {
            if pl.coords.len() < 2 {
                keep[idx] = false;
                continue;
            }
            let len_stub = pl.length();
            if len_stub > max_whisker_m {
                continue;
            }
            let p0 = pl.coords[0];
            let p1 = *pl.coords.last().unwrap();
            let k0 = p0.key_mm();
            let k1 = p1.key_mm();
            let d0 = adj.get(&k0).map_or(0, |v| v.len());
            let d1 = adj.get(&k1).map_or(0, |v| v.len());

            // Must have one degree-1 dead-end tip and one intersection hub (degree >= 3)
            let (junc_pt, tip_pt, junc_key) = if d0 == 1 && d1 >= 3 {
                (p1, p0, k1)
            } else if d1 == 1 && d0 >= 3 {
                (p0, p1, k0)
            } else {
                continue;
            };

            let v_stub = (junc_pt.x - tip_pt.x, junc_pt.y - tip_pt.y);
            let Some(neighbors) = adj.get(&junc_key) else {
                continue;
            };

            let mut has_collinear_main = false;
            let mut has_cross_street = false;

            for &nb_idx in neighbors {
                if nb_idx == idx || !keep[nb_idx] {
                    continue;
                }
                let nb = &current[nb_idx];
                let nb_p0 = nb.coords[0];
                let nb_p1 = *nb.coords.last().unwrap();
                let nb_other = if nb_p0.key_mm() == junc_key {
                    nb_p1
                } else {
                    nb_p0
                };
                let v_nb = (nb_other.x - junc_pt.x, nb_other.y - junc_pt.y);
                let turn_deg = angle_between_vectors(v_stub, v_nb);
                let len_nb = nb.length();

                if turn_deg <= 15.0 && len_stub <= stub_removal_ratio * (len_stub + len_nb) {
                    has_collinear_main = true;
                } else if turn_deg >= 20.0 {
                    has_cross_street = true;
                }
            }

            if has_collinear_main && has_cross_street {
                keep[idx] = false;
                removed_any = true;
            }
        }

        if !removed_any {
            break;
        }
        current = current
            .into_iter()
            .enumerate()
            .filter_map(|(i, pl)| if keep[i] { Some(pl) } else { None })
            .collect();
    }

    current
}

/// Returns `true` if `lines` is already a planarized 2-point segment map
/// (all polylines have 2 vertices and at least 50% of endpoints meet at shared vertices),
/// preserving 1-to-1 segment order and grade-separated overpass crossings without artificial splits.
pub fn is_prenoded_segment_map(lines: &[Polyline]) -> bool {
    if lines.len() < 3 || !lines.iter().all(|l| l.coords.len() == 2) {
        return false;
    }
    let mut ep_counts: HashMap<(i64, i64), usize> = HashMap::with_capacity(lines.len() * 2);
    for l in lines {
        *ep_counts.entry(l.coords[0].key_cm()).or_insert(0) += 1;
        *ep_counts.entry(l.coords[1].key_cm()).or_insert(0) += 1;
    }
    let total_ep = lines.len() * 2;
    let shared_ep: usize = lines
        .iter()
        .map(|l| {
            let c0 = usize::from(ep_counts.get(&l.coords[0].key_cm()).copied().unwrap_or(0) >= 2);
            let c1 = usize::from(ep_counts.get(&l.coords[1].key_cm()).copied().unwrap_or(0) >= 2);
            c0 + c1
        })
        .sum();
    (shared_ep as f64) / (total_ep as f64) >= 0.50
}

/// Planarizes and nodes `lines` while pruning axial crossing overshoot stubs (`depthmapX` `stubremoval` rule).
pub fn planarize_and_node_pruned(lines: &[Polyline], stub_removal_ratio: f64) -> Vec<Polyline> {
    if is_prenoded_segment_map(lines) {
        return lines.to_vec();
    }
    if stub_removal_ratio <= 0.0 {
        return planarize_and_node(lines);
    }
    let trimmed = trim_axial_overshoot_whiskers(lines, stub_removal_ratio);
    let noded = planarize_and_node(&trimmed);
    prune_noded_corner_whiskers(&noded, 22.0, stub_removal_ratio)
}

/// Converts polylines into planar 2-point `SegmentRecord`s (`length_m > 0.005`),
/// computing bearings `b0`, `b1` and `azimuth_deg = (b0.to_degrees() + 360.0) % 360.0`.
pub fn explode_to_2point_segments(lines: &[Polyline]) -> Vec<SegmentRecord> {
    let mut records = Vec::new();
    let mut next_id = 0usize;
    for line in lines {
        if line.coords.len() < 2 {
            continue;
        }
        for w in line.coords.windows(2) {
            let p0 = w[0];
            let p1 = w[1];
            let length_m = p0.dist(p1);
            if length_m > 0.005 {
                let b0 = (p1.x - p0.x).atan2(p1.y - p0.y);
                let b1 = (p0.x - p1.x).atan2(p0.y - p1.y);
                let azimuth_deg = (b0.to_degrees() + 360.0) % 360.0;
                records.push(SegmentRecord {
                    id: next_id,
                    p0,
                    p1,
                    b0,
                    b1,
                    length_m,
                    azimuth_deg,
                    level: line.level,
                    highway: line.highway.clone(),
                    name: line.name.clone(),
                });
                next_id += 1;
            }
        }
    }
    records
}

/// Snaps dangling endpoints within `snap_distance_m` (`0.001 < d < snap_distance_m`)
/// onto the nearest point on nearby polylines.
pub fn snap_endpoints_watertight(lines: &[Polyline], snap_distance_m: f64) -> Vec<Polyline> {
    if lines.is_empty() || snap_distance_m <= 0.0 {
        return lines.to_vec();
    }

    let mut snapped = Vec::with_capacity(lines.len());
    for (idx, line) in lines.iter().enumerate() {
        if line.coords.len() < 2 {
            continue;
        }
        let p_start = line.coords[0];
        let p_end = *line.coords.last().unwrap();

        let mut new_start = p_start;
        let mut min_dist_start = snap_distance_m;

        let mut new_end = p_end;
        let mut min_dist_end = snap_distance_m;

        for (other_idx, other) in lines.iter().enumerate() {
            if other_idx == idx || other.coords.len() < 2 {
                continue;
            }
            if (other.level - line.level).abs() > 1e-6 {
                continue;
            }
            let (proj_s, d_s) = project_point_to_polyline(p_start, other);
            if d_s > 0.001 && d_s < min_dist_start {
                min_dist_start = d_s;
                new_start = Point2D::new(
                    (proj_s.x * 1000.0).round() / 1000.0,
                    (proj_s.y * 1000.0).round() / 1000.0,
                );
            }

            let (proj_e, d_e) = project_point_to_polyline(p_end, other);
            if d_e > 0.001 && d_e < min_dist_end {
                min_dist_end = d_e;
                new_end = Point2D::new(
                    (proj_e.x * 1000.0).round() / 1000.0,
                    (proj_e.y * 1000.0).round() / 1000.0,
                );
            }
        }

        let mut new_coords = line.coords.clone();
        new_coords[0] = new_start;
        let last_i = new_coords.len() - 1;
        new_coords[last_i] = new_end;
        snapped.push(Polyline::with_meta(
            Polyline::new(new_coords),
            line.level,
            line.highway.clone(),
            line.name.clone(),
        ));
    }

    snapped
}

/// Generates transverse pedestrian cross-connector rungs between parallel lines
/// separated by `min_gap_m..max_gap_m` at intervals of `spacing_m`.
pub fn synthesize_dual_carriageway_rungs(
    lines: &[Polyline],
    min_gap_m: f64,
    max_gap_m: f64,
    spacing_m: f64,
) -> Vec<Polyline> {
    if lines.is_empty() || max_gap_m <= min_gap_m || spacing_m <= 0.0 {
        return Vec::new();
    }

    let has_highway_tags = lines.iter().any(|l| !l.highway.trim().is_empty());
    let candidate_indices: Vec<usize> = lines
        .iter()
        .enumerate()
        .filter(|(_, ln)| {
            let len_ok = ln.length() >= 20.0;
            if !len_ok {
                return false;
            }
            if has_highway_tags {
                let hw = ln.highway.trim().to_ascii_lowercase();
                MAJOR_HIGHWAYS.contains(&hw.as_str())
            } else {
                true
            }
        })
        .map(|(i, _)| i)
        .collect();

    let candidates: Vec<usize> = if candidate_indices.is_empty() {
        lines
            .iter()
            .enumerate()
            .filter(|(_, ln)| ln.length() >= 20.0)
            .map(|(i, _)| i)
            .collect()
    } else {
        candidate_indices
    };

    if candidates.len() < 2 {
        return Vec::new();
    }

    let mut rungs = Vec::new();
    for (local_idx, &line_idx) in candidates.iter().enumerate() {
        let line = &lines[line_idx];
        let length = line.length();
        let num_rungs = ((length / spacing_m).floor() as usize).max(1);
        let step_len = length / ((num_rungs + 1) as f64);

        for step in 1..=num_rungs {
            let sample_pt = interpolate_polyline(line, (step as f64) * step_len);
            let mut best_neighbor_pt: Option<Point2D> = None;
            let mut min_dist = max_gap_m;

            for (other_local, &other_idx) in candidates.iter().enumerate() {
                if other_local == local_idx {
                    continue;
                }
                let other_line = &lines[other_idx];
                if (other_line.level - line.level).abs() > 1e-6 {
                    continue;
                }
                let (other_proj, dist) = project_point_to_polyline(sample_pt, other_line);
                if dist >= min_gap_m && dist < min_dist {
                    min_dist = dist;
                    best_neighbor_pt = Some(other_proj);
                }
            }

            if let Some(target_pt) = best_neighbor_pt {
                rungs.push(Polyline::with_meta(
                    Polyline::new(vec![sample_pt, target_pt]),
                    line.level,
                    "footway",
                    "dual_carriageway_rung",
                ));
            }
        }
    }

    rungs
}

/// Merges unbranched consecutive polylines (`degree == 2`) where turn deflection angle
/// `<= max_turn_deflection_deg` (`15.0°`), strictly preserving `degree >= 3` junctions
/// and sharp corners (`> max_turn_deflection_deg`).
pub fn collinear_join_with_turn_conservation(
    lines: &[Polyline],
    max_turn_deflection_deg: f64,
) -> Vec<Polyline> {
    if lines.is_empty() {
        return Vec::new();
    }

    let mut adj: HashMap<(i64, i64), Vec<(usize, bool, (i64, i64))>> = HashMap::new();
    let mut v_degree: HashMap<(i64, i64), usize> = HashMap::new();
    let mut node_order: Vec<(i64, i64)> = Vec::new();

    for (idx, ln) in lines.iter().enumerate() {
        if ln.coords.len() < 2 {
            continue;
        }
        let p0 = ln.coords[0].key_mm();
        let p1 = ln.coords.last().unwrap().key_mm();
        if !v_degree.contains_key(&p0) {
            node_order.push(p0);
        }
        if !v_degree.contains_key(&p1) {
            node_order.push(p1);
        }
        *v_degree.entry(p0).or_insert(0) += 1;
        *v_degree.entry(p1).or_insert(0) += 1;
        adj.entry(p0).or_default().push((idx, true, p1));
        adj.entry(p1).or_default().push((idx, false, p0));
    }

    let mut mergeable_nodes: HashSet<(i64, i64)> = HashSet::new();
    for (&pt_key, &deg) in &v_degree {
        if deg == 2 {
            let inc = &adj[&pt_key];
            if inc.len() != 2 {
                continue;
            }
            let (idx1, is_start1, _) = inc[0];
            let (idx2, is_start2, _) = inc[1];
            if idx1 == idx2 {
                continue;
            }
            let l1 = &lines[idx1];
            let l2 = &lines[idx2];
            if (l1.level - l2.level).abs() > 1e-6 {
                continue;
            }
            let c1 = &l1.coords;
            let c2 = &l2.coords;
            let pt = if is_start1 { c1[0] } else { *c1.last().unwrap() };
            let prev1 = if is_start1 { c1[1] } else { c1[c1.len() - 2] };
            let next2 = if is_start2 { c2[1] } else { c2[c2.len() - 2] };
            let v1 = (pt.x - prev1.x, pt.y - prev1.y);
            let v2 = (next2.x - pt.x, next2.y - pt.y);
            let defl = angle_between_vectors(v1, v2);
            if defl <= max_turn_deflection_deg {
                mergeable_nodes.insert(pt_key);
            }
        }
    }

    let mut visited_edges: HashSet<usize> = HashSet::with_capacity(lines.len());
    let mut result_lines: Vec<Polyline> = Vec::new();

    for pt in &node_order {
        if mergeable_nodes.contains(pt) {
            continue;
        }
        if let Some(neighbors) = adj.get(pt) {
            for &(edge_idx, is_start, next_pt) in neighbors {
                if visited_edges.contains(&edge_idx) {
                    continue;
                }
                visited_edges.insert(edge_idx);
                let base_line = &lines[edge_idx];
                let mut chain_coords = base_line.coords.clone();
                if !is_start {
                    chain_coords.reverse();
                }
                let mut curr_pt = next_pt;
                while mergeable_nodes.contains(&curr_pt) {
                    let next_opt = adj.get(&curr_pt).and_then(|list| {
                        list.iter()
                            .find(|&&(e_i, _, _)| !visited_edges.contains(&e_i))
                            .copied()
                    });
                    let Some((next_e_idx, next_is_start, n_next_pt)) = next_opt else {
                        break;
                    };
                    visited_edges.insert(next_e_idx);
                    let mut next_coords = lines[next_e_idx].coords.clone();
                    if !next_is_start {
                        next_coords.reverse();
                    }
                    chain_coords.extend_from_slice(&next_coords[1..]);
                    curr_pt = n_next_pt;
                }
                result_lines.push(Polyline::with_meta(
                    Polyline::new(chain_coords),
                    base_line.level,
                    base_line.highway.clone(),
                    base_line.name.clone(),
                ));
            }
        }
    }

    // Handle any remaining closed degree-2 loops
    for (idx, ln) in lines.iter().enumerate() {
        if !visited_edges.contains(&idx) && ln.coords.len() >= 2 {
            visited_edges.insert(idx);
            let mut chain_coords = ln.coords.clone();
            let mut curr_pt = ln.coords.last().unwrap().key_mm();
            while mergeable_nodes.contains(&curr_pt) {
                let next_opt = adj.get(&curr_pt).and_then(|list| {
                    list.iter()
                        .find(|&&(e_i, _, _)| !visited_edges.contains(&e_i))
                        .copied()
                });
                let Some((next_e_idx, next_is_start, n_next_pt)) = next_opt else {
                    break;
                };
                visited_edges.insert(next_e_idx);
                let mut next_coords = lines[next_e_idx].coords.clone();
                if !next_is_start {
                    next_coords.reverse();
                }
                chain_coords.extend_from_slice(&next_coords[1..]);
                curr_pt = n_next_pt;
            }
            result_lines.push(Polyline::with_meta(
                Polyline::new(chain_coords),
                ln.level,
                ln.highway.clone(),
                ln.name.clone(),
            ));
        }
    }

    result_lines
}

/// Removes dead-end cul-de-sac / driveway stubs shorter than `max_stub_length_m`
/// that hang off an intersection (`(d0 == 1 && d1 >= 3) || (d1 == 1 && d0 >= 3)`).
pub fn remove_dead_end_stubs(lines: &[Polyline], max_stub_length_m: f64) -> Vec<Polyline> {
    if lines.is_empty() || max_stub_length_m <= 0.0 {
        return lines.to_vec();
    }

    let mut deg: HashMap<(i64, i64), usize> = HashMap::new();
    for ln in lines {
        if ln.coords.len() < 2 {
            continue;
        }
        *deg.entry(ln.coords[0].key_mm()).or_insert(0) += 1;
        *deg.entry(ln.coords.last().unwrap().key_mm()).or_insert(0) += 1;
    }

    let mut filtered = Vec::with_capacity(lines.len());
    for ln in lines {
        if ln.coords.len() < 2 {
            continue;
        }
        let d0 = deg.get(&ln.coords[0].key_mm()).copied().unwrap_or(0);
        let d1 = deg
            .get(&ln.coords.last().unwrap().key_mm())
            .copied()
            .unwrap_or(0);
        let is_hanging_stub =
            ln.length() < max_stub_length_m && ((d0 == 1 && d1 >= 3) || (d1 == 1 && d0 >= 3));
        if !is_hanging_stub {
            filtered.push(ln.clone());
        }
    }
    filtered
}

/// Counts the number of connected graph components in `O(V + E)` using BFS.
pub fn count_connected_components(lines: &[Polyline]) -> usize {
    if lines.is_empty() {
        return 0;
    }
    let mut adj: HashMap<(i64, i64), Vec<(i64, i64)>> = HashMap::new();
    for ln in lines {
        if ln.coords.len() < 2 {
            continue;
        }
        let p0 = ln.coords[0].key_mm();
        let p1 = ln.coords.last().unwrap().key_mm();
        adj.entry(p0).or_default().push(p1);
        adj.entry(p1).or_default().push(p0);
    }

    let mut visited: HashSet<(i64, i64)> = HashSet::with_capacity(adj.len());
    let mut components = 0usize;
    let mut stack = Vec::new();

    for &node in adj.keys() {
        if visited.insert(node) {
            components += 1;
            stack.push(node);
            while let Some(curr) = stack.pop() {
                if let Some(neighbors) = adj.get(&curr) {
                    for &nbr in neighbors {
                        if visited.insert(nbr) {
                            stack.push(nbr);
                        }
                    }
                }
            }
        }
    }
    components
}

/// Counts intersection nodes with topological degree `>= 3`.
pub fn count_graph_junctions(lines: &[Polyline]) -> usize {
    if lines.is_empty() {
        return 0;
    }
    let mut endpoint_counts: HashMap<(i64, i64), usize> = HashMap::new();
    for ln in lines {
        if ln.coords.len() < 2 {
            continue;
        }
        *endpoint_counts.entry(ln.coords[0].key_mm()).or_insert(0) += 1;
        *endpoint_counts
            .entry(ln.coords.last().unwrap().key_mm())
            .or_insert(0) += 1;
    }
    endpoint_counts.values().filter(|&&deg| deg >= 3).count()
}

/// Counts true dead-end vertices with topological degree `== 1`.
pub fn count_dangling_endpoints(lines: &[Polyline]) -> usize {
    if lines.is_empty() {
        return 0;
    }
    let mut endpoint_counts: HashMap<(i64, i64), usize> = HashMap::new();
    for ln in lines {
        if ln.coords.len() < 2 {
            continue;
        }
        *endpoint_counts.entry(ln.coords[0].key_mm()).or_insert(0) += 1;
        *endpoint_counts
            .entry(ln.coords.last().unwrap().key_mm())
            .or_insert(0) += 1;
    }
    endpoint_counts.values().filter(|&&deg| deg == 1).count()
}

/// Executes the full 5-Step P90 Adaptive Watertight Network Cleaning Pipeline.
///
/// Steps:
/// 0. Dangle Incidence Audit (`degree == 1` vs locked junctions `degree >= 2`).
/// 1. Explode polylines into 2-point segments and unit approach tangent vectors.
/// 2. Adaptive Forward Raycasting (`auto` P90 clamped to `[6.0, 20.0]` or manual `extend_distance_m`),
///    deflection angle guard (`15°..165°`), obliquity compensation (`d_perp = d * sin(theta)`),
///    crossing overshoot (`+0.05m`), and perpendicular T-junction lateral fallback.
/// 3. Planarization & Noding (`planarize_and_node`) + micro-stub pruning (`< min_stub_len_m`).
/// 4. Collinear Join with Turn Conservation (`<= max_turn_deflection_deg`) + dead-end stub pruning.
/// 5. Post-Flight Topology Audit & `CleanerReport` (with `circuity_avg`).
pub fn clean_network_adaptive_watertight(
    lines: &[Polyline],
    config: &CleanerConfig,
) -> (Vec<Polyline>, CleanerReport) {
    let t0 = Instant::now();
    let mode_clean = config.mode.trim().to_ascii_lowercase();

    let clean_input: Vec<Polyline> = lines
        .iter()
        .filter(|ln| ln.coords.len() >= 2 && ln.length() > 0.001)
        .cloned()
        .collect();

    if clean_input.is_empty() {
        return (
            Vec::new(),
            CleanerReport {
                mode: mode_clean,
                assertions_passed: true,
                ..Default::default()
            },
        );
    }

    // Step 0 & 1: Explode into 2-point segments & identify degree == 1 dangles
    let mut segments: Vec<Polyline> = Vec::new();
    let mut distinct_levels: HashSet<i64> = HashSet::new();
    for line in &clean_input {
        distinct_levels.insert((line.level * 1000.0).round() as i64);
        for w in line.coords.windows(2) {
            if w[0].dist(w[1]) > 0.001 {
                segments.push(Polyline::with_meta(
                    Polyline::new(vec![w[0], w[1]]),
                    line.level,
                    line.highway.clone(),
                    line.name.clone(),
                ));
            }
        }
    }
    let has_multi_levels = distinct_levels.len() > 1;

    let mut pt_degree: HashMap<(i64, i64), usize> = HashMap::new();
    for seg in &segments {
        *pt_degree.entry(seg.coords[0].key_mm()).or_insert(0) += 1;
        *pt_degree.entry(seg.coords[1].key_mm()).or_insert(0) += 1;
    }

    let dangling_pts: HashSet<(i64, i64)> = pt_degree
        .iter()
        .filter(|&(_, &deg)| deg == 1)
        .map(|(&pt, _)| pt)
        .collect();

    // (seg_idx, is_end, dangle_key)
    let mut dangle_records: Vec<(usize, bool, (i64, i64))> = Vec::new();
    for (s_idx, seg) in segments.iter().enumerate() {
        let p0_key = seg.coords[0].key_mm();
        let p1_key = seg.coords[1].key_mm();
        let seg_len = seg.length();
        for (is_end, p_key, other_key) in [(true, p1_key, p0_key), (false, p0_key, p1_key)] {
            if dangling_pts.contains(&p_key) {
                let other_deg = pt_degree.get(&other_key).copied().unwrap_or(0);
                if config.min_stub_len_m > 0.0 && seg_len < config.min_stub_len_m && other_deg >= 3
                {
                    continue;
                }
                dangle_records.push((s_idx, is_end, p_key));
            }
        }
    }

    // Precompute segment unit vectors and bounding boxes
    let m_segs = segments.len();
    let mut seg_u: Vec<(f64, f64)> = Vec::with_capacity(m_segs);
    for seg in &segments {
        let dx = seg.coords[1].x - seg.coords[0].x;
        let dy = seg.coords[1].y - seg.coords[0].y;
        let len = dx.hypot(dy);
        if len > 1e-9 {
            seg_u.push((dx / len, dy / len));
        } else {
            seg_u.push((0.0, 0.0));
        }
    }

    // Step 2a: Adaptive Distance Determination (P90 Clamped to [6.0, 20.0] in Auto Mode)
    let effective_dist = if mode_clean == "auto" {
        let mut candidate_gaps: Vec<f64> = Vec::new();
        for &(s_idx, is_end, _) in &dangle_records {
            let seg = &segments[s_idx];
            let origin = if is_end { seg.coords[1] } else { seg.coords[0] };
            let s_p0 = seg.coords[0].key_mm();
            let s_p1 = seg.coords[1].key_mm();
            let mut min_d = f64::INFINITY;
            for (idx, cand) in segments.iter().enumerate() {
                if idx == s_idx {
                    continue;
                }
                let c_p0 = cand.coords[0].key_mm();
                let c_p1 = cand.coords[1].key_mm();
                // Skip touching segments
                if c_p0 == s_p0 || c_p0 == s_p1 || c_p1 == s_p0 || c_p1 == s_p1 {
                    continue;
                }
                let (_, d, _) = project_point_to_segment(origin, cand.coords[0], cand.coords[1]);
                if d > 0.05 && d <= 30.0 && d < min_d {
                    min_d = d;
                }
            }
            if min_d.is_finite() {
                candidate_gaps.push(min_d);
            }
        }
        if candidate_gaps.len() >= 5 {
            candidate_gaps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let pos = 0.90 * ((candidate_gaps.len() - 1) as f64);
            let lo = pos.floor() as usize;
            let hi = pos.ceil() as usize;
            let frac = pos - (lo as f64);
            let p90 = candidate_gaps[lo] * (1.0 - frac) + candidate_gaps[hi] * frac;
            p90.clamp(6.0, 20.0)
        } else {
            18.39
        }
    } else {
        config.extend_distance_m.max(1.0)
    };

    // Step 2b: Forward Raycasting + Lateral Fallback
    let overshoot_eps = 0.05;
    let ray_search_max = effective_dist * 1.6;
    let cos_min_bound = config
        .angle_max_deg
        .min(180.0 - config.angle_min_deg)
        .to_radians()
        .cos();
    let cos_max_bound = config
        .angle_min_deg
        .min(180.0 - config.angle_max_deg)
        .to_radians()
        .cos();
    let cos_lat_bound = 45.0_f64.to_radians().cos();

    let mut ray_extensions: Vec<Polyline> = Vec::new();
    let mut lateral_extensions: Vec<Polyline> = Vec::new();

    for &(s_idx, is_end, _) in &dangle_records {
        let seg = &segments[s_idx];
        let (dx, dy, origin) = if is_end {
            (
                seg.coords[1].x - seg.coords[0].x,
                seg.coords[1].y - seg.coords[0].y,
                seg.coords[1],
            )
        } else {
            (
                seg.coords[0].x - seg.coords[1].x,
                seg.coords[0].y - seg.coords[1].y,
                seg.coords[0],
            )
        };
        let mag = dx.hypot(dy);
        if mag <= 0.001 {
            continue;
        }
        let ux = dx / mag;
        let uy = dy / mag;
        let target_pt = Point2D::new(
            origin.x + ray_search_max * ux,
            origin.y + ray_search_max * uy,
        );

        let mut best_hit: Option<Point2D> = None;
        let mut min_hit_dist = ray_search_max;

        for (h_idx, cand) in segments.iter().enumerate() {
            if h_idx == s_idx {
                continue;
            }
            if has_multi_levels && (cand.level - seg.level).abs() > 1e-6 {
                continue;
            }
            let abs_dot = (ux * seg_u[h_idx].0 + uy * seg_u[h_idx].1).abs();
            if abs_dot < cos_min_bound - 1e-9 || abs_dot > cos_max_bound + 1e-9 {
                continue;
            }
            if let Some((_, _, ipt)) =
                segment_intersection_params(origin, target_pt, cand.coords[0], cand.coords[1])
            {
                let d = origin.dist(ipt);
                let sin_ang = (1.0 - abs_dot * abs_dot).max(0.0).sqrt().max(0.5);
                let d_perp = d * sin_ang;
                if d > 0.02
                    && d < min_hit_dist
                    && (d <= effective_dist || d_perp <= effective_dist)
                {
                    min_hit_dist = d;
                    best_hit = Some(Point2D::new(
                        ipt.x + overshoot_eps * ux,
                        ipt.y + overshoot_eps * uy,
                    ));
                }
            }
        }

        if let Some(hit_pt) = best_hit {
            ray_extensions.push(Polyline::with_meta(
                Polyline::new(vec![origin, hit_pt]),
                seg.level,
                seg.highway.clone(),
                seg.name.clone(),
            ));
        } else if config.lateral_fallback {
            let lat_tol = config.snap_distance_m.max(3.0).min(effective_dist * 0.25);
            let s_p0 = seg.coords[0].key_mm();
            let s_p1 = seg.coords[1].key_mm();
            let mut best_lat: Option<Point2D> = None;
            let mut min_lat = lat_tol;

            for (idx, cand) in segments.iter().enumerate() {
                if idx == s_idx {
                    continue;
                }
                if has_multi_levels && (cand.level - seg.level).abs() > 1e-6 {
                    continue;
                }
                let c_p0 = cand.coords[0].key_mm();
                let c_p1 = cand.coords[1].key_mm();
                if c_p0 == s_p0 || c_p0 == s_p1 || c_p1 == s_p0 || c_p1 == s_p1 {
                    continue;
                }
                let (proj, d, _) = project_point_to_segment(origin, cand.coords[0], cand.coords[1]);
                if d > 0.02 && d < min_lat {
                    let lat_dx = proj.x - origin.x;
                    let lat_dy = proj.y - origin.y;
                    let lat_mag = lat_dx.hypot(lat_dy);
                    if lat_mag <= 1e-6 {
                        continue;
                    }
                    let lux = lat_dx / lat_mag;
                    let luy = lat_dy / lat_mag;
                    let abs_dot_lat = (lux * seg_u[idx].0 + luy * seg_u[idx].1).abs();
                    if abs_dot_lat <= cos_lat_bound + 1e-9 {
                        min_lat = d;
                        best_lat = Some(Point2D::new(
                            proj.x + overshoot_eps * lux,
                            proj.y + overshoot_eps * luy,
                        ));
                    }
                }
            }

            if let Some(lat_pt) = best_lat {
                lateral_extensions.push(Polyline::with_meta(
                    Polyline::new(vec![origin, lat_pt]),
                    seg.level,
                    seg.highway.clone(),
                    seg.name.clone(),
                ));
            }
        }
    }

    // Step 3: Trim & Planarize + Micro-Stub Pruning
    let mut all_elements = Vec::with_capacity(
        segments.len() + ray_extensions.len() + lateral_extensions.len(),
    );
    all_elements.extend(segments.iter().cloned());
    all_elements.extend(ray_extensions.iter().cloned());
    all_elements.extend(lateral_extensions.iter().cloned());

    let planar_lines = planarize_and_node(&all_elements);

    let mut deg_planar: HashMap<(i64, i64), usize> = HashMap::new();
    for pl in &planar_lines {
        if pl.coords.len() >= 2 {
            *deg_planar.entry(pl.coords[0].key_mm()).or_insert(0) += 1;
            *deg_planar
                .entry(pl.coords.last().unwrap().key_mm())
                .or_insert(0) += 1;
        }
    }

    let stub_cutoff = config.min_stub_len_m.max(overshoot_eps + 0.015);
    let noding_overshoots_count = ray_extensions.len() + lateral_extensions.len();
    let mut pruned_lines: Vec<Polyline> = Vec::with_capacity(planar_lines.len());
    let mut total_trimmed_tails = 0usize;

    for pl in planar_lines {
        let p0 = pl.coords[0].key_mm();
        let p1 = pl.coords.last().unwrap().key_mm();
        let is_dangle =
            deg_planar.get(&p0).copied().unwrap_or(0) == 1
                || deg_planar.get(&p1).copied().unwrap_or(0) == 1;
        if is_dangle && pl.length() < stub_cutoff {
            total_trimmed_tails += 1;
            continue;
        }
        pruned_lines.push(pl);
    }

    let pruned_stubs_count = if config.min_stub_len_m > 0.0 {
        total_trimmed_tails.saturating_sub(noding_overshoots_count)
    } else {
        0
    };

    // Step 4: Collinear Join with Turn Conservation + Dead-End Stub Removal
    let mut final_lines =
        collinear_join_with_turn_conservation(&pruned_lines, config.max_turn_deflection_deg);

    let mut dead_end_stubs_removed_count = 0usize;
    if config.dead_end_stub_len_m > 0.0 {
        let before_stub_count = final_lines.len();
        final_lines = remove_dead_end_stubs(&final_lines, config.dead_end_stub_len_m);
        dead_end_stubs_removed_count = before_stub_count.saturating_sub(final_lines.len());
    }

    // Step 5: Post-Flight Topology Audit
    let raw_junctions = count_graph_junctions(&clean_input);
    let cleaned_junctions = count_graph_junctions(&final_lines);
    let raw_components = count_connected_components(&clean_input);
    let cleaned_components = count_connected_components(&final_lines);
    let final_dangles = count_dangling_endpoints(&final_lines);
    let raw_total_len: f64 = clean_input.iter().map(|ln| ln.length()).sum();
    let cleaned_total_len: f64 = final_lines.iter().map(|ln| ln.length()).sum();
    let len_delta_pct = if raw_total_len > 0.0 {
        (cleaned_total_len - raw_total_len) / raw_total_len * 100.0
    } else {
        0.0
    };

    let mut deg_final: HashMap<(i64, i64), usize> = HashMap::new();
    for fl in &final_lines {
        if fl.coords.len() >= 2 {
            *deg_final.entry(fl.coords[0].key_mm()).or_insert(0) += 1;
            *deg_final
                .entry(fl.coords.last().unwrap().key_mm())
                .or_insert(0) += 1;
        }
    }
    let remaining_stubs = final_lines
        .iter()
        .filter(|fl| {
            config.min_stub_len_m > 0.0
                && fl.length() < config.min_stub_len_m
                && (deg_final.get(&fl.coords[0].key_mm()).copied().unwrap_or(0) == 1
                    || deg_final
                        .get(&fl.coords.last().unwrap().key_mm())
                        .copied()
                        .unwrap_or(0)
                        == 1)
        })
        .count();

    let mut assertions_passed = true;
    if dead_end_stubs_removed_count == 0 && cleaned_junctions < raw_junctions {
        assertions_passed = false;
    }
    if remaining_stubs > 0 {
        assertions_passed = false;
    }
    if cleaned_components > raw_components {
        assertions_passed = false;
    }
    if (dead_end_stubs_removed_count == 0 && len_delta_pct < -0.5) || len_delta_pct > 5.0 {
        assertions_passed = false;
    }

    let circuity_avg = compute_network_circuity_avg(&final_lines);
    let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;

    let report = CleanerReport {
        mode: mode_clean,
        effective_extend_distance: (effective_dist * 100.0).round() / 100.0,
        raw_lines_count: clean_input.len(),
        exploded_segments_count: segments.len(),
        raw_dangles_count: dangling_pts.len(),
        cleaned_dangles_count: final_dangles,
        ray_extensions_count: ray_extensions.len(),
        lateral_connectors_count: lateral_extensions.len(),
        pruned_micro_stubs_count: pruned_stubs_count,
        remaining_micro_stubs_count: remaining_stubs,
        dead_end_stubs_removed_count,
        cleaned_lines_count: final_lines.len(),
        raw_junctions_count: raw_junctions,
        cleaned_junctions_count: cleaned_junctions,
        raw_components_count: raw_components,
        cleaned_components_count: cleaned_components,
        raw_total_length_m: (raw_total_len * 100.0).round() / 100.0,
        cleaned_total_length_m: (cleaned_total_len * 100.0).round() / 100.0,
        length_delta_pct: (len_delta_pct * 100.0).round() / 100.0,
        circuity_avg,
        assertions_passed,
        elapsed_ms,
    };

    (final_lines, report)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_planarize_and_node_cross() {
        let l1 = Polyline::new(vec![Point2D::new(0.0, 50.0), Point2D::new(100.0, 50.0)]);
        let l2 = Polyline::new(vec![Point2D::new(50.0, 0.0), Point2D::new(50.0, 100.0)]);
        let noded = planarize_and_node(&[l1, l2]);
        assert_eq!(noded.len(), 4);
        assert_eq!(count_graph_junctions(&noded), 1);
    }

    #[test]
    fn test_clean_adaptive_watertight_undershoot() {
        let main_st = Polyline::new(vec![Point2D::new(0.0, 0.0), Point2D::new(0.0, 100.0)]);
        let side_st = Polyline::new(vec![Point2D::new(100.0, 50.0), Point2D::new(2.0, 50.0)]);
        let cfg = CleanerConfig::default();
        let (cleaned, report) = clean_network_adaptive_watertight(&[main_st, side_st], &cfg);
        assert_eq!(cleaned.len(), 3);
        assert_eq!(report.cleaned_junctions_count, 1);
        assert_eq!(report.cleaned_components_count, 1);
    }
}
