//! Compressed Sparse Row (CSR) Dual Graph for Turner's (2007) Angular Segment Analysis.
//!
//! Implements:
//! - Exact continuous (`f64`) and discrete `Tulip-1024` turn deflection (`calculate_turn_deflection`).
//! - Automated multi-level bridge/tunnel grade-separation unlinking (`detect_grade_unlinks`).
//! - Contiguous cache-aligned `CSRDualGraph` construction and neighbor traversal.

use crate::cleaner::{explode_to_2point_segments, is_prenoded_segment_map, planarize_and_node};
pub use crate::types::CSRDualGraph;
use crate::types::{Point2D, Polyline, SegmentRecord};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::f64::consts::PI;

/// Calculates bearing angle in radians from `p1` to `p2` (`atan2(dx, dy)`).
#[inline]
pub fn get_bearing(p1: Point2D, p2: Point2D) -> f64 {
    (p2.x - p1.x).atan2(p2.y - p1.y)
}

/// Calculates Turner (2007) / UCL `depthmapX` turn deflection angle between two outward segment bearings.
///
/// - `0.0` = straight through (`0°` deflection, zero turn cost)
/// - `1.0` = right-angle turn (`90°` deflection, cost `1.0`)
/// - `2.0` = complete `180°` reversal (cost `2.0`)
///
/// When `tulip_bins > 0` (e.g. `1024` in `depthmapX`), quantizes angular deflection into
/// discrete Tulip bins before scaling to `[0.0, 2.0]`.
#[inline]
pub fn calculate_turn_deflection(b1: f64, b2: f64, tulip_bins: usize) -> f64 {
    let mut diff = (b1 - b2).abs() % (2.0 * PI);
    if diff > PI {
        diff = 2.0 * PI - diff;
    }
    let deflection_rad = (diff - PI).abs();
    if tulip_bins > 0 {
        let bin_idx = ((deflection_rad / PI) * (tulip_bins as f64 * 0.5)).round();
        bin_idx / (tulip_bins as f64 * 0.25)
    } else {
        deflection_rad / (PI * 0.5)
    }
}

/// Automatically detects grade-separation unlink pairs `(min_id, max_id)` at shared vertices
/// where segments of different elevation/grade `level` cross (both levels have `>= 2` incident
/// segments at that vertex), while preserving 2-segment ramp transitions.
pub fn detect_grade_unlinks(segments: &[SegmentRecord]) -> Vec<(usize, usize)> {
    if segments.is_empty() {
        return Vec::new();
    }

    let mut endpoint_map: HashMap<(i64, i64), Vec<(usize, u8)>> = HashMap::new();
    for (i, seg) in segments.iter().enumerate() {
        endpoint_map.entry(seg.p0.key_cm()).or_default().push((i, 0));
        endpoint_map.entry(seg.p1.key_cm()).or_default().push((i, 1));
    }

    let mut unlinks: BTreeSet<(usize, usize)> = BTreeSet::new();
    for members in endpoint_map.values() {
        if members.len() < 2 {
            continue;
        }
        let mut level_counts: HashMap<i64, usize> = HashMap::new();
        for &(idx, _) in members {
            let lv_key = (segments[idx].level * 1000.0).round() as i64;
            *level_counts.entry(lv_key).or_insert(0) += 1;
        }
        if level_counts.len() < 2 {
            continue;
        }
        for a in 0..members.len() {
            for b in (a + 1)..members.len() {
                let i = members[a].0;
                let j = members[b].0;
                if i == j {
                    continue;
                }
                let lv_i = (segments[i].level * 1000.0).round() as i64;
                let lv_j = (segments[j].level * 1000.0).round() as i64;
                if lv_i != lv_j
                    && level_counts.get(&lv_i).copied().unwrap_or(0) >= 2
                    && level_counts.get(&lv_j).copied().unwrap_or(0) >= 2
                {
                    unlinks.insert((i.min(j), i.max(j)));
                }
            }
        }
    }

    unlinks.into_iter().collect()
}

impl CSRDualGraph {
    /// Planarizes input polylines, explodes them into 2-point `SegmentRecord`s,
    /// detects grade-separation unlinks, and constructs the `CSRDualGraph`.
    pub fn from_polylines(
        lines: &[Polyline],
        tulip_bins: usize,
        angular_weight_ratio: f64,
    ) -> (Self, Vec<SegmentRecord>) {
        let noded = if is_prenoded_segment_map(lines) {
            lines.to_vec()
        } else {
            planarize_and_node(lines)
        };
        let segments = explode_to_2point_segments(&noded);
        let unlinks = detect_grade_unlinks(&segments);
        let graph = Self::from_segments(&segments, &unlinks, tulip_bins, angular_weight_ratio);
        (graph, segments)
    }

    /// Builds the `CSRDualGraph` from planarized 2-point `SegmentRecord`s and explicit `unlinks`.
    pub fn from_segments(
        segments: &[SegmentRecord],
        unlinks: &[(usize, usize)],
        tulip_bins: usize,
        angular_weight_ratio: f64,
    ) -> Self {
        let n = segments.len();
        let mut unlinks_set: HashSet<(usize, usize)> = unlinks
            .iter()
            .map(|&(a, b)| (a.min(b), a.max(b)))
            .collect();
        for pair in detect_grade_unlinks(segments) {
            unlinks_set.insert(pair);
        }

        let mut segment_lengths = vec![0.0f32; n];
        let mut segment_weights = vec![1.0f64; n];
        let mut segment_coords = vec![[0.0f64; 4]; n];
        let mut segment_azimuths = vec![0.0f64; n];
        let mut endpoint_map: HashMap<(i64, i64), Vec<(usize, u8)>> = HashMap::new();
        let mut fw_dirs = vec![(1.0f64, 0.0f64); n];
        let mut bk_dirs = vec![(-1.0f64, 0.0f64); n];

        for (i, seg) in segments.iter().enumerate() {
            segment_lengths[i] = seg.length_m as f32;
            segment_weights[i] = seg.length_m;
            segment_coords[i] = [seg.p0.x, seg.p0.y, seg.p1.x, seg.p1.y];
            segment_azimuths[i] = seg.azimuth_deg;
            let len = seg.length_m.max(1e-12);
            let dx = (seg.p1.x - seg.p0.x) / len;
            let dy = (seg.p1.y - seg.p0.y) / len;
            fw_dirs[i] = (dx, dy);
            bk_dirs[i] = (-dx, -dy);

            endpoint_map.entry(seg.p0.key_cm()).or_default().push((i, 0));
            endpoint_map.entry(seg.p1.key_cm()).or_default().push((i, 1));
        }

        // adj[u] maps target_v -> (weight, avg_len, ang_cost)
        let mut adj: Vec<BTreeMap<usize, (f64, f64, f64)>> = vec![BTreeMap::new(); n];
        let mut forward_segconns: Vec<Vec<(i8, u32, f32)>> = vec![Vec::new(); n];
        let mut back_segconns: Vec<Vec<(i8, u32, f32)>> = vec![Vec::new(); n];

        for members in endpoint_map.values() {
            if members.len() < 2 {
                continue;
            }
            for a in 0..members.len() {
                for b in (a + 1)..members.len() {
                    let (i, end_i) = members[a];
                    let (j, end_j) = members[b];
                    if i == j {
                        continue;
                    }
                    let pair = (i.min(j), i.max(j));
                    if unlinks_set.contains(&pair) {
                        continue;
                    }

                    // Directed connection from i (exiting at end_i) into j (entering at end_j)
                    let dir_i_exit = if end_i == 1 { fw_dirs[i] } else { bk_dirs[i] };
                    let conn_dir_j: i8 = if end_j == 0 { 1 } else { -1 };
                    let dir_j_enter = if conn_dir_j == 1 { fw_dirs[j] } else { bk_dirs[j] };
                    let dot_ij = (dir_i_exit.0 * dir_j_enter.0 + dir_i_exit.1 * dir_j_enter.1).clamp(-1.0, 1.0);
                    let w_ij_f32 = (dot_ij.acos() / (PI * 0.5)) as f32;
                    if end_i == 1 {
                        forward_segconns[i].push((conn_dir_j, j as u32, w_ij_f32));
                    } else {
                        back_segconns[i].push((conn_dir_j, j as u32, w_ij_f32));
                    }

                    // Directed connection from j (exiting at end_j) into i (entering at end_i)
                    let dir_j_exit = if end_j == 1 { fw_dirs[j] } else { bk_dirs[j] };
                    let conn_dir_i: i8 = if end_i == 0 { 1 } else { -1 };
                    let dir_i_enter = if conn_dir_i == 1 { fw_dirs[i] } else { bk_dirs[i] };
                    let dot_ji = (dir_j_exit.0 * dir_i_enter.0 + dir_j_exit.1 * dir_i_enter.1).clamp(-1.0, 1.0);
                    let w_ji_f32 = (dot_ji.acos() / (PI * 0.5)) as f32;
                    if end_j == 1 {
                        forward_segconns[j].push((conn_dir_i, i as u32, w_ji_f32));
                    } else {
                        back_segconns[j].push((conn_dir_i, i as u32, w_ji_f32));
                    }

                    let b_i = if end_i == 0 {
                        segments[i].b0
                    } else {
                        segments[i].b1
                    };
                    let b_j = if end_j == 0 {
                        segments[j].b0
                    } else {
                        segments[j].b1
                    };

                    let ang_cost = calculate_turn_deflection(b_i, b_j, tulip_bins);
                    let avg_len = 0.5 * (segments[i].length_m + segments[j].length_m);
                    let w = (ang_cost + angular_weight_ratio * avg_len).max(1e-4);

                    match adj[i].get(&j) {
                        Some(&(prev_w, _, _)) if w >= prev_w => {}
                        _ => {
                            adj[i].insert(j, (w, avg_len, w_ij_f32 as f64));
                        }
                    }
                    match adj[j].get(&i) {
                        Some(&(prev_w, _, _)) if w >= prev_w => {}
                        _ => {
                            adj[j].insert(i, (w, avg_len, w_ji_f32 as f64));
                        }
                    }
                }
            }
        }

        for u in 0..n {
            forward_segconns[u].sort_by_key(|&(_, r, _)| r);
            forward_segconns[u].dedup_by_key(|&mut (_, r, _)| r);
            back_segconns[u].sort_by_key(|&(_, r, _)| r);
            back_segconns[u].dedup_by_key(|&mut (_, r, _)| r);
        }

        let total_edges: usize = adj.iter().map(|m| m.len()).sum();
        let mut row_offsets = vec![0u32; n + 1];
        let mut target_nodes = Vec::with_capacity(total_edges);
        let mut angular_weights = Vec::with_capacity(total_edges);
        let mut metric_lengths = Vec::with_capacity(total_edges);
        let mut connectivity = vec![0i32; n];
        let mut angular_connectivity = vec![0.0f64; n];

        let mut curr_offset = 0u32;
        for u in 0..n {
            row_offsets[u] = curr_offset;
            let neighbors = &adj[u];
            connectivity[u] = (forward_segconns[u].len() + back_segconns[u].len()) as i32;
            let mut ang_conn_f32 = 0.0f32;
            for &(_, _, w) in &forward_segconns[u] {
                ang_conn_f32 += w;
            }
            for &(_, _, w) in &back_segconns[u] {
                ang_conn_f32 += w;
            }
            for (&v, &(weight, metric_m, _)) in neighbors {
                target_nodes.push(v as u32);
                angular_weights.push(weight as f32);
                metric_lengths.push(metric_m as f32);
                curr_offset += 1;
            }
            angular_connectivity[u] = ang_conn_f32 as f64;
        }
        row_offsets[n] = curr_offset;

        Self {
            num_nodes: n,
            row_offsets,
            target_nodes,
            angular_weights,
            metric_lengths,
            segment_lengths,
            segment_weights,
            connectivity,
            angular_connectivity,
            segment_coords,
            segment_azimuths,
            forward_segconns,
            back_segconns,
        }
    }

    /// Returns `(target_nodes, angular_weights, metric_lengths)` slices for segment `u`.
    #[inline]
    pub fn neighbors(&self, u: usize) -> (&[u32], &[f32], &[f32]) {
        let start = self.row_offsets[u] as usize;
        let end = self.row_offsets[u + 1] as usize;
        (
            &self.target_nodes[start..end],
            &self.angular_weights[start..end],
            &self.metric_lengths[start..end],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_turn_deflection_values() {
        // Opposite outward bearings from shared vertex = straight through (0.0 cost)
        let b_north = get_bearing(Point2D::new(0.0, 0.0), Point2D::new(0.0, 10.0));
        let b_south = get_bearing(Point2D::new(0.0, 0.0), Point2D::new(0.0, -10.0));
        let b_east = get_bearing(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0));

        assert!((calculate_turn_deflection(b_north, b_south, 0) - 0.0).abs() < 1e-9);
        assert!((calculate_turn_deflection(b_north, b_east, 0) - 1.0).abs() < 1e-9);
        assert!((calculate_turn_deflection(b_north, b_north, 0) - 2.0).abs() < 1e-9);

        // Tulip-1024 quantization
        assert!((calculate_turn_deflection(b_north, b_south, 1024) - 0.0).abs() < 1e-9);
        assert!((calculate_turn_deflection(b_north, b_east, 1024) - 1.0).abs() < 1e-9);
    }
}
