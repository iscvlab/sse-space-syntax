//! High-Performance Angular Segment Analysis (`ASA`) & Multi-Source Origin Step Depth Solver.
//!
//! Implements:
//! 1. Turner (2007) Angular, Metric, and Topological Segment Analysis across multiple radii in a single pass.
//! 2. Brandes (2001) Betweenness Centrality (`Choice`) with unweighted, segment-length, and custom weighting.
//! 3. Closeness `Integration`, `Total_Depth`, `Mean_Depth`, `Node_Count`, `Control`, and `Controllability`.
//! 4. Hillier, Yang & Turner (2012) `NACH` and `NAIN` normalizations:
//!    - `NACH = ln(Choice + 1) / ln(TotalDepth + 3)`
//!    - `NAIN = (NodeCount ^ 1.2) / (TotalDepth + 2)`
//! 5. Space Syntax `Intelligibility R²`, `Synergy R²`, and Multi-Origin Step Depth.

use crate::types::{
    pearson_r2, AnalysisType, AsaConfig, AsaResults, CSRDualGraph, RadiusType, StepDepthResults,
    WeightMode,
};
use rayon::prelude::*;
use std::cmp::Ordering;
use std::collections::{BTreeMap, BinaryHeap};
use std::time::Instant;

#[derive(Copy, Clone, Debug)]
struct AsaHeapEntry {
    cost: f64,
    rad: f64,
    node: usize,
}

impl PartialEq for AsaHeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.cost.to_bits() == other.cost.to_bits()
            && self.rad.to_bits() == other.rad.to_bits()
            && self.node == other.node
    }
}

impl Eq for AsaHeapEntry {}

impl PartialOrd for AsaHeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for AsaHeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap ordering: reverse comparison
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
            .then_with(|| other.rad.partial_cmp(&self.rad).unwrap_or(Ordering::Equal))
            .then_with(|| other.node.cmp(&self.node))
    }
}

/// Computes segment-level Turner/Hillier `Control` and `Controllability` on `CSRDualGraph`:
/// - `control[u] = sum_{v in N(u)} (1.0 / max(connectivity[v], 1))`
/// - `controllability[u] = k_1 / max(k_1 + k_2, 1)`
pub fn compute_segment_control_metrics(graph: &CSRDualGraph) -> (Vec<f64>, Vec<f64>) {
    let n = graph.num_nodes;
    let mut control = vec![0.0f64; n];
    let mut controllability = vec![0.0f64; n];
    let mut visited_stamp = vec![-1i32; n];

    for u in 0..n {
        let start_u = graph.row_offsets[u] as usize;
        let end_u = graph.row_offsets[u + 1] as usize;
        let k_1 = graph.connectivity[u];
        if k_1 <= 0 {
            continue;
        }

        let stamp = u as i32;
        visited_stamp[u] = stamp;
        let mut ctrl_sum = 0.0;
        for e1 in start_u..end_u {
            let v = graph.target_nodes[e1] as usize;
            visited_stamp[v] = stamp;
            let deg_v = graph.connectivity[v].max(1) as f64;
            ctrl_sum += 1.0 / deg_v;
        }
        control[u] = ctrl_sum;

        let mut k_2 = 0usize;
        for e1 in start_u..end_u {
            let v = graph.target_nodes[e1] as usize;
            let start_v = graph.row_offsets[v] as usize;
            let end_v = graph.row_offsets[v + 1] as usize;
            for e2 in start_v..end_v {
                let w = graph.target_nodes[e2] as usize;
                if visited_stamp[w] != stamp {
                    visited_stamp[w] = stamp;
                    k_2 += 1;
                }
            }
        }

        let denom = (k_1 as usize + k_2).max(1) as f64;
        controllability[u] = (k_1 as f64) / denom;
    }

    (control, controllability)
}

#[inline]
fn format_radius_label(r: f64) -> String {
    if !r.is_finite() || r >= 1e8 {
        "Rn".to_string()
    } else {
        format!("R{}m", r.round() as i64)
    }
}

#[derive(Copy, Clone, Debug)]
struct TulipSegData {
    metricdepth: f32,
    dir: i8,
    ref_i: u32,
    prev_dir: i8,
    prev_ref: i32,
    segdepth: i32,
    coverage: u32,
}

#[inline]
fn insert_tulip_bin(bin: &mut Vec<TulipSegData>, item: TulipSegData) {
    // Matches depthmapX `std::lower_bound` with `a.metricdepth > b.metricdepth` (pop from back = shortest metricdepth)
    let mut lo = 0usize;
    let mut hi = bin.len();
    while lo < hi {
        let mid = (lo + hi) / 2;
        if bin[mid].metricdepth > item.metricdepth {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    bin.insert(lo, item);
}

fn solve_asa_tulip_directed(
    graph: &CSRDualGraph,
    config: &AsaConfig,
    t0: Instant,
    mut attributes: BTreeMap<String, Vec<f64>>,
    conn_f64: Vec<f64>,
) -> AsaResults {
    let n = graph.num_nodes;
    let tulip_bins = (config.tulip_bins / 2) + 1;
    let denom_conv = ((tulip_bins as f64) - 1.0) * 0.5;

    let weights_f32: Vec<f32> = match config.weight_mode {
        WeightMode::Custom => graph.segment_weights.iter().map(|&w| w as f32).collect(),
        _ => graph.segment_lengths.clone(),
    };
    let use_weighted_primary = !matches!(config.weight_mode, WeightMode::Unweighted);

    // Sort radii ascending with global (-1.0) at the end, as required by segmtulip.cpp bitmask shifting
    let r_count = config.radii.len().min(30);
    let mut sorted_radii: Vec<(usize, f64)> = config
        .radii
        .iter()
        .take(r_count)
        .copied()
        .enumerate()
        .map(|(idx, r)| {
            if !r.is_finite() || r >= 1e8 {
                (idx, -1.0)
            } else if matches!(config.radius_type, RadiusType::Angular) {
                (idx, (r * (tulip_bins as f64) * 0.5).floor())
            } else {
                (idx, r)
            }
        })
        .collect();
    sorted_radii.sort_by(|a, b| match (a.1 < 0.0, b.1 < 0.0) {
        (true, true) => a.0.cmp(&b.0),
        (true, false) => Ordering::Greater,
        (false, true) => Ordering::Less,
        (false, false) => a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal),
    });

    let radius_vals: Vec<f64> = sorted_radii.iter().map(|&(_, r)| r).collect();
    let radiusmask: u32 = if r_count == 32 {
        u32::MAX
    } else {
        (1u32 << r_count) - 1
    };

    let max_sources = if config.max_sources_per_radius == 0 {
        n
    } else {
        config.max_sources_per_radius
    };
    let (sources, scale_mult): (Vec<usize>, f64) = if n <= max_sources {
        ((0..n).collect(), 1.0)
    } else {
        let step = (n / max_sources).max(1);
        ((0..n).step_by(step).collect(), step as f64)
    };

    let num_sources = sources.len();
    let n_chunks = num_sources
        .min(rayon::current_num_threads().max(1) * 4)
        .max(1);

    struct TulipChunkOut {
        choice_unw: Vec<Vec<[f64; 2]>>,
        choice_wgt: Vec<Vec<[f64; 2]>>,
        rows: Vec<(
            usize,
            Vec<f64>,
            Vec<f64>,
            Vec<f64>,
            Vec<f64>,
            Vec<f64>,
            Vec<f64>,
        )>,
    }

    let chunk_outs: Vec<TulipChunkOut> = (0..n_chunks)
        .into_par_iter()
        .map(|chunk_id| {
            let start_s = (chunk_id * num_sources) / n_chunks;
            let end_s = ((chunk_id + 1) * num_sources) / n_chunks;

            let mut choice_unw = vec![vec![[0.0f64; 2]; n]; r_count];
            let mut choice_wgt = vec![vec![[0.0f64; 2]; n]; r_count];
            let mut rows = Vec::with_capacity(end_s.saturating_sub(start_s));

            let mut bins: Vec<Vec<TulipSegData>> = vec![Vec::new(); tulip_bins];
            let mut uncovered = vec![[radiusmask; 2]; n];
            let mut audit_depth = vec![vec![[0i32; 2]; n]; r_count];
            let mut audit_prev = vec![vec![[(0i8, -1i32); 2]; n]; r_count];
            let mut audit_leaf = vec![vec![[true; 2]; n]; r_count];
            let mut audit_choice_cov = vec![vec![[false; 2]; n]; r_count];
            let mut visited_nodes: Vec<u32> = Vec::with_capacity(n);
            let mut in_visited = vec![false; n];

            for &cursor in &sources[start_s..end_s] {
                for b in &mut bins {
                    b.clear();
                }
                let rootseglength = graph.segment_lengths[cursor];
                let rootweight = weights_f32[cursor] as f64;

                insert_tulip_bin(
                    &mut bins[0],
                    TulipSegData {
                        metricdepth: 0.5 * rootseglength,
                        dir: 0,
                        ref_i: cursor as u32,
                        prev_dir: 0,
                        prev_ref: -1,
                        segdepth: 0,
                        coverage: radiusmask,
                    },
                );

                let mut depthlevel: i32 = 0;
                let mut opencount: usize = 1;
                let mut currentbin: usize = 0;

                while opencount > 0 {
                    while bins[currentbin].is_empty() {
                        depthlevel += 1;
                        currentbin += 1;
                        if currentbin == tulip_bins {
                            currentbin = 0;
                        }
                    }
                    let lineindex = bins[currentbin].pop().unwrap();
                    opencount -= 1;

                    let ref_i = lineindex.ref_i as usize;
                    let dir = if lineindex.dir == 1 { 0usize } else { 1usize };
                    let coverage = lineindex.coverage & uncovered[ref_i][dir];
                    if coverage != 0 {
                        if !in_visited[ref_i] {
                            in_visited[ref_i] = true;
                            visited_nodes.push(ref_i as u32);
                        }
                        let rbinbase: usize;
                        if lineindex.prev_ref != -1 {
                            uncovered[ref_i][dir] &= !coverage;
                            let mut rbin = coverage.trailing_zeros() as usize;
                            rbinbase = rbin;
                            let prev_u = lineindex.prev_ref as usize;
                            let prev_d = if lineindex.prev_dir == 1 { 0usize } else { 1usize };
                            while rbin < r_count {
                                if ((coverage >> rbin) & 1) == 1 {
                                    audit_depth[rbin][ref_i][dir] = depthlevel;
                                    audit_prev[rbin][ref_i][dir] =
                                        (lineindex.prev_dir, lineindex.prev_ref);
                                    audit_leaf[rbin][prev_u][prev_d] = false;
                                }
                                rbin += 1;
                            }
                        } else {
                            rbinbase = 0;
                            uncovered[ref_i][0] &= !coverage;
                            uncovered[ref_i][1] &= !coverage;
                        }

                        if lineindex.dir != -1 {
                            for &(conn_dir, conn_ref_u32, w_f32) in &graph.forward_segconns[ref_i]
                            {
                                let conn_ref = conn_ref_u32 as usize;
                                let cd_idx = if conn_dir == 1 { 0usize } else { 1usize };
                                if (uncovered[conn_ref][cd_idx] & coverage) != 0 {
                                    let extradepth =
                                        ((w_f32 as f64) * (tulip_bins as f64) * 0.5).floor() as i32;
                                    let seglength = graph.segment_lengths[conn_ref];
                                    let mut rbin = rbinbase;
                                    match config.radius_type {
                                        RadiusType::Angular => {
                                            while rbin < r_count
                                                && radius_vals[rbin] >= 0.0
                                                && ((depthlevel + extradepth) as f64)
                                                    > radius_vals[rbin]
                                            {
                                                rbin += 1;
                                            }
                                        }
                                        RadiusType::Metric => {
                                            while rbin < r_count
                                                && radius_vals[rbin] >= 0.0
                                                && ((lineindex.metricdepth + seglength * 0.5)
                                                    as f64)
                                                    > radius_vals[rbin]
                                            {
                                                rbin += 1;
                                            }
                                        }
                                        RadiusType::Topological => {
                                            if rbin < r_count
                                                && radius_vals[rbin] >= 0.0
                                                && (lineindex.segdepth as f64) >= radius_vals[rbin]
                                            {
                                                rbin += 1;
                                            }
                                        }
                                    }
                                    if rbin < r_count && (coverage >> rbin) != 0 {
                                        let next_cov = (coverage >> rbin) << rbin;
                                        let b_idx = (currentbin
                                            + tulip_bins
                                            + (extradepth as usize))
                                            % tulip_bins;
                                        insert_tulip_bin(
                                            &mut bins[b_idx],
                                            TulipSegData {
                                                metricdepth: lineindex.metricdepth + seglength,
                                                dir: conn_dir,
                                                ref_i: conn_ref_u32,
                                                prev_dir: 1,
                                                prev_ref: ref_i as i32,
                                                segdepth: lineindex.segdepth + 1,
                                                coverage: next_cov,
                                            },
                                        );
                                        opencount += 1;
                                    }
                                }
                            }
                        }

                        if lineindex.dir != 1 {
                            for &(conn_dir, conn_ref_u32, w_f32) in &graph.back_segconns[ref_i] {
                                let conn_ref = conn_ref_u32 as usize;
                                let cd_idx = if conn_dir == 1 { 0usize } else { 1usize };
                                if (uncovered[conn_ref][cd_idx] & coverage) != 0 {
                                    let extradepth =
                                        ((w_f32 as f64) * (tulip_bins as f64) * 0.5).floor() as i32;
                                    let seglength = graph.segment_lengths[conn_ref];
                                    let mut rbin = rbinbase;
                                    match config.radius_type {
                                        RadiusType::Angular => {
                                            while rbin < r_count
                                                && radius_vals[rbin] >= 0.0
                                                && ((depthlevel + extradepth) as f64)
                                                    > radius_vals[rbin]
                                            {
                                                rbin += 1;
                                            }
                                        }
                                        RadiusType::Metric => {
                                            while rbin < r_count
                                                && radius_vals[rbin] >= 0.0
                                                && ((lineindex.metricdepth + seglength * 0.5)
                                                    as f64)
                                                    > radius_vals[rbin]
                                            {
                                                rbin += 1;
                                            }
                                        }
                                        RadiusType::Topological => {
                                            if rbin < r_count
                                                && radius_vals[rbin] >= 0.0
                                                && (lineindex.segdepth as f64) >= radius_vals[rbin]
                                            {
                                                rbin += 1;
                                            }
                                        }
                                    }
                                    if rbin < r_count && (coverage >> rbin) != 0 {
                                        let next_cov = (coverage >> rbin) << rbin;
                                        let b_idx = (currentbin
                                            + tulip_bins
                                            + (extradepth as usize))
                                            % tulip_bins;
                                        insert_tulip_bin(
                                            &mut bins[b_idx],
                                            TulipSegData {
                                                metricdepth: lineindex.metricdepth + seglength,
                                                dir: conn_dir,
                                                ref_i: conn_ref_u32,
                                                prev_dir: -1,
                                                prev_ref: ref_i as i32,
                                                segdepth: lineindex.segdepth + 1,
                                                coverage: next_cov,
                                            },
                                        );
                                        opencount += 1;
                                    }
                                }
                            }
                        }
                    }
                }

                visited_nodes.sort_unstable();

                let mut s_nc = vec![0.0f64; r_count];
                let mut s_td_unw = vec![0.0f64; r_count];
                let mut s_td_wgt = vec![0.0f64; r_count];
                let mut s_tot_w = vec![0.0f64; r_count];
                let mut s_integ_unw = vec![0.0f64; r_count];
                let mut s_integ_wgt = vec![0.0f64; r_count];

                for k in 0..r_count {
                    let mut curs_nc = 0.0f64;
                    let mut curs_td = 0.0f64;
                    let mut curs_tw = 0.0f64;
                    let mut curs_w_td = 0.0f64;

                    for &j_u32 in &visited_nodes {
                        let j = j_u32 as usize;
                        let m0 = ((uncovered[j][0] >> k) & 1) == 0;
                        let m1 = ((uncovered[j][1] >> k) & 1) == 0;
                        if m0 || m1 {
                            let dir = if m0 && m1 {
                                if audit_depth[k][j][0] < audit_depth[k][j][1] {
                                    0usize
                                } else {
                                    1usize
                                }
                            } else if m0 {
                                0usize
                            } else {
                                1usize
                            };

                            let d_val = audit_depth[k][j][dir] as f64;
                            let w_j = weights_f32[j] as f64;
                            curs_nc += 1.0;
                            curs_td += d_val;
                            curs_tw += w_j;
                            curs_w_td += d_val * w_j;

                            if audit_leaf[k][j][dir] && j != cursor {
                                let mut here_dir: i8 = if dir == 0 { 1 } else { -1 };
                                let mut here_ref: usize = j;
                                let mut choicecount = 0i64;
                                let mut choiceweight = 0.0f64;

                                while here_ref != cursor {
                                    let heredir = if here_dir == 1 { 0usize } else { 1usize };
                                    choice_unw[k][here_ref][heredir] +=
                                        (choicecount as f64) * scale_mult;
                                    choice_wgt[k][here_ref][heredir] += choiceweight * scale_mult;

                                    if !audit_choice_cov[k][here_ref][heredir] {
                                        choicecount += 1;
                                        let prod_w = (weights_f32[here_ref] as f64) * rootweight;
                                        choiceweight += prod_w;
                                        audit_choice_cov[k][here_ref][heredir] = true;
                                        choice_wgt[k][here_ref][heredir] +=
                                            (prod_w * 0.5) * scale_mult;
                                    }
                                    let (p_dir, p_ref) = audit_prev[k][here_ref][heredir];
                                    if p_ref < 0 {
                                        break;
                                    }
                                    here_dir = p_dir;
                                    here_ref = p_ref as usize;
                                }
                                let root_heredir = if here_dir == 1 { 0usize } else { 1usize };
                                choice_wgt[k][here_ref][root_heredir] +=
                                    (choiceweight * 0.5) * scale_mult;
                            }
                        }
                    }

                    let td_conv = (curs_td / denom_conv) as f32 as f64;
                    let w_td_conv = (curs_w_td / denom_conv) as f32 as f64;
                    s_nc[k] = (curs_nc as f32) as f64;
                    s_tot_w[k] = (curs_tw as f32) as f64;
                    if curs_nc > 1.0 {
                        s_td_unw[k] = td_conv;
                        s_td_wgt[k] = w_td_conv;
                        s_integ_unw[k] = if td_conv > 1e-9 {
                            ((curs_nc * curs_nc / td_conv) as f32) as f64
                        } else {
                            -1.0
                        };
                        s_integ_wgt[k] = if w_td_conv > 1e-9 {
                            ((curs_tw * curs_tw / w_td_conv) as f32) as f64
                        } else {
                            -1.0
                        };
                    } else {
                        s_td_unw[k] = -1.0;
                        s_td_wgt[k] = -1.0;
                        s_integ_unw[k] = -1.0;
                        s_integ_wgt[k] = -1.0;
                    }
                }

                rows.push((
                    cursor,
                    s_nc,
                    s_td_unw,
                    s_td_wgt,
                    s_tot_w,
                    s_integ_unw,
                    s_integ_wgt,
                ));

                for &j_u32 in &visited_nodes {
                    let j = j_u32 as usize;
                    uncovered[j] = [radiusmask; 2];
                    in_visited[j] = false;
                    for k in 0..r_count {
                        audit_depth[k][j] = [0; 2];
                        audit_prev[k][j] = [(0, -1); 2];
                        audit_leaf[k][j] = [true; 2];
                        audit_choice_cov[k][j] = [false; 2];
                    }
                }
                visited_nodes.clear();
            }

            TulipChunkOut {
                choice_unw,
                choice_wgt,
                rows,
            }
        })
        .collect();

    let mut choice_unw_all = vec![vec![0.0f64; n]; r_count];
    let mut choice_wgt_all = vec![vec![0.0f64; n]; r_count];
    let mut nc_all = vec![vec![0.0f64; n]; r_count];
    let mut td_unw_all = vec![vec![0.0f64; n]; r_count];
    let mut td_wgt_all = vec![vec![0.0f64; n]; r_count];
    let mut tot_w_all = vec![vec![0.0f64; n]; r_count];
    let mut integ_unw_all = vec![vec![0.0f64; n]; r_count];
    let mut integ_wgt_all = vec![vec![0.0f64; n]; r_count];

    let mut accum_unw = vec![vec![[0.0f64; 2]; n]; r_count];
    let mut accum_wgt = vec![vec![[0.0f64; 2]; n]; r_count];

    for chunk in chunk_outs {
        for k in 0..r_count {
            for u in 0..n {
                accum_unw[k][u][0] += chunk.choice_unw[k][u][0];
                accum_unw[k][u][1] += chunk.choice_unw[k][u][1];
                accum_wgt[k][u][0] += chunk.choice_wgt[k][u][0];
                accum_wgt[k][u][1] += chunk.choice_wgt[k][u][1];
            }
        }
        for (s, s_nc, s_td_u, s_td_w, s_tw, s_ig_u, s_ig_w) in chunk.rows {
            for k in 0..r_count {
                nc_all[k][s] = s_nc[k];
                td_unw_all[k][s] = s_td_u[k];
                td_wgt_all[k][s] = s_td_w[k];
                tot_w_all[k][s] = s_tw[k];
                integ_unw_all[k][s] = s_ig_u[k];
                integ_wgt_all[k][s] = s_ig_w[k];
            }
        }
    }

    for k in 0..r_count {
        for u in 0..n {
            choice_unw_all[k][u] = ((accum_unw[k][u][0] + accum_unw[k][u][1]) as f32) as f64;
            choice_wgt_all[k][u] = ((accum_wgt[k][u][0] + accum_wgt[k][u][1]) as f32) as f64;
        }
    }

    if num_sources < n && !sources.is_empty() {
        let step = (n / max_sources).max(1);
        for u in 0..n {
            let nearest_s = ((u / step) * step).min(n - 1);
            if nearest_s != u {
                for k in 0..r_count {
                    nc_all[k][u] = nc_all[k][nearest_s];
                    td_unw_all[k][u] = td_unw_all[k][nearest_s];
                    td_wgt_all[k][u] = td_wgt_all[k][nearest_s];
                    tot_w_all[k][u] = tot_w_all[k][nearest_s];
                    integ_unw_all[k][u] = integ_unw_all[k][nearest_s];
                    integ_wgt_all[k][u] = integ_wgt_all[k][nearest_s];
                }
            }
        }
    }

    let mut r_labels = Vec::with_capacity(r_count);
    for (k, &(orig_idx, _)) in sorted_radii.iter().enumerate() {
        let orig_r = config.radii[orig_idx];
        let r_label = format_radius_label(orig_r);
        r_labels.push(r_label.clone());

        let choice_primary = if use_weighted_primary {
            &choice_wgt_all[k]
        } else {
            &choice_unw_all[k]
        };
        let closeness_primary = if use_weighted_primary {
            &integ_wgt_all[k]
        } else {
            &integ_unw_all[k]
        };
        let td_primary = if use_weighted_primary {
            &td_wgt_all[k]
        } else {
            &td_unw_all[k]
        };
        let node_count = &nc_all[k];

        let mut mean_depth = vec![0.0f64; n];
        let mut nach = vec![0.0f64; n];
        let mut nain = vec![0.0f64; n];

        for i in 0..n {
            let safe_c = choice_primary[i].max(0.0);
            let safe_td = td_primary[i].max(0.0);
            let safe_nc = node_count[i].max(1.0);
            mean_depth[i] = safe_td / safe_nc;
            nach[i] = (safe_c + 1.0).ln() / (safe_td + 3.0).ln();
            nain[i] = safe_nc.powf(1.2) / (td_unw_all[k][i].max(0.0) + 2.0);
        }

        if config.include_normalized {
            let max_c = choice_primary.iter().copied().fold(0.0f64, f64::max);
            let max_i = closeness_primary.iter().copied().fold(0.0f64, f64::max);
            let mut nach_norm = vec![0.0f64; n];
            let mut nain_norm = vec![0.0f64; n];
            if max_c > 0.0 {
                let denom_c = (max_c + 1.0).ln();
                for i in 0..n {
                    nach_norm[i] = (choice_primary[i].max(0.0) + 1.0).ln() / denom_c;
                }
            }
            if max_i > 0.0 {
                let denom_i = (max_i + 1.0).ln();
                for i in 0..n {
                    nain_norm[i] = (closeness_primary[i].max(0.0) + 1.0).ln() / denom_i;
                }
            }
            attributes.insert(format!("NACH_Norm_{}", r_label), nach_norm);
            attributes.insert(format!("NAIN_Norm_{}", r_label), nain_norm);
        }

        attributes.insert(format!("Choice_{}", r_label), choice_primary.clone());
        attributes.insert(format!("Integration_{}", r_label), closeness_primary.clone());
        attributes.insert(format!("Total_Depth_{}", r_label), td_primary.clone());
        attributes.insert(format!("Mean_Depth_{}", r_label), mean_depth);
        attributes.insert(format!("Node_Count_{}", r_label), node_count.clone());
        attributes.insert(format!("NACH_{}", r_label), nach);
        attributes.insert(format!("NAIN_{}", r_label), nain);

        // Also expose unweighted & weighted T1024 columns explicitly for 1-to-1 depthmapX column lookup
        attributes.insert(format!("T1024_Choice_{}", r_label), choice_unw_all[k].clone());
        attributes.insert(
            format!("T1024_Choice_Wgt_{}", r_label),
            choice_wgt_all[k].clone(),
        );
        attributes.insert(
            format!("T1024_Integration_{}", r_label),
            integ_unw_all[k].clone(),
        );
        attributes.insert(
            format!("T1024_Integration_Wgt_{}", r_label),
            integ_wgt_all[k].clone(),
        );
        attributes.insert(
            format!("T1024_Total_Depth_{}", r_label),
            td_unw_all[k].clone(),
        );
        attributes.insert(
            format!("T1024_Total_Depth_Wgt_{}", r_label),
            td_wgt_all[k].clone(),
        );
        attributes.insert(
            format!("T1024_Total_Segment_Length_{}", r_label),
            tot_w_all[k].clone(),
        );
    }

    let global_label = if r_labels.iter().any(|l| l == "Rn") {
        "Rn".to_string()
    } else {
        r_labels.last().cloned().unwrap_or_else(|| "Rn".to_string())
    };

    let nain_global_key = format!("NAIN_{}", global_label);
    let integ_global_key = format!("Integration_{}", global_label);

    let intelligibility_r2 = attributes
        .get(&nain_global_key)
        .map(|nain_rn| pearson_r2(&conn_f64, nain_rn))
        .unwrap_or(0.0);

    let intelligibility_integration_r2 = attributes
        .get(&integ_global_key)
        .map(|integ_rn| pearson_r2(&conn_f64, integ_rn))
        .unwrap_or(0.0);

    let mut synergy_r2 = BTreeMap::new();
    if let Some(nain_rn) = attributes.get(&nain_global_key) {
        for r_label in &r_labels {
            if r_label != &global_label {
                let local_key = format!("NAIN_{}", r_label);
                if let Some(nain_loc) = attributes.get(&local_key) {
                    synergy_r2.insert(r_label.clone(), pearson_r2(nain_loc, nain_rn));
                }
            }
        }
    }

    AsaResults {
        attributes,
        intelligibility_r2,
        intelligibility_integration_r2,
        synergy_r2,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
    }
}

/// Solves multi-radius Angular, Metric, or Topological Segment Analysis (`ASA`) in parallel using `rayon`.
pub fn solve_angular_segment_analysis(graph: &CSRDualGraph, config: &AsaConfig) -> AsaResults {
    let t0 = Instant::now();
    let n = graph.num_nodes;
    let (control, controllability) = compute_segment_control_metrics(graph);

    let mut attributes: BTreeMap<String, Vec<f64>> = BTreeMap::new();
    let conn_f64: Vec<f64> = graph.connectivity.iter().map(|&c| c as f64).collect();
    let seg_len_f64: Vec<f64> = graph.segment_lengths.iter().map(|&l| l as f64).collect();

    attributes.insert("Connectivity".to_string(), conn_f64.clone());
    attributes.insert(
        "Angular_Connectivity".to_string(),
        graph.angular_connectivity.clone(),
    );
    attributes.insert("Control".to_string(), control);
    attributes.insert("Controllability".to_string(), controllability);
    attributes.insert("Segment_Length".to_string(), seg_len_f64);

    if n == 0 || config.radii.is_empty() {
        return AsaResults {
            attributes,
            intelligibility_r2: 0.0,
            intelligibility_integration_r2: 0.0,
            synergy_r2: BTreeMap::new(),
            elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
        };
    }

    if matches!(config.analysis_type, AnalysisType::Angular)
        && config.tulip_bins > 0
        && !graph.forward_segconns.is_empty()
    {
        return solve_asa_tulip_directed(graph, config, t0, attributes, conn_f64);
    }

    let (w_norm, is_weighted) = match config.weight_mode {
        WeightMode::SegmentLength => {
            let w_node: Vec<f64> = graph
                .segment_lengths
                .iter()
                .map(|&l| (l as f64).max(1e-3))
                .collect();
            let mean_w = (w_node.iter().sum::<f64>() / (n as f64)).max(1e-6);
            (w_node.into_iter().map(|w| w / mean_w).collect(), true)
        }
        WeightMode::Custom => {
            let w_node: Vec<f64> = graph
                .segment_weights
                .iter()
                .map(|&w| w.max(1e-3))
                .collect();
            let mean_w = (w_node.iter().sum::<f64>() / (n as f64)).max(1e-6);
            (w_node.into_iter().map(|w| w / mean_w).collect(), true)
        }
        WeightMode::Unweighted => (vec![1.0f64; n], false),
    };

    let max_sources = if config.max_sources_per_radius == 0 {
        n
    } else {
        config.max_sources_per_radius
    };
    let (sources, scale_mult): (Vec<usize>, f64) = if n <= max_sources {
        ((0..n).collect(), 1.0)
    } else {
        let step = (n / max_sources).max(1);
        ((0..n).step_by(step).collect(), step as f64)
    };

    let r_count = config.radii.len();
    let is_global_arr: Vec<bool> = config
        .radii
        .iter()
        .map(|&r| !r.is_finite() || r >= 1e8)
        .collect();
    let any_global = is_global_arr.iter().any(|&g| g);
    let max_radius = config
        .radii
        .iter()
        .zip(is_global_arr.iter())
        .filter(|(_, &g)| !g)
        .map(|(&r, _)| r)
        .fold(0.0f64, f64::max);

    let same_mode = matches!(
        (config.analysis_type, config.radius_type),
        (AnalysisType::Angular, RadiusType::Angular)
            | (AnalysisType::Metric, RadiusType::Metric)
            | (AnalysisType::Topological, RadiusType::Topological)
    );
    let prune_dijkstra = !any_global && same_mode;

    let num_sources = sources.len();
    let n_chunks = num_sources.min(rayon::current_num_threads().max(1) * 2).max(1);

    struct ChunkOutput {
        choice: Vec<Vec<f64>>,
        closeness_pairs: Vec<(usize, Vec<f64>, Vec<f64>, Vec<f64>)>,
    }

    let chunk_outputs: Vec<ChunkOutput> = (0..n_chunks)
        .into_par_iter()
        .map(|chunk_id| {
            let start_s = (chunk_id * num_sources) / n_chunks;
            let end_s = ((chunk_id + 1) * num_sources) / n_chunks;

            let mut choice_chunk = vec![vec![0.0f64; n]; r_count];
            let mut closeness_pairs = Vec::with_capacity(end_s.saturating_sub(start_s));

            let mut settled = Vec::with_capacity(n);
            let mut d_cost = vec![f64::INFINITY; n];
            let mut d_rad = vec![f64::INFINITY; n];
            let mut sigma = vec![0.0f64; n];
            let mut delta = vec![0.0f64; n];

            let pred_cap = (graph.target_nodes.len() + 4).max(16);
            let mut pred_head = vec![-1i32; n];
            let mut pred_node = vec![0usize; pred_cap];
            let mut pred_next = vec![-1i32; pred_cap];
            let mut heap = BinaryHeap::with_capacity(n * 2);

            for &s in &sources[start_s..end_s] {
                settled.clear();
                let mut pred_ptr = 0usize;

                d_cost[s] = 0.0;
                d_rad[s] = 0.0;
                sigma[s] = 1.0;
                heap.clear();
                heap.push(AsaHeapEntry {
                    cost: 0.0,
                    rad: 0.0,
                    node: s,
                });

                while let Some(AsaHeapEntry {
                    cost: dist_c,
                    rad: dist_r,
                    node: u,
                }) = heap.pop()
                {
                    if dist_c > d_cost[u] {
                        continue;
                    }
                    settled.push(u);

                    let row_start = graph.row_offsets[u] as usize;
                    let row_end = graph.row_offsets[u + 1] as usize;
                    let sig_u = sigma[u];

                    for e_idx in row_start..row_end {
                        let v = graph.target_nodes[e_idx] as usize;
                        let step_r = match config.radius_type {
                            RadiusType::Angular => graph.angular_weights[e_idx] as f64,
                            RadiusType::Topological => 1.0,
                            RadiusType::Metric => graph.metric_lengths[e_idx] as f64,
                        };
                        let new_rad = dist_r + step_r;
                        if prune_dijkstra && new_rad > max_radius {
                            continue;
                        }

                        let step_c = match config.analysis_type {
                            AnalysisType::Metric => graph.metric_lengths[e_idx] as f64,
                            AnalysisType::Topological => 1.0,
                            AnalysisType::Angular => graph.angular_weights[e_idx] as f64,
                        };
                        let new_cost = dist_c + step_c;
                        let dv = d_cost[v];

                        if new_cost < dv - 1e-7 {
                            d_cost[v] = new_cost;
                            d_rad[v] = new_rad;
                            heap.push(AsaHeapEntry {
                                cost: new_cost,
                                rad: new_rad,
                                node: v,
                            });
                            sigma[v] = sig_u;
                            if pred_ptr >= pred_node.len() {
                                pred_node.push(u);
                                pred_next.push(-1);
                            } else {
                                pred_node[pred_ptr] = u;
                                pred_next[pred_ptr] = -1;
                            }
                            pred_head[v] = pred_ptr as i32;
                            pred_ptr += 1;
                        } else if (new_cost - dv).abs() <= 1e-7 {
                            if new_rad < d_rad[v] {
                                d_rad[v] = new_rad;
                            }
                            sigma[v] += sig_u;
                            if pred_ptr >= pred_node.len() {
                                pred_node.push(u);
                                pred_next.push(pred_head[v]);
                            } else {
                                pred_node[pred_ptr] = u;
                                pred_next[pred_ptr] = pred_head[v];
                            }
                            pred_head[v] = pred_ptr as i32;
                            pred_ptr += 1;
                        }
                    }
                }

                let w_s = w_norm[s];
                let mut s_closeness = vec![0.0f64; r_count];
                let mut s_td = vec![0.0f64; r_count];
                let mut s_nc = vec![0.0f64; r_count];

                for r_idx in 0..r_count {
                    let r_m = config.radii[r_idx];
                    let is_glob = is_global_arr[r_idx];

                    for &w in &settled {
                        delta[w] = 0.0;
                    }

                    let mut k_reach = 0usize;
                    let mut sum_cost = 0.0f64;

                    if !is_weighted {
                        for &w in settled.iter().rev() {
                            let in_hor = is_glob || (d_rad[w] <= r_m);
                            let del_w = delta[w];
                            if !in_hor && del_w == 0.0 {
                                continue;
                            }
                            let w_seed = if in_hor { 1.0 } else { 0.0 };
                            let factor = (w_seed + del_w) / sigma[w];

                            let mut p_cur = pred_head[w];
                            while p_cur != -1 {
                                let idx_p = p_cur as usize;
                                let v_pred = pred_node[idx_p];
                                delta[v_pred] += sigma[v_pred] * factor;
                                p_cur = pred_next[idx_p];
                            }

                            if w != s && in_hor {
                                choice_chunk[r_idx][w] += del_w * scale_mult;
                                if d_cost[w] > 0.0 {
                                    k_reach += 1;
                                    sum_cost += d_cost[w];
                                }
                            }
                        }
                    } else {
                        for &w in settled.iter().rev() {
                            let in_hor = is_glob || (d_rad[w] <= r_m);
                            let del_w = delta[w];
                            if !in_hor && del_w == 0.0 {
                                continue;
                            }
                            let w_seed = if in_hor { w_norm[w] } else { 0.0 };
                            let factor = (w_seed + del_w) / sigma[w];

                            let mut p_cur = pred_head[w];
                            while p_cur != -1 {
                                let idx_p = p_cur as usize;
                                let v_pred = pred_node[idx_p];
                                delta[v_pred] += sigma[v_pred] * factor;
                                p_cur = pred_next[idx_p];
                            }

                            if w != s && in_hor {
                                choice_chunk[r_idx][w] += w_s * del_w * scale_mult;
                                if d_cost[w] > 0.0 {
                                    k_reach += 1;
                                    sum_cost += d_cost[w] * w_norm[w];
                                }
                            }
                        }
                    }

                    s_nc[r_idx] = k_reach as f64;
                    if k_reach > 0 {
                        s_td[r_idx] = sum_cost;
                        s_closeness[r_idx] = (k_reach as f64) / sum_cost.max(1e-4);
                    }
                }

                closeness_pairs.push((s, s_closeness, s_td, s_nc));

                for &w in &settled {
                    d_cost[w] = f64::INFINITY;
                    d_rad[w] = f64::INFINITY;
                    sigma[w] = 0.0;
                    pred_head[w] = -1;
                }
            }

            ChunkOutput {
                choice: choice_chunk,
                closeness_pairs,
            }
        })
        .collect();

    let mut choice_all = vec![vec![0.0f64; n]; r_count];
    let mut closeness_all = vec![vec![0.0f64; n]; r_count];
    let mut total_depth_all = vec![vec![0.0f64; n]; r_count];
    let mut node_count_all = vec![vec![0.0f64; n]; r_count];

    for chunk in chunk_outputs {
        for r_idx in 0..r_count {
            for u in 0..n {
                choice_all[r_idx][u] += chunk.choice[r_idx][u];
            }
        }
        for (s, s_close, s_td, s_nc) in chunk.closeness_pairs {
            for r_idx in 0..r_count {
                closeness_all[r_idx][s] = s_close[r_idx];
                total_depth_all[r_idx][s] = s_td[r_idx];
                node_count_all[r_idx][s] = s_nc[r_idx];
            }
        }
    }

    // If strided sampling was used, fill un-sampled closeness/TD/NC from nearest sampled index
    if num_sources < n && !sources.is_empty() {
        let step = (n / max_sources).max(1);
        for u in 0..n {
            let nearest_s = (u / step) * step;
            let src_u = nearest_s.min(n - 1);
            if src_u != u {
                for r_idx in 0..r_count {
                    closeness_all[r_idx][u] = closeness_all[r_idx][src_u];
                    total_depth_all[r_idx][u] = total_depth_all[r_idx][src_u];
                    node_count_all[r_idx][u] = node_count_all[r_idx][src_u];
                }
            }
        }
    }

    let mut r_labels = Vec::with_capacity(r_count);
    for (idx, &r) in config.radii.iter().enumerate() {
        let r_label = format_radius_label(r);
        r_labels.push(r_label.clone());

        let choice = &choice_all[idx];
        let closeness = &closeness_all[idx];
        let total_depth = &total_depth_all[idx];
        let node_count = &node_count_all[idx];

        let mut mean_depth = vec![0.0f64; n];
        let mut nach = vec![0.0f64; n];
        let mut nain = vec![0.0f64; n];

        for i in 0..n {
            let safe_c = choice[i].max(0.0);
            let safe_td = total_depth[i].max(0.0);
            let safe_nc = node_count[i].max(1.0);
            mean_depth[i] = safe_td / safe_nc;
            nach[i] = (safe_c + 1.0).ln() / (safe_td + 3.0).ln();
            nain[i] = safe_nc.powf(1.2) / (safe_td + 2.0);
        }

        if config.include_normalized {
            let max_c = choice.iter().copied().fold(0.0f64, f64::max);
            let max_i = closeness.iter().copied().fold(0.0f64, f64::max);
            let mut nach_norm = vec![0.0f64; n];
            let mut nain_norm = vec![0.0f64; n];
            if max_c > 0.0 {
                let denom_c = (max_c + 1.0).ln();
                for i in 0..n {
                    nach_norm[i] = (choice[i].max(0.0) + 1.0).ln() / denom_c;
                }
            }
            if max_i > 0.0 {
                let denom_i = (max_i + 1.0).ln();
                for i in 0..n {
                    nain_norm[i] = (closeness[i].max(0.0) + 1.0).ln() / denom_i;
                }
            }
            attributes.insert(format!("NACH_Norm_{}", r_label), nach_norm);
            attributes.insert(format!("NAIN_Norm_{}", r_label), nain_norm);
        }

        attributes.insert(format!("Choice_{}", r_label), choice.clone());
        attributes.insert(format!("Integration_{}", r_label), closeness.clone());
        attributes.insert(format!("Total_Depth_{}", r_label), total_depth.clone());
        attributes.insert(format!("Mean_Depth_{}", r_label), mean_depth);
        attributes.insert(format!("Node_Count_{}", r_label), node_count.clone());
        attributes.insert(format!("NACH_{}", r_label), nach);
        attributes.insert(format!("NAIN_{}", r_label), nain);
    }

    let global_label = if r_labels.iter().any(|l| l == "Rn") {
        "Rn".to_string()
    } else {
        r_labels.last().cloned().unwrap_or_else(|| "Rn".to_string())
    };

    let nain_global_key = format!("NAIN_{}", global_label);
    let integ_global_key = format!("Integration_{}", global_label);

    let intelligibility_r2 = attributes
        .get(&nain_global_key)
        .map(|nain_rn| pearson_r2(&conn_f64, nain_rn))
        .unwrap_or(0.0);

    let intelligibility_integration_r2 = attributes
        .get(&integ_global_key)
        .map(|integ_rn| pearson_r2(&conn_f64, integ_rn))
        .unwrap_or(0.0);

    let mut synergy_r2 = BTreeMap::new();
    if let Some(nain_rn) = attributes.get(&nain_global_key) {
        for r_label in &r_labels {
            if r_label != &global_label {
                let local_key = format!("NAIN_{}", r_label);
                if let Some(nain_loc) = attributes.get(&local_key) {
                    synergy_r2.insert(r_label.clone(), pearson_r2(nain_loc, nain_rn));
                }
            }
        }
    }

    AsaResults {
        attributes,
        intelligibility_r2,
        intelligibility_integration_r2,
        synergy_r2,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
    }
}

#[derive(Copy, Clone, Debug)]
struct StepHeapEntry {
    ang: f64,
    metric: f64,
    topo: i32,
    node: usize,
}

impl PartialEq for StepHeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.ang.to_bits() == other.ang.to_bits()
            && self.metric.to_bits() == other.metric.to_bits()
            && self.topo == other.topo
            && self.node == other.node
    }
}

impl Eq for StepHeapEntry {}

impl PartialOrd for StepHeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for StepHeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .ang
            .partial_cmp(&self.ang)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                other
                    .metric
                    .partial_cmp(&self.metric)
                    .unwrap_or(Ordering::Equal)
            })
            .then_with(|| other.topo.cmp(&self.topo))
            .then_with(|| other.node.cmp(&self.node))
    }
}

/// Runs Multi-Source Dijkstra from `origin_ids` to compute:
/// - `angular_step_depth`: shortest angular turn cost from nearest origin (`0.0` at origins, `-1.0` if unreachable)
/// - `metric_step_depth_m`: shortest metric distance in meters from nearest origin (`0.0` at origins, `-1.0` if unreachable)
/// - `topo_step_depth`: topological step count from nearest origin (`0` at origins, `-1` if unreachable)
/// - `nearest_origin_id`: segment ID of closest origin (`-1` if unreachable)
/// - `is_origin`: `1` if segment is in `origin_ids` else `0`
pub fn solve_origin_step_depth(
    graph: &CSRDualGraph,
    origin_ids: &[usize],
    max_radius: f64,
    radius_type: RadiusType,
) -> StepDepthResults {
    let t0 = Instant::now();
    let n = graph.num_nodes;
    let mut valid_origins: Vec<usize> = origin_ids
        .iter()
        .copied()
        .filter(|&idx| idx < n)
        .collect();
    valid_origins.sort_unstable();
    valid_origins.dedup();

    let mut angular_step_depth = vec![-1.0f64; n];
    let mut metric_step_depth_m = vec![-1.0f64; n];
    let mut topo_step_depth = vec![-1i32; n];
    let mut nearest_origin_id = vec![-1i32; n];
    let mut is_origin = vec![0i32; n];

    if valid_origins.is_empty() || n == 0 || max_radius < 0.0 {
        return StepDepthResults {
            origin_ids: valid_origins,
            angular_step_depth,
            metric_step_depth_m,
            topo_step_depth,
            nearest_origin_id,
            is_origin,
            elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
        };
    }

    let is_global = !max_radius.is_finite() || max_radius >= 1e8;
    let mut d_ang = vec![f64::INFINITY; n];
    let mut d_metric = vec![f64::INFINITY; n];
    let mut d_topo = vec![-1i32; n];
    let mut heap = BinaryHeap::with_capacity(n * 2);

    for &o in &valid_origins {
        d_ang[o] = 0.0;
        d_metric[o] = 0.0;
        d_topo[o] = 0;
        nearest_origin_id[o] = o as i32;
        is_origin[o] = 1;
        heap.push(StepHeapEntry {
            ang: 0.0,
            metric: 0.0,
            topo: 0,
            node: o,
        });
    }

    while let Some(StepHeapEntry {
        ang: dist_a,
        metric: dist_m,
        topo: dist_t,
        node: u,
    }) = heap.pop()
    {
        if dist_a > d_ang[u] + 1e-9 {
            continue;
        }
        let row_start = graph.row_offsets[u] as usize;
        let row_end = graph.row_offsets[u + 1] as usize;
        let orig_u = nearest_origin_id[u];

        for e_idx in row_start..row_end {
            let v = graph.target_nodes[e_idx] as usize;
            let new_ang = dist_a + (graph.angular_weights[e_idx] as f64);
            let new_metric = dist_m + (graph.metric_lengths[e_idx] as f64);
            let new_topo = dist_t + 1;

            let horizon_val = match radius_type {
                RadiusType::Angular => new_ang,
                RadiusType::Topological => new_topo as f64,
                RadiusType::Metric => new_metric,
            };
            if !is_global && horizon_val > max_radius {
                continue;
            }

            let dav = d_ang[v];
            if new_ang < dav - 1e-7
                || ((new_ang - dav).abs() <= 1e-7 && new_metric < d_metric[v] - 1e-7)
            {
                d_ang[v] = new_ang;
                d_metric[v] = new_metric;
                d_topo[v] = new_topo;
                nearest_origin_id[v] = orig_u;
                heap.push(StepHeapEntry {
                    ang: new_ang,
                    metric: new_metric,
                    topo: new_topo,
                    node: v,
                });
            }
        }
    }

    for i in 0..n {
        if d_ang[i].is_finite() {
            angular_step_depth[i] = (d_ang[i] * 10000.0).round() / 10000.0;
            metric_step_depth_m[i] = (d_metric[i] * 100.0).round() / 100.0;
            topo_step_depth[i] = d_topo[i];
        }
    }
    for &o in &valid_origins {
        angular_step_depth[o] = 0.0;
        metric_step_depth_m[o] = 0.0;
        topo_step_depth[o] = 0;
    }

    StepDepthResults {
        origin_ids: valid_origins,
        angular_step_depth,
        metric_step_depth_m,
        topo_step_depth,
        nearest_origin_id,
        is_origin,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Point2D, Polyline};

    #[test]
    fn test_benchmark_a_collinear_3_segment_chain() {
        // Benchmark A: 1D 3-Segment Collinear Chain S0 - S1 - S2 (100m each)
        let lines = vec![
            Polyline::new(vec![Point2D::new(0.0, 0.0), Point2D::new(100.0, 0.0)]),
            Polyline::new(vec![Point2D::new(100.0, 0.0), Point2D::new(200.0, 0.0)]),
            Polyline::new(vec![Point2D::new(200.0, 0.0), Point2D::new(300.0, 0.0)]),
        ];
        let (graph, _segs) = CSRDualGraph::from_polylines(&lines, 0, 0.0001);
        assert_eq!(graph.num_nodes, 3);

        // Verify unweighted Choice and Metric Mean Depth
        let cfg = AsaConfig {
            radii: vec![f64::INFINITY],
            analysis_type: AnalysisType::Metric,
            radius_type: RadiusType::Metric,
            weight_mode: WeightMode::Unweighted,
            tulip_bins: 0,
            include_normalized: true,
            max_sources_per_radius: 5000,
        };
        let res = solve_angular_segment_analysis(&graph, &cfg);
        let choice_rn = &res.attributes["Choice_Rn"];
        let md_rn = &res.attributes["Mean_Depth_Rn"];

        assert!((choice_rn[1] - 2.0).abs() < 1e-6);
        assert!((choice_rn[0] - 0.0).abs() < 1e-6);
        assert!((choice_rn[2] - 0.0).abs() < 1e-6);

        assert!((md_rn[1] - 100.0).abs() < 1e-6);
        assert!((md_rn[0] - 150.0).abs() < 1e-6);
        assert!((md_rn[2] - 150.0).abs() < 1e-6);
    }
}
