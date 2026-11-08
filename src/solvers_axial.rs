//! Topological Axial Map Analysis Solver (Hillier & Hanson 1984; `depthmapX v0.8.0` 16-Attribute Parity).
//!
//! Implements:
//! 1. `Connectivity`, `Control`, `Controllability`, `Line_Length`
//! 2. `Node_Count`, `Total_Depth`, `Mean_Depth`, `Harmonic_Mean_Depth`
//! 3. `RA` (Relative Asymmetry), `RRA` (Real Relative Asymmetry via Hillier Diamond `D_n`), `RA_Penn`
//! 4. `Integration_HH`, `Integration_Pval`, `Integration_Tekl`
//! 5. `Entropy`, `Relativised_Entropy`, `Intensity`
//! 6. `Choice` and `Choice_Norm` across multiple topological step radii (`"n"`, `"R3"`, `"R5"`, etc.).

use crate::types::{pearson_r2, AxialResults, Point2D, Polyline, SegmentRecord};
use rayon::prelude::*;
use std::collections::BTreeMap;
use std::time::Instant;

/// Computes Hillier & Hanson's (1984) diamond benchmark `D_k` for an axial system of `k` lines:
/// `D_k = 2 * { k * [ log2((k + 2) / 3) - 1 ] + 1 } / [ (k - 1) * (k - 2) ]`
#[inline]
pub fn calculate_hillier_diamond_dn(k: usize) -> f64 {
    if k <= 2 {
        return 1.0;
    }
    let kf = k as f64;
    let numerator = 2.0 * (kf * (((kf + 2.0) / 3.0).log2() - 1.0) + 1.0);
    let denominator = (kf - 1.0) * (kf - 2.0);
    (numerator / denominator).max(1e-12)
}

#[inline]
fn point_to_segment_dist(p: Point2D, a: Point2D, b: Point2D) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len2 = dx * dx + dy * dy;
    if len2 <= 1e-18 {
        return p.dist(a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
    p.dist(Point2D::new(a.x + t * dx, a.y + t * dy))
}

#[inline]
fn segments_intersect_2d(a0: Point2D, a1: Point2D, b0: Point2D, b1: Point2D, eps: f64) -> bool {
    if a0.x.max(a1.x) + eps < b0.x.min(b1.x)
        || b0.x.max(b1.x) + eps < a0.x.min(a1.x)
        || a0.y.max(a1.y) + eps < b0.y.min(b1.y)
        || b0.y.max(b1.y) + eps < a0.y.min(a1.y)
    {
        return false;
    }

    let dax = a1.x - a0.x;
    let day = a1.y - a0.y;
    let dbx = b1.x - b0.x;
    let dby = b1.y - b0.y;
    let denom = dax * dby - day * dbx;

    if denom.abs() > 1e-11 {
        let wx = b0.x - a0.x;
        let wy = b0.y - a0.y;
        let t = (wx * dby - wy * dbx) / denom;
        let u = (wx * day - wy * dax) / denom;
        if t >= -1e-6 && t <= 1.0 + 1e-6 && u >= -1e-6 && u <= 1.0 + 1e-6 {
            return true;
        }
    }

    point_to_segment_dist(a0, b0, b1) <= eps
        || point_to_segment_dist(a1, b0, b1) <= eps
        || point_to_segment_dist(b0, a0, a1) <= eps
        || point_to_segment_dist(b1, a0, a1) <= eps
}

fn polylines_intersect(la: &Polyline, lb: &Polyline, eps: f64) -> bool {
    if (la.level - lb.level).abs() > 1e-6 {
        return false;
    }
    for wa in la.coords.windows(2) {
        for wb in lb.coords.windows(2) {
            if segments_intersect_2d(wa[0], wa[1], wb[0], wb[1], eps) {
                return true;
            }
        }
    }
    false
}

/// Solves full `depthmapX v0.8.0` 16-attribute Topological Axial Analysis across `radii_steps`.
///
/// Radius convention:
/// - `0` or `usize::MAX` -> global radius `"n"`
/// - `k > 0` -> step horizon `"R{k}"` (e.g. `"R3"`, `"R5"`)
pub fn solve_axial_analysis(
    lines: &[Polyline],
    radii_steps: &[usize],
    include_choice: bool,
) -> AxialResults {
    let t0 = Instant::now();

    let valid_lines: Vec<&Polyline> = lines
        .iter()
        .filter(|ln| ln.coords.len() >= 2 && ln.length() > 1e-5)
        .collect();
    let n = valid_lines.len();

    if n == 0 {
        return AxialResults::default();
    }

    let mut axial_records = Vec::with_capacity(n);
    let mut bboxes = Vec::with_capacity(n);
    let mut line_lengths = Vec::with_capacity(n);

    for (i, ln) in valid_lines.iter().enumerate() {
        let p0 = ln.coords[0];
        let p1 = *ln.coords.last().unwrap();
        let length_m = ln.length();
        let b0 = (p1.x - p0.x).atan2(p1.y - p0.y);
        let b1 = (p0.x - p1.x).atan2(p0.y - p1.y);
        let azimuth_deg = (b0.to_degrees() + 360.0) % 360.0;

        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for pt in &ln.coords {
            min_x = min_x.min(pt.x);
            max_x = max_x.max(pt.x);
            min_y = min_y.min(pt.y);
            max_y = max_y.max(pt.y);
        }
        bboxes.push((min_x - 1e-3, max_x + 1e-3, min_y - 1e-3, max_y + 1e-3));
        line_lengths.push(length_m);

        axial_records.push(SegmentRecord {
            id: i,
            p0,
            p1,
            b0,
            b1,
            length_m,
            azimuth_deg,
            level: ln.level,
            highway: ln.highway.clone(),
            name: ln.name.clone(),
        });
    }

    // Normalize radii_steps (0 or usize::MAX => global 0)
    let mut radii_clean: Vec<usize> = Vec::new();
    if radii_steps.is_empty() {
        radii_clean.extend_from_slice(&[0, 3, 5]);
    } else {
        for &r in radii_steps {
            let norm_r = if r == usize::MAX { 0 } else { r };
            if !radii_clean.contains(&norm_r) {
                radii_clean.push(norm_r);
            }
        }
    }
    let r_count = radii_clean.len();

    // 1. Build axial intersection CSR graph using sweep-line bounding-box pruning
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| {
        bboxes[a]
            .0
            .partial_cmp(&bboxes[b].0)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for idx_a in 0..n {
        let i = order[idx_a];
        let (min_xa, max_xa, min_ya, max_ya) = bboxes[i];
        for idx_b in (idx_a + 1)..n {
            let j = order[idx_b];
            let (min_xb, max_xb, min_yb, max_yb) = bboxes[j];
            if min_xb > max_xa {
                break;
            }
            if max_yb < min_ya || min_yb > max_ya || max_xb < min_xa {
                continue;
            }
            if polylines_intersect(valid_lines[i], valid_lines[j], 1e-3) {
                adj[i].push(j);
                adj[j].push(i);
            }
        }
    }

    let mut indptr = vec![0usize; n + 1];
    let mut indices = Vec::new();
    let mut connectivity = vec![0.0f64; n];

    for u in 0..n {
        adj[u].sort_unstable();
        adj[u].dedup();
        indptr[u] = indices.len();
        connectivity[u] = adj[u].len() as f64;
        indices.extend_from_slice(&adj[u]);
    }
    indptr[n] = indices.len();

    // 2. Control: sum_{j in N(i)} (1 / deg(j))
    let mut control = vec![-1.0f64; n];
    for u in 0..n {
        let deg_u = adj[u].len();
        if deg_u > 0 {
            let mut sum_ctrl = 0.0;
            for &v in &adj[u] {
                let deg_v = adj[v].len().max(1) as f64;
                sum_ctrl += 1.0 / deg_v;
            }
            control[u] = sum_ctrl;
        }
    }

    // 3. Parallel multi-radius BFS + Brandes Topological Choice
    let n_chunks = n.min(rayon::current_num_threads().max(1) * 2).max(1);

    struct AxialChunkOut {
        start_s: usize,
        // [r_idx][metric_idx][local_s]
        metrics: Vec<[Vec<f64>; 13]>,
        controllability: Vec<f64>,
        choice: Vec<Vec<f64>>,
    }

    let chunk_outs: Vec<AxialChunkOut> = (0..n_chunks)
        .into_par_iter()
        .map(|chunk_id| {
            let start_s = (chunk_id * n) / n_chunks;
            let end_s = ((chunk_id + 1) * n) / n_chunks;
            let m_len = end_s.saturating_sub(start_s);

            let mut metrics: Vec<[Vec<f64>; 13]> = (0..r_count)
                .map(|_| std::array::from_fn(|_| vec![-1.0f64; m_len]))
                .collect();
            let mut controllability = vec![-1.0f64; m_len];
            let mut choice = if include_choice {
                vec![vec![0.0f64; n]; r_count]
            } else {
                Vec::new()
            };

            let mut queue = vec![0usize; n];
            let mut depth = vec![-1i32; n];
            let mut sigma = vec![0.0f64; n];
            let mut delta = vec![0.0f64; n];
            let mut counts_all = vec![0usize; n + 1];

            let pred_cap = (indices.len() + 4).max(16);
            let mut pred_head = vec![-1i32; n];
            let mut pred_node = vec![0usize; pred_cap];
            let mut pred_next = vec![-1i32; pred_cap];

            for (local_i, s) in (start_s..end_s).enumerate() {
                queue[0] = s;
                depth[s] = 0;
                sigma[s] = 1.0;
                let mut head = 0usize;
                let mut tail = 1usize;
                let mut pred_ptr = 0usize;
                let mut max_d_global = 0usize;

                while head < tail {
                    let u = queue[head];
                    head += 1;
                    let d_u = depth[u] as usize;
                    if d_u > max_d_global {
                        max_d_global = d_u;
                    }
                    counts_all[d_u] += 1;
                    let sig_u = sigma[u];

                    for &v in &indices[indptr[u]..indptr[u + 1]] {
                        let dv = depth[v];
                        if dv == -1 {
                            depth[v] = (d_u + 1) as i32;
                            sigma[v] = sig_u;
                            queue[tail] = v;
                            tail += 1;
                            if include_choice {
                                if pred_ptr >= pred_node.len() {
                                    pred_node.push(u);
                                    pred_next.push(-1);
                                } else {
                                    pred_node[pred_ptr] = u;
                                    pred_next[pred_ptr] = -1;
                                }
                                pred_head[v] = pred_ptr as i32;
                                pred_ptr += 1;
                            }
                        } else if include_choice && (dv as usize) == d_u + 1 {
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

                let k_visited = tail;
                if max_d_global >= 1 {
                    let c1 = counts_all[1];
                    let c2 = if max_d_global >= 2 { counts_all[2] } else { 0 };
                    if c1 + c2 > 0 {
                        controllability[local_i] = (c1 as f64) / ((c1 + c2) as f64);
                    }
                }

                for (r_idx, &r_lim) in radii_clean.iter().enumerate() {
                    let (sub_len, b_bins, max_wave) = if r_lim > 0 && max_d_global >= r_lim {
                        (r_lim, r_lim + 1, r_lim)
                    } else {
                        (max_d_global, max_d_global + 2, max_d_global)
                    };

                    let mut nc_sum = 0usize;
                    for d_i in 1..=sub_len {
                        nc_sum += counts_all[d_i];
                    }
                    let nc = 1 + nc_sum;
                    metrics[r_idx][0][local_i] = nc as f64;

                    if nc <= 1 {
                        continue;
                    }

                    let mut td = 0usize;
                    for d_i in 1..=sub_len {
                        td += counts_all[d_i] * d_i;
                    }
                    let md = (td as f64) / ((nc - 1) as f64);
                    metrics[r_idx][2][local_i] = md;

                    let mut ent = 0.0f64;
                    let mut rel_ent = 0.0f64;
                    let mut fact = 1.0f64;
                    let mut h_denom = 0.0f64;
                    let mut overflow = false;
                    let exp_neg_md = (-md).exp();

                    for i_bin in 0..sub_len {
                        let cnt = counts_all[i_bin + 1];
                        fact *= (i_bin + 1) as f64;
                        if cnt > 0 {
                            let prob = (cnt as f64) / (nc as f64);
                            ent -= prob * prob.log2();
                            if !overflow {
                                let p_md = md.powi(i_bin as i32);
                                if !p_md.is_finite() || !fact.is_finite() || fact <= 0.0 {
                                    overflow = true;
                                } else {
                                    let q = (p_md / fact) * exp_neg_md;
                                    if q > 0.0 && q.is_finite() {
                                        rel_ent += prob * (prob / q).log2();
                                    } else {
                                        overflow = true;
                                    }
                                }
                            }
                            h_denom += 1.0 / (cnt as f64);
                        }
                    }

                    metrics[r_idx][9][local_i] = ent;
                    metrics[r_idx][10][local_i] = if overflow { 0.0 } else { rel_ent };
                    metrics[r_idx][11][local_i] = if h_denom > 0.0 {
                        (b_bins as f64) / h_denom
                    } else {
                        -1.0
                    };

                    if td > nc {
                        metrics[r_idx][12][local_i] = (nc as f64) * ent / ((td - nc) as f64);
                    }

                    if nc > 2 && md >= 1.0 {
                        metrics[r_idx][1][local_i] = td as f64;
                        let ra = 2.0 * (md - 1.0) / ((nc - 2) as f64);
                        let dn = calculate_hillier_diamond_dn(nc);
                        let rra = ra / dn;
                        metrics[r_idx][3][local_i] = ra;
                        metrics[r_idx][4][local_i] = rra;

                        if rra > 1e-12 {
                            let ihh = 1.0 / rra;
                            metrics[r_idx][6][local_i] = ihh;
                            metrics[r_idx][7][local_i] = ihh;
                        } else {
                            // Star hub (MD == 1.0, RA == 0.0, RRA == 0.0)
                            let ihh = 1.0 / dn;
                            metrics[r_idx][6][local_i] = ihh;
                            metrics[r_idx][7][local_i] = ihh;
                        }

                        if td > nc {
                            metrics[r_idx][8][local_i] = (0.5 * ((nc - 2) as f64)).ln()
                                / (((td - nc + 1) as f64).ln());
                        }

                        let l_val = (b_bins - 1) as f64;
                        let pal = 0.5 * l_val * (l_val + 1.0);
                        let denom_p = (l_val - 1.0) * (nc as f64) - pal + 1.0;
                        if denom_p > 0.0 {
                            metrics[r_idx][5][local_i] =
                                (l_val * (nc as f64) - (td as f64) - pal) / denom_p;
                        }
                    }

                    if include_choice && max_wave >= 2 {
                        for &w in &queue[0..k_visited] {
                            delta[w] = 0.0;
                        }
                        for &w in queue[1..k_visited].iter().rev() {
                            if (depth[w] as usize) > max_wave {
                                continue;
                            }
                            let sig_w = sigma[w];
                            let factor = (1.0 + delta[w]) / sig_w;
                            let mut p_cur = pred_head[w];
                            while p_cur != -1 {
                                let idx_p = p_cur as usize;
                                let u_pred = pred_node[idx_p];
                                delta[u_pred] += sigma[u_pred] * factor;
                                p_cur = pred_next[idx_p];
                            }
                            if w != s {
                                choice[r_idx][w] += delta[w];
                            }
                        }
                    }
                }

                for d_i in 0..=max_d_global {
                    counts_all[d_i] = 0;
                }
                for &w in &queue[0..k_visited] {
                    depth[w] = -1;
                    sigma[w] = 0.0;
                    pred_head[w] = -1;
                }
            }

            AxialChunkOut {
                start_s,
                metrics,
                controllability,
                choice,
            }
        })
        .collect();

    let mut all_metrics: Vec<[Vec<f64>; 13]> = (0..r_count)
        .map(|_| std::array::from_fn(|_| vec![-1.0f64; n]))
        .collect();
    let mut controllability = vec![-1.0f64; n];
    let mut choice_totals = vec![vec![0.0f64; n]; r_count];

    for chunk in chunk_outs {
        let s0 = chunk.start_s;
        let m_len = chunk.controllability.len();
        controllability[s0..(s0 + m_len)].copy_from_slice(&chunk.controllability);
        for r_idx in 0..r_count {
            for m_i in 0..13 {
                all_metrics[r_idx][m_i][s0..(s0 + m_len)]
                    .copy_from_slice(&chunk.metrics[r_idx][m_i]);
            }
            if include_choice {
                for u in 0..n {
                    choice_totals[r_idx][u] += chunk.choice[r_idx][u];
                }
            }
        }
    }

    let mut attributes: BTreeMap<String, Vec<f64>> = BTreeMap::new();
    attributes.insert("Connectivity".to_string(), connectivity.clone());
    attributes.insert("Control".to_string(), control);
    attributes.insert("Controllability".to_string(), controllability);
    attributes.insert("Line_Length".to_string(), line_lengths);

    for (r_idx, &r_lim) in radii_clean.iter().enumerate() {
        let r_label = if r_lim == 0 {
            "n".to_string()
        } else {
            format!("R{}", r_lim)
        };

        let nc_arr = &all_metrics[r_idx][0];
        attributes.insert(format!("Node_Count_{}", r_label), nc_arr.clone());
        attributes.insert(
            format!("Total_Depth_{}", r_label),
            all_metrics[r_idx][1].clone(),
        );
        attributes.insert(
            format!("Mean_Depth_{}", r_label),
            all_metrics[r_idx][2].clone(),
        );
        attributes.insert(format!("RA_{}", r_label), all_metrics[r_idx][3].clone());
        attributes.insert(format!("RRA_{}", r_label), all_metrics[r_idx][4].clone());
        attributes.insert(
            format!("RA_Penn_{}", r_label),
            all_metrics[r_idx][5].clone(),
        );
        attributes.insert(
            format!("Integration_HH_{}", r_label),
            all_metrics[r_idx][6].clone(),
        );
        attributes.insert(
            format!("Integration_Pval_{}", r_label),
            all_metrics[r_idx][7].clone(),
        );
        attributes.insert(
            format!("Integration_Tekl_{}", r_label),
            all_metrics[r_idx][8].clone(),
        );
        attributes.insert(
            format!("Entropy_{}", r_label),
            all_metrics[r_idx][9].clone(),
        );
        attributes.insert(
            format!("Relativised_Entropy_{}", r_label),
            all_metrics[r_idx][10].clone(),
        );
        attributes.insert(
            format!("Harmonic_Mean_Depth_{}", r_label),
            all_metrics[r_idx][11].clone(),
        );
        attributes.insert(
            format!("Intensity_{}", r_label),
            all_metrics[r_idx][12].clone(),
        );

        if include_choice {
            let ch_raw = &choice_totals[r_idx];
            let mut ch_final = vec![-1.0f64; n];
            let mut ch_norm = vec![-1.0f64; n];
            for i in 0..n {
                let nc = nc_arr[i];
                if nc > 2.0 {
                    ch_final[i] = ch_raw[i];
                    let denom = ((nc - 1.0) * (nc - 2.0)).max(1.0);
                    ch_norm[i] = 2.0 * ch_raw[i] / denom;
                }
            }
            attributes.insert(format!("Choice_{}", r_label), ch_final);
            attributes.insert(format!("Choice_Norm_{}", r_label), ch_norm);
        }
    }

    // Intelligibility R²: Pearson R² between Connectivity and Global Integration_HH_n
    let global_hh_key = if attributes.contains_key("Integration_HH_n") {
        Some("Integration_HH_n".to_string())
    } else {
        attributes
            .keys()
            .find(|k| k.starts_with("Integration_HH_"))
            .cloned()
    };

    let intelligibility_r2 = if let Some(key) = global_hh_key {
        let int_col = &attributes[&key];
        let mut x_valid = Vec::with_capacity(n);
        let mut y_valid = Vec::with_capacity(n);
        for i in 0..n {
            if connectivity[i] > 0.0 && int_col[i] > 0.0 && int_col[i].is_finite() {
                x_valid.push(connectivity[i]);
                y_valid.push(int_col[i]);
            }
        }
        pearson_r2(&x_valid, &y_valid)
    } else {
        0.0
    };

    AxialResults {
        lines: axial_records,
        attributes,
        intelligibility_r2,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_benchmark_b_hillier_diamond_and_5_line_star() {
        // Benchmark B: D_5 = 0.35199368 and 5-Line Axial Star K_{1,4}
        let d5 = calculate_hillier_diamond_dn(5);
        assert!((d5 - 0.35199368).abs() < 1e-6);

        // Hub line (index 0): y = 0 from x = 0 to x = 100
        // 4 Leaf lines (indices 1..=4): vertical lines crossing the Hub at x = 20, 40, 60, 80
        let lines = vec![
            Polyline::new(vec![Point2D::new(0.0, 0.0), Point2D::new(100.0, 0.0)]),
            Polyline::new(vec![Point2D::new(20.0, -20.0), Point2D::new(20.0, 20.0)]),
            Polyline::new(vec![Point2D::new(40.0, -20.0), Point2D::new(40.0, 20.0)]),
            Polyline::new(vec![Point2D::new(60.0, -20.0), Point2D::new(60.0, 20.0)]),
            Polyline::new(vec![Point2D::new(80.0, -20.0), Point2D::new(80.0, 20.0)]),
        ];

        let res = solve_axial_analysis(&lines, &[0], true);
        let td = &res.attributes["Total_Depth_n"];
        let md = &res.attributes["Mean_Depth_n"];
        let ra = &res.attributes["RA_n"];
        let rra = &res.attributes["RRA_n"];
        let ihh = &res.attributes["Integration_HH_n"];
        let choice = &res.attributes["Choice_n"];

        // Hub (index 0): TD = 4, MD = 1.0, RA = 0.0, RRA = 0.0, Choice = 12.0
        assert!((td[0] - 4.0).abs() < 1e-6);
        assert!((md[0] - 1.0).abs() < 1e-6);
        assert!((ra[0] - 0.0).abs() < 1e-6);
        assert!((rra[0] - 0.0).abs() < 1e-6);
        assert!((choice[0] - 12.0).abs() < 1e-6);

        // Leaves (indices 1..=4): TD = 7, MD = 1.75, RA = 0.5, RRA = 1.420480, Integration_HH = 0.703987
        for leaf in 1..=4 {
            assert!((td[leaf] - 7.0).abs() < 1e-6);
            assert!((md[leaf] - 1.75).abs() < 1e-6);
            assert!((ra[leaf] - 0.5).abs() < 1e-6);
            assert!((rra[leaf] - 1.420480).abs() < 1e-5);
            assert!((ihh[leaf] - 0.703987).abs() < 1e-5);
            assert!((choice[leaf] - 0.0).abs() < 1e-6);
        }
    }
}
