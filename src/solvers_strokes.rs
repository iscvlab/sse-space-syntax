//! Natural Roads Gestalt Stroke Concatenation Solver (Jiang & Claramunt 2004; Thomson 2003).
//!
//! Implements:
//! - Greedy best-fit collinear continuation across junctions (`deflection <= max_deflection_deg`).
//! - Open corridor and closed-loop stroke tracing.
//! - Stroke `connectivity`, `length_m`, `chord_length_m`, `seg_count`, and closed-loop safe
//!   `sinuosity: Option<f64>` (`None` when `chord_length_m < 1e-6`).
//! - Network-wide `circuity_avg: Option<f64>` (`None` when total chord length `<= 1e-9`,
//!   resolving the `gboeing/osmnx#1410` closed-loop zero-chord singularity).

use crate::graph::get_bearing;
use crate::types::{NaturalRoadsResults, Point2D, Polyline, StrokeRecord};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::f64::consts::PI;
use std::time::Instant;

/// Calculates angular deflection in radians `[0.0, PI]` between two outward bearings at a junction
/// (`0.0` = straight through continuation, `PI` = `180°` reversal).
#[inline]
pub fn angular_deflection_rad(b1: f64, b2: f64) -> f64 {
    let mut diff = (b1 - b2).abs() % (2.0 * PI);
    if diff > PI {
        diff = 2.0 * PI - diff;
    }
    (diff - PI).abs()
}

/// Solves Gestalt Natural Road Stroke concatenation and computes stroke-level and network-wide
/// sinuosity/circuity metrics.
pub fn solve_natural_roads(lines: &[Polyline], max_deflection_deg: f64) -> NaturalRoadsResults {
    let t0 = Instant::now();

    let valid_lines: Vec<&Polyline> = lines
        .iter()
        .filter(|ln| ln.coords.len() >= 2 && ln.length() > 1e-5)
        .collect();

    if valid_lines.is_empty() {
        return NaturalRoadsResults {
            strokes: Vec::new(),
            attributes: BTreeMap::new(),
            circuity_avg: None,
            max_deflection_deg,
            elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
        };
    }

    let n = valid_lines.len();
    let max_deflection_rad = max_deflection_deg.to_radians();

    let mut endpoint_map: HashMap<(i64, i64), Vec<(usize, bool)>> = HashMap::new();
    let mut seg_bearings: Vec<(f64, f64, &[Point2D])> = Vec::with_capacity(n);

    for (i, line) in valid_lines.iter().enumerate() {
        let coords = &line.coords;
        let p_start = coords[0].key_cm();
        let p_end = coords.last().unwrap().key_cm();

        let b_start = get_bearing(coords[0], coords[1]);
        let b_end = get_bearing(coords[coords.len() - 1], coords[coords.len() - 2]);

        // If a single input Polyline is already a self-closed loop, do not merge its closed vertex with itself
        if p_start != p_end {
            endpoint_map.entry(p_start).or_default().push((i, true));
            endpoint_map.entry(p_end).or_default().push((i, false));
        } else {
            endpoint_map.entry(p_start).or_default().push((i, true));
        }
        seg_bearings.push((b_start, b_end, coords.as_slice()));
    }

    // Greedy best-fit collinear pairing at each junction
    let mut connections: HashMap<(usize, bool), (usize, bool)> = HashMap::new();

    for members in endpoint_map.values() {
        if members.len() < 2 {
            continue;
        }
        let mut candidate_pairs: Vec<(f64, (usize, bool), (usize, bool))> = Vec::new();
        for a in 0..members.len() {
            for b in (a + 1)..members.len() {
                let (i, is_start_i) = members[a];
                let (j, is_start_j) = members[b];
                if i == j {
                    continue;
                }
                let b_i = if is_start_i {
                    seg_bearings[i].0
                } else {
                    seg_bearings[i].1
                };
                let b_j = if is_start_j {
                    seg_bearings[j].0
                } else {
                    seg_bearings[j].1
                };
                let defl = angular_deflection_rad(b_i, b_j);
                if defl <= max_deflection_rad + 1e-9 {
                    candidate_pairs.push((defl, members[a], members[b]));
                }
            }
        }
        candidate_pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        for (_, m1, m2) in candidate_pairs {
            if !connections.contains_key(&m1) && !connections.contains_key(&m2) {
                connections.insert(m1, m2);
                connections.insert(m2, m1);
            }
        }
    }

    let mut visited_seg = vec![false; n];
    let mut raw_strokes: Vec<(Vec<Point2D>, Vec<usize>)> = Vec::new();

    // Pass 1: Trace open strokes starting from un-paired or terminal ends
    for i in 0..n {
        if visited_seg[i] {
            continue;
        }
        for is_start in [true, false] {
            let end_key = (i, is_start);
            let other_key = (i, !is_start);
            // Skip if i is in the middle of a stroke (both ends connected)
            if connections.contains_key(&end_key) && connections.contains_key(&other_key) {
                continue;
            }
            if visited_seg[i] {
                break;
            }

            let mut stroke_coords: Vec<Point2D> = Vec::new();
            let mut seg_ids: Vec<usize> = Vec::new();

            visited_seg[i] = true;
            seg_ids.push(i);
            let base_coords = seg_bearings[i].2;
            let mut curr_exit_end = if is_start {
                stroke_coords.extend_from_slice(base_coords);
                (i, false)
            } else {
                stroke_coords.extend(base_coords.iter().rev().copied());
                (i, true)
            };

            while let Some(&(next_i, next_is_start)) = connections.get(&curr_exit_end) {
                if visited_seg[next_i] {
                    break;
                }
                visited_seg[next_i] = true;
                seg_ids.push(next_i);
                let next_coords = seg_bearings[next_i].2;
                if next_is_start {
                    stroke_coords.extend_from_slice(&next_coords[1..]);
                    curr_exit_end = (next_i, false);
                } else {
                    for &pt in next_coords.iter().rev().skip(1) {
                        stroke_coords.push(pt);
                    }
                    curr_exit_end = (next_i, true);
                }
            }

            if stroke_coords.len() >= 2 {
                raw_strokes.push((stroke_coords, seg_ids));
            }
        }
    }

    // Pass 2: Trace any remaining closed-loop stroke cycles
    for i in 0..n {
        if visited_seg[i] {
            continue;
        }
        visited_seg[i] = true;
        let mut stroke_coords: Vec<Point2D> = Vec::new();
        let mut seg_ids: Vec<usize> = vec![i];
        let base_coords = seg_bearings[i].2;
        stroke_coords.extend_from_slice(base_coords);
        let mut curr_exit_end = (i, false);

        while let Some(&(next_i, next_is_start)) = connections.get(&curr_exit_end) {
            if visited_seg[next_i] {
                break;
            }
            visited_seg[next_i] = true;
            seg_ids.push(next_i);
            let next_coords = seg_bearings[next_i].2;
            if next_is_start {
                stroke_coords.extend_from_slice(&next_coords[1..]);
                curr_exit_end = (next_i, false);
            } else {
                for &pt in next_coords.iter().rev().skip(1) {
                    stroke_coords.push(pt);
                }
                curr_exit_end = (next_i, true);
            }
        }

        if stroke_coords.len() >= 2 {
            raw_strokes.push((stroke_coords, seg_ids));
        }
    }

    // Compute stroke-to-stroke connectivity via shared vertices
    let num_strokes = raw_strokes.len();
    let mut vertex_to_strokes: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (s_id, (coords, _)) in raw_strokes.iter().enumerate() {
        let mut seen_v: HashSet<(i64, i64)> = HashSet::new();
        for &pt in coords {
            let k = pt.key_cm();
            if seen_v.insert(k) {
                vertex_to_strokes.entry(k).or_default().push(s_id);
            }
        }
    }

    let mut stroke_neighbors: Vec<HashSet<usize>> = vec![HashSet::new(); num_strokes];
    for members in vertex_to_strokes.values() {
        if members.len() < 2 {
            continue;
        }
        for a in 0..members.len() {
            for b in (a + 1)..members.len() {
                let sa = members[a];
                let sb = members[b];
                if sa != sb {
                    stroke_neighbors[sa].insert(sb);
                    stroke_neighbors[sb].insert(sa);
                }
            }
        }
    }

    let mut strokes = Vec::with_capacity(num_strokes);
    let mut attr_len = Vec::with_capacity(num_strokes);
    let mut attr_chord = Vec::with_capacity(num_strokes);
    let mut attr_seg_cnt = Vec::with_capacity(num_strokes);
    let mut attr_conn = Vec::with_capacity(num_strokes);
    let mut attr_sinuosity = Vec::with_capacity(num_strokes);

    let mut total_len_m = 0.0f64;
    let mut total_chord_m = 0.0f64;

    for (stroke_id, (coords, segment_ids)) in raw_strokes.into_iter().enumerate() {
        let length_m: f64 = coords.windows(2).map(|w| w[0].dist(w[1])).sum();
        let chord_length_m = match (coords.first(), coords.last()) {
            (Some(&a), Some(&b)) => a.dist(b),
            _ => 0.0,
        };
        let seg_count = segment_ids.len();
        let connectivity = stroke_neighbors[stroke_id].len();
        let sinuosity = if chord_length_m >= 1e-6 {
            Some(length_m / chord_length_m)
        } else {
            None
        };

        total_len_m += length_m;
        total_chord_m += chord_length_m;

        attr_len.push(length_m);
        attr_chord.push(chord_length_m);
        attr_seg_cnt.push(seg_count as f64);
        attr_conn.push(connectivity as f64);
        attr_sinuosity.push(sinuosity.unwrap_or(-1.0));

        strokes.push(StrokeRecord {
            stroke_id,
            coords,
            segment_ids,
            length_m,
            chord_length_m,
            seg_count,
            connectivity,
            sinuosity,
        });
    }

    let circuity_avg = if total_chord_m <= 1e-9 {
        None
    } else {
        Some(total_len_m / total_chord_m)
    };

    let mut attributes = BTreeMap::new();
    attributes.insert("Stroke_Connectivity".to_string(), attr_conn);
    attributes.insert("Stroke_Length".to_string(), attr_len);
    attributes.insert("Chord_Length".to_string(), attr_chord);
    attributes.insert("Segment_Count".to_string(), attr_seg_cnt);
    attributes.insert("Sinuosity".to_string(), attr_sinuosity);

    NaturalRoadsResults {
        strokes,
        attributes,
        circuity_avg,
        max_deflection_deg,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
    }
}

/// Concatenates contiguous line segments into perceptual Natural Road `Polyline` strokes.
pub fn concatenate_natural_road_strokes(lines: &[Polyline], max_deflection_deg: f64) -> Vec<Polyline> {
    let res = solve_natural_roads(lines, max_deflection_deg);
    res.strokes
        .into_iter()
        .map(|s| Polyline::new(s.coords))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_closed_loop_sinuosity_and_circuity_none() {
        // Closed-loop stroke (osmnx#1410 zero-chord singularity guard)
        let closed_ring = Polyline::new(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(100.0, 0.0),
            Point2D::new(100.0, 100.0),
            Point2D::new(0.0, 100.0),
            Point2D::new(0.0, 0.0),
        ]);
        let res = solve_natural_roads(&[closed_ring], 45.0);
        assert_eq!(res.strokes.len(), 1);
        assert!(res.strokes[0].chord_length_m < 1e-6);
        assert_eq!(res.strokes[0].sinuosity, None);
        assert_eq!(res.circuity_avg, None);
    }

    #[test]
    fn test_collinear_corridor_concatenation() {
        let s0 = Polyline::new(vec![Point2D::new(0.0, 0.0), Point2D::new(50.0, 0.0)]);
        let s1 = Polyline::new(vec![Point2D::new(50.0, 0.0), Point2D::new(100.0, 5.0)]);
        let cross = Polyline::new(vec![Point2D::new(50.0, -30.0), Point2D::new(50.0, 30.0)]);
        let res = solve_natural_roads(&[s0, s1, cross], 45.0);
        assert_eq!(res.strokes.len(), 2);
        assert!(res.circuity_avg.is_some());
        assert!(res.circuity_avg.unwrap() >= 1.0);
    }
}
