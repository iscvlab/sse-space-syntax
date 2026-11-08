//! Shared domain types, configuration contracts, and statistical utilities for SSE (`sse_core`).

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// 2D metric coordinate in planar space (meters).
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn dist(self, other: Self) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }

    /// 1mm topological vertex key `(round(x * 1000), round(y * 1000))`.
    #[inline]
    pub fn key_mm(self) -> (i64, i64) {
        ((self.x * 1000.0).round() as i64, (self.y * 1000.0).round() as i64)
    }

    /// 1cm topological vertex key `(round(x * 100), round(y * 100))`.
    #[inline]
    pub fn key_cm(self) -> (i64, i64) {
        ((self.x * 100.0).round() as i64, (self.y * 100.0).round() as i64)
    }
}

/// A 2D polyline with optional grade/elevation level and highway classification.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Polyline {
    pub coords: Vec<Point2D>,
    pub level: f64,
    pub highway: String,
    pub name: String,
}

impl Polyline {
    pub fn new(coords: Vec<Point2D>) -> Self {
        Self {
            coords,
            level: 0.0,
            highway: String::new(),
            name: String::new(),
        }
    }

    pub fn with_level(mut self, level: f64) -> Self {
        self.level = level;
        self
    }

    pub fn with_meta(mut self, level: f64, highway: impl Into<String>, name: impl Into<String>) -> Self {
        self.level = level;
        self.highway = highway.into();
        self.name = name.into();
        self
    }

    pub fn length(&self) -> f64 {
        self.coords
            .windows(2)
            .map(|w| w[0].dist(w[1]))
            .sum()
    }

    pub fn chord_length(&self) -> f64 {
        match (self.coords.first(), self.coords.last()) {
            (Some(&a), Some(&b)) => a.dist(b),
            _ => 0.0,
        }
    }
}

/// A planarized 2-point street segment or axial line record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SegmentRecord {
    pub id: usize,
    pub p0: Point2D,
    pub p1: Point2D,
    pub b0: f64,
    pub b1: f64,
    pub length_m: f64,
    pub azimuth_deg: f64,
    pub level: f64,
    pub highway: String,
    pub name: String,
}

impl SegmentRecord {
    #[inline]
    pub fn midpoint(&self) -> Point2D {
        Point2D::new(0.5 * (self.p0.x + self.p1.x), 0.5 * (self.p0.y + self.p1.y))
    }
}

/// Compressed Sparse Row (CSR) representation of Turner's (2007) Angular Dual Graph.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CSRDualGraph {
    pub num_nodes: usize,
    pub row_offsets: Vec<u32>,
    pub target_nodes: Vec<u32>,
    pub angular_weights: Vec<f32>,
    pub metric_lengths: Vec<f32>,
    pub segment_lengths: Vec<f32>,
    pub segment_weights: Vec<f64>,
    pub connectivity: Vec<i32>,
    pub angular_connectivity: Vec<f64>,
    pub segment_coords: Vec<[f64; 4]>,
    pub segment_azimuths: Vec<f64>,
    /// Directed endpoint connections exiting at `p1` (`forward: p0 -> p1`): `(conn_dir, conn_ref, turn_weight_f32)`.
    pub forward_segconns: Vec<Vec<(i8, u32, f32)>>,
    /// Directed endpoint connections exiting at `p0` (`back: p1 -> p0`): `(conn_dir, conn_ref, turn_weight_f32)`.
    pub back_segconns: Vec<Vec<(i8, u32, f32)>>,
}

impl CSRDualGraph {
    pub fn memory_footprint_bytes(&self) -> usize {
        self.row_offsets.len() * 4
            + self.target_nodes.len() * 4
            + self.angular_weights.len() * 4
            + self.metric_lengths.len() * 4
            + self.segment_lengths.len() * 4
            + self.segment_weights.len() * 8
            + self.connectivity.len() * 4
            + self.angular_connectivity.len() * 8
            + self.segment_coords.len() * 32
            + self.segment_azimuths.len() * 8
    }
}

/// Shortest-path impedance mode for Angular Segment Analysis (`ASA`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AnalysisType {
    Angular,
    Metric,
    Topological,
}

impl AnalysisType {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Angular => "Angular",
            Self::Metric => "Metric",
            Self::Topological => "Topological",
        }
    }
}

/// Horizon radius cutoff unit for `ASA` and `Origin Step Depth`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RadiusType {
    Metric,
    Angular,
    Topological,
}

impl RadiusType {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Metric => "Metric (m)",
            Self::Angular => "Angular (turns)",
            Self::Topological => "Topological (steps)",
        }
    }
}

/// Segment weighting mode for Brandes Choice and Closeness Integration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum WeightMode {
    Unweighted,
    SegmentLength,
    Custom,
}

impl WeightMode {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Unweighted => "Unweighted",
            Self::SegmentLength => "Segment Length",
            Self::Custom => "Custom Weight",
        }
    }
}

/// Configuration for Angular Segment Analysis (`ASA`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AsaConfig {
    pub radii: Vec<f64>,
    pub analysis_type: AnalysisType,
    pub radius_type: RadiusType,
    pub weight_mode: WeightMode,
    pub tulip_bins: usize,
    pub include_normalized: bool,
    pub max_sources_per_radius: usize,
}

impl Default for AsaConfig {
    fn default() -> Self {
        Self {
            radii: vec![400.0, 800.0, 1200.0, 2000.0, 5000.0, f64::INFINITY],
            analysis_type: AnalysisType::Angular,
            radius_type: RadiusType::Metric,
            weight_mode: WeightMode::SegmentLength,
            tulip_bins: 1024,
            include_normalized: true,
            max_sources_per_radius: 0,
        }
    }
}

/// Output of Angular Segment Analysis (`ASA`).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AsaResults {
    pub attributes: BTreeMap<String, Vec<f64>>,
    pub intelligibility_r2: f64,
    pub intelligibility_integration_r2: f64,
    pub synergy_r2: BTreeMap<String, f64>,
    pub elapsed_ms: f64,
}

/// Output of Multi-Source Origin Step Depth.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct StepDepthResults {
    pub origin_ids: Vec<usize>,
    pub angular_step_depth: Vec<f64>,
    pub metric_step_depth_m: Vec<f64>,
    pub topo_step_depth: Vec<i32>,
    pub nearest_origin_id: Vec<i32>,
    pub is_origin: Vec<i32>,
    pub elapsed_ms: f64,
}

/// Output of Topological Axial Map Analysis (Hillier & Hanson 1984; depthmapX v0.8.0 16 attributes).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AxialResults {
    pub lines: Vec<SegmentRecord>,
    pub attributes: BTreeMap<String, Vec<f64>>,
    pub intelligibility_r2: f64,
    pub elapsed_ms: f64,
}

/// A Gestalt Natural Road stroke (Jiang & Claramunt 2004).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StrokeRecord {
    pub stroke_id: usize,
    pub coords: Vec<Point2D>,
    pub segment_ids: Vec<usize>,
    pub length_m: f64,
    pub chord_length_m: f64,
    pub seg_count: usize,
    pub connectivity: usize,
    /// Sinuosity (`length_m / chord_length_m`). `None` when closed loop (`chord_length_m < 1e-6`).
    pub sinuosity: Option<f64>,
}

/// Output of Natural Roads Stroke Analysis.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NaturalRoadsResults {
    pub strokes: Vec<StrokeRecord>,
    pub attributes: BTreeMap<String, Vec<f64>>,
    /// Network-wide average circuity (`sum(length_m) / sum(chord_length_m)`).
    /// `None` when total straight-line distance is zero (`< 1e-9`), matching `gboeing/osmnx#1410`.
    pub circuity_avg: Option<f64>,
    pub max_deflection_deg: f64,
    pub elapsed_ms: f64,
}

/// Configuration for the 5-Step P90 Adaptive Watertight Network Cleaner.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CleanerConfig {
    pub mode: String,
    pub extend_distance_m: f64,
    pub angle_min_deg: f64,
    pub angle_max_deg: f64,
    pub min_stub_len_m: f64,
    pub max_turn_deflection_deg: f64,
    pub dead_end_stub_len_m: f64,
    pub snap_distance_m: f64,
    pub lateral_fallback: bool,
}

impl Default for CleanerConfig {
    fn default() -> Self {
        Self {
            mode: "auto".to_string(),
            extend_distance_m: 15.0,
            angle_min_deg: 15.0,
            angle_max_deg: 165.0,
            min_stub_len_m: 0.5,
            max_turn_deflection_deg: 15.0,
            dead_end_stub_len_m: 0.0,
            snap_distance_m: 3.0,
            lateral_fallback: true,
        }
    }
}

/// Post-flight topology audit report from the 5-Step Cleaner.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CleanerReport {
    pub mode: String,
    pub effective_extend_distance: f64,
    pub raw_lines_count: usize,
    pub exploded_segments_count: usize,
    pub raw_dangles_count: usize,
    pub cleaned_dangles_count: usize,
    pub ray_extensions_count: usize,
    pub lateral_connectors_count: usize,
    pub pruned_micro_stubs_count: usize,
    pub remaining_micro_stubs_count: usize,
    pub dead_end_stubs_removed_count: usize,
    pub cleaned_lines_count: usize,
    pub raw_junctions_count: usize,
    pub cleaned_junctions_count: usize,
    pub raw_components_count: usize,
    pub cleaned_components_count: usize,
    pub raw_total_length_m: f64,
    pub cleaned_total_length_m: f64,
    pub length_delta_pct: f64,
    pub circuity_avg: Option<f64>,
    pub assertions_passed: bool,
    pub elapsed_ms: f64,
}

/// A single discretized cell in a Visibility Graph Analysis (`VGA`) PointMap.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct VgaGridPoint {
    pub id: usize,
    pub x: f64,
    pub y: f64,
    pub grid_i: i32,
    pub grid_j: i32,
    pub depthmap_ref: i32,
}

/// Output of Visibility Graph Analysis (`VGA`).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VgaResults {
    pub points: Vec<VgaGridPoint>,
    pub cell_size_m: f64,
    pub row_offsets: Vec<u32>,
    pub col_indices: Vec<u32>,
    pub bin_indices: Vec<u8>,
    pub attributes: BTreeMap<String, Vec<f64>>,
    pub elapsed_ms: f64,
}

/// Analytical 2D Benedikt (1979) Isovist polygon and indicators from a vantage point.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IsovistResult {
    pub origin: Point2D,
    pub polygon: Vec<Point2D>,
    pub area: f64,
    pub perimeter: f64,
    pub compactness: f64,
    pub occlusivity: f64,
    pub min_radial: f64,
    pub max_radial: f64,
    pub drift_magnitude: f64,
    pub drift_angle_deg: f64,
    pub drift_target: Point2D,
}

fn default_agents_per_click() -> usize {
    1
}

fn default_max_trails() -> usize {
    50
}

/// Visual target selection rule for Gibson Ecological Visual Agents (Turner & Penn 2002; UCL `depthmapX` `AgentProgram`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AgentMovementRule {
    /// `SEL_STANDARD` (`0x1001`, default in `depthmapX`): Uniform random target cell selection across visible VGA nodes in the FOV cone.
    Standard,
    /// `SEL_LOS` (`0x0001`): Angular bin roulette selection weighted by line-of-sight length (`onLoSLook`).
    LineOfSightLength,
}

impl Default for AgentMovementRule {
    fn default() -> Self {
        Self::Standard
    }
}

impl AgentMovementRule {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Standard => "Standard (Turner & Penn 2002)",
            Self::LineOfSightLength => "Line of Sight Length",
        }
    }
}

/// Configuration for Gibson Ecological Visual Agent Simulation (Turner & Penn 2002; UCL `depthmapX` `CAgentAnalysisDlg`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentConfig {
    /// Analysis length in timesteps (`depthmapX` default = `5000`).
    pub timesteps: usize,
    /// Release rate in agents per timestep (`depthmapX` default = `0.1`).
    pub release_rate: f64,
    /// Agent lifetime / timesteps in system (`depthmapX` default = `1000`).
    pub agent_life_steps: usize,
    /// Field of view in angular bins (`1..=32`, `depthmapX` default = `15` ≈ `170°`, `vbin = 7`).
    pub agent_fov_bins: usize,
    /// Mean steps before turn decision (`depthmapX` default = `3`, Bernoulli probability `1 / m_steps`).
    pub agent_steps_before_turn: usize,
    /// Visual look / movement rule (`Standard` vs `LineOfSightLength`).
    #[serde(default)]
    pub movement_rule: AgentMovementRule,
    /// When `true`, release agents only from user-selected `release_cells` (`depthmapX` `"Release from selected locations"`).
    #[serde(default)]
    pub release_from_selected_only: bool,
    pub max_active_agents: usize,
    pub seed: u64,
    pub record_trails: bool,
    /// Maximum number of agent trails to record (`depthmapX` default = `50`).
    #[serde(default = "default_max_trails")]
    pub max_trails: usize,
    /// Optional manually placed release cell indices (UCL `depthmapX` manual agent release locations).
    #[serde(default)]
    pub release_cells: Vec<usize>,
    /// Number of agents spawned when clicking a grid cell in manual placement mode (`depthmapX` `Q3DView::CreateAgent` = `1`).
    #[serde(default = "default_agents_per_click")]
    pub agents_per_click: usize,
}

impl Default for AgentConfig {
    fn default() -> Self {
        Self {
            timesteps: 5000,
            release_rate: 0.10,
            agent_life_steps: 1000,
            agent_fov_bins: 15,
            agent_steps_before_turn: 3,
            movement_rule: AgentMovementRule::Standard,
            release_from_selected_only: false,
            max_active_agents: 200,
            seed: 42,
            record_trails: false,
            max_trails: 50,
            release_cells: Vec::new(),
            agents_per_click: 1,
        }
    }
}

/// A single active Gibson Visual Agent (Turner & Penn 2002; UCL `depthmapX` `Agent`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActiveAgent {
    pub agent_id: usize,
    pub node_idx: usize,
    #[serde(default)]
    pub prev_pos: Point2D,
    pub pos: Point2D,
    #[serde(default)]
    pub dir: Point2D,
    #[serde(default)]
    pub target_pos: Point2D,
    #[serde(default)]
    pub has_target: bool,
    pub bin: u8,
    pub steps_taken: usize,
    pub birth_step: usize,
    pub trail: Vec<Point2D>,
}

/// Live & Batch state of Gibson Ecological Visual Agents on a VGA PointMap.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AgentSimState {
    pub current_step: usize,
    pub gate_counts: Vec<f64>,
    pub active_agents: Vec<ActiveAgent>,
    pub finished_trails: Vec<Vec<Point2D>>,
    pub rng_state: u64,
    pub next_agent_id: usize,
    pub release_accumulator: f64,
    /// Cached list of walkable node indices with degree > 0 (avoids per-frame allocation).
    #[serde(default)]
    pub valid_nodes: Vec<usize>,
    /// Pre-indexed 8-connected Moore neighbors `(neighbor_idx, bin, step_len_sq)` per node (`O(1)` step lookup).
    #[serde(default)]
    pub moore_neighbors: Vec<Vec<(u32, u8, i32)>>,
}

/// Descriptive distribution statistics for any attribute column.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct SummaryStats {
    pub count: usize,
    pub min: f64,
    pub max: f64,
    pub mean: f64,
    pub std: f64,
    pub p10: f64,
    pub p50: f64,
    pub p90: f64,
}

impl SummaryStats {
    pub fn from_slice(values: &[f64]) -> Self {
        let mut finite: Vec<f64> = values
            .iter()
            .copied()
            .filter(|v| v.is_finite() && *v > -0.999999)
            .collect();
        if finite.is_empty() {
            finite = values.iter().copied().filter(|v| v.is_finite()).collect();
        }
        if finite.is_empty() {
            return Self::default();
        }
        finite.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n = finite.len();
        let min = finite[0];
        let max = finite[n - 1];
        let mean = finite.iter().sum::<f64>() / (n as f64);
        let var = finite.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n as f64);
        let std = var.sqrt();
        let pct = |q: f64| -> f64 {
            if n == 1 {
                return finite[0];
            }
            let pos = q * ((n - 1) as f64);
            let lo = pos.floor() as usize;
            let hi = pos.ceil() as usize;
            let frac = pos - (lo as f64);
            finite[lo] * (1.0 - frac) + finite[hi] * frac
        };
        Self {
            count: n,
            min,
            max,
            mean,
            std,
            p10: pct(0.10),
            p50: pct(0.50),
            p90: pct(0.90),
        }
    }
}

/// Computes Ordinary Least Squares (`y = slope * x + intercept`, `pearson_r`, `r2`)
/// over finite paired values.
pub fn ols_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64, f64) {
    let n = x.len().min(y.len());
    if n < 3 {
        return (0.0, 0.0, 0.0, 0.0);
    }
    let pairs: Vec<(f64, f64)> = x
        .iter()
        .copied()
        .zip(y.iter().copied())
        .filter(|(vx, vy)| vx.is_finite() && vy.is_finite() && *vx > -0.999999 && *vy > -0.999999)
        .collect();
    if pairs.len() < 3 {
        return (0.0, 0.0, 0.0, 0.0);
    }
    let nf = pairs.len() as f64;
    let mx = pairs.iter().map(|p| p.0).sum::<f64>() / nf;
    let my = pairs.iter().map(|p| p.1).sum::<f64>() / nf;
    let mut sxx = 0.0;
    let mut syy = 0.0;
    let mut sxy = 0.0;
    for (vx, vy) in pairs {
        let dx = vx - mx;
        let dy = vy - my;
        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    if sxx <= 1e-18 || syy <= 1e-18 {
        return (0.0, my, 0.0, 0.0);
    }
    let slope = sxy / sxx;
    let intercept = my - slope * mx;
    let r = (sxy / (sxx * syy).sqrt()).clamp(-1.0, 1.0);
    let r2 = r * r;
    (slope, intercept, r, r2)
}

#[inline]
pub fn pearson_r2(x: &[f64], y: &[f64]) -> f64 {
    ols_regression(x, y).3
}

/// Computes network-wide average circuity (`sum(length) / sum(chord_length)`).
/// Returns `None` when total straight-line distance is `<= 1e-9` (closed-loop guard for `osmnx#1410`).
pub fn compute_network_circuity_avg(lines: &[Polyline]) -> Option<f64> {
    if lines.is_empty() {
        return None;
    }
    let total_len: f64 = lines.iter().map(|l| l.length()).sum();
    let total_chord: f64 = lines.iter().map(|l| l.chord_length()).sum();
    if total_chord <= 1e-9 {
        None
    } else {
        Some(total_len / total_chord)
    }
}

/// Classic UCL `depthmapX v0.8.0` 11-Stop Blue-to-Red Spectral Colour Scale (`t in [0.0, 1.0]`).
pub fn depthmapx_color_rgb(t: f64, palette: ColorPalette) -> [u8; 3] {
    let t = t.clamp(0.0, 1.0);
    match palette {
        ColorPalette::DepthmapClassic => {
            const STOPS: [[u8; 3]; 11] = [
                [51, 51, 221],   // 0.0 Deep Blue
                [51, 136, 221],  // 0.1 Royal Cyan-Blue
                [34, 204, 221],  // 0.2 Cyan
                [34, 204, 170],  // 0.3 Blue-Green
                [34, 221, 136],  // 0.4 Spring Green
                [136, 221, 34],  // 0.5 Lime Green
                [204, 221, 34],  // 0.6 Yellow-Green
                [255, 204, 34],  // 0.7 Warm Yellow
                [255, 153, 34],  // 0.8 Orange
                [255, 85, 34],   // 0.9 Red-Orange
                [221, 34, 34],   // 1.0 Crimson Red
            ];
            let scaled = t * 10.0;
            let idx = (scaled.floor() as usize).min(9);
            let frac = scaled - (idx as f64);
            let c0 = STOPS[idx];
            let c1 = STOPS[idx + 1];
            [
                (c0[0] as f64 + frac * (c1[0] as f64 - c0[0] as f64)).round() as u8,
                (c0[1] as f64 + frac * (c1[1] as f64 - c0[1] as f64)).round() as u8,
                (c0[2] as f64 + frac * (c1[2] as f64 - c0[2] as f64)).round() as u8,
            ]
        }
        ColorPalette::Viridis => {
            const STOPS: [[u8; 3]; 5] = [
                [68, 1, 84],
                [59, 82, 139],
                [33, 145, 140],
                [94, 201, 98],
                [253, 231, 37],
            ];
            let scaled = t * 4.0;
            let idx = (scaled.floor() as usize).min(3);
            let frac = scaled - (idx as f64);
            let c0 = STOPS[idx];
            let c1 = STOPS[idx + 1];
            [
                (c0[0] as f64 + frac * (c1[0] as f64 - c0[0] as f64)).round() as u8,
                (c0[1] as f64 + frac * (c1[1] as f64 - c0[1] as f64)).round() as u8,
                (c0[2] as f64 + frac * (c1[2] as f64 - c0[2] as f64)).round() as u8,
            ]
        }
        ColorPalette::MonochromeAcademic => {
            let v = (225.0 - t * 200.0).round() as u8;
            [v, v, (v as u16 + 10).min(255) as u8]
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ColorPalette {
    DepthmapClassic,
    Viridis,
    MonochromeAcademic,
}

impl ColorPalette {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::DepthmapClassic => "Classic DepthmapX (11-Stop Blue-Red)",
            Self::Viridis => "Viridis (Perceptually Uniform)",
            Self::MonochromeAcademic => "Monochrome Academic (Print Slate)",
        }
    }
}
