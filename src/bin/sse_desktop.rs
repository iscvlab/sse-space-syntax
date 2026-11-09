#![windows_subsystem = "windows"]
#![allow(float_literal_f32_fallback)]

//! # SSE — Space Syntax Engine (`sse-desktop`)
//!
//! Native Rust Academic Spatial Network & Visibility Workstation (`UCL depthmapX v0.8.0` replacement).
//! Provides interactive Angular Segment Analysis (`ASA`), Topological Axial Analysis (Hillier & Hanson 1984),
//! Gestalt Natural Roads (`sinuosity` & `circuity_avg` closed-loop guard), 5-Step P90 Adaptive Watertight
//! Network Cleaning, Visibility Graph Analysis (`VGA PointMap`), analytical 2D Benedikt Isovists,
//! and interactive Gibson Ecological Visual Agent Simulation (Turner & Penn 2002).

use eframe::egui::{self, Color32, FontFamily, FontId, Pos2, Rect, RichText, Sense, Stroke, Vec2};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::PathBuf;

use sse_core::benchmarks::{load_benchmark, BenchmarkDataset, BenchmarkPreset};
use sse_core::cleaner::{
    clean_network_adaptive_watertight, count_connected_components, count_dangling_endpoints,
    count_graph_junctions, explode_to_2point_segments, planarize_and_node_pruned,
    snap_endpoints_watertight, synthesize_dual_carriageway_rungs, trim_axial_overshoot_whiskers,
};
use sse_core::graph::{calculate_turn_deflection, detect_grade_unlinks, get_bearing};
use sse_core::io::{
    export_segments_csv, export_segments_dxf, export_segments_geojson, export_segments_mif_mid,
    export_vga_csv, fetch_osm_overpass_boundary_staged, geocode_osm_location,
    load_osm_boundary_file, load_polylines_auto,
};
use sse_core::solvers_agents::{
    init_agent_simulation, solve_agents_batch, spawn_agents_at_cell,
    step_agent_simulation_with_mode,
};
use sse_core::solvers_asa::{solve_angular_segment_analysis, solve_origin_step_depth};
use sse_core::solvers_axial::{calculate_hillier_diamond_dn, solve_axial_analysis};
use sse_core::solvers_strokes::solve_natural_roads;
use sse_core::solvers_vga::{
    build_walkable_vga_grid_lattice, compute_isovist_at_point, effective_vga_grid_spacing,
    generate_vga_grid, generate_vga_grid_flood_fill, solve_vga, solve_vga_with_progress, whichbin,
    MAX_INTERACTIVE_VGA_CELLS,
};
use sse_core::types::*;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AnalysisDomain {
    StreetNetwork,
    SpatialVisibility,
}

impl AnalysisDomain {
    fn short_label(&self) -> &'static str {
        match self {
            Self::StreetNetwork => "URBAN STREET NETWORK",
            Self::SpatialVisibility => "SPATIAL VISIBILITY & AGENTS",
        }
    }
}

fn configure_segoe_ui_only(ctx: &egui::Context) {
    let mut fonts = egui::FontDefinitions::default();

    // Load Windows Segoe UI exclusively and remove default fallback fonts where possible
    if let Ok(bytes) = std::fs::read(r"C:\Windows\Fonts\segoeui.ttf") {
        fonts.font_data.clear();
        fonts
            .font_data
            .insert("win_segoeui".to_string(), egui::FontData::from_owned(bytes).into());
        fonts
            .families
            .insert(FontFamily::Proportional, vec!["win_segoeui".to_string()]);
        fonts
            .families
            .insert(FontFamily::Monospace, vec!["win_segoeui".to_string()]);
    }

    ctx.set_fonts(fonts);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum OsmBasemapStyle {
    OsmStandard,
    EsriSatellite,
    EsriHybrid,
}

impl OsmBasemapStyle {
    fn label(&self) -> &'static str {
        match self {
            Self::OsmStandard => "OpenStreetMap (Standard)",
            Self::EsriSatellite => "Satellite (Esri World Imagery)",
            Self::EsriHybrid => "Hybrid (Satellite + Roads & Labels)",
        }
    }

    fn attribution(&self) -> &'static str {
        match self {
            Self::OsmStandard => "© OpenStreetMap contributors (Zero API Key)",
            Self::EsriSatellite => "Tiles © Esri — Source: Esri, Maxar, Earthstar Geographics (Zero API Key)",
            Self::EsriHybrid => "Tiles © Esri World Imagery + © OpenStreetMap / CARTO Labels (Zero API Key)",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OsmDrawTool {
    PanNavigate,
    DrawRectangle,
    DrawPolygon,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct SlippyTileKey {
    style: OsmBasemapStyle,
    z: i32,
    x: i32,
    y: i32,
}

struct SlippyTileMsg {
    key: SlippyTileKey,
    rgba: Option<(usize, usize, Vec<u8>)>,
}

struct SlippyTileManager {
    textures: HashMap<SlippyTileKey, egui::TextureHandle>,
    in_flight: BTreeSet<(u8, i32, i32, i32)>,
    failed: BTreeSet<(u8, i32, i32, i32)>,
    tx: Sender<SlippyTileMsg>,
    rx: Receiver<SlippyTileMsg>,
}

impl SlippyTileManager {
    fn new() -> Self {
        let (tx, rx) = mpsc::channel();
        Self {
            textures: HashMap::new(),
            in_flight: BTreeSet::new(),
            failed: BTreeSet::new(),
            tx,
            rx,
        }
    }

    fn style_id(style: OsmBasemapStyle) -> u8 {
        match style {
            OsmBasemapStyle::OsmStandard => 0,
            OsmBasemapStyle::EsriSatellite => 1,
            OsmBasemapStyle::EsriHybrid => 2,
        }
    }

    fn poll_loaded(&mut self, ctx: &egui::Context) {
        while let Ok(msg) = self.rx.try_recv() {
            let tuple_key = (
                Self::style_id(msg.key.style),
                msg.key.z,
                msg.key.x,
                msg.key.y,
            );
            self.in_flight.remove(&tuple_key);
            if let Some((w, h, rgba)) = msg.rgba {
                let image = egui::ColorImage::from_rgba_unmultiplied([w, h], &rgba);
                let tex = ctx.load_texture(
                    format!(
                        "osm_tile_{}_{}_{}_{}",
                        tuple_key.0, msg.key.z, msg.key.x, msg.key.y
                    ),
                    image,
                    egui::TextureOptions::LINEAR,
                );
                self.textures.insert(msg.key, tex);
            } else {
                self.failed.insert(tuple_key);
            }
        }
    }

    fn get_or_request(&mut self, ctx: &egui::Context, key: SlippyTileKey) -> Option<&egui::TextureHandle> {
        if self.textures.contains_key(&key) {
            return self.textures.get(&key);
        }
        let tuple_key = (Self::style_id(key.style), key.z, key.x, key.y);
        if !self.in_flight.contains(&tuple_key)
            && !self.failed.contains(&tuple_key)
            && self.in_flight.len() < 24
        {
            self.in_flight.insert(tuple_key);
            let tx = self.tx.clone();
            let ctx_clone = ctx.clone();
            std::thread::spawn(move || {
                let rgba = fetch_slippy_tile_rgba(key);
                let _ = tx.send(SlippyTileMsg { key, rgba });
                ctx_clone.request_repaint();
            });
        }
        None
    }
}

fn fetch_url_bytes(url: &str) -> Option<Vec<u8>> {
    let agent = ureq::AgentBuilder::new()
        .timeout_connect(Duration::from_secs(6))
        .timeout_read(Duration::from_secs(10))
        .user_agent("SSE-SpaceSyntaxEngine/1.1 (Academic Urban Morphology Workstation)")
        .build();
    let resp = agent.get(url).call().ok()?;
    let mut reader = resp.into_reader();
    let mut buf = Vec::with_capacity(32768);
    std::io::Read::read_to_end(&mut reader, &mut buf).ok()?;
    if buf.is_empty() {
        None
    } else {
        Some(buf)
    }
}

fn fetch_slippy_tile_rgba(key: SlippyTileKey) -> Option<(usize, usize, Vec<u8>)> {
    match key.style {
        OsmBasemapStyle::OsmStandard => {
            let url = format!("https://tile.openstreetmap.org/{}/{}/{}.png", key.z, key.x, key.y);
            let bytes = fetch_url_bytes(&url)?;
            let img = image::load_from_memory(&bytes).ok()?.to_rgba8();
            let (w, h) = img.dimensions();
            Some((w as usize, h as usize, img.into_raw()))
        }
        OsmBasemapStyle::EsriSatellite => {
            // Note: ArcGIS World_Imagery uses {z}/{y}/{x}
            let url = format!(
                "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{}/{}/{}",
                key.z, key.y, key.x
            );
            let bytes = fetch_url_bytes(&url)?;
            let img = image::load_from_memory(&bytes).ok()?.to_rgba8();
            let (w, h) = img.dimensions();
            Some((w as usize, h as usize, img.into_raw()))
        }
        OsmBasemapStyle::EsriHybrid => {
            let sat_url = format!(
                "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{}/{}/{}",
                key.z, key.y, key.x
            );
            let sat_bytes = fetch_url_bytes(&sat_url)?;
            let mut base_img = image::load_from_memory(&sat_bytes).ok()?.to_rgba8();
            let (w, h) = base_img.dimensions();

            // Composite Esri World Transportation + Carto Voyager Labels (both zero API key)
            let trans_url = format!(
                "https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Transportation/MapServer/tile/{}/{}/{}",
                key.z, key.y, key.x
            );
            if let Some(t_bytes) = fetch_url_bytes(&trans_url) {
                if let Ok(t_img) = image::load_from_memory(&t_bytes) {
                    let over = t_img.to_rgba8();
                    if over.dimensions() == (w, h) {
                        alpha_composite_rgba(base_img.as_mut(), over.as_raw());
                    }
                }
            }

            let lbl_url = format!(
                "https://basemaps.cartocdn.com/rastertiles/voyager_only_labels/{}/{}/{}.png",
                key.z, key.x, key.y
            );
            if let Some(l_bytes) = fetch_url_bytes(&lbl_url) {
                if let Ok(l_img) = image::load_from_memory(&l_bytes) {
                    let over = l_img.to_rgba8();
                    if over.dimensions() == (w, h) {
                        alpha_composite_rgba(base_img.as_mut(), over.as_raw());
                    }
                }
            }

            Some((w as usize, h as usize, base_img.into_raw()))
        }
    }
}

fn alpha_composite_rgba(base: &mut [u8], over: &[u8]) {
    for (b_px, o_px) in base.chunks_exact_mut(4).zip(over.chunks_exact(4)) {
        let a = o_px[3] as u32;
        if a == 0 {
            continue;
        }
        let inv_a = 255 - a;
        b_px[0] = ((o_px[0] as u32 * a + b_px[0] as u32 * inv_a) / 255) as u8;
        b_px[1] = ((o_px[1] as u32 * a + b_px[1] as u32 * inv_a) / 255) as u8;
        b_px[2] = ((o_px[2] as u32 * a + b_px[2] as u32 * inv_a) / 255) as u8;
        b_px[3] = 255;
    }
}

#[inline]
fn slippy_lon_to_world_px(lon: f64, zoom: f64) -> f64 {
    let world_px = 256.0 * 2.0_f64.powf(zoom);
    ((lon + 180.0) / 360.0) * world_px
}

#[inline]
fn slippy_lat_to_world_px(lat: f64, zoom: f64) -> f64 {
    let world_px = 256.0 * 2.0_f64.powf(zoom);
    let clamped = lat.clamp(-85.05112878, 85.05112878).to_radians();
    let y_norm = (1.0 - (clamped.tan() + 1.0 / clamped.cos()).ln() / std::f64::consts::PI) * 0.5;
    y_norm * world_px
}

#[inline]
fn slippy_world_px_to_lon(x_px: f64, zoom: f64) -> f64 {
    let world_px = 256.0 * 2.0_f64.powf(zoom);
    ((x_px / world_px) * 360.0 - 180.0).clamp(-180.0, 180.0)
}

#[inline]
fn slippy_world_px_to_lat(y_px: f64, zoom: f64) -> f64 {
    let world_px = 256.0 * 2.0_f64.powf(zoom);
    let y_norm = (y_px / world_px).clamp(0.0, 1.0);
    let n = std::f64::consts::PI * (1.0 - 2.0 * y_norm);
    n.sinh().atan().to_degrees().clamp(-85.05112878, 85.05112878)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ActiveMapMode {
    SegmentMapAsa,
    AxialMap,
    NaturalRoadsStrokes,
    VgaPointMap,
}

impl ActiveMapMode {
    fn label(&self) -> &'static str {
        match self {
            Self::SegmentMapAsa => "[ASA] Segment Map",
            Self::AxialMap => "[AXIAL] Topological Axial Map",
            Self::NaturalRoadsStrokes => "[STROKES] Gestalt Natural Roads",
            Self::VgaPointMap => "[VGA] Visibility PointMap & Agents",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WorkspaceTab {
    SpatialViewport,
    ScatterPlotRegression,
    AttributeDataSheet,
    TopologyAuditLog,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CanvasClickMode {
    InspectAndSelectOrigin,
    FillVgaGridAtSeed,
    PlaceAgentsAtClick,
    CastAnalyticalIsovist,
}

struct VgaBackgroundJob {
    rx: Receiver<Option<VgaResults>>,
    progress_pct: Arc<AtomicUsize>,
    cancel_flag: Arc<AtomicBool>,
    cell_count: usize,
    effective_cell_m: f64,
    requested_cell_m: f64,
}

#[derive(Debug, Clone)]
enum OsmStageState {
    Pending,
    Running(String),
    Completed(String),
    Failed(String),
}

struct OsmPipelineProgress {
    progress_pct: f32,
    stages: [OsmStageState; 5],
    live_detail: String,
    logs: Vec<String>,
    started_at: Instant,
    finished_elapsed_s: Option<f64>,
    error_msg: Option<String>,
    is_done: bool,
}

struct OsmPipelineOutput {
    title: String,
    subtitle: String,
    raw_polylines: Vec<Polyline>,
    working_polylines: Vec<Polyline>,
    cleaner_report: CleanerReport,
    segments: Vec<SegmentRecord>,
    grade_unlinks: Vec<(usize, usize)>,
    csr_graph: CSRDualGraph,
    asa_results: AsaResults,
    total_elapsed_ms: f64,
    summary_line: String,
}

struct OsmPipelineJob {
    rx: Option<Receiver<Result<OsmPipelineOutput, String>>>,
    progress: Arc<Mutex<OsmPipelineProgress>>,
    cancel_flag: Arc<AtomicBool>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ColorScaleStretch {
    LinearMinMax,
    QuantileRank,
    Logarithmic,
}

impl ColorScaleStretch {
    fn as_str(&self) -> &'static str {
        match self {
            Self::LinearMinMax => "Linear (Min-Max)",
            Self::QuantileRank => "Equal Quantile (P0-P100)",
            Self::Logarithmic => "Log10 Stretch (Choice)",
        }
    }
}

#[derive(Debug, Clone)]
struct VerificationCheckRow {
    code: &'static str,
    title: &'static str,
    expected: String,
    observed: String,
    passed: bool,
}

#[derive(Debug, Clone)]
struct ProjectDocument {
    tab_title: String,
    project_domain: AnalysisDomain,
    active_preset: Option<BenchmarkPreset>,
    dataset_title: String,
    dataset_subtitle: String,
    raw_polylines: Vec<Polyline>,
    working_polylines: Vec<Polyline>,
    boundary_rings: Vec<Vec<Point2D>>,
    obstacles: Vec<Polyline>,
    segments: Vec<SegmentRecord>,
    grade_unlinks: Vec<(usize, usize)>,
    csr_graph: Option<CSRDualGraph>,
    asa_config: AsaConfig,
    radii_input_str: String,
    prune_axial_whiskers: bool,
    axial_stub_ratio: f64,
    asa_results: AsaResults,
    step_depth_results: Option<StepDepthResults>,
    axial_results: Option<AxialResults>,
    strokes_results: Option<NaturalRoadsResults>,
    cleaner_config: CleanerConfig,
    cleaner_report: Option<CleanerReport>,
    vga_cell_size_m: f64,
    vga_max_vis_m: f64,
    vga_compute_isovist_cols: bool,
    vga_results: Option<VgaResults>,
    cast_isovists: Vec<IsovistResult>,
    agent_config: AgentConfig,
    agent_state: Option<AgentSimState>,
    is_playing_agents: bool,
    active_map_mode: ActiveMapMode,
    active_tab: WorkspaceTab,
    active_attribute: String,
    selected_ids: BTreeSet<usize>,
    scatter_x_attr: String,
    scatter_y_attr: String,
    cam_center: Point2D,
    cam_zoom: f64,
    needs_zoom_to_fit: bool,
    last_solver_ms: f64,
}

impl ProjectDocument {
    fn new_blank(tab_title: String) -> Self {
        Self {
            tab_title: tab_title.clone(),
            project_domain: AnalysisDomain::StreetNetwork,
            active_preset: None,
            dataset_title: format!("{} (Empty Workspace)", tab_title),
            dataset_subtitle: "Open a vector file (.mif / .dxf / .geojson / .osm / .csv) or load an official UCL depthmapX benchmark.".to_string(),
            raw_polylines: Vec::new(),
            working_polylines: Vec::new(),
            boundary_rings: Vec::new(),
            obstacles: Vec::new(),
            segments: Vec::new(),
            grade_unlinks: Vec::new(),
            csr_graph: None,
            asa_config: AsaConfig::default(),
            radii_input_str: "400, 800, 1200, 2000, n".to_string(),
            prune_axial_whiskers: true,
            axial_stub_ratio: 0.40,
            asa_results: AsaResults::default(),
            step_depth_results: None,
            axial_results: None,
            strokes_results: None,
            cleaner_config: CleanerConfig::default(),
            cleaner_report: None,
            vga_cell_size_m: 1.0,
            vga_max_vis_m: 50.0,
            vga_compute_isovist_cols: false,
            vga_results: None,
            cast_isovists: Vec::new(),
            agent_config: AgentConfig::default(),
            agent_state: None,
            is_playing_agents: false,
            active_map_mode: ActiveMapMode::SegmentMapAsa,
            active_tab: WorkspaceTab::SpatialViewport,
            active_attribute: "NACH_Rn".to_string(),
            selected_ids: BTreeSet::new(),
            scatter_x_attr: "Connectivity".to_string(),
            scatter_y_attr: "NAIN_Rn".to_string(),
            cam_center: Point2D::new(50.0, 50.0),
            cam_zoom: 4.0,
            needs_zoom_to_fit: false,
            last_solver_ms: 0.0,
        }
    }
}

struct McpGuiRequest {
    cmd: serde_json::Value,
    reply_tx: Sender<serde_json::Value>,
}

struct SseWorkstationApp {
    // Academic visual theme, font & viewport state
    dark_theme: bool,
    color_palette: ColorPalette,
    color_stretch: ColorScaleStretch,
    line_width: f32,
    wall_line_width: f32,
    agent_trail_width: f32,
    show_network_colors: bool,
    show_segment_ids: bool,
    show_endpoints: bool,
    show_grade_unlinks: bool,
    show_cleaner_diff: bool,
    show_vga_cells: bool,
    show_vga_los_edges: bool,
    show_obstacles: bool,
    show_isovists: bool,
    show_isovist_rays: bool,
    show_isovist_drift: bool,
    show_agents_and_trails: bool,
    show_agent_bodies: bool,
    show_agent_active_trails: bool,
    show_agent_history_trails: bool,

    // Multi-project document tabs
    projects: Vec<ProjectDocument>,
    active_project_idx: usize,
    next_untitled_counter: usize,
    tab_title: String,
    project_domain: AnalysisDomain,

    // Pan & zoom state (world meters -> screen pixels)
    cam_center: Point2D,
    cam_zoom: f64, // pixels per meter
    needs_zoom_to_fit: bool,
    last_cursor_world: Point2D,
    hovered_feature_idx: Option<usize>,

    // Active dataset & layers
    active_preset: Option<BenchmarkPreset>,
    dataset_title: String,
    dataset_subtitle: String,
    raw_polylines: Vec<Polyline>,
    working_polylines: Vec<Polyline>,
    boundary_rings: Vec<Vec<Point2D>>,
    obstacles: Vec<Polyline>,

    // Dual graph & solver outputs
    segments: Vec<SegmentRecord>,
    grade_unlinks: Vec<(usize, usize)>,
    csr_graph: Option<CSRDualGraph>,
    asa_config: AsaConfig,
    radii_input_str: String,
    prune_axial_whiskers: bool,
    axial_stub_ratio: f64,
    asa_results: AsaResults,
    step_depth_results: Option<StepDepthResults>,
    axial_results: Option<AxialResults>,
    strokes_results: Option<NaturalRoadsResults>,
    cleaner_config: CleanerConfig,
    cleaner_report: Option<CleanerReport>,

    // VGA, Isovists & Gibson Visual Agents
    vga_cell_size_m: f64,
    vga_max_vis_m: f64,
    vga_compute_isovist_cols: bool,
    vga_results: Option<VgaResults>,
    vga_job: Option<VgaBackgroundJob>,
    cast_isovists: Vec<IsovistResult>,
    agent_config: AgentConfig,
    agent_state: Option<AgentSimState>,
    is_playing_agents: bool,
    agent_steps_per_sec: f64,
    agent_step_accum: f64,
    last_agent_frame_instant: Option<Instant>,
    agent_manual_single_mode: bool,

    // Selection, Active Map Mode, Attributes & Scatter Plot
    active_map_mode: ActiveMapMode,
    active_tab: WorkspaceTab,
    click_mode: CanvasClickMode,
    active_attribute: String,
    selected_ids: BTreeSet<usize>,
    scatter_x_attr: String,
    scatter_y_attr: String,
    table_sort_col: String,
    table_sort_desc: bool,

    // Modals, Interactive Slippy Map & Diagnostics
    show_overpass_modal: bool,
    overpass_south: f64,
    overpass_west: f64,
    overpass_north: f64,
    overpass_east: f64,
    overpass_mode: String,
    overpass_city_idx: usize,
    osm_basemap: OsmBasemapStyle,
    osm_draw_tool: OsmDrawTool,
    osm_map_center_lat: f64,
    osm_map_center_lon: f64,
    osm_map_zoom: f64,
    osm_zoom_scroll_accum: f64,
    last_osm_zoom_instant: Option<Instant>,
    last_osm_map_size: (f64, f64),
    osm_search_query: String,
    osm_search_status: String,
    osm_rect_drag_start: Option<(f64, f64)>,
    osm_polygon_lat_lon: Vec<(f64, f64)>,
    osm_polygon_closed: bool,
    osm_boundary_source: String,
    osm_cursor_lat_lon: Option<(f64, f64)>,
    osm_tile_mgr: SlippyTileManager,
    osm_pipeline_job: Option<OsmPipelineJob>,
    show_osm_pipeline_modal: bool,
    show_cleaner_modal: bool,
    show_math_verification_modal: bool,
    show_png_export_modal: bool,
    png_width: u32,
    png_height: u32,
    png_transparent_bg: bool,
    png_bg_rgb: [u8; 3],
    png_line_width: f32,
    png_wall_width: f32,
    png_wall_rgb: [u8; 3],
    png_margin_pct: f32,
    png_draw_legend: bool,
    png_draw_isovists: bool,
    png_draw_agents: bool,
    verification_rows: Vec<VerificationCheckRow>,
    status_message: String,
    last_solver_ms: f64,
    audit_log_entries: Vec<String>,
    mcp_rx: Option<Receiver<McpGuiRequest>>,
}

impl SseWorkstationApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let initial_doc = ProjectDocument::new_blank("Untitled Project 1".to_string());
        let mut app = Self {
            dark_theme: false,
            color_palette: ColorPalette::DepthmapClassic,
            color_stretch: ColorScaleStretch::LinearMinMax,
            line_width: 2.4,
            wall_line_width: 2.0,
            agent_trail_width: 1.5,
            show_network_colors: true,
            show_segment_ids: false,
            show_endpoints: false,
            show_grade_unlinks: true,
            show_cleaner_diff: false,
            // Default Spatial/Agent mode: show ONLY layout (obstacles) + heatmap (vga_cells)
            show_vga_cells: true,
            show_obstacles: true,
            show_vga_los_edges: false,
            show_isovists: false,
            show_isovist_rays: false,
            show_isovist_drift: false,
            show_agents_and_trails: true,
            show_agent_bodies: false,
            show_agent_active_trails: false,
            show_agent_history_trails: false,

            projects: vec![initial_doc.clone()],
            active_project_idx: 0,
            next_untitled_counter: 1,
            tab_title: initial_doc.tab_title.clone(),
            project_domain: initial_doc.project_domain,

            cam_center: initial_doc.cam_center,
            cam_zoom: initial_doc.cam_zoom,
            needs_zoom_to_fit: initial_doc.needs_zoom_to_fit,
            last_cursor_world: Point2D::new(0.0, 0.0),
            hovered_feature_idx: None,

            active_preset: initial_doc.active_preset,
            dataset_title: initial_doc.dataset_title,
            dataset_subtitle: initial_doc.dataset_subtitle,
            raw_polylines: Vec::new(),
            working_polylines: Vec::new(),
            boundary_rings: Vec::new(),
            obstacles: Vec::new(),

            segments: Vec::new(),
            grade_unlinks: Vec::new(),
            csr_graph: None,
            asa_config: AsaConfig::default(),
            radii_input_str: "400, 800, 1200, 2000, n".to_string(),
            prune_axial_whiskers: true,
            axial_stub_ratio: 0.40,
            asa_results: AsaResults::default(),
            step_depth_results: None,
            axial_results: None,
            strokes_results: None,
            cleaner_config: CleanerConfig::default(),
            cleaner_report: None,

            vga_cell_size_m: 1.0,
            vga_max_vis_m: 50.0,
            vga_compute_isovist_cols: false,
            vga_results: None,
            vga_job: None,
            cast_isovists: Vec::new(),
            agent_config: AgentConfig::default(),
            agent_state: None,
            is_playing_agents: false,
            agent_steps_per_sec: 2.0,
            agent_step_accum: 0.0,
            last_agent_frame_instant: None,
            agent_manual_single_mode: false,

            active_map_mode: ActiveMapMode::SegmentMapAsa,
            active_tab: WorkspaceTab::SpatialViewport,
            click_mode: CanvasClickMode::InspectAndSelectOrigin,
            active_attribute: "NACH_Rn".to_string(),
            selected_ids: BTreeSet::new(),
            scatter_x_attr: "Connectivity".to_string(),
            scatter_y_attr: "NAIN_Rn".to_string(),
            table_sort_col: "id".to_string(),
            table_sort_desc: false,

            show_overpass_modal: false,
            overpass_south: 13.7200,
            overpass_west: 100.5200,
            overpass_north: 13.7320,
            overpass_east: 100.5350,
            overpass_mode: "walk".to_string(),
            overpass_city_idx: 0,
            osm_basemap: OsmBasemapStyle::OsmStandard,
            osm_draw_tool: OsmDrawTool::DrawRectangle,
            osm_map_center_lat: 13.7260,
            osm_map_center_lon: 100.5275,
            osm_map_zoom: 15.0,
            osm_zoom_scroll_accum: 0.0,
            last_osm_zoom_instant: None,
            last_osm_map_size: (860.0, 420.0),
            osm_search_query: String::new(),
            osm_search_status: String::new(),
            osm_rect_drag_start: None,
            osm_polygon_lat_lon: Vec::new(),
            osm_polygon_closed: false,
            osm_boundary_source: "Rectangle BBox (Bangkok: Sathorn/Silom)".to_string(),
            osm_cursor_lat_lon: None,
            osm_tile_mgr: SlippyTileManager::new(),
            osm_pipeline_job: None,
            show_osm_pipeline_modal: false,
            show_cleaner_modal: false,
            show_math_verification_modal: false,
            show_png_export_modal: false,
            png_width: 2560,
            png_height: 1440,
            png_transparent_bg: false,
            png_bg_rgb: [247, 248, 250],
            png_line_width: 2.8,
            png_wall_width: 2.0,
            png_wall_rgb: [30, 38, 48],
            png_margin_pct: 6.0,
            png_draw_legend: true,
            png_draw_isovists: true,
            png_draw_agents: true,
            verification_rows: Vec::new(),
            status_message: "Ready — Blank Project Workspace initialized. Press Ctrl+O to open a vector file or choose a UCL depthmapX benchmark.".to_string(),
            last_solver_ms: 0.0,
            audit_log_entries: vec![
                "[INIT] Initialized blank project 'Untitled Project 1'. Zero preloaded map.".to_string(),
                "[MCP BRIDGE] Live SSE Workstation TCP Bridge listening on 127.0.0.1:9884.".to_string(),
            ],
            mcp_rx: None,
        };

        // Spawn non-blocking Live MCP TCP Bridge on 127.0.0.1:9884
        let (mcp_tx, mcp_rx) = mpsc::channel::<McpGuiRequest>();
        app.mcp_rx = Some(mcp_rx);
        let ctx_clone = cc.egui_ctx.clone();
        std::thread::spawn(move || {
            use std::io::{BufRead, BufReader, Write};
            if let Ok(listener) = std::net::TcpListener::bind("127.0.0.1:9884") {
                for stream in listener.incoming().flatten() {
                    let _ = stream.set_read_timeout(Some(Duration::from_secs(5)));
                    let _ = stream.set_write_timeout(Some(Duration::from_secs(5)));
                    let mut writer = match stream.try_clone() {
                        Ok(s) => s,
                        Err(_) => continue,
                    };
                    let mut reader = BufReader::new(stream);
                    let mut line = String::new();
                    if reader.read_line(&mut line).is_ok() && !line.trim().is_empty() {
                        if let Ok(cmd) = serde_json::from_str::<serde_json::Value>(line.trim()) {
                            let (reply_tx, reply_rx) = mpsc::channel();
                            if mcp_tx.send(McpGuiRequest { cmd, reply_tx }).is_ok() {
                                ctx_clone.request_repaint();
                                let resp = reply_rx
                                    .recv_timeout(Duration::from_secs(8))
                                    .unwrap_or_else(|_| {
                                        serde_json::json!({"ok": false, "error": "GUI bridge timeout"})
                                    });
                                let _ = writeln!(writer, "{}", resp);
                                let _ = writer.flush();
                            }
                        }
                    }
                }
            }
        });

        configure_segoe_ui_only(&cc.egui_ctx);
        app.apply_academic_theme(&cc.egui_ctx);
        app.run_self_verification_suite();
        app
    }

    fn fit_osm_map_to_current_boundary(&mut self, view_w: f64, view_h: f64) {
        let s = self.overpass_south.min(self.overpass_north);
        let n = self.overpass_south.max(self.overpass_north);
        let w = self.overpass_west.min(self.overpass_east);
        let e = self.overpass_west.max(self.overpass_east);

        self.osm_map_center_lat = ((s + n) * 0.5).clamp(-80.0, 80.0);
        self.osm_map_center_lon = ((w + e) * 0.5).clamp(-180.0, 180.0);

        let mut best_z = 15.0;
        for z_test in (3..=17).rev() {
            let zf = z_test as f64;
            let px_w = (slippy_lon_to_world_px(e, zf) - slippy_lon_to_world_px(w, zf)).abs();
            let px_h = (slippy_lat_to_world_px(s, zf) - slippy_lat_to_world_px(n, zf)).abs();
            if px_w <= view_w * 0.72 && px_h <= view_h * 0.72 {
                best_z = zf;
                break;
            }
        }
        self.osm_map_zoom = best_z;
    }

    fn run_osm_location_search(&mut self) {
        let q = self.osm_search_query.trim().to_string();
        if q.is_empty() {
            return;
        }
        match geocode_osm_location(&q) {
            Ok(loc) => {
                self.overpass_south = loc.south;
                self.overpass_north = loc.north;
                self.overpass_west = loc.west;
                self.overpass_east = loc.east;
                self.osm_polygon_lat_lon.clear();
                self.osm_polygon_closed = false;
                self.overpass_city_idx = usize::MAX;
                let short_name: String = loc.display_name.chars().take(48).collect();
                self.osm_boundary_source = format!("Search BBox ({})", short_name);
                self.osm_search_status =
                    format!("Located: {} ({:.5}, {:.5})", loc.display_name, loc.lat, loc.lon);
                let (vw, vh) = self.last_osm_map_size;
                self.fit_osm_map_to_current_boundary(vw.max(600.0), vh.max(260.0));
                self.log_audit(format!(
                    "[OSM SEARCH] '{}' -> {:.5}, {:.5} ({})",
                    q, loc.lat, loc.lon, loc.display_name
                ));
            }
            Err(e) => {
                self.osm_search_status = format!("Search Error: {}", e);
                self.status_message = format!("OSM Search Error: {}", e);
                self.log_audit(format!("[OSM SEARCH ERROR] {}", e));
            }
        }
    }

    fn cancel_osm_pipeline_job(&mut self) {
        if let Some(job) = &mut self.osm_pipeline_job {
            job.cancel_flag.store(true, Ordering::Relaxed);
            job.rx = None;
            if let Ok(mut prog) = job.progress.lock() {
                prog.error_msg = Some("Pipeline cancelled by user.".to_string());
                prog.is_done = true;
                prog.finished_elapsed_s = Some(prog.started_at.elapsed().as_secs_f64());
                prog.logs
                    .push("[CANCELLED] User cancelled the active OSM pipeline.".to_string());
            }
        }
    }

    fn start_osm_pipeline_job(&mut self) {
        self.cancel_osm_pipeline_job();
        self.parse_radii_input();

        let south = self.overpass_south;
        let west = self.overpass_west;
        let north = self.overpass_north;
        let east = self.overpass_east;
        let poly_opt = if self.osm_polygon_lat_lon.len() >= 3 {
            Some(self.osm_polygon_lat_lon.clone())
        } else {
            None
        };
        let mode = self.overpass_mode.clone();
        let boundary_src = self.osm_boundary_source.clone();
        let cleaner_cfg = self.cleaner_config.clone();
        let asa_cfg = self.asa_config.clone();
        let stub_ratio = if self.prune_axial_whiskers {
            self.axial_stub_ratio
        } else {
            0.0
        };

        let progress = Arc::new(Mutex::new(OsmPipelineProgress {
            progress_pct: 4.0,
            stages: [
                OsmStageState::Running("Connecting to OSM / Overpass API...".to_string()),
                OsmStageState::Pending,
                OsmStageState::Pending,
                OsmStageState::Pending,
                OsmStageState::Pending,
            ],
            live_detail: "Stage 1/5: Requesting live street network from OpenStreetMap...".to_string(),
            logs: vec![format!(
                "[0.00s] Stage 1/5: Initiated OSM street network fetch ({}, mode='{}').",
                boundary_src, mode
            )],
            started_at: Instant::now(),
            finished_elapsed_s: None,
            error_msg: None,
            is_done: false,
        }));
        let cancel_flag = Arc::new(AtomicBool::new(false));
        let prog_clone = Arc::clone(&progress);
        let cancel_clone = Arc::clone(&cancel_flag);
        let (tx, rx) = mpsc::channel();

        std::thread::spawn(move || {
            let t_pipeline = Instant::now();

            // Stages 1, 2, 3: Fetch OSM -> Filter & Clip Boundary -> Project WGS84 to Metric UTM
            let fetch_res = fetch_osm_overpass_boundary_staged(
                south,
                west,
                north,
                east,
                poly_opt.as_deref(),
                &mode,
                Some(&cancel_clone),
                |stage_idx, pct, msg| {
                    if let Ok(mut p) = prog_clone.lock() {
                        let el = p.started_at.elapsed().as_secs_f64();
                        p.progress_pct = pct;
                        p.live_detail = msg.clone();
                        p.logs.push(format!("[{:.2}s] {}", el, msg));
                        if stage_idx == 1 {
                            p.stages[0] = OsmStageState::Running(msg);
                        } else if stage_idx == 2 {
                            p.stages[0] = OsmStageState::Completed(
                                "Raw OSM JSON downloaded".to_string(),
                            );
                            p.stages[1] = OsmStageState::Running(msg);
                        } else if stage_idx == 3 {
                            p.stages[1] = OsmStageState::Completed(
                                "Highway mode filtered & boundary clipped".to_string(),
                            );
                            p.stages[2] = OsmStageState::Running(msg);
                        }
                    }
                },
            );

            let fetch_report = match fetch_res {
                Ok(r) => r,
                Err(e) => {
                    if let Ok(mut p) = prog_clone.lock() {
                        let el = p.started_at.elapsed().as_secs_f64();
                        for st in &mut p.stages {
                            if matches!(st, OsmStageState::Running(_)) {
                                *st = OsmStageState::Failed(e.clone());
                            }
                        }
                        p.error_msg = Some(e.clone());
                        p.is_done = true;
                        p.finished_elapsed_s = Some(el);
                        p.logs.push(format!("[{:.2}s] [ERROR] {}", el, e));
                    }
                    let _ = tx.send(Err(e));
                    return;
                }
            };

            if cancel_clone.load(Ordering::Relaxed) {
                let _ = tx.send(Err("Pipeline cancelled by user.".to_string()));
                return;
            }

            let epsg_label = fetch_report
                .utm_epsg
                .map_or_else(|| "Metric Planar".to_string(), |code| format!("EPSG:{}", code));
            if let Ok(mut p) = prog_clone.lock() {
                let el = p.started_at.elapsed().as_secs_f64();
                p.stages[0] = OsmStageState::Completed(format!(
                    "{} raw OSM elements from {}",
                    fetch_report.raw_elements_count, fetch_report.source_endpoint
                ));
                p.stages[1] = OsmStageState::Completed(format!(
                    "{} '{}' street polylines inside boundary",
                    fetch_report.filtered_ways_count, mode
                ));
                p.stages[2] = OsmStageState::Completed(format!(
                    "Projected WGS84 -> Metric UTM ({})",
                    epsg_label
                ));
                p.stages[3] = OsmStageState::Running(
                    "Running 5-Step P90 Adaptive Watertight Cleaner & planarization...".to_string(),
                );
                p.progress_pct = 68.0;
                p.live_detail =
                    "Stage 4/5: Running 5-Step P90 Adaptive Watertight Network Cleaner...".to_string();
                p.logs.push(format!(
                    "[{:.2}s] Stage 3/5 complete: Projected {} polylines to {}. Starting 5-Step Cleaner...",
                    el, fetch_report.filtered_ways_count, epsg_label
                ));
            }

            // Stage 4: 5-Step P90 Adaptive Watertight Network Cleaner & Planarization
            let raw_polylines = fetch_report.polylines;
            let (cleaned_polylines, cleaner_report) =
                clean_network_adaptive_watertight(&raw_polylines, &cleaner_cfg);
            let noded = planarize_and_node_pruned(&cleaned_polylines, stub_ratio);
            let segments = explode_to_2point_segments(&noded);
            let grade_unlinks = detect_grade_unlinks(&segments);

            if cancel_clone.load(Ordering::Relaxed) {
                let _ = tx.send(Err("Pipeline cancelled by user.".to_string()));
                return;
            }

            if let Ok(mut p) = prog_clone.lock() {
                let el = p.started_at.elapsed().as_secs_f64();
                p.stages[3] = OsmStageState::Completed(format!(
                    "Cleaned dangles {} -> {}, planarized into {} segments ({:.1} ms)",
                    cleaner_report.raw_dangles_count,
                    cleaner_report.cleaned_dangles_count,
                    segments.len(),
                    cleaner_report.elapsed_ms
                ));
                p.stages[4] = OsmStageState::Running(format!(
                    "Building CSR Dual Graph & solving ASA on {} segments...",
                    segments.len()
                ));
                p.progress_pct = 85.0;
                p.live_detail = format!(
                    "Stage 5/5: Solving Angular Segment Analysis (ASA) across {} radii on {} segments...",
                    asa_cfg.radii.len(),
                    segments.len()
                );
                p.logs.push(format!(
                    "[{:.2}s] Stage 4/5 complete: {} 2-point segments ({} grade unlinks). Solving ASA...",
                    el,
                    segments.len(),
                    grade_unlinks.len()
                ));
            }

            // Stage 5: CSR Dual Graph + Angular Segment Analysis (ASA)
            let csr_graph =
                CSRDualGraph::from_segments(&segments, &grade_unlinks, asa_cfg.tulip_bins, 0.0001);
            let asa_results = solve_angular_segment_analysis(&csr_graph, &asa_cfg);
            let total_elapsed_ms = t_pipeline.elapsed().as_secs_f64() * 1000.0;
            let summary_line = format!(
                "Solved {} segments ({}) in {:.2} s | Intelligibility R² = {:.4}",
                segments.len(),
                epsg_label,
                total_elapsed_ms / 1000.0,
                asa_results.intelligibility_r2
            );

            if let Ok(mut p) = prog_clone.lock() {
                let el = p.started_at.elapsed().as_secs_f64();
                p.stages[4] = OsmStageState::Completed(format!(
                    "Solved {} segments across {} radii in {:.1} ms (R² = {:.4})",
                    segments.len(),
                    asa_cfg.radii.len(),
                    asa_results.elapsed_ms,
                    asa_results.intelligibility_r2
                ));
                p.progress_pct = 100.0;
                p.live_detail = format!("Pipeline Complete — {}", summary_line);
                p.finished_elapsed_s = Some(el);
                p.is_done = true;
                p.logs.push(format!("[{:.2}s] [COMPLETE] {}", el, summary_line));
            }

            let out = OsmPipelineOutput {
                title: format!("OSM ({:.4}, {:.4})", south, west),
                subtitle: format!(
                    "Live OSM Street Network ({} — Projected to {})",
                    boundary_src, epsg_label
                ),
                raw_polylines,
                working_polylines: cleaned_polylines,
                cleaner_report,
                segments,
                grade_unlinks,
                csr_graph,
                asa_results,
                total_elapsed_ms,
                summary_line,
            };
            let _ = tx.send(Ok(out));
        });

        self.osm_pipeline_job = Some(OsmPipelineJob {
            rx: Some(rx),
            progress,
            cancel_flag,
        });
        self.show_osm_pipeline_modal = true;
    }

    fn poll_osm_pipeline_job(&mut self, ctx: &egui::Context) {
        let mut received: Option<Result<OsmPipelineOutput, String>> = None;
        let mut still_running = false;

        if let Some(job) = &mut self.osm_pipeline_job {
            if let Some(rx) = &job.rx {
                match rx.try_recv() {
                    Ok(res) => {
                        received = Some(res);
                        job.rx = None;
                    }
                    Err(mpsc::TryRecvError::Empty) => {
                        still_running = true;
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        job.rx = None;
                    }
                }
            }
        }

        if still_running {
            ctx.request_repaint_after(Duration::from_millis(40));
        }

        if let Some(res) = received {
            match res {
                Ok(out) => {
                    self.cancel_vga_background_job();
                    self.active_preset = None;
                    self.project_domain = AnalysisDomain::StreetNetwork;
                    self.is_playing_agents = false;
                    self.click_mode = CanvasClickMode::InspectAndSelectOrigin;
                    self.selected_ids.clear();
                    self.cast_isovists.clear();
                    self.agent_config.release_cells.clear();
                    self.step_depth_results = None;
                    self.axial_results = None;
                    self.strokes_results = None;
                    self.vga_results = None;
                    self.agent_state = None;
                    self.obstacles.clear();
                    self.boundary_rings.clear();

                    self.tab_title = out.title.clone();
                    self.dataset_title = out.title;
                    self.dataset_subtitle = out.subtitle;
                    self.raw_polylines = out.raw_polylines;
                    self.working_polylines = out.working_polylines;
                    self.cleaner_report = Some(out.cleaner_report);
                    self.segments = out.segments;
                    self.grade_unlinks = out.grade_unlinks;
                    self.csr_graph = Some(out.csr_graph);
                    self.asa_results = out.asa_results;
                    self.last_solver_ms = out.total_elapsed_ms;
                    self.active_map_mode = ActiveMapMode::SegmentMapAsa;
                    self.active_attribute = "NACH_Rn".to_string();
                    self.scatter_x_attr = "Connectivity".to_string();
                    self.scatter_y_attr = "NAIN_Rn".to_string();
                    self.needs_zoom_to_fit = true;
                    self.show_overpass_modal = false;

                    self.save_active_to_project_slot();
                    self.status_message = format!("OSM Pipeline Complete: {}", out.summary_line);
                    self.log_audit(format!("[OSM PIPELINE] {}", out.summary_line));
                    ctx.request_repaint();
                }
                Err(e) => {
                    self.status_message = format!("OSM Pipeline Error: {}", e);
                    self.log_audit(format!("[OSM PIPELINE ERROR] {}", e));
                    ctx.request_repaint();
                }
            }
        }
    }

    fn dialog_import_osm_boundary_file(&mut self) {
        let picked = rfd::FileDialog::new()
            .set_title("Import OSM Boundary Polygon (.gpkg / .dxf / .geojson)")
            .add_filter(
                "Geospatial Boundary Files (.gpkg, .dxf, .geojson, .json, .mif)",
                &["gpkg", "dxf", "geojson", "json", "mif"],
            )
            .add_filter("OGC GeoPackage (.gpkg)", &["gpkg"])
            .add_filter("AutoCAD DXF (.dxf)", &["dxf"])
            .add_filter("GeoJSON (.geojson, .json)", &["geojson", "json"])
            .pick_file();

        if let Some(path) = picked {
            match load_osm_boundary_file(&path, self.osm_map_center_lat, self.osm_map_center_lon) {
                Ok(imported) => {
                    self.overpass_south = imported.south;
                    self.overpass_west = imported.west;
                    self.overpass_north = imported.north;
                    self.overpass_east = imported.east;
                    self.osm_polygon_lat_lon = imported.polygon_lat_lon;
                    self.osm_polygon_closed = true;
                    self.osm_draw_tool = OsmDrawTool::PanNavigate;
                    let fname = path
                        .file_name()
                        .and_then(|s| s.to_str())
                        .unwrap_or("boundary");
                    self.osm_boundary_source =
                        format!("{} ({})", fname, imported.source_summary);
                    self.fit_osm_map_to_current_boundary(820.0, 380.0);
                    self.log_audit(format!(
                        "[OSM BOUNDARY] Imported '{}' -> {} ({:.4}, {:.4} to {:.4}, {:.4})",
                        fname,
                        imported.source_summary,
                        self.overpass_south,
                        self.overpass_west,
                        self.overpass_north,
                        self.overpass_east
                    ));
                }
                Err(e) => {
                    self.status_message = format!("Boundary Import Error: {}", e);
                    self.log_audit(format!("[OSM BOUNDARY ERROR] {}", e));
                }
            }
        }
    }

    #[inline]
    fn bar_horizontal_divider(&self, ui: &mut egui::Ui) {
        ui.add_space(2.0);
        let rect = ui.available_rect_before_wrap();
        let y = rect.min.y;
        let line_col = if self.dark_theme {
            Color32::from_rgb(45, 55, 72)
        } else {
            Color32::from_rgb(198, 206, 218)
        };
        ui.painter()
            .hline(rect.min.x..=rect.max.x, y, Stroke::new(1.0, line_col));
        ui.add_space(3.0);
    }

    #[inline]
    fn accent_text_color(&self) -> Color32 {
        if self.dark_theme {
            Color32::from_rgb(125, 195, 255)
        } else {
            Color32::from_rgb(15, 62, 145)
        }
    }

    #[inline]
    fn heading_text_color(&self) -> Color32 {
        if self.dark_theme {
            Color32::from_rgb(225, 233, 245)
        } else {
            Color32::from_rgb(15, 25, 42)
        }
    }

    #[inline]
    fn pill_text(&self, text: impl Into<String>, selected: bool) -> RichText {
        let col = if selected {
            Color32::WHITE
        } else if self.dark_theme {
            Color32::from_rgb(222, 230, 242)
        } else {
            Color32::from_rgb(16, 26, 44)
        };
        RichText::new(text).strong().color(col)
    }

    #[inline]
    fn btn_text(&self, text: impl Into<String>) -> RichText {
        let col = if self.dark_theme {
            Color32::from_rgb(225, 233, 245)
        } else {
            Color32::from_rgb(15, 25, 42)
        };
        RichText::new(text).strong().color(col)
    }

    fn apply_academic_theme(&self, ctx: &egui::Context) {
        let mut visuals = if self.dark_theme {
            egui::Visuals::dark()
        } else {
            egui::Visuals::light()
        };

        if self.dark_theme {
            visuals.override_text_color = None;
            visuals.panel_fill = Color32::from_rgb(18, 22, 29);
            visuals.window_fill = Color32::from_rgb(22, 27, 35);
            visuals.extreme_bg_color = Color32::from_rgb(11, 14, 19);
            visuals.faint_bg_color = Color32::from_rgb(26, 32, 42);
            visuals.widgets.noninteractive.bg_fill = Color32::from_rgb(22, 27, 35);
            visuals.widgets.noninteractive.bg_stroke =
                Stroke::new(1.0, Color32::from_rgb(45, 55, 72));
            visuals.widgets.noninteractive.fg_stroke =
                Stroke::new(1.0, Color32::from_rgb(218, 226, 238));
            visuals.widgets.inactive.bg_fill = Color32::from_rgb(30, 38, 50);
            visuals.widgets.inactive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(56, 68, 89));
            visuals.widgets.inactive.fg_stroke =
                Stroke::new(1.0, Color32::from_rgb(218, 226, 238));
            visuals.widgets.hovered.bg_fill = Color32::from_rgb(42, 56, 78);
            visuals.widgets.hovered.fg_stroke = Stroke::new(1.0, Color32::WHITE);
            visuals.widgets.active.bg_fill = Color32::from_rgb(24, 90, 185);
            visuals.widgets.active.fg_stroke = Stroke::new(1.0, Color32::WHITE);
            visuals.widgets.open.bg_fill = Color32::from_rgb(36, 48, 66);
            visuals.widgets.open.fg_stroke = Stroke::new(1.0, Color32::WHITE);
            visuals.selection.bg_fill = Color32::from_rgb(24, 90, 185);
            visuals.selection.stroke = Stroke::new(1.0, Color32::WHITE);
            visuals.hyperlink_color = Color32::from_rgb(115, 195, 255);
        } else {
            // Note: In egui, RichText::strong() reads visuals.widgets.active.fg_stroke.color!
            // Therefore widgets.active.fg_stroke MUST be dark navy ink in Light Mode!
            visuals.override_text_color = None;
            visuals.panel_fill = Color32::from_rgb(244, 246, 249);
            visuals.window_fill = Color32::from_rgb(252, 252, 253);
            visuals.extreme_bg_color = Color32::from_rgb(255, 255, 255);
            visuals.faint_bg_color = Color32::from_rgb(233, 237, 243);
            visuals.widgets.noninteractive.bg_fill = Color32::from_rgb(248, 249, 251);
            visuals.widgets.noninteractive.bg_stroke =
                Stroke::new(1.0, Color32::from_rgb(192, 200, 212));
            visuals.widgets.noninteractive.fg_stroke =
                Stroke::new(1.0, Color32::from_rgb(20, 30, 48));
            visuals.widgets.inactive.bg_fill = Color32::from_rgb(228, 233, 241);
            visuals.widgets.inactive.bg_stroke =
                Stroke::new(1.0, Color32::from_rgb(176, 186, 202));
            visuals.widgets.inactive.fg_stroke =
                Stroke::new(1.0, Color32::from_rgb(15, 25, 42));
            visuals.widgets.hovered.bg_fill = Color32::from_rgb(206, 218, 236);
            visuals.widgets.hovered.fg_stroke =
                Stroke::new(1.0, Color32::from_rgb(8, 16, 32));
            visuals.widgets.active.bg_fill = Color32::from_rgb(185, 206, 238);
            visuals.widgets.active.fg_stroke =
                Stroke::new(1.0, Color32::from_rgb(10, 18, 34));
            visuals.widgets.open.bg_fill = Color32::from_rgb(210, 222, 240);
            visuals.widgets.open.fg_stroke =
                Stroke::new(1.0, Color32::from_rgb(10, 18, 34));
            visuals.selection.bg_fill = Color32::from_rgb(24, 90, 185);
            visuals.selection.stroke = Stroke::new(1.0, Color32::WHITE);
            visuals.hyperlink_color = Color32::from_rgb(15, 62, 145);
        }
        ctx.set_visuals(visuals);
        ctx.style_mut(|style| {
            style.spacing.item_spacing = egui::vec2(6.0, 5.0);
            style.spacing.button_padding = egui::vec2(7.0, 3.5);
            style.spacing.interact_size.y = 22.0;
        });
    }

    fn save_active_to_project_slot(&mut self) {
        if self.active_project_idx >= self.projects.len() {
            return;
        }
        let doc = ProjectDocument {
            tab_title: self.tab_title.clone(),
            project_domain: self.project_domain,
            active_preset: self.active_preset,
            dataset_title: self.dataset_title.clone(),
            dataset_subtitle: self.dataset_subtitle.clone(),
            raw_polylines: self.raw_polylines.clone(),
            working_polylines: self.working_polylines.clone(),
            boundary_rings: self.boundary_rings.clone(),
            obstacles: self.obstacles.clone(),
            segments: self.segments.clone(),
            grade_unlinks: self.grade_unlinks.clone(),
            csr_graph: self.csr_graph.clone(),
            asa_config: self.asa_config.clone(),
            radii_input_str: self.radii_input_str.clone(),
            prune_axial_whiskers: self.prune_axial_whiskers,
            axial_stub_ratio: self.axial_stub_ratio,
            asa_results: self.asa_results.clone(),
            step_depth_results: self.step_depth_results.clone(),
            axial_results: self.axial_results.clone(),
            strokes_results: self.strokes_results.clone(),
            cleaner_config: self.cleaner_config.clone(),
            cleaner_report: self.cleaner_report.clone(),
            vga_cell_size_m: self.vga_cell_size_m,
            vga_max_vis_m: self.vga_max_vis_m,
            vga_compute_isovist_cols: self.vga_compute_isovist_cols,
            vga_results: self.vga_results.clone(),
            cast_isovists: self.cast_isovists.clone(),
            agent_config: self.agent_config.clone(),
            agent_state: self.agent_state.clone(),
            is_playing_agents: self.is_playing_agents,
            active_map_mode: self.active_map_mode,
            active_tab: self.active_tab,
            active_attribute: self.active_attribute.clone(),
            selected_ids: self.selected_ids.clone(),
            scatter_x_attr: self.scatter_x_attr.clone(),
            scatter_y_attr: self.scatter_y_attr.clone(),
            cam_center: self.cam_center,
            cam_zoom: self.cam_zoom,
            needs_zoom_to_fit: self.needs_zoom_to_fit,
            last_solver_ms: self.last_solver_ms,
        };
        self.projects[self.active_project_idx] = doc;
    }

    fn load_from_project_slot(&mut self, idx: usize) {
        let Some(doc) = self.projects.get(idx).cloned() else {
            return;
        };
        self.active_project_idx = idx;
        self.tab_title = doc.tab_title;
        self.project_domain = doc.project_domain;
        self.active_preset = doc.active_preset;
        self.dataset_title = doc.dataset_title;
        self.dataset_subtitle = doc.dataset_subtitle;
        self.raw_polylines = doc.raw_polylines;
        self.working_polylines = doc.working_polylines;
        self.boundary_rings = doc.boundary_rings;
        self.obstacles = doc.obstacles;
        self.segments = doc.segments;
        self.grade_unlinks = doc.grade_unlinks;
        self.csr_graph = doc.csr_graph;
        self.asa_config = doc.asa_config;
        self.radii_input_str = doc.radii_input_str;
        self.prune_axial_whiskers = doc.prune_axial_whiskers;
        self.axial_stub_ratio = doc.axial_stub_ratio;
        self.asa_results = doc.asa_results;
        self.step_depth_results = doc.step_depth_results;
        self.axial_results = doc.axial_results;
        self.strokes_results = doc.strokes_results;
        self.cleaner_config = doc.cleaner_config;
        self.cleaner_report = doc.cleaner_report;
        self.vga_cell_size_m = doc.vga_cell_size_m;
        self.vga_max_vis_m = doc.vga_max_vis_m;
        self.vga_compute_isovist_cols = doc.vga_compute_isovist_cols;
        self.vga_results = doc.vga_results;
        self.cast_isovists = doc.cast_isovists;
        self.agent_config = doc.agent_config;
        self.agent_state = doc.agent_state;
        self.is_playing_agents = doc.is_playing_agents;
        self.active_map_mode = doc.active_map_mode;
        self.active_tab = doc.active_tab;
        self.active_attribute = doc.active_attribute;
        self.selected_ids = doc.selected_ids;
        self.scatter_x_attr = doc.scatter_x_attr;
        self.scatter_y_attr = doc.scatter_y_attr;
        self.cam_center = doc.cam_center;
        self.cam_zoom = doc.cam_zoom;
        self.needs_zoom_to_fit = doc.needs_zoom_to_fit;
        self.last_solver_ms = doc.last_solver_ms;
        self.hovered_feature_idx = None;
        self.cancel_vga_background_job();
    }

    fn create_new_blank_project_tab(&mut self) {
        self.save_active_to_project_slot();
        self.next_untitled_counter += 1;
        let title = format!("Untitled Project {}", self.next_untitled_counter);
        let new_doc = ProjectDocument::new_blank(title.clone());
        self.projects.push(new_doc);
        let new_idx = self.projects.len() - 1;
        self.load_from_project_slot(new_idx);
        self.log_audit(format!("[PROJECT] Created new blank tab '{}'.", title));
    }

    fn switch_to_project_tab(&mut self, idx: usize) {
        if idx < self.projects.len() && idx != self.active_project_idx {
            self.save_active_to_project_slot();
            self.load_from_project_slot(idx);
        }
    }

    fn close_project_tab(&mut self, idx: usize) {
        if idx >= self.projects.len() {
            return;
        }
        self.save_active_to_project_slot();
        let closed_name = self.projects[idx].tab_title.clone();
        if self.projects.len() <= 1 {
            self.next_untitled_counter = 1;
            self.projects[0] = ProjectDocument::new_blank("Untitled Project 1".to_string());
            self.load_from_project_slot(0);
            self.log_audit(format!(
                "[PROJECT] Closed '{}' and reset to blank 'Untitled Project 1'.",
                closed_name
            ));
        } else {
            self.projects.remove(idx);
            let next_idx = if self.active_project_idx >= self.projects.len() {
                self.projects.len() - 1
            } else if idx < self.active_project_idx {
                self.active_project_idx - 1
            } else {
                self.active_project_idx
            };
            self.load_from_project_slot(next_idx);
            self.log_audit(format!("[PROJECT] Closed project tab '{}'.", closed_name));
        }
    }

    fn log_audit(&mut self, msg: impl Into<String>) {
        let entry = msg.into();
        self.status_message = entry.clone();
        self.audit_log_entries.push(entry);
    }

    fn parse_radii_input(&mut self) {
        let mut parsed = Vec::new();
        for tok in self.radii_input_str.split(|c: char| c == ',' || c == ';' || c.is_whitespace()) {
            let t = tok.trim().to_lowercase();
            if t.is_empty() {
                continue;
            }
            if t == "n" || t == "rn" || t == "inf" || t == "global" {
                parsed.push(f64::INFINITY);
            } else if let Ok(v) = t.trim_start_matches('r').trim_end_matches('m').parse::<f64>() {
                if v > 0.0 {
                    parsed.push(v);
                }
            }
        }
        if parsed.is_empty() {
            parsed.push(f64::INFINITY);
        }
        self.asa_config.radii = parsed;
    }

    fn load_preset(&mut self, preset: BenchmarkPreset) {
        self.cancel_vga_background_job();
        self.active_preset = Some(preset);
        self.project_domain = if preset.is_spatial_vga() {
            AnalysisDomain::SpatialVisibility
        } else {
            AnalysisDomain::StreetNetwork
        };
        self.is_playing_agents = false;
        self.click_mode = CanvasClickMode::InspectAndSelectOrigin;
        self.selected_ids.clear();
        self.cast_isovists.clear();
        self.agent_config.release_cells.clear();
        self.segments.clear();
        self.grade_unlinks.clear();
        self.csr_graph = None;
        self.asa_results = AsaResults::default();
        self.step_depth_results = None;
        self.cleaner_report = None;
        self.axial_results = None;
        self.strokes_results = None;
        self.vga_results = None;
        self.agent_state = None;

        let ds: BenchmarkDataset = load_benchmark(preset);
        self.dataset_title = ds.name.to_string();
        self.dataset_subtitle = ds.subtitle.to_string();
        self.tab_title = match preset {
            BenchmarkPreset::UrbanSuperblockGrid => "barnsbury_segment_lines.mif".to_string(),
            BenchmarkPreset::AxialStarDiamond => "barnsbury_extended1.dxf".to_string(),
            BenchmarkPreset::ArchitecturalGalleryVga => "gallery.dxf".to_string(),
            BenchmarkPreset::UclRoomsFloorplan => "rooms.dxf".to_string(),
            BenchmarkPreset::RingRoadCulDeSacLoop => "Ring-Road Loop Guard".to_string(),
        };
        self.raw_polylines = ds.lines.clone();
        self.working_polylines = ds.lines;
        self.boundary_rings = ds.boundary_rings;
        self.obstacles = ds.obstacles;
        self.vga_cell_size_m = ds.default_vga_cell_m;

        match preset {
            BenchmarkPreset::UrbanSuperblockGrid => {
                self.vga_max_vis_m = 250.0;
                self.rebuild_segment_graph_and_solve_asa();
                self.run_natural_roads_solver();
                self.active_map_mode = ActiveMapMode::SegmentMapAsa;
                self.active_attribute = "NACH_Rn".to_string();
                self.scatter_x_attr = "Connectivity".to_string();
                self.scatter_y_attr = "NAIN_Rn".to_string();
            }
            BenchmarkPreset::AxialStarDiamond => {
                self.vga_max_vis_m = 250.0;
                self.rebuild_segment_graph_and_solve_asa();
                self.run_axial_solver();
                self.active_map_mode = ActiveMapMode::SegmentMapAsa;
                self.active_attribute = "NACH_Rn".to_string();
                self.scatter_x_attr = "Connectivity".to_string();
                self.scatter_y_attr = "NAIN_Rn".to_string();
            }
            BenchmarkPreset::ArchitecturalGalleryVga => {
                self.vga_max_vis_m = 50.0;
                self.run_vga_solver_sync();
                self.active_map_mode = ActiveMapMode::VgaPointMap;
                self.active_attribute = "Visual_Integration_HH".to_string();
                self.scatter_x_attr = "Connectivity".to_string();
                self.scatter_y_attr = "Visual_Integration_HH".to_string();
            }
            BenchmarkPreset::UclRoomsFloorplan => {
                self.vga_max_vis_m = 30.0;
                self.run_vga_solver_sync();
                self.active_map_mode = ActiveMapMode::VgaPointMap;
                self.active_attribute = "Visual_Integration_HH".to_string();
                self.scatter_x_attr = "Connectivity".to_string();
                self.scatter_y_attr = "Visual_Integration_HH".to_string();
            }
            BenchmarkPreset::RingRoadCulDeSacLoop => {
                self.vga_max_vis_m = 250.0;
                self.rebuild_segment_graph_and_solve_asa();
                self.run_natural_roads_solver();
                self.active_map_mode = ActiveMapMode::NaturalRoadsStrokes;
                self.active_attribute = "Stroke_Connectivity".to_string();
                self.scatter_x_attr = "Stroke_Length".to_string();
                self.scatter_y_attr = "Stroke_Connectivity".to_string();
            }
        }
        let (bmin, bmax) = self.world_bounds();
        self.cam_center = Point2D::new(0.5 * (bmin.x + bmax.x), 0.5 * (bmin.y + bmax.y));
        let bw = (bmax.x - bmin.x).max(0.5);
        let bh = (bmax.y - bmin.y).max(0.5);
        self.cam_zoom = (1050.0 / bw).min(700.0 / bh).clamp(0.05, 2500.0);
        self.needs_zoom_to_fit = true;
        self.save_active_to_project_slot();
        if self.project_domain == AnalysisDomain::SpatialVisibility {
            let cell_n = self.vga_results.as_ref().map_or(0, |v| v.points.len());
            self.log_audit(format!(
                "[PRESET] Loaded '{}' [{}] ({} wall polylines, {} VGA cells).",
                self.dataset_title,
                self.project_domain.short_label(),
                self.obstacles.len(),
                cell_n
            ));
        } else {
            self.log_audit(format!(
                "[PRESET] Loaded '{}' [{}] ({} polylines, {} segments).",
                self.dataset_title,
                self.project_domain.short_label(),
                self.working_polylines.len(),
                self.segments.len()
            ));
        }
    }

    fn load_custom_polylines(
        &mut self,
        title: String,
        subtitle: String,
        lines: Vec<Polyline>,
        domain: AnalysisDomain,
    ) {
        self.cancel_vga_background_job();
        self.active_preset = None;
        self.project_domain = domain;
        self.is_playing_agents = false;
        self.click_mode = CanvasClickMode::InspectAndSelectOrigin;
        self.selected_ids.clear();
        self.cast_isovists.clear();
        self.agent_config.release_cells.clear();
        self.step_depth_results = None;
        self.cleaner_report = None;
        self.axial_results = None;
        self.strokes_results = None;
        self.vga_results = None;
        self.agent_state = None;

        self.tab_title = title.clone();
        self.dataset_title = title.clone();
        self.dataset_subtitle = subtitle;
        self.raw_polylines = lines.clone();
        self.working_polylines = lines.clone();
        self.boundary_rings.clear();
        self.needs_zoom_to_fit = true;

        match domain {
            AnalysisDomain::StreetNetwork => {
                self.obstacles.clear();
                self.rebuild_segment_graph_and_solve_asa();
                self.active_map_mode = ActiveMapMode::SegmentMapAsa;
                self.active_attribute = "NACH_Rn".to_string();
                self.scatter_x_attr = "Connectivity".to_string();
                self.scatter_y_attr = "NAIN_Rn".to_string();
            }
            AnalysisDomain::SpatialVisibility => {
                self.obstacles = lines;
                self.segments.clear();
                let (bmin, bmax) = self.world_bounds();
                let raw_span = (bmax.x - bmin.x).max(bmax.y - bmin.y).max(1e-3);
                // If an imported architectural floorplan was digitized in small paper/inch units (span < 8.0),
                // scale it by 5.0x into metric floorplan coordinates so 0.30 m VGA grid works naturally
                if raw_span < 8.0 {
                    for pl in &mut self.raw_polylines {
                        for p in &mut pl.coords {
                            p.x *= 5.0;
                            p.y *= 5.0;
                        }
                    }
                    self.working_polylines = self.raw_polylines.clone();
                    self.obstacles = self.raw_polylines.clone();
                }
                let (bmin, bmax) = self.world_bounds();
                let span = (bmax.x - bmin.x).max(bmax.y - bmin.y).max(1.0);
                self.vga_cell_size_m = (span / 60.0).clamp(0.10, 5.0);
                self.vga_max_vis_m = (span * 2.0).max(30.0);
                // Create the walkable grid immediately; user can then adjust Grid Cell Size, run VGA, or place agents manually
                self.create_or_fill_vga_grid(None);
                self.active_map_mode = ActiveMapMode::VgaPointMap;
                self.active_attribute = "Connectivity".to_string();
                self.scatter_x_attr = "Connectivity".to_string();
                self.scatter_y_attr = "Visual_Integration_HH".to_string();
            }
        }
        self.save_active_to_project_slot();
        self.log_audit(format!(
            "[IO] Loaded '{}' in {} mode ({} polylines).",
            title,
            domain.short_label(),
            self.working_polylines.len(),
        ));
    }

    fn switch_project_domain(&mut self, new_domain: AnalysisDomain) {
        if self.project_domain == new_domain {
            return;
        }
        self.cancel_vga_background_job();
        self.project_domain = new_domain;
        self.is_playing_agents = false;
        self.selected_ids.clear();
        match new_domain {
            AnalysisDomain::StreetNetwork => {
                self.obstacles.clear();
                self.boundary_rings.clear();
                self.click_mode = CanvasClickMode::InspectAndSelectOrigin;
                if self.segments.is_empty() && !self.working_polylines.is_empty() {
                    self.rebuild_segment_graph_and_solve_asa();
                }
                self.active_map_mode = ActiveMapMode::SegmentMapAsa;
                if !self.asa_results.attributes.contains_key(&self.active_attribute) {
                    self.active_attribute = "NACH_Rn".to_string();
                }
            }
            AnalysisDomain::SpatialVisibility => {
                if self.obstacles.is_empty() && !self.working_polylines.is_empty() {
                    self.obstacles = self.working_polylines.clone();
                }
                if self.vga_results.is_none() && !self.obstacles.is_empty() {
                    self.create_or_fill_vga_grid(None);
                }
                self.active_map_mode = ActiveMapMode::VgaPointMap;
                if let Some(vga) = &self.vga_results {
                    if !vga.attributes.contains_key(&self.active_attribute) {
                        self.active_attribute = "Visual_Integration_HH".to_string();
                    }
                }
            }
        }
        self.save_active_to_project_slot();
    }

    fn rebuild_segment_graph_and_solve_asa(&mut self) {
        self.parse_radii_input();
        let stub_ratio = if self.prune_axial_whiskers {
            self.axial_stub_ratio
        } else {
            0.0
        };
        let noded = planarize_and_node_pruned(&self.working_polylines, stub_ratio);
        let segs = explode_to_2point_segments(&noded);
        let unlinks = detect_grade_unlinks(&segs);
        let graph =
            CSRDualGraph::from_segments(&segs, &unlinks, self.asa_config.tulip_bins, 0.0001);
        let res = solve_angular_segment_analysis(&graph, &self.asa_config);
        self.last_solver_ms = res.elapsed_ms;
        self.segments = segs;
        self.grade_unlinks = unlinks;
        self.csr_graph = Some(graph);
        self.asa_results = res;

        if !self.asa_results.attributes.contains_key(&self.active_attribute) {
            if self.asa_results.attributes.contains_key("NACH_Rn") {
                self.active_attribute = "NACH_Rn".to_string();
            } else if let Some(first_k) = self.asa_results.attributes.keys().next() {
                self.active_attribute = first_k.clone();
            }
        }
        if !self.asa_results.attributes.contains_key(&self.scatter_x_attr) {
            self.scatter_x_attr = "Connectivity".to_string();
        }
        if !self.asa_results.attributes.contains_key(&self.scatter_y_attr) {
            self.scatter_y_attr = if self.asa_results.attributes.contains_key("NAIN_Rn") {
                "NAIN_Rn".to_string()
            } else {
                self.asa_results
                    .attributes
                    .keys()
                    .nth(1)
                    .or_else(|| self.asa_results.attributes.keys().next())
                    .cloned()
                    .unwrap_or_default()
            };
        }
        self.log_audit(format!(
            "[ASA] Solved {} segments across {} radii in {:.2} ms (Intelligibility R² = {:.4}).",
            self.segments.len(),
            self.asa_config.radii.len(),
            self.last_solver_ms,
            self.asa_results.intelligibility_r2
        ));
    }

    fn run_cleaner_5step(&mut self) {
        let (cleaned, report) =
            clean_network_adaptive_watertight(&self.raw_polylines, &self.cleaner_config);
        self.last_solver_ms = report.elapsed_ms;
        self.working_polylines = cleaned;
        self.log_audit(format!(
            "[CLEANER] 5-Step P90 Cleaner (ext={:.2}m): dangles {} -> {}, junctions {} -> {}, components {} -> {} ({:.2} ms).",
            report.effective_extend_distance,
            report.raw_dangles_count,
            report.cleaned_dangles_count,
            report.raw_junctions_count,
            report.cleaned_junctions_count,
            report.raw_components_count,
            report.cleaned_components_count,
            report.elapsed_ms
        ));
        self.cleaner_report = Some(report);
        self.rebuild_segment_graph_and_solve_asa();
    }

    fn run_step_depth_from_selected(&mut self) {
        let Some(graph) = &self.csr_graph else {
            return;
        };
        let origins: Vec<usize> = if self.selected_ids.is_empty() {
            if self.segments.is_empty() {
                return;
            }
            vec![self.segments.len() / 2]
        } else {
            self.selected_ids
                .iter()
                .copied()
                .filter(|&id| id < self.segments.len())
                .collect()
        };
        if origins.is_empty() {
            return;
        }
        let sd = solve_origin_step_depth(
            graph,
            &origins,
            f64::INFINITY,
            self.asa_config.radius_type,
        );
        self.last_solver_ms = sd.elapsed_ms;
        self.asa_results
            .attributes
            .insert("Angular_Step_Depth".to_string(), sd.angular_step_depth.clone());
        self.asa_results
            .attributes
            .insert("Metric_Step_Depth_m".to_string(), sd.metric_step_depth_m.clone());
        self.asa_results.attributes.insert(
            "Topo_Step_Depth".to_string(),
            sd.topo_step_depth.iter().map(|&v| v as f64).collect(),
        );
        self.step_depth_results = Some(sd);
        self.active_map_mode = ActiveMapMode::SegmentMapAsa;
        self.active_attribute = "Angular_Step_Depth".to_string();
        self.log_audit(format!(
            "[STEP DEPTH] Solved Multi-Source Origin Step Depth from {:?} in {:.2} ms.",
            origins, self.last_solver_ms
        ));
    }

    fn run_axial_solver(&mut self) {
        // Solve Hillier & Hanson (1984) Axial Analysis on unbroken sightlines (never planarize/chop!)
        let axial_input: Vec<Polyline> = if self.prune_axial_whiskers {
            trim_axial_overshoot_whiskers(&self.working_polylines, self.axial_stub_ratio)
        } else {
            self.working_polylines.clone()
        };
        let res = solve_axial_analysis(&axial_input, &[3, 5, 0], true);
        self.last_solver_ms = res.elapsed_ms;
        let line_count = res.lines.len();
        let r2 = res.intelligibility_r2;
        self.axial_results = Some(res);
        self.active_map_mode = ActiveMapMode::AxialMap;
        if let Some(ax) = &self.axial_results {
            if !ax.attributes.contains_key(&self.active_attribute) {
                self.active_attribute = if ax.attributes.contains_key("Integration_HH_n") {
                    "Integration_HH_n".to_string()
                } else {
                    ax.attributes.keys().next().cloned().unwrap_or_default()
                };
            }
            if !ax.attributes.contains_key(&self.scatter_x_attr) {
                self.scatter_x_attr = "Connectivity".to_string();
            }
            if !ax.attributes.contains_key(&self.scatter_y_attr) {
                self.scatter_y_attr = "Integration_HH_n".to_string();
            }
        }
        self.log_audit(format!(
            "[AXIAL] Solved 16-Attribute Hillier & Hanson Axial Analysis on {} unbroken sightlines in {:.2} ms (Intelligibility R² = {:.4}).",
            line_count, self.last_solver_ms, r2
        ));
    }

    fn run_natural_roads_solver(&mut self) {
        let stroke_input: Vec<Polyline> = if self.prune_axial_whiskers {
            planarize_and_node_pruned(&self.working_polylines, self.axial_stub_ratio)
        } else {
            self.working_polylines.clone()
        };
        let res = solve_natural_roads(&stroke_input, 45.0);
        self.last_solver_ms = res.elapsed_ms;
        let count = res.strokes.len();
        let circ_str = match res.circuity_avg {
            Some(c) => format!("{:.4}", c),
            None => "None (Closed Loop Guard)".to_string(),
        };
        self.strokes_results = Some(res);
        self.active_map_mode = ActiveMapMode::NaturalRoadsStrokes;
        if let Some(st) = &self.strokes_results {
            if !st.attributes.contains_key(&self.active_attribute) {
                self.active_attribute = if st.attributes.contains_key("Stroke_Connectivity") {
                    "Stroke_Connectivity".to_string()
                } else {
                    st.attributes.keys().next().cloned().unwrap_or_default()
                };
            }
            if !st.attributes.contains_key(&self.scatter_x_attr) {
                self.scatter_x_attr = "Stroke_Length".to_string();
            }
            if !st.attributes.contains_key(&self.scatter_y_attr) {
                self.scatter_y_attr = "Stroke_Connectivity".to_string();
            }
        }
        let max_n = self.active_feature_count();
        self.selected_ids.retain(|&id| id < max_n);
        self.log_audit(format!(
            "[STROKES] Gestalt Natural Roads built {} strokes in {:.2} ms (circuity_avg = {}).",
            count, self.last_solver_ms, circ_str
        ));
    }

    fn cancel_vga_background_job(&mut self) {
        if let Some(job) = self.vga_job.take() {
            job.cancel_flag.store(true, Ordering::Relaxed);
        }
    }

    fn create_or_fill_vga_grid(&mut self, custom_seed: Option<Point2D>) {
        self.cancel_vga_background_job();
        self.is_playing_agents = false;
        self.agent_config.release_cells.clear();
        self.selected_ids.clear();

        let obs = if self.obstacles.is_empty() {
            &self.working_polylines
        } else {
            &self.obstacles
        };
        let requested_step = self.vga_cell_size_m.max(0.05);
        let pts = if let Some(seed) = custom_seed {
            generate_vga_grid_flood_fill(obs, seed, requested_step)
        } else {
            generate_vga_grid(&self.boundary_rings, obs, requested_step, true)
        };
        let eff_step = effective_vga_grid_spacing(&pts, requested_step);
        let lattice = build_walkable_vga_grid_lattice(&pts, obs, eff_step);
        self.last_solver_ms = lattice.elapsed_ms;
        let pt_count = lattice.points.len();
        self.vga_results = Some(lattice);
        self.init_or_reset_agents();
        self.active_map_mode = ActiveMapMode::VgaPointMap;
        if let Some(vga) = &self.vga_results {
            if !vga.attributes.contains_key(&self.active_attribute) {
                self.active_attribute = "Connectivity".to_string();
            }
        }
        if (eff_step - requested_step).abs() > 1e-4 {
            self.log_audit(format!(
                "[GRID] Created walkable VGA grid: {} cells in {:.2} ms (requested {:.2}m auto-scaled to {:.2}m for {} interactive cap).",
                pt_count, self.last_solver_ms, requested_step, eff_step, MAX_INTERACTIVE_VGA_CELLS
            ));
        } else {
            self.log_audit(format!(
                "[GRID] Created walkable VGA grid: {} cells (spacing={:.2}m) in {:.2} ms. Ready to compute VGA or place agents manually.",
                pt_count, eff_step, self.last_solver_ms
            ));
        }
    }

    fn run_vga_solver_sync(&mut self) {
        self.cancel_vga_background_job();
        let obs = if self.obstacles.is_empty() {
            &self.working_polylines
        } else {
            &self.obstacles
        };
        let requested_step = self.vga_cell_size_m.max(0.05);
        let pts = generate_vga_grid(&self.boundary_rings, obs, requested_step, true);
        let eff_step = effective_vga_grid_spacing(&pts, requested_step);
        let res = solve_vga(
            &pts,
            obs,
            eff_step,
            self.vga_max_vis_m,
            true,
            self.vga_compute_isovist_cols,
        );
        self.last_solver_ms = res.elapsed_ms;
        let pt_count = res.points.len();
        self.vga_results = Some(res);
        self.init_or_reset_agents();
        self.active_map_mode = ActiveMapMode::VgaPointMap;
        if let Some(vga) = &self.vga_results {
            if !vga.attributes.contains_key(&self.active_attribute) {
                self.active_attribute = if vga.attributes.contains_key("Visual_Integration_HH") {
                    "Visual_Integration_HH".to_string()
                } else {
                    vga.attributes.keys().next().cloned().unwrap_or_default()
                };
            }
        }
        self.log_audit(format!(
            "[VGA] Solved Visibility Graph Analysis across {} PointMap cells (cell={:.2}m) in {:.2} ms.",
            pt_count, eff_step, self.last_solver_ms
        ));
    }

    fn run_vga_solver(&mut self) {
        self.cancel_vga_background_job();
        let obs_vec: Vec<Polyline> = if self.obstacles.is_empty() {
            self.working_polylines.clone()
        } else {
            self.obstacles.clone()
        };
        let requested_step = self.vga_cell_size_m.max(0.05);
        let pts = generate_vga_grid(&self.boundary_rings, &obs_vec, requested_step, true);
        let eff_step = effective_vga_grid_spacing(&pts, requested_step);

        // Immediately build and display the walkable grid lattice (< 1.5 ms) so the viewport updates right away
        let lattice = build_walkable_vga_grid_lattice(&pts, &obs_vec, eff_step);
        let pt_count = lattice.points.len();
        self.vga_results = Some(lattice);
        self.init_or_reset_agents();
        self.active_map_mode = ActiveMapMode::VgaPointMap;

        // Spawn non-blocking background worker thread for full all-pairs LOS + BFS VGA metrics
        let max_vis_m = self.vga_max_vis_m;
        let compute_iso = self.vga_compute_isovist_cols;
        let progress_pct = Arc::new(AtomicUsize::new(1));
        let cancel_flag = Arc::new(AtomicBool::new(false));
        let prog_clone = Arc::clone(&progress_pct);
        let cancel_clone = Arc::clone(&cancel_flag);
        let (tx, rx) = mpsc::channel();

        std::thread::spawn(move || {
            let res = solve_vga_with_progress(
                &pts,
                &obs_vec,
                eff_step,
                max_vis_m,
                true,
                compute_iso,
                Some(&prog_clone),
                Some(&cancel_clone),
            );
            let _ = tx.send(res);
        });

        self.vga_job = Some(VgaBackgroundJob {
            rx,
            progress_pct,
            cancel_flag,
            cell_count: pt_count,
            effective_cell_m: eff_step,
            requested_cell_m: requested_step,
        });

        if (eff_step - requested_step).abs() > 1e-4 {
            self.status_message = format!(
                "[VGA] Computing {} cells in background (requested {:.2}m -> effective {:.2}m to fit {} interactive cap)...",
                pt_count, requested_step, eff_step, MAX_INTERACTIVE_VGA_CELLS
            );
        } else {
            self.status_message = format!(
                "[VGA] Computing Visibility Graph across {} cells (cell={:.2}m) in background...",
                pt_count, eff_step
            );
        }
    }

    fn poll_vga_background_job(&mut self, ctx: &egui::Context) {
        let Some(job) = &self.vga_job else {
            return;
        };
        match job.rx.try_recv() {
            Ok(Some(res)) => {
                let eff_m = job.effective_cell_m;
                let req_m = job.requested_cell_m;
                self.vga_job = None;
                self.last_solver_ms = res.elapsed_ms;
                let pt_count = res.points.len();
                self.vga_results = Some(res);
                self.init_or_reset_agents();
                self.active_map_mode = ActiveMapMode::VgaPointMap;
                if let Some(vga) = &self.vga_results {
                    if !vga.attributes.contains_key(&self.active_attribute)
                        || self.active_attribute == "Connectivity"
                    {
                        if vga.attributes.contains_key("Visual_Integration_HH") {
                            self.active_attribute = "Visual_Integration_HH".to_string();
                        }
                    }
                }
                if (eff_m - req_m).abs() > 1e-4 {
                    self.log_audit(format!(
                        "[VGA] Solved {} PointMap cells in {:.2} ms (requested {:.2}m auto-scaled to {:.2}m for interactive performance).",
                        pt_count, self.last_solver_ms, req_m, eff_m
                    ));
                } else {
                    self.log_audit(format!(
                        "[VGA] Solved Visibility Graph Analysis across {} PointMap cells (cell={:.2}m) in {:.2} ms.",
                        pt_count, eff_m, self.last_solver_ms
                    ));
                }
                self.save_active_to_project_slot();
                ctx.request_repaint();
            }
            Ok(None) => {
                self.vga_job = None;
                self.log_audit("[VGA] Background VGA computation cancelled.".to_string());
                ctx.request_repaint();
            }
            Err(mpsc::TryRecvError::Empty) => {
                let pct = job.progress_pct.load(Ordering::Relaxed);
                self.status_message = format!(
                    "[VGA] Computing Visibility Graph ({} cells, {:.2}m): {}%...",
                    job.cell_count, job.effective_cell_m, pct
                );
                ctx.request_repaint_after(Duration::from_millis(30));
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.vga_job = None;
            }
        }
    }

    fn place_agents_at_world_point(&mut self, wp: Point2D) {
        if self.vga_results.is_none() {
            self.create_or_fill_vga_grid(Some(wp));
        }
        let Some(vga) = &self.vga_results else {
            return;
        };
        let mut best = (usize::MAX, (vga.cell_size_m * 1.6).max(0.5));
        for (idx, pt) in vga.points.iter().enumerate() {
            let d = wp.dist(Point2D::new(pt.x, pt.y));
            if d < best.1 {
                best = (idx, d);
            }
        }
        let cell_idx = best.0;
        if cell_idx == usize::MAX {
            self.status_message =
                "Click inside a walkable VGA grid cell to place/release agents.".to_string();
            return;
        }
        if !self.agent_config.release_cells.contains(&cell_idx) {
            self.agent_config.release_cells.push(cell_idx);
        }
        if self.agent_state.is_none() {
            let st = init_agent_simulation(vga, &self.agent_config);
            self.agent_state = Some(st);
        }
        let cell_pt = Point2D::new(vga.points[cell_idx].x, vga.points[cell_idx].y);
        let (active_len, pin_len) = if let (Some(vga_ref), Some(st)) =
            (&self.vga_results, &mut self.agent_state)
        {
            spawn_agents_at_cell(vga_ref, st, &self.agent_config, cell_idx, 1);
            (st.active_agents.len(), self.agent_config.release_cells.len())
        } else {
            (0, 0)
        };
        self.show_agent_bodies = true;
        self.agent_manual_single_mode = true;
        self.agent_step_accum = 0.0;
        self.sync_agent_gate_counts_to_vga();
        self.active_map_mode = ActiveMapMode::VgaPointMap;
        self.active_attribute = "Gate_Counts".to_string();
        self.log_audit(format!(
            "[AGENTS] Placed 1 stationary agent at cell #{} ({:.2}, {:.2}) m | Selected Locations: {} | Active Agents: {} (Press Play to start movement).",
            cell_idx, cell_pt.x, cell_pt.y, pin_len, active_len
        ));
    }

    fn init_or_reset_agents(&mut self) {
        self.agent_step_accum = 0.0;
        self.last_agent_frame_instant = None;
        self.agent_manual_single_mode = false;
        if let Some(vga) = &self.vga_results {
            let st = init_agent_simulation(vga, &self.agent_config);
            self.agent_state = Some(st);
            self.sync_agent_gate_counts_to_vga();
        }
    }

    fn sync_agent_gate_counts_to_vga(&mut self) {
        if let (Some(vga), Some(st)) = (&mut self.vga_results, &self.agent_state) {
            if st.gate_counts.len() == vga.points.len() {
                vga.attributes
                    .insert("Gate_Counts".to_string(), st.gate_counts.clone());
            }
        }
    }

    fn run_agents_batch_solver(&mut self) {
        if self.vga_results.is_none() {
            self.create_or_fill_vga_grid(None);
        }
        if let Some(vga) = &self.vga_results {
            let t0 = Instant::now();
            let st = solve_agents_batch(vga, &self.agent_config);
            self.last_solver_ms = t0.elapsed().as_secs_f64() * 1000.0;
            self.agent_state = Some(st);
            self.sync_agent_gate_counts_to_vga();
            self.active_map_mode = ActiveMapMode::VgaPointMap;
            self.active_attribute = "Gate_Counts".to_string();
            let mode_str = if self.agent_config.release_cells.is_empty() {
                "Random Floorplan Release".to_string()
            } else {
                format!("{} Manual Release Pins", self.agent_config.release_cells.len())
            };
            self.log_audit(format!(
                "[AGENTS] Simulated {} Gibson Visual Agent steps ({}, FOV={} bins) in {:.2} ms.",
                self.agent_config.timesteps, mode_str, self.agent_config.agent_fov_bins, self.last_solver_ms
            ));
        }
    }

    fn run_self_verification_suite(&mut self) {
        let mut rows = Vec::new();

        // Check 1: Benchmark A — 1D 3-Segment Collinear Chain (S0-S1-S2, 100m each)
        let chain_lines = vec![
            Polyline::new(vec![Point2D::new(0.0, 0.0), Point2D::new(100.0, 0.0)]),
            Polyline::new(vec![Point2D::new(100.0, 0.0), Point2D::new(200.0, 0.0)]),
            Polyline::new(vec![Point2D::new(200.0, 0.0), Point2D::new(300.0, 0.0)]),
        ];
        let (g_chain, _) = CSRDualGraph::from_polylines(&chain_lines, 0, 0.0001);
        let cfg_chain = AsaConfig {
            radii: vec![f64::INFINITY],
            analysis_type: AnalysisType::Metric,
            radius_type: RadiusType::Metric,
            weight_mode: WeightMode::Unweighted,
            tulip_bins: 0,
            include_normalized: true,
            max_sources_per_radius: 100,
        };
        let res_chain = solve_angular_segment_analysis(&g_chain, &cfg_chain);
        let ch = res_chain
            .attributes
            .get("Choice_Rn")
            .cloned()
            .unwrap_or_default();
        let md = res_chain
            .attributes
            .get("Mean_Depth_Rn")
            .cloned()
            .unwrap_or_default();
        let (c0, c1, md0, md1) = if ch.len() >= 3 && md.len() >= 3 {
            (ch[0], ch[1], md[0], md[1])
        } else {
            (-1.0, -1.0, -1.0, -1.0)
        };
        rows.push(VerificationCheckRow {
            code: "UL-01 / Bench-A",
            title: "3-Segment Collinear Chain (S0-S1-S2, 100m): Brandes Choice & Metric MD",
            expected: "Choice(S1)=2.0, Choice(S0)=0.0, MD(S1)=100.0m, MD(S0)=150.0m".to_string(),
            observed: format!(
                "Choice(S1)={:.4}, Choice(S0)={:.4}, MD(S1)={:.2}m, MD(S0)={:.2}m",
                c1, c0, md1, md0
            ),
            passed: (c1 - 2.0).abs() < 1e-4
                && c0.abs() < 1e-4
                && (md1 - 100.0).abs() < 1e-2
                && (md0 - 150.0).abs() < 1e-2,
        });

        // Check 2: Benchmark B — Hillier Diamond D_5 and 5-Line Axial Star K_{1,4}
        let d5 = calculate_hillier_diamond_dn(5);
        let star_lines = vec![
            Polyline::new(vec![Point2D::new(0.0, 0.0), Point2D::new(100.0, 0.0)]),
            Polyline::new(vec![Point2D::new(20.0, -30.0), Point2D::new(20.0, 30.0)]),
            Polyline::new(vec![Point2D::new(40.0, -30.0), Point2D::new(40.0, 30.0)]),
            Polyline::new(vec![Point2D::new(60.0, -30.0), Point2D::new(60.0, 30.0)]),
            Polyline::new(vec![Point2D::new(80.0, -30.0), Point2D::new(80.0, 30.0)]),
        ];
        let ax_star = solve_axial_analysis(&star_lines, &[0], true);
        let rra = ax_star
            .attributes
            .get("RRA_n")
            .cloned()
            .unwrap_or_default();
        let ihh = ax_star
            .attributes
            .get("Integration_HH_n")
            .cloned()
            .unwrap_or_default();
        let (rra_hub, rra_leaf, ihh_leaf) = if rra.len() == 5 && ihh.len() == 5 {
            (rra[0], rra[1], ihh[1])
        } else {
            (-1.0, -1.0, -1.0)
        };
        rows.push(VerificationCheckRow {
            code: "UL-02 / Bench-B",
            title: "Hillier Diamond D_5 & 5-Line Axial Star (K_{1,4}) RRA / Integration [HH]",
            expected: "D_5=0.351994, Hub RRA=0.0000, Leaf RRA=1.4205, Leaf Int[HH]=0.7040".to_string(),
            observed: format!(
                "D_5={:.6}, Hub RRA={:.4}, Leaf RRA={:.4}, Leaf Int[HH]={:.4}",
                d5, rra_hub, rra_leaf, ihh_leaf
            ),
            passed: (d5 - 0.35199368).abs() < 1e-5
                && rra_hub.abs() < 1e-4
                && (rra_leaf - 1.420480).abs() < 1e-3
                && (ihh_leaf - 0.703987).abs() < 1e-3,
        });

        // Check 3: Closed-Loop Zero-Chord Sinuosity & Circuity Guard (osmnx#1410)
        let ring = vec![
            Polyline::new(vec![Point2D::new(0.0, 0.0), Point2D::new(20.0, 0.0)]),
            Polyline::new(vec![Point2D::new(20.0, 0.0), Point2D::new(20.0, 1.0)]),
            Polyline::new(vec![Point2D::new(20.0, 1.0), Point2D::new(0.0, 1.0)]),
            Polyline::new(vec![Point2D::new(0.0, 1.0), Point2D::new(0.0, 0.0)]),
        ];
        let closed_poly = vec![Polyline::new(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(10.0, 0.0),
            Point2D::new(10.0, 10.0),
            Point2D::new(0.0, 0.0),
        ])];
        let circ_closed = compute_network_circuity_avg(&closed_poly);
        let nr_closed = solve_natural_roads(&closed_poly, 180.0);
        let stroke_sin = nr_closed.strokes.first().and_then(|s| s.sinuosity);
        let _ = ring;
        rows.push(VerificationCheckRow {
            code: "UL-03 / osmnx#1410",
            title: "Closed-Loop Zero-Chord Singularity Guard (sinuosity & circuity_avg)",
            expected: "Closed-Loop sinuosity = None, circuity_avg = None (No 1e6 blow-up)".to_string(),
            observed: format!("sinuosity = {:?}, circuity_avg = {:?}", stroke_sin, circ_closed),
            passed: stroke_sin.is_none() && circ_closed.is_none(),
        });

        // Check 4: Analytical 2D Benedikt Isovist & 32-Bin Octant Quantizer
        let box_obs = vec![Polyline::new(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(20.0, 0.0),
            Point2D::new(20.0, 20.0),
            Point2D::new(0.0, 20.0),
            Point2D::new(0.0, 0.0),
        ])];
        let iso = compute_isovist_at_point(Point2D::new(10.0, 10.0), &box_obs, 100.0);
        let b_east = whichbin(1.0, 0.0);
        let b_west = whichbin(-1.0, 0.0);
        rows.push(VerificationCheckRow {
            code: "UL-05 / Isovist-VGA",
            title: "20m x 20m Room Analytical Corner-Ray Isovist & 32-Bin whichbin()",
            expected: "Area=400.0 m², Compactness=0.7854 (pi/4), whichbin(+x)!=whichbin(-x)".to_string(),
            observed: format!(
                "Area={:.2} m², Compactness={:.4}, bins=({}, {})",
                iso.area, iso.compactness, b_east, b_west
            ),
            passed: (iso.area - 400.0).abs() < 2.0
                && (iso.compactness - std::f64::consts::FRAC_PI_4).abs() < 0.02
                && b_east != b_west,
        });

        self.verification_rows = rows;
    }

    fn active_feature_count(&self) -> usize {
        match self.active_map_mode {
            ActiveMapMode::SegmentMapAsa => self.segments.len(),
            ActiveMapMode::AxialMap => self.axial_results.as_ref().map_or(0, |r| r.lines.len()),
            ActiveMapMode::NaturalRoadsStrokes => {
                self.strokes_results.as_ref().map_or(0, |r| r.strokes.len())
            }
            ActiveMapMode::VgaPointMap => self.vga_results.as_ref().map_or(0, |r| r.points.len()),
        }
    }

    fn active_attributes_map(&self) -> Option<&BTreeMap<String, Vec<f64>>> {
        match self.active_map_mode {
            ActiveMapMode::SegmentMapAsa => Some(&self.asa_results.attributes),
            ActiveMapMode::AxialMap => self.axial_results.as_ref().map(|r| &r.attributes),
            ActiveMapMode::NaturalRoadsStrokes => {
                self.strokes_results.as_ref().map(|r| &r.attributes)
            }
            ActiveMapMode::VgaPointMap => self.vga_results.as_ref().map(|r| &r.attributes),
        }
    }

    fn active_attribute_values(&self) -> Vec<f64> {
        self.active_attributes_map()
            .and_then(|m| m.get(&self.active_attribute))
            .cloned()
            .unwrap_or_default()
    }

    fn compute_normalized_t_values(&self, values: &[f64]) -> Vec<f64> {
        if values.is_empty() {
            return Vec::new();
        }
        match self.color_stretch {
            ColorScaleStretch::LinearMinMax => {
                let stats = SummaryStats::from_slice(values);
                let span = (stats.max - stats.min).max(1e-12);
                values
                    .iter()
                    .map(|&v| {
                        if v.is_finite() && v > -0.999999 {
                            ((v - stats.min) / span).clamp(0.0, 1.0)
                        } else {
                            0.0
                        }
                    })
                    .collect()
            }
            ColorScaleStretch::QuantileRank => {
                let mut valid: Vec<(usize, f64)> = values
                    .iter()
                    .copied()
                    .enumerate()
                    .filter(|(_, v)| v.is_finite() && *v > -0.999999)
                    .collect();
                valid.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
                let mut out = vec![0.0; values.len()];
                let denom = (valid.len().saturating_sub(1)).max(1) as f64;
                for (rank, (idx, _)) in valid.into_iter().enumerate() {
                    out[idx] = (rank as f64 / denom).clamp(0.0, 1.0);
                }
                out
            }
            ColorScaleStretch::Logarithmic => {
                let stats = SummaryStats::from_slice(values);
                let offset = (-stats.min).max(0.0) + 1.0;
                let log_vals: Vec<f64> = values
                    .iter()
                    .map(|&v| {
                        if v.is_finite() && v > -0.999999 {
                            (v + offset).max(1e-9).log10()
                        } else {
                            0.0
                        }
                    })
                    .collect();
                let lstats = SummaryStats::from_slice(&log_vals);
                let span = (lstats.max - lstats.min).max(1e-12);
                log_vals
                    .into_iter()
                    .map(|lv| ((lv - lstats.min) / span).clamp(0.0, 1.0))
                    .collect()
            }
        }
    }

    fn world_bounds(&self) -> (Point2D, Point2D) {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        let mut ingest = |p: Point2D| {
            if p.x.is_finite() && p.y.is_finite() {
                min_x = min_x.min(p.x);
                min_y = min_y.min(p.y);
                max_x = max_x.max(p.x);
                max_y = max_y.max(p.y);
            }
        };

        match self.project_domain {
            AnalysisDomain::SpatialVisibility => {
                for poly in &self.obstacles {
                    for &p in &poly.coords {
                        ingest(p);
                    }
                }
                if let Some(vga) = &self.vga_results {
                    for pt in &vga.points {
                        ingest(Point2D::new(pt.x, pt.y));
                    }
                }
                if self.obstacles.is_empty() {
                    for poly in &self.working_polylines {
                        for &p in &poly.coords {
                            ingest(p);
                        }
                    }
                }
                for ring in &self.boundary_rings {
                    if ring.len() >= 3 {
                        for &p in ring {
                            ingest(p);
                        }
                    }
                }
            }
            AnalysisDomain::StreetNetwork => {
                for s in &self.segments {
                    ingest(s.p0);
                    ingest(s.p1);
                }
                for poly in &self.working_polylines {
                    for &p in &poly.coords {
                        ingest(p);
                    }
                }
            }
        }
        if !min_x.is_finite() {
            return (Point2D::new(0.0, 0.0), Point2D::new(100.0, 100.0));
        }
        (Point2D::new(min_x, min_y), Point2D::new(max_x, max_y))
    }

    fn zoom_to_fit(&mut self, rect: Rect) {
        let (bmin, bmax) = self.world_bounds();
        let w = (bmax.x - bmin.x).max(0.5);
        let h = (bmax.y - bmin.y).max(0.5);
        self.cam_center = Point2D::new(0.5 * (bmin.x + bmax.x), 0.5 * (bmin.y + bmax.y));
        let avail_w = (rect.width() as f64 - 80.0).max(100.0);
        let avail_h = (rect.height() as f64 - 100.0).max(100.0);
        self.cam_zoom = (avail_w / w).min(avail_h / h).clamp(0.05, 2500.0);
        self.needs_zoom_to_fit = false;
    }

    #[inline]
    fn world_to_screen(&self, p: Point2D, rect: Rect) -> Pos2 {
        let cx = rect.center().x as f64;
        let cy = rect.center().y as f64;
        let sx = cx + (p.x - self.cam_center.x) * self.cam_zoom;
        let sy = cy - (p.y - self.cam_center.y) * self.cam_zoom;
        Pos2::new(sx as f32, sy as f32)
    }

    #[inline]
    fn screen_to_world(&self, pos: Pos2, rect: Rect) -> Point2D {
        let cx = rect.center().x as f64;
        let cy = rect.center().y as f64;
        let wx = self.cam_center.x + ((pos.x as f64) - cx) / self.cam_zoom;
        let wy = self.cam_center.y - ((pos.y as f64) - cy) / self.cam_zoom;
        Point2D::new(wx, wy)
    }

    fn dialog_open_vector_file(&mut self) {
        self.dialog_open_vector_file_with_domain(self.project_domain);
    }

    fn dialog_open_vector_file_with_domain(&mut self, domain: AnalysisDomain) {
        let title = match domain {
            AnalysisDomain::StreetNetwork => {
                "Open Street Network Vector File (GeoJSON / DXF / MapInfo MIF / OSM XML)"
            }
            AnalysisDomain::SpatialVisibility => {
                "Open Architectural Floorplan / Spatial Boundary File (DXF / MIF / GeoJSON)"
            }
        };
        let Some(path) = rfd::FileDialog::new()
            .set_title(title)
            .add_filter("Spatial Vector Files", &["geojson", "json", "dxf", "mif", "osm", "xml", "csv"])
            .pick_file()
        else {
            return;
        };
        match load_polylines_auto(&path) {
            Ok(lines) => {
                let name = path
                    .file_name()
                    .and_then(|s| s.to_str())
                    .unwrap_or("Imported Map")
                    .to_string();
                let sub = format!("Imported from {}", path.display());
                self.load_custom_polylines(name, sub, lines, domain);
            }
            Err(e) => {
                self.log_audit(format!("[ERROR] Failed to open {}: {}", path.display(), e));
            }
        }
    }

    fn dialog_export_active_map(&mut self, ext: &str) {
        let Some(path): Option<PathBuf> = rfd::FileDialog::new()
            .set_title("Export Active Space Syntax Layer")
            .set_file_name(format!("sse_export.{}", ext))
            .save_file()
        else {
            return;
        };

        let res = match self.active_map_mode {
            ActiveMapMode::VgaPointMap => {
                if let Some(vga) = &self.vga_results {
                    export_vga_csv(&path, vga)
                } else {
                    Err("No VGA results to export.".to_string())
                }
            }
            ActiveMapMode::AxialMap => {
                if let Some(ax) = &self.axial_results {
                    match ext {
                        "csv" => export_segments_csv(&path, &ax.lines, &ax.attributes),
                        "dxf" => export_segments_dxf(
                            &path,
                            &ax.lines,
                            &ax.attributes,
                            Some(&self.active_attribute),
                        ),
                        "mif" => export_segments_mif_mid(&path, &ax.lines, &ax.attributes),
                        _ => export_segments_geojson(&path, &ax.lines, &ax.attributes),
                    }
                } else {
                    Err("No Axial results to export.".to_string())
                }
            }
            ActiveMapMode::NaturalRoadsStrokes => {
                if let Some(st) = &self.strokes_results {
                    let stroke_segs: Vec<SegmentRecord> = st
                        .strokes
                        .iter()
                        .map(|s| {
                            let p0 = s.coords.first().copied().unwrap_or(Point2D::new(0.0, 0.0));
                            let p1 = s.coords.last().copied().unwrap_or(p0);
                            let b0 = get_bearing(p0, p1);
                            let b1 = get_bearing(p1, p0);
                            SegmentRecord {
                                id: s.stroke_id,
                                p0,
                                p1,
                                b0,
                                b1,
                                length_m: s.length_m,
                                azimuth_deg: (b0.to_degrees() + 360.0) % 360.0,
                                level: 0.0,
                                highway: "stroke".to_string(),
                                name: format!("stroke_{}", s.stroke_id),
                            }
                        })
                        .collect();
                    match ext {
                        "csv" => export_segments_csv(&path, &stroke_segs, &st.attributes),
                        "dxf" => export_segments_dxf(
                            &path,
                            &stroke_segs,
                            &st.attributes,
                            Some(&self.active_attribute),
                        ),
                        "mif" => export_segments_mif_mid(&path, &stroke_segs, &st.attributes),
                        _ => export_segments_geojson(&path, &stroke_segs, &st.attributes),
                    }
                } else {
                    Err("No Natural Roads results to export.".to_string())
                }
            }
            _ => match ext {
                "csv" => export_segments_csv(&path, &self.segments, &self.asa_results.attributes),
                "dxf" => export_segments_dxf(
                    &path,
                    &self.segments,
                    &self.asa_results.attributes,
                    Some(&self.active_attribute),
                ),
                "mif" => export_segments_mif_mid(&path, &self.segments, &self.asa_results.attributes),
                _ => export_segments_geojson(&path, &self.segments, &self.asa_results.attributes),
            },
        };

        match res {
            Ok(()) => self.log_audit(format!("[EXPORT] Saved active map to {}", path.display())),
            Err(e) => self.log_audit(format!("[EXPORT ERROR] {}", e)),
        }
    }

    fn dialog_render_and_save_png(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .set_title("Save Publication Map Image (PNG)")
            .add_filter("PNG Image", &["png"])
            .set_file_name("sse_map_export.png")
            .save_file()
        else {
            return;
        };
        let _ = self.render_and_save_png_to_path(&path);
    }

    fn render_and_save_png_to_path(&mut self, path: &std::path::Path) -> Result<(), String> {
        let w = self.png_width.clamp(400, 7680);
        let h = self.png_height.clamp(400, 4320);
        let bg_px = if self.png_transparent_bg {
            image::Rgba([0, 0, 0, 0])
        } else {
            image::Rgba([self.png_bg_rgb[0], self.png_bg_rgb[1], self.png_bg_rgb[2], 255])
        };
        let mut img = image::RgbaImage::from_pixel(w, h, bg_px);

        let (bmin, bmax) = self.world_bounds();
        let world_w = (bmax.x - bmin.x).max(0.5);
        let world_h = (bmax.y - bmin.y).max(0.5);
        let wc = Point2D::new(0.5 * (bmin.x + bmax.x), 0.5 * (bmin.y + bmax.y));

        let margin_frac = (self.png_margin_pct as f64 / 100.0).clamp(0.02, 0.25);
        let avail_w = (w as f64) * (1.0 - 2.0 * margin_frac);
        let avail_h = (h as f64) * (1.0 - 2.0 * margin_frac);
        let px_per_m = (avail_w / world_w).min(avail_h / world_h).max(1e-4);

        let cx = 0.5 * (w as f64);
        let cy = 0.5 * (h as f64);
        let to_px = |p: Point2D| -> (f64, f64) {
            (cx + (p.x - wc.x) * px_per_m, cy - (p.y - wc.y) * px_per_m)
        };

        let vals = self.active_attribute_values();
        let norm_t = self.compute_normalized_t_values(&vals);
        let lw = (self.png_line_width as f64).clamp(0.5, 20.0);

        // 1. Optional Isovist polygons & rays
        if self.png_draw_isovists && self.project_domain == AnalysisDomain::SpatialVisibility {
            for iso in &self.cast_isovists {
                if iso.polygon.len() >= 3 {
                    let orig_px = to_px(iso.origin);
                    for i in 0..iso.polygon.len() {
                        let a = to_px(iso.polygon[i]);
                        let b = to_px(iso.polygon[(i + 1) % iso.polygon.len()]);
                        fill_triangle_rgba(&mut img, orig_px, a, b, [45, 220, 255, 62]);
                    }
                    if self.show_isovist_rays {
                        for &pt in &iso.polygon {
                            let p_px = to_px(pt);
                            draw_aa_capsule_segment(
                                &mut img,
                                orig_px.0,
                                orig_px.1,
                                p_px.0,
                                p_px.1,
                                (lw * 0.45).max(0.8),
                                [80, 235, 255, 95],
                            );
                        }
                    }
                    for i in 0..iso.polygon.len() {
                        let a = to_px(iso.polygon[i]);
                        let b = to_px(iso.polygon[(i + 1) % iso.polygon.len()]);
                        draw_aa_capsule_segment(
                            &mut img,
                            a.0,
                            a.1,
                            b.0,
                            b.1,
                            (lw * 0.75).max(1.4),
                            [55, 235, 255, 235],
                        );
                    }
                }
            }
        }

        // 2. Active Map Geometry
        match self.active_map_mode {
            ActiveMapMode::SegmentMapAsa => {
                for (idx, seg) in self.segments.iter().enumerate() {
                    let (x0, y0) = to_px(seg.p0);
                    let (x1, y1) = to_px(seg.p1);
                    let t = norm_t.get(idx).copied().unwrap_or(0.0);
                    let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
                    draw_aa_capsule_segment(&mut img, x0, y0, x1, y1, lw, [r, g, b, 255]);
                }
            }
            ActiveMapMode::AxialMap => {
                if let Some(ax) = &self.axial_results {
                    for (idx, line) in ax.lines.iter().enumerate() {
                        let (x0, y0) = to_px(line.p0);
                        let (x1, y1) = to_px(line.p1);
                        let t = norm_t.get(idx).copied().unwrap_or(0.0);
                        let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
                        draw_aa_capsule_segment(&mut img, x0, y0, x1, y1, lw + 0.4, [r, g, b, 255]);
                    }
                }
            }
            ActiveMapMode::NaturalRoadsStrokes => {
                if let Some(nr) = &self.strokes_results {
                    for (idx, st) in nr.strokes.iter().enumerate() {
                        let t = norm_t.get(idx).copied().unwrap_or(0.0);
                        let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
                        for win in st.coords.windows(2) {
                            let (x0, y0) = to_px(win[0]);
                            let (x1, y1) = to_px(win[1]);
                            draw_aa_capsule_segment(
                                &mut img,
                                x0,
                                y0,
                                x1,
                                y1,
                                lw + 0.5,
                                [r, g, b, 255],
                            );
                        }
                    }
                }
            }
            ActiveMapMode::VgaPointMap => {
                if self.show_vga_cells {
                    if let Some(vga) = &self.vga_results {
                        let half = (vga.cell_size_m * px_per_m * 0.48).max(1.5);
                        for (idx, pt) in vga.points.iter().enumerate() {
                            let (px, py) = to_px(Point2D::new(pt.x, pt.y));
                            let t = norm_t.get(idx).copied().unwrap_or(0.0);
                            let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
                            fill_rect_rgba(
                                &mut img,
                                (px - half).floor() as i32,
                                (py - half).floor() as i32,
                                (px + half).ceil() as i32,
                                (py + half).ceil() as i32,
                                [r, g, b, 255],
                            );
                        }
                    }
                }
            }
        }

        // 3. Obstacle / Wall Polylines (Only in SpatialVisibility domain)
        if self.project_domain == AnalysisDomain::SpatialVisibility
            && self.show_obstacles
            && self.png_wall_width > 0.0
        {
            let ww = (self.png_wall_width as f64).clamp(0.5, 15.0);
            let wc_rgba = [
                self.png_wall_rgb[0],
                self.png_wall_rgb[1],
                self.png_wall_rgb[2],
                235,
            ];
            for poly in &self.obstacles {
                for win in poly.coords.windows(2) {
                    let (x0, y0) = to_px(win[0]);
                    let (x1, y1) = to_px(win[1]);
                    draw_aa_capsule_segment(&mut img, x0, y0, x1, y1, ww, wc_rgba);
                }
            }
        }

        // 4. Optional Gibson Ecological Visual Agent Trails (Only in SpatialVisibility domain)
        if self.png_draw_agents && self.project_domain == AnalysisDomain::SpatialVisibility {
            if let Some(st) = &self.agent_state {
                if self.show_agent_history_trails {
                    for tr in st.finished_trails.iter().rev().take(80) {
                        for win in tr.windows(2) {
                            let (x0, y0) = to_px(win[0]);
                            let (x1, y1) = to_px(win[1]);
                            draw_aa_capsule_segment(
                                &mut img,
                                x0,
                                y0,
                                x1,
                                y1,
                                (lw * 0.55).max(1.0),
                                [255, 200, 45, 110],
                            );
                        }
                    }
                }
                if self.show_agent_active_trails {
                    for ag in &st.active_agents {
                        for win in ag.trail.windows(2) {
                            let (x0, y0) = to_px(win[0]);
                            let (x1, y1) = to_px(win[1]);
                            draw_aa_capsule_segment(
                                &mut img,
                                x0,
                                y0,
                                x1,
                                y1,
                                (lw * 0.7).max(1.4),
                                [255, 255, 255, 205],
                            );
                        }
                    }
                }
            }
        }

        // 5. Optional Spectral Color Scale Legend Bar (Bottom-Left)
        if self.png_draw_legend {
            let bar_w = ((w as f64) * 0.22).clamp(220.0, 520.0) as i32;
            let bar_h = ((h as f64) * 0.018).clamp(14.0, 32.0) as i32;
            let pad = 24_i32;
            let x_start = pad;
            let y_start = (h as i32) - pad - bar_h;
            fill_rect_rgba(
                &mut img,
                x_start - 6,
                y_start - 6,
                x_start + bar_w + 6,
                y_start + bar_h + 6,
                [15, 20, 28, 215],
            );
            for dx in 0..bar_w {
                let t = (dx as f64) / ((bar_w - 1).max(1) as f64);
                let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
                fill_rect_rgba(
                    &mut img,
                    x_start + dx,
                    y_start,
                    x_start + dx + 1,
                    y_start + bar_h,
                    [r, g, b, 255],
                );
            }
        }

        match img.save(path) {
            Ok(()) => {
                self.log_audit(format!(
                    "[EXPORT PNG] Saved publication map ({}x{} px) to {}",
                    w,
                    h,
                    path.display()
                ));
                Ok(())
            }
            Err(e) => {
                let err_msg = format!("Failed to save PNG: {}", e);
                self.log_audit(format!("[EXPORT PNG ERROR] {}", err_msg));
                Err(err_msg)
            }
        }
    }
}

#[inline]
fn blend_rgba_pixel(img: &mut image::RgbaImage, x: i32, y: i32, src: [u8; 4]) {
    if x < 0 || y < 0 || x >= (img.width() as i32) || y >= (img.height() as i32) {
        return;
    }
    let sa = (src[3] as f32) / 255.0;
    if sa <= 0.0 {
        return;
    }
    let dst = img.get_pixel_mut(x as u32, y as u32);
    let da = (dst.0[3] as f32) / 255.0;
    let out_a = sa + da * (1.0 - sa);
    if out_a <= 1e-6 {
        *dst = image::Rgba([0, 0, 0, 0]);
        return;
    }
    let r = ((src[0] as f32) * sa + (dst.0[0] as f32) * da * (1.0 - sa)) / out_a;
    let g = ((src[1] as f32) * sa + (dst.0[1] as f32) * da * (1.0 - sa)) / out_a;
    let b = ((src[2] as f32) * sa + (dst.0[2] as f32) * da * (1.0 - sa)) / out_a;
    *dst = image::Rgba([
        r.round().clamp(0.0, 255.0) as u8,
        g.round().clamp(0.0, 255.0) as u8,
        b.round().clamp(0.0, 255.0) as u8,
        (out_a * 255.0).round().clamp(0.0, 255.0) as u8,
    ]);
}

fn fill_rect_rgba(img: &mut image::RgbaImage, x0: i32, y0: i32, x1: i32, y1: i32, rgba: [u8; 4]) {
    let w = img.width() as i32;
    let h = img.height() as i32;
    let min_x = x0.min(x1).clamp(0, w);
    let max_x = x0.max(x1).clamp(0, w);
    let min_y = y0.min(y1).clamp(0, h);
    let max_y = y0.max(y1).clamp(0, h);
    for py in min_y..max_y {
        for px in min_x..max_x {
            blend_rgba_pixel(img, px, py, rgba);
        }
    }
}

fn fill_triangle_rgba(
    img: &mut image::RgbaImage,
    p0: (f64, f64),
    p1: (f64, f64),
    p2: (f64, f64),
    rgba: [u8; 4],
) {
    let w = img.width() as i32;
    let h = img.height() as i32;
    let min_x = p0.0.min(p1.0).min(p2.0).floor() as i32;
    let max_x = p0.0.max(p1.0).max(p2.0).ceil() as i32;
    let min_y = p0.1.min(p1.1).min(p2.1).floor() as i32;
    let max_y = p0.1.max(p1.1).max(p2.1).ceil() as i32;

    let min_x = min_x.clamp(0, w - 1);
    let max_x = max_x.clamp(0, w - 1);
    let min_y = min_y.clamp(0, h - 1);
    let max_y = max_y.clamp(0, h - 1);

    let edge = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| -> f64 {
        (c.0 - a.0) * (b.1 - a.1) - (c.1 - a.1) * (b.0 - a.0)
    };
    let area = edge(p0, p1, p2);
    if area.abs() <= 1e-8 {
        return;
    }

    for py in min_y..=max_y {
        let yf = (py as f64) + 0.5;
        for px in min_x..=max_x {
            let xf = (px as f64) + 0.5;
            let w0 = edge(p1, p2, (xf, yf));
            let w1 = edge(p2, p0, (xf, yf));
            let w2 = edge(p0, p1, (xf, yf));
            let inside = if area > 0.0 {
                w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0
            } else {
                w0 <= 0.0 && w1 <= 0.0 && w2 <= 0.0
            };
            if inside {
                blend_rgba_pixel(img, px, py, rgba);
            }
        }
    }
}

fn draw_aa_capsule_segment(
    img: &mut image::RgbaImage,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    width_px: f64,
    rgba: [u8; 4],
) {
    let w = img.width() as i32;
    let h = img.height() as i32;
    let r = (0.5 * width_px).max(0.35);
    let pad = r + 1.5;

    let min_y = ((y0.min(y1) - pad).floor() as i32).clamp(0, h - 1);
    let max_y = ((y0.max(y1) + pad).ceil() as i32).clamp(0, h - 1);
    let dx = x1 - x0;
    let dy = y1 - y0;
    let l2 = dx * dx + dy * dy;

    for py in min_y..=max_y {
        let yf = (py as f64) + 0.5;
        let (row_min_x, row_max_x) = if dy.abs() > 1e-6 {
            let t0 = (yf - pad - y0) / dy;
            let t1 = (yf + pad - y0) / dy;
            let ta = t0.min(t1).clamp(0.0, 1.0);
            let tb = t0.max(t1).clamp(0.0, 1.0);
            let xa = x0 + ta * dx;
            let xb = x0 + tb * dx;
            (
                ((xa.min(xb) - pad).floor() as i32).clamp(0, w - 1),
                ((xa.max(xb) + pad).ceil() as i32).clamp(0, w - 1),
            )
        } else {
            (
                ((x0.min(x1) - pad).floor() as i32).clamp(0, w - 1),
                ((x0.max(x1) + pad).ceil() as i32).clamp(0, w - 1),
            )
        };

        for px in row_min_x..=row_max_x {
            let xf = (px as f64) + 0.5;
            let d = if l2 <= 1e-12 {
                (xf - x0).hypot(yf - y0)
            } else {
                let t = (((xf - x0) * dx + (yf - y0) * dy) / l2).clamp(0.0, 1.0);
                (xf - (x0 + t * dx)).hypot(yf - (y0 + t * dy))
            };
            if d <= r + 0.75 {
                let cov = (r + 0.5 - d).clamp(0.0, 1.0);
                let a = ((rgba[3] as f64) * cov).round() as u8;
                if a > 0 {
                    blend_rgba_pixel(img, px, py, [rgba[0], rgba[1], rgba[2], a]);
                }
            }
        }
    }
}

impl SseWorkstationApp {
    fn render_top_menu_and_ribbon(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        let accent = self.accent_text_color();
        let heading = self.heading_text_color();

        egui::menu::bar(ui, |ui| {
            ui.label(
                RichText::new("SSE WORKSTATION v1.1.0")
                    .strong()
                    .color(accent),
            );
            ui.separator();

            ui.menu_button("File", |ui| {
                if ui
                    .button("New Blank Project Tab (Ctrl+N)")
                    .clicked()
                {
                    ui.close_menu();
                    self.create_new_blank_project_tab();
                }
                if ui
                    .button("Open Street Network File (.mif / .dxf / .geojson / .osm / .csv)... (Ctrl+O)")
                    .clicked()
                {
                    ui.close_menu();
                    self.dialog_open_vector_file_with_domain(AnalysisDomain::StreetNetwork);
                }
                if ui
                    .button("Open Architectural Floorplan / Spatial Boundary (.dxf / .mif / .geojson)...")
                    .clicked()
                {
                    ui.close_menu();
                    self.dialog_open_vector_file_with_domain(AnalysisDomain::SpatialVisibility);
                }
                if ui
                    .button("Close Active Project Tab (Ctrl+W)")
                    .clicked()
                {
                    ui.close_menu();
                    let cur = self.active_project_idx;
                    self.close_project_tab(cur);
                }
                ui.separator();
                ui.menu_button("Load Street Network Benchmark (ASA / Axial / Natural Roads)", |ui| {
                    if ui
                        .button("UCL Barnsbury Street Network — Islington, London (barnsbury_segment_lines.mif)")
                        .clicked()
                    {
                        ui.close_menu();
                        self.load_preset(BenchmarkPreset::UrbanSuperblockGrid);
                    }
                    if ui
                        .button("UCL Barnsbury Extended Axial Network — North London (barnsbury_extended1.dxf)")
                        .clicked()
                    {
                        ui.close_menu();
                        self.load_preset(BenchmarkPreset::AxialStarDiamond);
                    }
                    if ui
                        .button("Closed-Loop Ring-Road & Cul-de-Sac Network (osmnx#1410 Sinuosity Guard)")
                        .clicked()
                    {
                        ui.close_menu();
                        self.load_preset(BenchmarkPreset::RingRoadCulDeSacLoop);
                    }
                });
                ui.menu_button("Load Architectural Floorplan Benchmark (VGA / Isovists / Agents)", |ui| {
                    if ui
                        .button("UCL National Gallery Floorplan — Sainsbury Wing VGA & Agents (gallery.dxf)")
                        .clicked()
                    {
                        ui.close_menu();
                        self.load_preset(BenchmarkPreset::ArchitecturalGalleryVga);
                    }
                    if ui
                        .button("UCL Multi-Room Architectural Floorplan — VGA & Isovists (rooms.dxf)")
                        .clicked()
                    {
                        ui.close_menu();
                        self.load_preset(BenchmarkPreset::UclRoomsFloorplan);
                    }
                });
                if ui.button("Download OSM Overpass Street Network...").clicked() {
                    ui.close_menu();
                    self.show_overpass_modal = true;
                }
                ui.separator();
                ui.menu_button("Export Active Map", |ui| {
                    if ui.button("Export Map Image (PNG)...").clicked() {
                        ui.close_menu();
                        self.show_png_export_modal = true;
                    }
                    ui.separator();
                    if ui.button("Export as GeoJSON (.geojson)...").clicked() {
                        ui.close_menu();
                        self.dialog_export_active_map("geojson");
                    }
                    if ui.button("Export as Attribute Table (.csv)...").clicked() {
                        ui.close_menu();
                        self.dialog_export_active_map("csv");
                    }
                    if ui.button("Export as AutoCAD DXF AC1018 (.dxf)...").clicked() {
                        ui.close_menu();
                        self.dialog_export_active_map("dxf");
                    }
                    if ui
                        .button("Export as MapInfo Interchange (.mif / .mid)...")
                        .clicked()
                    {
                        ui.close_menu();
                        self.dialog_export_active_map("mif");
                    }
                });
            });

            ui.menu_button("Map", |ui| {
                ui.label(RichText::new("Street Network Maps:").strong().color(heading));
                if ui.button("Convert to Segment Map [ASA]").clicked() {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::StreetNetwork);
                    self.rebuild_segment_graph_and_solve_asa();
                    self.active_map_mode = ActiveMapMode::SegmentMapAsa;
                }
                if ui.button("Convert to Topological Axial Map [AXIAL]").clicked() {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::StreetNetwork);
                    self.run_axial_solver();
                }
                if ui
                    .button("Convert to Gestalt Natural Roads [STROKES]")
                    .clicked()
                {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::StreetNetwork);
                    self.run_natural_roads_solver();
                }
                ui.separator();
                ui.label(RichText::new("Spatial Visibility & Building Maps:").strong().color(heading));
                if ui.button("Create / Fill Walkable VGA Grid").clicked() {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::SpatialVisibility);
                    self.create_or_fill_vga_grid(None);
                }
                if ui.button("Compute VGA Visibility Graph [VGA]").clicked() {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::SpatialVisibility);
                    self.run_vga_solver();
                }
                if ui.button("Clear Agent Pins, Isovists & Trails").clicked() {
                    ui.close_menu();
                    self.cast_isovists.clear();
                    self.agent_config.release_cells.clear();
                    self.init_or_reset_agents();
                }
            });

            ui.menu_button("Tools", |ui| {
                ui.label(RichText::new("Urban Street Network Tools:").strong().color(heading));
                if ui
                    .button("Network Cleaner -> Configure & Run 5-Step P90 Watertight Cleaner...")
                    .clicked()
                {
                    ui.close_menu();
                    self.show_cleaner_modal = true;
                }
                if ui
                    .button("Network Cleaner -> QuickSnap Endpoints (3.0m) + Planarize")
                    .clicked()
                {
                    ui.close_menu();
                    self.working_polylines = snap_endpoints_watertight(&self.working_polylines, 3.0);
                    self.rebuild_segment_graph_and_solve_asa();
                }
                if ui
                    .button("Network Cleaner -> Synthesize Dual-Carriageway Cross Rungs")
                    .clicked()
                {
                    ui.close_menu();
                    let rungs =
                        synthesize_dual_carriageway_rungs(&self.working_polylines, 4.0, 22.0, 60.0);
                    let added = rungs.len();
                    self.working_polylines.extend(rungs);
                    self.rebuild_segment_graph_and_solve_asa();
                    self.log_audit(format!(
                        "[CLEANER] Synthesized {} dual-carriageway transverse rungs.",
                        added
                    ));
                }
                if ui
                    .button("Segment -> Run Angular Segment Analysis (ASA)")
                    .clicked()
                {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::StreetNetwork);
                    self.rebuild_segment_graph_and_solve_asa();
                    self.active_map_mode = ActiveMapMode::SegmentMapAsa;
                }
                if ui
                    .button("Segment -> Compute Step Depth from Selected Segment(s)")
                    .clicked()
                {
                    ui.close_menu();
                    self.run_step_depth_from_selected();
                }
                if ui
                    .button("Axial -> Run 16-Attribute Topological Axial Analysis")
                    .clicked()
                {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::StreetNetwork);
                    self.run_axial_solver();
                }
                if ui
                    .button("Natural Roads -> Run Gestalt Stroke Concatenation (45 deg)")
                    .clicked()
                {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::StreetNetwork);
                    self.run_natural_roads_solver();
                }
                ui.separator();
                ui.label(RichText::new("Spatial Visibility & Agent Tools:").strong().color(heading));
                if ui
                    .button("Visibility (VGA) -> Create / Fill Walkable Grid")
                    .clicked()
                {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::SpatialVisibility);
                    self.create_or_fill_vga_grid(None);
                }
                if ui
                    .button("Visibility (VGA) -> Compute Visibility Graph Analysis")
                    .clicked()
                {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::SpatialVisibility);
                    self.run_vga_solver();
                }
                if ui
                    .button("Agents -> Place / Release Agents at Clicked Cell (Click Map Mode)")
                    .clicked()
                {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::SpatialVisibility);
                    self.click_mode = CanvasClickMode::PlaceAgentsAtClick;
                    self.active_tab = WorkspaceTab::SpatialViewport;
                }
                if ui
                    .button("Visibility (VGA) -> Cast Analytical 2D Isovist (Click Map Mode)")
                    .clicked()
                {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::SpatialVisibility);
                    self.click_mode = CanvasClickMode::CastAnalyticalIsovist;
                    self.active_tab = WorkspaceTab::SpatialViewport;
                }
                if ui
                    .button("Agents -> Run Agent Analysis Batch (5000 Timesteps)")
                    .clicked()
                {
                    ui.close_menu();
                    self.switch_project_domain(AnalysisDomain::SpatialVisibility);
                    self.run_agents_batch_solver();
                }
            });

            ui.menu_button("View", |ui| {
                if ui.button("Zoom to Fit [F]").clicked() {
                    ui.close_menu();
                    self.needs_zoom_to_fit = true;
                }
                ui.separator();
                ui.label(RichText::new("Color Scale Palette:").strong());
                for pal in [
                    ColorPalette::DepthmapClassic,
                    ColorPalette::Viridis,
                    ColorPalette::MonochromeAcademic,
                ] {
                    if ui
                        .radio(self.color_palette == pal, pal.as_str())
                        .clicked()
                    {
                        self.color_palette = pal;
                    }
                }
                ui.separator();
                ui.label(RichText::new("Color Stretch Mode:").strong());
                for st in [
                    ColorScaleStretch::LinearMinMax,
                    ColorScaleStretch::QuantileRank,
                    ColorScaleStretch::Logarithmic,
                ] {
                    if ui.radio(self.color_stretch == st, st.as_str()).clicked() {
                        self.color_stretch = st;
                    }
                }
                ui.separator();
                match self.project_domain {
                    AnalysisDomain::StreetNetwork => {
                        ui.label(RichText::new("Street Network Overlays:").strong());
                        ui.checkbox(
                            &mut self.show_network_colors,
                            "Show Network Attribute Colors (Heatmap)",
                        );
                        ui.checkbox(&mut self.show_segment_ids, "Show Segment / Line ID Labels");
                        ui.checkbox(&mut self.show_endpoints, "Show Topological Intersection Nodes");
                        ui.checkbox(
                            &mut self.show_cleaner_diff,
                            "Show 5-Step Cleaner Diff (Raw vs Cleaned)",
                        );
                        ui.checkbox(&mut self.show_grade_unlinks, "Show Grade-Separation Unlinks");
                    }
                    AnalysisDomain::SpatialVisibility => {
                        ui.label(RichText::new("Spatial Visibility & Agent Lines:").strong());
                        ui.checkbox(&mut self.show_vga_cells, "Show VGA Grid Cells (Heatmap)");
                        ui.checkbox(&mut self.show_obstacles, "Show Floorplan Walls / Plan Lines");
                        ui.checkbox(
                            &mut self.show_vga_los_edges,
                            "Show VGA Visibility Graph Lines (LOS)",
                        );
                        ui.checkbox(
                            &mut self.show_agent_bodies,
                            "Show Active Agent Bodies & Headings",
                        );
                        ui.checkbox(
                            &mut self.show_agent_active_trails,
                            "Show Active Agent Trail Lines",
                        );
                        ui.checkbox(
                            &mut self.show_agent_history_trails,
                            "Show Historical Agent Trail Lines",
                        );
                        ui.checkbox(&mut self.show_isovists, "Show Cast 2D Isovist Polygon & Border");
                        ui.checkbox(&mut self.show_isovist_rays, "Show Cast Isovist Radial Ray Lines");
                        ui.checkbox(
                            &mut self.show_isovist_drift,
                            "Show Cast Isovist Drift Vector Line",
                        );
                    }
                }
                ui.separator();
                if ui
                    .button(if self.dark_theme {
                        "Switch to Academic Light Paper Theme"
                    } else {
                        "Switch to Academic Dark Slate Theme"
                    })
                    .clicked()
                {
                    self.dark_theme = !self.dark_theme;
                    self.png_bg_rgb = if self.dark_theme {
                        [14, 18, 24]
                    } else {
                        [247, 248, 250]
                    };
                    self.png_wall_rgb = if self.dark_theme {
                        [215, 224, 236]
                    } else {
                        [30, 38, 48]
                    };
                    self.apply_academic_theme(ctx);
                    ui.close_menu();
                }
            });

            ui.menu_button("Help", |ui| {
                if ui
                    .button("Mathematical Reference, Literature Links & UCL depthmapX v0.8.0 Parity...")
                    .clicked()
                {
                    ui.close_menu();
                    self.run_self_verification_suite();
                    self.show_math_verification_modal = true;
                }
            });

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button(self.btn_text("[Verify Math Suite]")).clicked() {
                    self.run_self_verification_suite();
                    self.show_math_verification_modal = true;
                }
                if ui.button(self.btn_text("[EXPORT PNG]")).clicked() {
                    self.show_png_export_modal = true;
                }
                if ui.button(self.btn_text("[Zoom Fit]")).clicked() {
                    self.needs_zoom_to_fit = true;
                }
                ui.separator();
                for (preset, label) in [
                    (BenchmarkPreset::UclRoomsFloorplan, "UCL Rooms VGA"),
                    (BenchmarkPreset::ArchitecturalGalleryVga, "UCL Gallery VGA"),
                    (BenchmarkPreset::RingRoadCulDeSacLoop, "Loop Guard"),
                    (BenchmarkPreset::AxialStarDiamond, "Barnsbury Axial"),
                    (BenchmarkPreset::UrbanSuperblockGrid, "Barnsbury MIF"),
                ] {
                    let active = self.active_preset == Some(preset);
                    if ui
                        .selectable_label(active, self.pill_text(label, active))
                        .clicked()
                    {
                        self.load_preset(preset);
                    }
                }
            });
        });

        self.bar_horizontal_divider(ui);

        // Multi-Project Document Tab Bar (Vertically Centered 30px Bar)
        self.save_active_to_project_slot();
        let tab_names: Vec<String> = self.projects.iter().map(|p| p.tab_title.clone()).collect();
        let mut switch_target: Option<usize> = None;
        let mut close_target: Option<usize> = None;
        let mut create_new_tab = false;

        ui.allocate_ui_with_layout(
            egui::vec2(ui.available_width(), 30.0),
            egui::Layout::left_to_right(egui::Align::Center),
            |ui| {
                ui.spacing_mut().interact_size.y = 24.0;
                ui.spacing_mut().button_padding = egui::vec2(8.0, 4.0);
                ui.label(RichText::new("PROJECTS:").strong().color(heading));
                for (idx, t_name) in tab_names.iter().enumerate() {
                    let is_active = idx == self.active_project_idx;
                    if ui
                        .selectable_label(
                            is_active,
                            self.pill_text(format!(" {} ", t_name), is_active),
                        )
                        .clicked()
                    {
                        switch_target = Some(idx);
                    }
                    if ui
                        .button(self.btn_text("×"))
                        .on_hover_text("Close project tab (Ctrl+W)")
                        .clicked()
                    {
                        close_target = Some(idx);
                    }
                    ui.separator();
                }
                if ui
                    .button(self.btn_text("+ New Project (Ctrl+N)"))
                    .clicked()
                {
                    create_new_tab = true;
                }
            },
        );

        if let Some(idx) = switch_target {
            self.switch_to_project_tab(idx);
        }
        if let Some(idx) = close_target {
            self.close_project_tab(idx);
        }
        if create_new_tab {
            self.create_new_blank_project_tab();
        }

        self.bar_horizontal_divider(ui);

        // Domain-Separated Scientific Ribbon Bar (Vertically Centered 34px Bar)
        ui.allocate_ui_with_layout(
            egui::vec2(ui.available_width(), 34.0),
            egui::Layout::left_to_right(egui::Align::Center),
            |ui| {
                ui.spacing_mut().interact_size.y = 24.0;
                ui.spacing_mut().button_padding = egui::vec2(8.0, 4.0);
                ui.label(RichText::new("DOMAIN:").strong().color(heading));
                let is_net = self.project_domain == AnalysisDomain::StreetNetwork;
                if ui
                    .selectable_label(
                        is_net,
                        self.pill_text(" [URBAN STREET NETWORK] ", is_net),
                    )
                    .on_hover_text("Street Network Mode: Segment ASA, Axial Map, Gestalt Natural Roads & 5-Step Cleaner")
                    .clicked()
                {
                    self.switch_project_domain(AnalysisDomain::StreetNetwork);
                }
                let is_vga = self.project_domain == AnalysisDomain::SpatialVisibility;
                if ui
                    .selectable_label(
                        is_vga,
                        self.pill_text(" [SPATIAL VISIBILITY & AGENTS] ", is_vga),
                    )
                    .on_hover_text("Architectural / Spatial Visibility Mode: VGA PointMap, 2D Benedikt Isovists & Gibson Ecological Visual Agents")
                    .clicked()
                {
                    self.switch_project_domain(AnalysisDomain::SpatialVisibility);
                }

                ui.separator();
                ui.label(RichText::new("SOLVERS:").strong().color(heading));
                match self.project_domain {
                    AnalysisDomain::StreetNetwork => {
                        if ui.button(self.btn_text("[CLEANER 5-Step]")).clicked() {
                            self.run_cleaner_5step();
                        }
                        if ui.button(self.btn_text("[RUN ASA]")).clicked() {
                            self.rebuild_segment_graph_and_solve_asa();
                            self.active_map_mode = ActiveMapMode::SegmentMapAsa;
                        }
                        if ui.button(self.btn_text("[STEP DEPTH]")).clicked() {
                            self.run_step_depth_from_selected();
                        }
                        if ui.button(self.btn_text("[RUN AXIAL]")).clicked() {
                            self.run_axial_solver();
                        }
                        if ui.button(self.btn_text("[NATURAL ROADS]")).clicked() {
                            self.run_natural_roads_solver();
                        }
                    }
                    AnalysisDomain::SpatialVisibility => {
                        if ui
                            .button(self.btn_text("[CREATE GRID]"))
                            .on_hover_text(
                                "Create / flood-fill walkable grid cells at current Grid Cell Size (m)",
                            )
                            .clicked()
                        {
                            self.create_or_fill_vga_grid(None);
                        }
                        if let Some(job) = &self.vga_job {
                            let pct = job.progress_pct.load(Ordering::Relaxed);
                            if ui
                                .button(self.btn_text(format!("[CANCEL VGA ({}%)]", pct)))
                                .clicked()
                            {
                                self.cancel_vga_background_job();
                            }
                        } else if ui
                            .button(self.btn_text("[RUN VGA]"))
                            .on_hover_text(
                                "Compute Visibility Graph Analysis (non-blocking background thread)",
                            )
                            .clicked()
                        {
                            self.run_vga_solver();
                        }
                        let place_active = self.click_mode == CanvasClickMode::PlaceAgentsAtClick;
                        if ui
                            .selectable_label(
                                place_active,
                                self.pill_text("[PLACE AGENT (CLICK MAP)]", place_active),
                            )
                            .on_hover_text(
                                "Click any walkable grid cell on the map to place a stationary agent (UCL depthmapX CreateAgent)",
                            )
                            .clicked()
                        {
                            self.click_mode = if place_active {
                                CanvasClickMode::InspectAndSelectOrigin
                            } else {
                                CanvasClickMode::PlaceAgentsAtClick
                            };
                        }
                        let cast_active = self.click_mode == CanvasClickMode::CastAnalyticalIsovist;
                        if ui
                            .selectable_label(
                                cast_active,
                                self.pill_text("[CAST 2D ISOVIST]", cast_active),
                            )
                            .clicked()
                        {
                            self.click_mode = if cast_active {
                                CanvasClickMode::InspectAndSelectOrigin
                            } else {
                                CanvasClickMode::CastAnalyticalIsovist
                            };
                        }
                        if ui
                            .button(self.btn_text("[RUN AGENT ANALYSIS]"))
                            .on_hover_text(
                                "Run statistical Agent Analysis batch simulation (Turner & Penn 2002)",
                            )
                            .clicked()
                        {
                            self.run_agents_batch_solver();
                        }
                        let agent_btn_label = if self.is_playing_agents {
                            "[PAUSE AGENTS]"
                        } else {
                            "[PLAY AGENTS]"
                        };
                        if ui
                            .selectable_label(
                                self.is_playing_agents,
                                self.pill_text(agent_btn_label, self.is_playing_agents),
                            )
                            .clicked()
                        {
                            if self.vga_results.is_none() {
                                self.create_or_fill_vga_grid(None);
                            }
                            if self.agent_state.is_none() {
                                self.init_or_reset_agents();
                            }
                            self.show_agent_bodies = true;
                            self.is_playing_agents = !self.is_playing_agents;
                            if self.is_playing_agents {
                                self.last_agent_frame_instant = Some(Instant::now());
                            }
                            self.active_map_mode = ActiveMapMode::VgaPointMap;
                        }
                        if ui.button(self.btn_text("[CLEAR PINS/ISOVISTS]")).clicked() {
                            self.is_playing_agents = false;
                            self.cast_isovists.clear();
                            self.agent_config.release_cells.clear();
                            self.init_or_reset_agents();
                        }
                    }
                }

                ui.separator();
                ui.label(RichText::new("STRETCH:").strong().color(heading));
                egui::ComboBox::from_id_salt("ribbon_stretch_combo")
                    .selected_text(self.color_stretch.as_str())
                    .width(155.0)
                    .show_ui(ui, |ui| {
                        for st in [
                            ColorScaleStretch::LinearMinMax,
                            ColorScaleStretch::QuantileRank,
                            ColorScaleStretch::Logarithmic,
                        ] {
                            ui.selectable_value(&mut self.color_stretch, st, st.as_str());
                        }
                    });

                ui.add_space(4.0);
                if self.project_domain == AnalysisDomain::StreetNetwork {
                    ui.add(egui::Slider::new(&mut self.line_width, 1.0..=6.0).text("Line px"));
                } else {
                    ui.add(egui::Slider::new(&mut self.wall_line_width, 0.5..=6.0).text("Wall px"));
                }
            },
        );
    }

    fn render_left_dock(&mut self, ui: &mut egui::Ui) {
        let accent = self.accent_text_color();
        let heading = self.heading_text_color();
        ui.set_min_width(290.0);
        egui::ScrollArea::vertical()
            .id_salt("left_dock_scroll")
            .show(ui, |ui| {
                // 1. Collapsible Section: Map Hierarchy, Active Layer & Layer Display (Top)
                egui::CollapsingHeader::new(
                    RichText::new("MAP HIERARCHY & LAYER DISPLAY")
                        .strong()
                        .color(heading),
                )
                .id_salt("dock_sec_map_hierarchy")
                .default_open(true)
                .show(ui, |ui| {
                    ui.label(
                        RichText::new(&self.dataset_title)
                            .size(12.5)
                            .strong()
                            .color(accent),
                    );

                    match self.project_domain {
                        AnalysisDomain::StreetNetwork => {
                            for mode in [
                                ActiveMapMode::SegmentMapAsa,
                                ActiveMapMode::AxialMap,
                                ActiveMapMode::NaturalRoadsStrokes,
                            ] {
                                let is_sel = self.active_map_mode == mode;
                                if ui
                                    .selectable_label(is_sel, self.pill_text(mode.label(), is_sel))
                                    .clicked()
                                {
                                    match mode {
                                        ActiveMapMode::AxialMap if self.axial_results.is_none() => {
                                            self.run_axial_solver();
                                        }
                                        ActiveMapMode::NaturalRoadsStrokes
                                            if self.strokes_results.is_none() =>
                                        {
                                            self.run_natural_roads_solver();
                                        }
                                        _ => {
                                            self.active_map_mode = mode;
                                        }
                                    }
                                    let mut next_active = None;
                                    let mut next_sx = None;
                                    let mut next_sy = None;
                                    if let Some(attrs) = self.active_attributes_map() {
                                        if !attrs.contains_key(&self.active_attribute) {
                                            next_active = attrs.keys().next().cloned();
                                        }
                                        if !attrs.contains_key(&self.scatter_x_attr) {
                                            next_sx = Some(if attrs.contains_key("Connectivity") {
                                                "Connectivity".to_string()
                                            } else if attrs.contains_key("Stroke_Length") {
                                                "Stroke_Length".to_string()
                                            } else {
                                                attrs.keys().next().cloned().unwrap_or_default()
                                            });
                                        }
                                        if !attrs.contains_key(&self.scatter_y_attr) {
                                            next_sy = Some(if attrs.contains_key("NAIN_Rn") {
                                                "NAIN_Rn".to_string()
                                            } else if attrs.contains_key("Integration_HH_n") {
                                                "Integration_HH_n".to_string()
                                            } else if attrs.contains_key("Stroke_Connectivity") {
                                                "Stroke_Connectivity".to_string()
                                            } else {
                                                attrs
                                                    .keys()
                                                    .nth(1)
                                                    .or_else(|| attrs.keys().next())
                                                    .cloned()
                                                    .unwrap_or_default()
                                            });
                                        }
                                    }
                                    if let Some(a) = next_active {
                                        self.active_attribute = a;
                                    }
                                    if let Some(sx) = next_sx {
                                        self.scatter_x_attr = sx;
                                    }
                                    if let Some(sy) = next_sy {
                                        self.scatter_y_attr = sy;
                                    }
                                    let max_n = self.active_feature_count();
                                    self.selected_ids.retain(|&id| id < max_n);
                                }
                            }

                            ui.add_space(4.0);
                            ui.checkbox(
                                &mut self.show_network_colors,
                                "Show Network Attribute Colors (Heatmap)",
                            );
                            ui.checkbox(
                                &mut self.show_segment_ids,
                                "Show Segment / Line ID Labels",
                            );
                            ui.checkbox(
                                &mut self.show_endpoints,
                                "Show Topological Intersection Nodes",
                            );
                            ui.checkbox(
                                &mut self.show_cleaner_diff,
                                "Show 5-Step Cleaner Diff (Raw vs Cleaned)",
                            );
                            ui.checkbox(
                                &mut self.show_grade_unlinks,
                                "Show Grade-Separation Unlinks",
                            );
                            ui.add(
                                egui::Slider::new(&mut self.line_width, 1.0..=6.0)
                                    .text("Network Line Width (px)"),
                            );
                        }
                        AnalysisDomain::SpatialVisibility => {
                            let is_sel = self.active_map_mode == ActiveMapMode::VgaPointMap;
                            if ui
                                .selectable_label(
                                    is_sel,
                                    self.pill_text(ActiveMapMode::VgaPointMap.label(), is_sel),
                                )
                                .clicked()
                            {
                                if self.vga_results.is_none() {
                                    self.run_vga_solver();
                                } else {
                                    self.active_map_mode = ActiveMapMode::VgaPointMap;
                                }
                            }

                            ui.add_space(4.0);
                            ui.checkbox(
                                &mut self.show_vga_cells,
                                "Show VGA Grid Cells (Heatmap)",
                            );
                            ui.checkbox(
                                &mut self.show_obstacles,
                                "Show Floorplan Walls / Plan Lines",
                            );
                            ui.checkbox(
                                &mut self.show_vga_los_edges,
                                "Show VGA Visibility Graph Lines (LOS)",
                            )
                            .on_hover_text(
                                "Draw inter-visibility line-of-sight edges (for selected/hovered cell, or sampled across the floorplan)",
                            );
                            ui.checkbox(
                                &mut self.show_agent_bodies,
                                "Show Active Agent Bodies & Headings",
                            );
                            ui.checkbox(
                                &mut self.show_agent_active_trails,
                                "Show Active Agent Trail Lines",
                            );
                            ui.checkbox(
                                &mut self.show_agent_history_trails,
                                "Show Historical Agent Trail Lines",
                            );
                            ui.checkbox(
                                &mut self.show_isovists,
                                "Show Cast 2D Isovist Polygon & Border",
                            );
                            ui.checkbox(
                                &mut self.show_isovist_rays,
                                "Show Cast Isovist Radial Ray Lines",
                            );
                            ui.checkbox(
                                &mut self.show_isovist_drift,
                                "Show Cast Isovist Drift Vector Line",
                            );
                            ui.add(
                                egui::Slider::new(&mut self.wall_line_width, 0.5..=6.0)
                                    .text("Wall Line Width (px)"),
                            );
                            ui.add(
                                egui::Slider::new(&mut self.agent_trail_width, 0.5..=5.0)
                                    .text("Agent Trail Width (px)"),
                            );
                        }
                    }
                });

                ui.separator();

                // 2. Collapsible Section: Domain-Specific Solver Parameters & Agent Analysis Controls
                match self.project_domain {
                    AnalysisDomain::StreetNetwork => {
                        egui::CollapsingHeader::new(
                            RichText::new("STREET NETWORK SOLVER PARAMETERS")
                                .strong()
                                .color(heading),
                        )
                        .id_salt("dock_sec_street_solver")
                        .default_open(true)
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                ui.label("Radii:");
                                ui.text_edit_singleline(&mut self.radii_input_str);
                            });

                            egui::Grid::new("asa_params_grid")
                                .num_columns(2)
                                .spacing([8.0, 4.0])
                                .show(ui, |ui| {
                                    ui.label("Impedance:");
                                    egui::ComboBox::from_id_salt("asa_atype_combo")
                                        .selected_text(self.asa_config.analysis_type.as_str())
                                        .show_ui(ui, |ui| {
                                            for t in [
                                                AnalysisType::Angular,
                                                AnalysisType::Metric,
                                                AnalysisType::Topological,
                                            ] {
                                                ui.selectable_value(
                                                    &mut self.asa_config.analysis_type,
                                                    t,
                                                    t.as_str(),
                                                );
                                            }
                                        });
                                    ui.end_row();

                                    ui.label("Horizon Unit:");
                                    egui::ComboBox::from_id_salt("asa_rtype_combo")
                                        .selected_text(self.asa_config.radius_type.as_str())
                                        .show_ui(ui, |ui| {
                                            for t in [
                                                RadiusType::Metric,
                                                RadiusType::Angular,
                                                RadiusType::Topological,
                                            ] {
                                                ui.selectable_value(
                                                    &mut self.asa_config.radius_type,
                                                    t,
                                                    t.as_str(),
                                                );
                                            }
                                        });
                                    ui.end_row();

                                    ui.label("Weighting:");
                                    egui::ComboBox::from_id_salt("asa_wmode_combo")
                                        .selected_text(self.asa_config.weight_mode.as_str())
                                        .show_ui(ui, |ui| {
                                            for w in [
                                                WeightMode::SegmentLength,
                                                WeightMode::Unweighted,
                                                WeightMode::Custom,
                                            ] {
                                                ui.selectable_value(
                                                    &mut self.asa_config.weight_mode,
                                                    w,
                                                    w.as_str(),
                                                );
                                            }
                                        });
                                    ui.end_row();

                                    ui.label("Tulip Bins:");
                                    egui::ComboBox::from_id_salt("asa_tulip_combo")
                                        .selected_text(if self.asa_config.tulip_bins == 0 {
                                            "0 (Exact Continuous f64)"
                                        } else {
                                            "1024 (DepthmapX Tulip)"
                                        })
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(
                                                &mut self.asa_config.tulip_bins,
                                                0,
                                                "0 (Exact Continuous f64)",
                                            );
                                            ui.selectable_value(
                                                &mut self.asa_config.tulip_bins,
                                                1024,
                                                "1024 (DepthmapX Tulip)",
                                            );
                                        });
                                    ui.end_row();
                                });

                            ui.add_space(4.0);
                            let mut changed_whiskers = false;
                            if ui
                                .checkbox(
                                    &mut self.prune_axial_whiskers,
                                    "Prune Axial Crossing Whiskers (depthmapX Stub Rule)",
                                )
                                .on_hover_text(
                                    "Automatically removes short axial overshoot tails at street intersections (UCL depthmapX stubremoval = 0.40)",
                                )
                                .changed()
                            {
                                changed_whiskers = true;
                            }
                            if self.prune_axial_whiskers {
                                if ui
                                    .add(
                                        egui::Slider::new(&mut self.axial_stub_ratio, 0.05..=0.48)
                                            .text("Stub Ratio"),
                                    )
                                    .changed()
                                {
                                    changed_whiskers = true;
                                }
                            }
                            if changed_whiskers {
                                self.rebuild_segment_graph_and_solve_asa();
                                if self.active_map_mode == ActiveMapMode::AxialMap
                                    || self.axial_results.is_some()
                                {
                                    let prev_mode = self.active_map_mode;
                                    self.run_axial_solver();
                                    self.active_map_mode = prev_mode;
                                }
                            }

                            if ui
                                .button(self.btn_text("[Recompute ASA Dual Graph Solver]"))
                                .clicked()
                            {
                                self.rebuild_segment_graph_and_solve_asa();
                                self.active_map_mode = ActiveMapMode::SegmentMapAsa;
                            }
                        });
                    }
                    AnalysisDomain::SpatialVisibility => {
                        egui::CollapsingHeader::new(
                            RichText::new("VISIBILITY GRAPH (VGA) & ISOVIST SOLVER")
                                .strong()
                                .color(heading),
                        )
                        .id_salt("dock_sec_vga_solver")
                        .default_open(true)
                        .show(ui, |ui| {
                            egui::Grid::new("vga_params_grid")
                                .num_columns(2)
                                .spacing([8.0, 4.0])
                                .show(ui, |ui| {
                                    ui.label("Grid Cell Size (m):");
                                    ui.add(
                                        egui::Slider::new(&mut self.vga_cell_size_m, 0.05..=5.0)
                                            .step_by(0.05),
                                    );
                                    ui.end_row();
                                    ui.label("Max LOS Horizon (m):");
                                    ui.add(
                                        egui::Slider::new(&mut self.vga_max_vis_m, 10.0..=500.0)
                                            .step_by(5.0),
                                    );
                                    ui.end_row();
                                });
                            ui.checkbox(
                                &mut self.vga_compute_isovist_cols,
                                "Include Analytical Isovist Columns",
                            )
                            .on_hover_text(
                                "Compute per-cell Benedikt Isovist Area/Compactness/Drift columns during VGA",
                            );

                            ui.horizontal_wrapped(|ui| {
                                if ui
                                    .button(self.btn_text("[Create / Fill Walkable Grid]"))
                                    .on_hover_text(
                                        "Create walkable grid cells on the map first (UCL depthmapX workflow)",
                                    )
                                    .clicked()
                                {
                                    self.create_or_fill_vga_grid(None);
                                }
                                if ui
                                    .button(self.btn_text("[Compute VGA Visibility Graph]"))
                                    .on_hover_text(
                                        "Compute all-pairs LOS visibility graph in background",
                                    )
                                    .clicked()
                                {
                                    self.run_vga_solver();
                                }
                            });

                            let vga_job_status = self.vga_job.as_ref().map(|job| {
                                (
                                    job.progress_pct.load(Ordering::Relaxed),
                                    job.cell_count,
                                )
                            });
                            if let Some((pct, cell_count)) = vga_job_status {
                                let frac = (pct as f32 / 100.0).clamp(0.0, 1.0);
                                ui.horizontal(|ui| {
                                    ui.add(
                                        egui::ProgressBar::new(frac)
                                            .desired_width(185.0)
                                            .text(format!("VGA ({} cells): {}%", cell_count, pct)),
                                    );
                                    if ui.button(self.btn_text("[Cancel]")).clicked() {
                                        self.cancel_vga_background_job();
                                    }
                                });
                            }

                            ui.add_space(4.0);
                            ui.label(
                                RichText::new("CANVAS CLICK INTERACTION MODE:")
                                    .strong()
                                    .color(heading),
                            );
                            ui.radio_value(
                                &mut self.click_mode,
                                CanvasClickMode::InspectAndSelectOrigin,
                                "Inspect / Select VGA Cell",
                            );
                            ui.radio_value(
                                &mut self.click_mode,
                                CanvasClickMode::FillVgaGridAtSeed,
                                "Set Grid Seed & Fill Walkable Grid at Click",
                            );
                            ui.radio_value(
                                &mut self.click_mode,
                                CanvasClickMode::PlaceAgentsAtClick,
                                "Place Individual Agent at Clicked Grid Cell",
                            );
                            ui.radio_value(
                                &mut self.click_mode,
                                CanvasClickMode::CastAnalyticalIsovist,
                                "Cast 2D Benedikt Corner-Ray Isovist at Click",
                            );
                        });

                        ui.separator();

                        egui::CollapsingHeader::new(
                            RichText::new("AGENT ANALYSIS (TURNER & PENN 2002)")
                                .strong()
                                .color(heading),
                        )
                        .id_salt("dock_sec_agent_analysis")
                        .default_open(true)
                        .show(ui, |ui| {
                            ui.horizontal_wrapped(|ui| {
                                let place_mode =
                                    self.click_mode == CanvasClickMode::PlaceAgentsAtClick;
                                if ui
                                    .selectable_label(
                                        place_mode,
                                        self.pill_text("[Place Agent (Click Map)]", place_mode),
                                    )
                                    .on_hover_text(
                                        "Click any walkable VGA cell to spawn 1 stationary agent at that cell (UCL depthmapX Q3DView::CreateAgent). Press [Play] to start walking.",
                                    )
                                    .clicked()
                                {
                                    self.click_mode = if place_mode {
                                        CanvasClickMode::InspectAndSelectOrigin
                                    } else {
                                        CanvasClickMode::PlaceAgentsAtClick
                                    };
                                }
                                if ui
                                    .button(self.btn_text("[Clear Locations]"))
                                    .on_hover_text(
                                        "Clear selected release locations, remove active agents, and reset Gate_Counts",
                                    )
                                    .clicked()
                                {
                                    self.is_playing_agents = false;
                                    self.agent_config.release_cells.clear();
                                    self.init_or_reset_agents();
                                }
                            });

                            ui.horizontal_wrapped(|ui| {
                                let lbl = if self.is_playing_agents {
                                    "[Pause]"
                                } else {
                                    "[Play]"
                                };
                                if ui
                                    .selectable_label(
                                        self.is_playing_agents,
                                        self.pill_text(lbl, self.is_playing_agents),
                                    )
                                    .on_hover_text(
                                        "Play/Pause interactive agent movement (depthmapX Q3DView pace: ~2.0 steps/s with smooth sub-step interpolation)",
                                    )
                                    .clicked()
                                {
                                    if self.vga_results.is_none() {
                                        self.create_or_fill_vga_grid(None);
                                    }
                                    if self.agent_state.is_none() {
                                        self.init_or_reset_agents();
                                    }
                                    self.show_agent_bodies = true;
                                    self.is_playing_agents = !self.is_playing_agents;
                                    if self.is_playing_agents {
                                        self.last_agent_frame_instant = Some(Instant::now());
                                    }
                                    self.active_map_mode = ActiveMapMode::VgaPointMap;
                                    self.active_attribute = "Gate_Counts".to_string();
                                }
                                if ui
                                    .button(self.btn_text("[+1 Step]"))
                                    .on_hover_text("Advance active agents by exactly 1 grid timestep")
                                    .clicked()
                                {
                                    if self.vga_results.is_none() {
                                        self.create_or_fill_vga_grid(None);
                                    }
                                    if self.agent_state.is_none() {
                                        self.init_or_reset_agents();
                                    }
                                    let allow_continuous = !self.agent_manual_single_mode
                                        || self.agent_config.release_from_selected_only;
                                    if let (Some(vga), Some(st)) =
                                        (&self.vga_results, &mut self.agent_state)
                                    {
                                        step_agent_simulation_with_mode(
                                            vga,
                                            st,
                                            &self.agent_config,
                                            1,
                                            allow_continuous,
                                        );
                                    }
                                    self.agent_step_accum = 0.0;
                                    self.show_agent_bodies = true;
                                    self.sync_agent_gate_counts_to_vga();
                                    self.active_attribute = "Gate_Counts".to_string();
                                }
                                if ui
                                    .button(self.btn_text("[+20 Steps]"))
                                    .on_hover_text("Advance active agents by 20 grid timesteps")
                                    .clicked()
                                {
                                    if self.vga_results.is_none() {
                                        self.create_or_fill_vga_grid(None);
                                    }
                                    if self.agent_state.is_none() {
                                        self.init_or_reset_agents();
                                    }
                                    let allow_continuous = !self.agent_manual_single_mode
                                        || self.agent_config.release_from_selected_only;
                                    if let (Some(vga), Some(st)) =
                                        (&self.vga_results, &mut self.agent_state)
                                    {
                                        step_agent_simulation_with_mode(
                                            vga,
                                            st,
                                            &self.agent_config,
                                            20,
                                            allow_continuous,
                                        );
                                    }
                                    self.agent_step_accum = 0.0;
                                    self.show_agent_bodies = true;
                                    self.sync_agent_gate_counts_to_vga();
                                    self.active_attribute = "Gate_Counts".to_string();
                                }
                                if ui.button(self.btn_text("[Reset]")).clicked() {
                                    self.is_playing_agents = false;
                                    self.init_or_reset_agents();
                                }
                            });

                            ui.add(
                                egui::Slider::new(&mut self.agent_steps_per_sec, 0.5..=15.0)
                                    .step_by(0.5)
                                    .text("Playback Speed (steps/s)"),
                            )
                            .on_hover_text(
                                "Interactive playback speed in steps per second (UCL depthmapX Q3DView default ≈ 2.08 steps/s, i.e., 480 ms per grid step)",
                            );

                            ui.separator();
                            ui.label(
                                RichText::new("UCL depthmapX Agent Analysis Parameters:")
                                    .strong()
                                    .color(heading),
                            );

                            egui::Grid::new("depthmapx_agent_dlg_grid")
                                .num_columns(2)
                                .spacing([8.0, 4.0])
                                .show(ui, |ui| {
                                    ui.label("Analysis Timesteps:")
                                        .on_hover_text("depthmapX CAgentAnalysisDlg: Analysis length (timesteps), default = 5000");
                                    ui.add(
                                        egui::DragValue::new(&mut self.agent_config.timesteps)
                                            .range(100..=100_000)
                                            .speed(100),
                                    );
                                    ui.end_row();

                                    ui.label("Release Rate (/step):")
                                        .on_hover_text("depthmapX AgentSet: Release rate (agents per timestep via Poisson draw), default = 0.10");
                                    ui.add(
                                        egui::Slider::new(
                                            &mut self.agent_config.release_rate,
                                            0.01..=2.0,
                                        )
                                        .step_by(0.01),
                                    );
                                    ui.end_row();

                                    ui.label("Field of View (bins):")
                                        .on_hover_text("depthmapX AgentProgram: Field of view (bins, 1..32). Default = 15 bins (2*7+1 ≈ 168.75°)");
                                    ui.add(egui::Slider::new(
                                        &mut self.agent_config.agent_fov_bins,
                                        1..=32,
                                    ));
                                    ui.end_row();

                                    ui.label("Steps Before Turn:")
                                        .on_hover_text("depthmapX AgentProgram: Steps before turn decision (Bernoulli probability 1/m_steps per step), default = 3");
                                    ui.add(egui::Slider::new(
                                        &mut self.agent_config.agent_steps_before_turn,
                                        1..=20,
                                    ));
                                    ui.end_row();

                                    ui.label("Lifetime (steps):")
                                        .on_hover_text("depthmapX AgentSet: Timesteps in system before agent retirement, default = 1000");
                                    ui.add(
                                        egui::DragValue::new(
                                            &mut self.agent_config.agent_life_steps,
                                        )
                                        .range(25..=10_000)
                                        .speed(25),
                                    );
                                    ui.end_row();

                                    ui.label("Movement Rule:")
                                        .on_hover_text("depthmapX AgentProgram: Visual target selection rule (Standard SEL_STANDARD vs Line of Sight Length SEL_LOS)");
                                    egui::ComboBox::from_id_salt("agent_movement_rule_combo")
                                        .selected_text(self.agent_config.movement_rule.as_str())
                                        .width(155.0)
                                        .show_ui(ui, |ui| {
                                            for rule in [
                                                AgentMovementRule::Standard,
                                                AgentMovementRule::LineOfSightLength,
                                            ] {
                                                ui.selectable_value(
                                                    &mut self.agent_config.movement_rule,
                                                    rule,
                                                    rule.as_str(),
                                                );
                                            }
                                        });
                                    ui.end_row();
                                });

                            ui.horizontal(|ui| {
                                ui.radio_value(
                                    &mut self.agent_config.release_from_selected_only,
                                    false,
                                    "Release from any location",
                                );
                                ui.radio_value(
                                    &mut self.agent_config.release_from_selected_only,
                                    true,
                                    "Selected locations",
                                );
                            });

                            ui.horizontal(|ui| {
                                if ui
                                    .checkbox(
                                        &mut self.agent_config.record_trails,
                                        "Record trails for",
                                    )
                                    .changed()
                                    && self.agent_config.record_trails
                                {
                                    self.show_agent_active_trails = true;
                                    self.show_agent_history_trails = true;
                                }
                                ui.add(
                                    egui::DragValue::new(&mut self.agent_config.max_trails)
                                        .range(1..=500)
                                        .speed(5),
                                );
                                ui.label("agents");
                            });

                            if ui
                                .button(
                                    self.btn_text(format!(
                                        "[Run Agent Analysis ({} Timesteps)]",
                                        self.agent_config.timesteps
                                    )),
                                )
                                .on_hover_text(
                                    "Execute full batch Agent Analysis (depthmapX AgentEngine::run) and populate Gate_Counts",
                                )
                                .clicked()
                            {
                                self.is_playing_agents = false;
                                self.agent_manual_single_mode = false;
                                self.run_agents_batch_solver();
                            }

                            if let Some(st) = &self.agent_state {
                                ui.label(
                                    RichText::new(format!(
                                        "Step: {} | Active: {} | Pins: {} | Trails: {}",
                                        st.current_step,
                                        st.active_agents.len(),
                                        self.agent_config.release_cells.len(),
                                        st.finished_trails.len()
                                    ))
                                    .monospace()
                                    .size(11.5)
                                    .color(heading),
                                );
                            }
                        });
                    }
                }

                ui.separator();

                // 3. Collapsible Section: Attributes & Spectral Selector
                egui::CollapsingHeader::new(
                    RichText::new("ATTRIBUTES & SPECTRAL SELECTOR")
                        .strong()
                        .color(heading),
                )
                .id_salt("dock_sec_attributes")
                .default_open(true)
                .show(ui, |ui| {
                    let attr_keys: Vec<String> = self
                        .active_attributes_map()
                        .map(|m| m.keys().cloned().collect())
                        .unwrap_or_default();

                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        egui::ScrollArea::vertical()
                            .id_salt("attr_selector_scroll")
                            .max_height(175.0)
                            .show(ui, |ui| {
                                for k in &attr_keys {
                                    let is_sel = &self.active_attribute == k;
                                    if ui
                                        .selectable_label(
                                            is_sel,
                                            self.pill_text(k, is_sel).size(12.5),
                                        )
                                        .clicked()
                                    {
                                        self.active_attribute = k.clone();
                                    }
                                }
                            });
                    });
                });

                ui.separator();

                // 4. Collapsible Section: Attribute Summary & Diagnostics (Bottom of Left Dock)
                egui::CollapsingHeader::new(
                    RichText::new("ATTRIBUTE SUMMARY & DIAGNOSTICS")
                        .strong()
                        .color(heading),
                )
                .id_salt("dock_sec_summary_stats")
                .default_open(true)
                .show(ui, |ui| {
                    let vals = self.active_attribute_values();
                    let stats = SummaryStats::from_slice(&vals);
                    egui::Grid::new("summary_stats_grid")
                        .num_columns(2)
                        .striped(true)
                        .spacing([12.0, 3.0])
                        .show(ui, |ui| {
                            ui.label(RichText::new("Attribute").monospace());
                            ui.label(
                                RichText::new(&self.active_attribute)
                                    .monospace()
                                    .strong()
                                    .color(heading),
                            );
                            ui.end_row();

                            ui.label(RichText::new("Valid Count (N)").monospace());
                            ui.label(RichText::new(format!("{}", stats.count)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Minimum").monospace());
                            ui.label(RichText::new(format!("{:.4}", stats.min)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("P10 Quantile").monospace());
                            ui.label(RichText::new(format!("{:.4}", stats.p10)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Median (P50)").monospace());
                            ui.label(RichText::new(format!("{:.4}", stats.p50)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Mean").monospace());
                            ui.label(RichText::new(format!("{:.4}", stats.mean)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("P90 Quantile").monospace());
                            ui.label(RichText::new(format!("{:.4}", stats.p90)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Maximum").monospace());
                            ui.label(RichText::new(format!("{:.4}", stats.max)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Std Dev").monospace());
                            ui.label(RichText::new(format!("{:.4}", stats.std)).monospace());
                            ui.end_row();

                            if self.project_domain == AnalysisDomain::StreetNetwork {
                                let circ_opt = if let Some(nr) = &self.strokes_results {
                                    nr.circuity_avg
                                } else if !self.working_polylines.is_empty() {
                                    compute_network_circuity_avg(&self.working_polylines)
                                } else {
                                    None
                                };
                                ui.label(RichText::new("Intelligibility R²").monospace());
                                ui.label(
                                    RichText::new(format!(
                                        "{:.4}",
                                        self.asa_results.intelligibility_r2
                                    ))
                                    .monospace()
                                    .strong()
                                    .color(heading),
                                );
                                ui.end_row();

                                let syn_r2 = self
                                    .asa_results
                                    .synergy_r2
                                    .values()
                                    .next()
                                    .copied()
                                    .unwrap_or(0.0);
                                ui.label(RichText::new("Synergy R² (Local/Rn)").monospace());
                                ui.label(RichText::new(format!("{:.4}", syn_r2)).monospace());
                                ui.end_row();

                                ui.label(RichText::new("Circuity Avg").monospace());
                                let c_txt = match circ_opt {
                                    Some(c) => format!("{:.4}", c),
                                    None => "None (Closed Loop)".to_string(),
                                };
                                ui.label(
                                    RichText::new(c_txt)
                                        .monospace()
                                        .strong()
                                        .color(heading),
                                );
                                ui.end_row();
                            } else if let Some(vga) = &self.vga_results {
                                ui.label(RichText::new("VGA Grid Spacing").monospace());
                                ui.label(
                                    RichText::new(format!("{:.2} m", vga.cell_size_m)).monospace(),
                                );
                                ui.end_row();

                                ui.label(RichText::new("LOS Graph Edges").monospace());
                                ui.label(
                                    RichText::new(format!("{}", vga.col_indices.len())).monospace(),
                                );
                                ui.end_row();
                            }
                        });
                });
            });
    }

    fn render_bottom_status_bar(&self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(
                RichText::new(format!(
                    "Cursor: ({:.2}, {:.2}) m",
                    self.last_cursor_world.x, self.last_cursor_world.y
                ))
                .monospace()
                .size(11.5),
            );
            ui.separator();
            ui.label(
                RichText::new(format!(
                    "{} | Attr: {}",
                    self.active_map_mode.label(),
                    self.active_attribute
                ))
                .monospace()
                .size(11.5),
            );
            ui.separator();
            let count_n = match self.active_map_mode {
                ActiveMapMode::SegmentMapAsa => self.segments.len(),
                ActiveMapMode::AxialMap => {
                    self.axial_results.as_ref().map(|a| a.lines.len()).unwrap_or(0)
                }
                ActiveMapMode::NaturalRoadsStrokes => self
                    .strokes_results
                    .as_ref()
                    .map(|s| s.strokes.len())
                    .unwrap_or(0),
                ActiveMapMode::VgaPointMap => {
                    self.vga_results.as_ref().map(|v| v.points.len()).unwrap_or(0)
                }
            };
            ui.label(
                RichText::new(format!(
                    "Elements: {} | Selected: {}",
                    count_n,
                    self.selected_ids.len()
                ))
                .monospace()
                .size(11.5),
            );
            ui.separator();
            ui.label(
                RichText::new(format!("Last Solver: {:.2} ms", self.last_solver_ms))
                    .monospace()
                    .size(11.5),
            );
            ui.separator();
            let status_col = if self.dark_theme {
                Color32::from_rgb(125, 200, 150)
            } else {
                Color32::from_rgb(18, 110, 55)
            };
            ui.label(
                RichText::new(&self.status_message)
                    .size(11.5)
                    .color(status_col),
            );
        });
    }
}

fn point_to_segment_dist(p: Point2D, a: Point2D, b: Point2D) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let l2 = dx * dx + dy * dy;
    if l2 <= 1e-12 {
        return p.dist(a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / l2).clamp(0.0, 1.0);
    let proj = Point2D::new(a.x + t * dx, a.y + t * dy);
    p.dist(proj)
}

impl SseWorkstationApp {
    fn render_spatial_viewport(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        let is_empty_project = self.segments.is_empty() && self.obstacles.is_empty();
        if is_empty_project {
            let rect = ui.available_rect_before_wrap();
            let bg_col = if self.dark_theme {
                Color32::from_rgb(10, 13, 18)
            } else {
                Color32::from_rgb(248, 249, 252)
            };
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 0.0, bg_col);

            let grid_col = if self.dark_theme {
                Color32::from_rgba_unmultiplied(45, 58, 78, 45)
            } else {
                Color32::from_rgba_unmultiplied(195, 205, 218, 95)
            };
            let step_px = 48.0_f32;
            let mut gx = rect.left();
            while gx <= rect.right() {
                painter.line_segment(
                    [Pos2::new(gx, rect.top()), Pos2::new(gx, rect.bottom())],
                    Stroke::new(1.0, grid_col),
                );
                gx += step_px;
            }
            let mut gy = rect.top();
            while gy <= rect.bottom() {
                painter.line_segment(
                    [Pos2::new(rect.left(), gy), Pos2::new(rect.right(), gy)],
                    Stroke::new(1.0, grid_col),
                );
                gy += step_px;
            }

            let accent = self.accent_text_color();
            let heading = self.heading_text_color();
            let top_pad = ((rect.height() - 470.0) * 0.38).max(14.0);

            ui.vertical_centered(|ui| {
                ui.add_space(top_pad);
                egui::Frame::window(ui.style())
                    .inner_margin(egui::Margin::symmetric(28.0, 20.0))
                    .show(ui, |ui| {
                        ui.set_max_width(660.0);
                        ui.vertical_centered(|ui| {
                            ui.label(
                                RichText::new(format!(
                                    "{} — Empty Project Workspace",
                                    self.tab_title
                                ))
                                .size(18.0)
                                .strong()
                                .color(accent),
                            );
                            ui.add_space(4.0);
                            ui.label(
                                RichText::new(
                                    "Choose an Analysis Mode below: Urban Street Network Analysis and Spatial / Building Visibility Analysis use separate maps and dedicated tools.",
                                )
                                .size(12.5)
                                .color(heading),
                            );
                        });
                        ui.add_space(10.0);
                        ui.separator();
                        ui.add_space(6.0);

                        // A. Urban Street Network Section
                        ui.label(
                            RichText::new(
                                "A. URBAN STREET NETWORK ANALYSIS (Segment ASA / Axial Map / Natural Roads / 5-Step Cleaner):",
                            )
                            .strong()
                            .size(13.0)
                            .color(accent),
                        );
                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            if ui
                                .button(
                                    self.btn_text(
                                        "[Open Street Network File (.mif / .dxf / .geojson / .osm / .csv)...]",
                                    )
                                    .size(12.5),
                                )
                                .clicked()
                            {
                                self.dialog_open_vector_file_with_domain(
                                    AnalysisDomain::StreetNetwork,
                                );
                            }
                            if ui
                                .button(
                                    self.btn_text("[Download OpenStreetMap Overpass Network...]")
                                        .size(12.5),
                                )
                                .clicked()
                            {
                                self.show_overpass_modal = true;
                            }
                        });
                        ui.add_space(4.0);
                        for (preset, btn_text) in [
                            (
                                BenchmarkPreset::UrbanSuperblockGrid,
                                "[UCL Barnsbury Street Network — Islington, London (barnsbury_segment_lines.mif)]",
                            ),
                            (
                                BenchmarkPreset::AxialStarDiamond,
                                "[UCL Barnsbury Extended Axial Network — North London (barnsbury_extended1.dxf)]",
                            ),
                            (
                                BenchmarkPreset::RingRoadCulDeSacLoop,
                                "[Closed-Loop Ring-Road & Cul-de-Sac Network (osmnx#1410 Sinuosity Guard)]",
                            ),
                        ] {
                            if ui
                                .add_sized(
                                    [635.0, 27.0],
                                    egui::Button::new(self.btn_text(btn_text).size(12.2)),
                                )
                                .clicked()
                            {
                                self.load_preset(preset);
                            }
                            ui.add_space(2.5);
                        }

                        ui.add_space(8.0);
                        ui.separator();
                        ui.add_space(6.0);

                        // B. Spatial & Building Visibility Section
                        ui.label(
                            RichText::new(
                                "B. SPATIAL & BUILDING VISIBILITY ANALYSIS (VGA PointMap / 2D Benedikt Isovists / Gibson Agents):",
                            )
                            .strong()
                            .size(13.0)
                            .color(accent),
                        );
                        ui.add_space(4.0);
                        if ui
                            .button(
                                self.btn_text(
                                    "[Open Architectural Floorplan / Spatial Boundary File (.dxf / .mif / .geojson)...]",
                                )
                                .size(12.5),
                            )
                            .clicked()
                        {
                            self.dialog_open_vector_file_with_domain(
                                AnalysisDomain::SpatialVisibility,
                            );
                        }
                        ui.add_space(4.0);
                        for (preset, btn_text) in [
                            (
                                BenchmarkPreset::ArchitecturalGalleryVga,
                                "[UCL National Gallery Floorplan — Sainsbury Wing VGA & Agents (gallery.dxf)]",
                            ),
                            (
                                BenchmarkPreset::UclRoomsFloorplan,
                                "[UCL Multi-Room Architectural Floorplan — VGA & Isovists (rooms.dxf)]",
                            ),
                        ] {
                            if ui
                                .add_sized(
                                    [635.0, 27.0],
                                    egui::Button::new(self.btn_text(btn_text).size(12.2)),
                                )
                                .clicked()
                            {
                                self.load_preset(preset);
                            }
                            ui.add_space(2.5);
                        }
                    });
            });
            return;
        }

        let avail = ui.available_size();
        let (response, painter) = ui.allocate_painter(avail, Sense::click_and_drag());
        let rect = response.rect;

        let bg_col = if self.dark_theme {
            Color32::from_rgb(10, 13, 18)
        } else {
            Color32::from_rgb(252, 252, 250)
        };
        painter.rect_filled(rect, 0.0, bg_col);

        if self.needs_zoom_to_fit && rect.width() > 50.0 && rect.height() > 50.0 {
            self.zoom_to_fit(rect);
        }

        // Keyboard shortcut 'F' -> Zoom to Fit
        if ctx.input(|i| i.key_pressed(egui::Key::F)) {
            self.zoom_to_fit(rect);
        }

        // Pan & Scroll-Wheel Zoom around cursor
        if response.dragged() {
            let delta = response.drag_delta();
            self.cam_center.x -= (delta.x as f64) / self.cam_zoom;
            self.cam_center.y += (delta.y as f64) / self.cam_zoom;
        }
        if response.hovered() {
            let scroll_y = ctx.input(|i| i.smooth_scroll_delta.y);
            if scroll_y.abs() > 0.01 {
                let factor = (1.0018_f64).powf(scroll_y as f64);
                if let Some(hover_pos) = response.hover_pos() {
                    let world_before = self.screen_to_world(hover_pos, rect);
                    self.cam_zoom = (self.cam_zoom * factor).clamp(0.02, 5000.0);
                    let world_after = self.screen_to_world(hover_pos, rect);
                    self.cam_center.x += world_before.x - world_after.x;
                    self.cam_center.y += world_before.y - world_after.y;
                } else {
                    self.cam_zoom = (self.cam_zoom * factor).clamp(0.02, 5000.0);
                }
            }
        }

        // Subtle CAD coordinate grid lines (adaptive from architectural 0.5m to urban 500m)
        let grid_step_m = if self.cam_zoom > 150.0 {
            0.5
        } else if self.cam_zoom > 60.0 {
            1.0
        } else if self.cam_zoom > 25.0 {
            5.0
        } else if self.cam_zoom > 5.0 {
            25.0
        } else if self.cam_zoom > 1.0 {
            100.0
        } else {
            500.0
        };
        let w_min = self.screen_to_world(Pos2::new(rect.left(), rect.bottom()), rect);
        let w_max = self.screen_to_world(Pos2::new(rect.right(), rect.top()), rect);
        let grid_col = if self.dark_theme {
            Color32::from_rgba_unmultiplied(45, 58, 78, 55)
        } else {
            Color32::from_rgba_unmultiplied(200, 208, 220, 110)
        };
        let gx_start = (w_min.x / grid_step_m).floor() as i64;
        let gx_end = (w_max.x / grid_step_m).ceil() as i64;
        if (gx_end - gx_start) <= 120 {
            for gx in gx_start..=gx_end {
                let wx = (gx as f64) * grid_step_m;
                let p0 = self.world_to_screen(Point2D::new(wx, w_min.y), rect);
                let p1 = self.world_to_screen(Point2D::new(wx, w_max.y), rect);
                painter.line_segment([p0, p1], Stroke::new(1.0, grid_col));
            }
        }
        let gy_start = (w_min.y / grid_step_m).floor() as i64;
        let gy_end = (w_max.y / grid_step_m).ceil() as i64;
        if (gy_end - gy_start) <= 120 {
            for gy in gy_start..=gy_end {
                let wy = (gy as f64) * grid_step_m;
                let p0 = self.world_to_screen(Point2D::new(w_min.x, wy), rect);
                let p1 = self.world_to_screen(Point2D::new(w_max.x, wy), rect);
                painter.line_segment([p0, p1], Stroke::new(1.0, grid_col));
            }
        }

        // Update cursor world coordinates & hover feature detection
        self.hovered_feature_idx = None;
        if let Some(hpos) = response.hover_pos() {
            let wp = self.screen_to_world(hpos, rect);
            self.last_cursor_world = wp;
            let tol_m = (8.0 / self.cam_zoom).max(0.05);

            match self.active_map_mode {
                ActiveMapMode::SegmentMapAsa => {
                    let mut best = (usize::MAX, tol_m);
                    for (idx, s) in self.segments.iter().enumerate() {
                        let d = point_to_segment_dist(wp, s.p0, s.p1);
                        if d < best.1 {
                            best = (idx, d);
                        }
                    }
                    if best.0 != usize::MAX {
                        self.hovered_feature_idx = Some(best.0);
                    }
                }
                ActiveMapMode::AxialMap => {
                    if let Some(ax) = &self.axial_results {
                        let mut best = (usize::MAX, tol_m);
                        for (idx, s) in ax.lines.iter().enumerate() {
                            let d = point_to_segment_dist(wp, s.p0, s.p1);
                            if d < best.1 {
                                best = (idx, d);
                            }
                        }
                        if best.0 != usize::MAX {
                            self.hovered_feature_idx = Some(best.0);
                        }
                    }
                }
                ActiveMapMode::NaturalRoadsStrokes => {
                    if let Some(nr) = &self.strokes_results {
                        let mut best = (usize::MAX, tol_m);
                        for (idx, st) in nr.strokes.iter().enumerate() {
                            for w in st.coords.windows(2) {
                                let d = point_to_segment_dist(wp, w[0], w[1]);
                                if d < best.1 {
                                    best = (idx, d);
                                }
                            }
                        }
                        if best.0 != usize::MAX {
                            self.hovered_feature_idx = Some(best.0);
                        }
                    }
                }
                ActiveMapMode::VgaPointMap => {
                    if let Some(vga) = &self.vga_results {
                        let mut best = (usize::MAX, vga.cell_size_m * 0.85);
                        for (idx, pt) in vga.points.iter().enumerate() {
                            let d = wp.dist(Point2D::new(pt.x, pt.y));
                            if d < best.1 {
                                best = (idx, d);
                            }
                        }
                        if best.0 != usize::MAX {
                            self.hovered_feature_idx = Some(best.0);
                        }
                    }
                }
            }
        }

        // Handle Canvas Click (Select Segment / Fill Grid Seed / Place Agents / Cast 2D Isovist)
        if response.clicked() {
            if let Some(cpos) = response.interact_pointer_pos() {
                let wp = self.screen_to_world(cpos, rect);
                match (self.project_domain, self.click_mode) {
                    (AnalysisDomain::SpatialVisibility, CanvasClickMode::FillVgaGridAtSeed) => {
                        self.create_or_fill_vga_grid(Some(wp));
                    }
                    (AnalysisDomain::SpatialVisibility, CanvasClickMode::PlaceAgentsAtClick) => {
                        self.place_agents_at_world_point(wp);
                    }
                    (AnalysisDomain::SpatialVisibility, CanvasClickMode::CastAnalyticalIsovist) => {
                        let obs = if self.obstacles.is_empty() {
                            &self.working_polylines
                        } else {
                            &self.obstacles
                        };
                        let iso = compute_isovist_at_point(wp, obs, self.vga_max_vis_m);
                        self.show_isovists = true;
                        self.log_audit(format!(
                            "[ISOVIST] Cast Benedikt 2D Isovist at ({:.2}, {:.2}) -> Area={:.2} m², Compactness={:.4}, Occlusivity={:.2} m.",
                            wp.x, wp.y, iso.area, iso.compactness, iso.occlusivity
                        ));
                        if !ctx.input(|i| i.modifiers.shift) && self.cast_isovists.len() >= 3 {
                            self.cast_isovists.remove(0);
                        }
                        self.cast_isovists.push(iso);
                    }
                    _ => {
                        let shift = ctx.input(|i| i.modifiers.shift);
                        if !shift {
                            self.selected_ids.clear();
                        }
                        if let Some(h_idx) = self.hovered_feature_idx {
                            if self.selected_ids.contains(&h_idx) && shift {
                                self.selected_ids.remove(&h_idx);
                            } else {
                                self.selected_ids.insert(h_idx);
                            }
                        }
                    }
                }
            }
        }

        // Advance Gibson Visual Agents at wall-clock pace (depthmapX Q3DView reference: ~2.08 steps/sec with sub-step interpolation)
        if self.is_playing_agents && self.project_domain == AnalysisDomain::SpatialVisibility {
            let now = Instant::now();
            let dt = self
                .last_agent_frame_instant
                .map(|prev| now.duration_since(prev).as_secs_f64().min(0.25))
                .unwrap_or(1.0 / 60.0);
            self.last_agent_frame_instant = Some(now);
            self.agent_step_accum += dt * self.agent_steps_per_sec.clamp(0.2, 30.0);
            let steps_to_run = self.agent_step_accum.floor() as usize;
            if steps_to_run > 0 {
                self.agent_step_accum -= steps_to_run as f64;
                let allow_continuous =
                    !self.agent_manual_single_mode || self.agent_config.release_from_selected_only;
                if let (Some(vga), Some(st)) = (&self.vga_results, &mut self.agent_state) {
                    step_agent_simulation_with_mode(
                        vga,
                        st,
                        &self.agent_config,
                        steps_to_run.min(10),
                        allow_continuous,
                    );
                }
                self.sync_agent_gate_counts_to_vga();
            }
            ctx.request_repaint_after(Duration::from_millis(16));
        } else {
            self.last_agent_frame_instant = None;
        }

        // Compute normalized color [0..1] for active map
        let vals = self.active_attribute_values();
        let norm_t = self.compute_normalized_t_values(&vals);
        let sel_halo_col = if self.dark_theme {
            Color32::WHITE
        } else {
            Color32::from_rgb(15, 23, 42)
        };
        let node_dot_col = if self.dark_theme {
            Color32::from_rgb(210, 220, 235)
        } else {
            Color32::from_rgb(45, 58, 78)
        };

        // 0. Optional 5-Step Cleaner Diff Overlay (Raw Uncleaned vs Cleaned Network)
        if self.show_cleaner_diff && self.project_domain == AnalysisDomain::StreetNetwork {
            let ghost_raw_col = if self.dark_theme {
                Color32::from_rgba_unmultiplied(235, 95, 255, 145)
            } else {
                Color32::from_rgba_unmultiplied(175, 35, 210, 150)
            };
            for poly in &self.raw_polylines {
                for w in poly.coords.windows(2) {
                    let p0 = self.world_to_screen(w[0], rect);
                    let p1 = self.world_to_screen(w[1], rect);
                    painter.line_segment([p0, p1], Stroke::new(self.line_width + 2.2, ghost_raw_col));
                }
            }
            // Highlight raw dangles vs cleaned endpoints so healed/snapped undershoots & pruned whiskers stand out
            let quantize_cm = |pt: Point2D| -> (i64, i64) {
                ((pt.x * 100.0).round() as i64, (pt.y * 100.0).round() as i64)
            };
            let mut raw_deg: HashMap<(i64, i64), (usize, Point2D)> = HashMap::new();
            for poly in &self.raw_polylines {
                if let (Some(&first), Some(&last)) = (poly.coords.first(), poly.coords.last()) {
                    let e0 = raw_deg.entry(quantize_cm(first)).or_insert((0, first));
                    e0.0 += 1;
                    let e1 = raw_deg.entry(quantize_cm(last)).or_insert((0, last));
                    e1.0 += 1;
                }
            }
            let mut clean_deg: HashMap<(i64, i64), usize> = HashMap::new();
            for seg in &self.segments {
                *clean_deg.entry(quantize_cm(seg.p0)).or_insert(0) += 1;
                *clean_deg.entry(quantize_cm(seg.p1)).or_insert(0) += 1;
            }
            for (k, (r_deg, pt)) in &raw_deg {
                if *r_deg == 1 {
                    let c_deg = clean_deg.get(k).copied().unwrap_or(0);
                    if c_deg != 1 {
                        // This raw dangle was healed (snapped to junction or pruned stub)!
                        let sp = self.world_to_screen(*pt, rect);
                        if rect.contains(sp) {
                            painter.circle_stroke(
                                sp,
                                6.5,
                                Stroke::new(2.0, Color32::from_rgb(225, 35, 185)),
                            );
                            painter.circle_filled(sp, 2.6, Color32::from_rgb(16, 185, 129));
                        }
                    }
                }
            }
        }

        // Helper for rendering legible ID labels with screen-space de-cluttering + high-contrast pill background
        let pill_bg = if self.dark_theme {
            Color32::from_rgba_unmultiplied(15, 21, 31, 220)
        } else {
            Color32::from_rgba_unmultiplied(255, 255, 255, 232)
        };
        let pill_border = if self.dark_theme {
            Color32::from_rgba_unmultiplied(85, 110, 145, 180)
        } else {
            Color32::from_rgba_unmultiplied(148, 163, 184, 200)
        };
        let pill_text_col = if self.dark_theme {
            Color32::from_rgb(240, 246, 255)
        } else {
            Color32::from_rgb(15, 23, 42)
        };

        let neutral_wire_col = if self.dark_theme {
            Color32::from_rgb(148, 163, 184)
        } else {
            Color32::from_rgb(100, 116, 139)
        };
        let offset_label_from_seg = |p0: Pos2, p1: Pos2, mid: Pos2, line_w: f32| -> (Pos2, Pos2, Pos2) {
            let d = p1 - p0;
            let len = d.length();
            let mut n = if len > 1e-3 {
                Vec2::new(-d.y / len, d.x / len)
            } else {
                Vec2::new(0.0, -1.0)
            };
            if n.y > 0.0 || (n.y.abs() < 1e-3 && n.x < 0.0) {
                n = -n;
            }
            let tick_start = mid + n * (line_w * 0.5 + 1.5);
            let tick_end = mid + n * (line_w * 0.5 + 5.0);
            let label_center = mid + n * (line_w * 0.5 + 11.5);
            (tick_start, tick_end, label_center)
        };

        // 1. Render Active Map Geometry
        match self.active_map_mode {
            ActiveMapMode::SegmentMapAsa => {
                for (idx, seg) in self.segments.iter().enumerate() {
                    let p0 = self.world_to_screen(seg.p0, rect);
                    let p1 = self.world_to_screen(seg.p1, rect);
                    let col = if self.show_network_colors {
                        let t = norm_t.get(idx).copied().unwrap_or(0.0);
                        let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
                        Color32::from_rgb(r, g, b)
                    } else {
                        neutral_wire_col
                    };

                    let is_sel = self.selected_ids.contains(&idx);
                    let is_hov = self.hovered_feature_idx == Some(idx);
                    if is_sel || is_hov {
                        painter.line_segment(
                            [p0, p1],
                            Stroke::new(self.line_width + 3.5, sel_halo_col),
                        );
                    }
                    painter.line_segment([p0, p1], Stroke::new(self.line_width, col));
                }

                if self.show_endpoints {
                    // Distinguish Welded Topological Junctions (deg >= 2) vs Dangling Dead-Ends (deg == 1)
                    let mut v_deg: HashMap<(i64, i64), (usize, Point2D)> =
                        HashMap::with_capacity(self.segments.len());
                    for seg in &self.segments {
                        for pt in [seg.p0, seg.p1] {
                            let key = ((pt.x * 1000.0).round() as i64, (pt.y * 1000.0).round() as i64);
                            let entry = v_deg.entry(key).or_insert((0, pt));
                            entry.0 += 1;
                        }
                    }
                    let junc_fill = if self.dark_theme {
                        Color32::from_rgb(56, 189, 248)
                    } else {
                        Color32::from_rgb(2, 132, 199)
                    };
                    let junc_rim = if self.dark_theme {
                        Color32::from_rgb(15, 23, 42)
                    } else {
                        Color32::WHITE
                    };
                    let dangle_col = Color32::from_rgb(239, 68, 68);
                    for (_k, (deg, pt)) in v_deg {
                        let sp = self.world_to_screen(pt, rect);
                        if !rect.contains(sp) {
                            continue;
                        }
                        if deg == 1 {
                            painter.circle_stroke(sp, 4.2, Stroke::new(1.5, dangle_col));
                            painter.circle_filled(sp, 1.8, dangle_col);
                        } else {
                            painter.circle_filled(sp, 3.0, junc_fill);
                            painter.circle_stroke(sp, 3.0, Stroke::new(0.9, junc_rim));
                        }
                    }
                }

                if self.show_segment_ids {
                    let mut occupied_cells: HashSet<(i32, i32)> = HashSet::new();
                    let dense_zoom = self.cam_zoom > 3.5;
                    let cell_w = if dense_zoom { 1.0 } else { 76.0 };
                    let cell_h = if dense_zoom { 1.0 } else { 30.0 };

                    // First pass: always draw selected or hovered segment IDs (offset perpendicular to segment so color stays uncovered)
                    for (idx, seg) in self.segments.iter().enumerate() {
                        if !self.selected_ids.contains(&idx) && self.hovered_feature_idx != Some(idx) {
                            continue;
                        }
                        let p0 = self.world_to_screen(seg.p0, rect);
                        let p1 = self.world_to_screen(seg.p1, rect);
                        let mid = self.world_to_screen(seg.midpoint(), rect);
                        if !rect.contains(mid) {
                            continue;
                        }
                        let (t0, t1, lpos) = offset_label_from_seg(p0, p1, mid, self.line_width);
                        let cell = ((lpos.x / cell_w).floor() as i32, (lpos.y / cell_h).floor() as i32);
                        occupied_cells.insert(cell);
                        let label = format!("#{}", seg.id);
                        let box_w = (label.len() as f32 * 6.2 + 6.0).max(20.0);
                        let bg_rect = Rect::from_center_size(lpos, Vec2::new(box_w, 14.0));
                        painter.line_segment([t0, t1], Stroke::new(1.2, sel_halo_col));
                        painter.rect_filled(bg_rect, 3.0, pill_bg);
                        painter.rect_stroke(bg_rect, 3.0, Stroke::new(1.2, sel_halo_col));
                        painter.text(
                            lpos,
                            egui::Align2::CENTER_CENTER,
                            label,
                            FontId::new(10.0, FontFamily::Monospace),
                            pill_text_col,
                        );
                    }

                    // Second pass: draw non-overlapping segment IDs offset perpendicular to each segment so segment colors remain 100% visible
                    for seg in &self.segments {
                        let p0 = self.world_to_screen(seg.p0, rect);
                        let p1 = self.world_to_screen(seg.p1, rect);
                        if !dense_zoom && p0.distance(p1) < 15.0 {
                            continue;
                        }
                        let mid = self.world_to_screen(seg.midpoint(), rect);
                        if !rect.contains(mid) {
                            continue;
                        }
                        let (t0, t1, lpos) = offset_label_from_seg(p0, p1, mid, self.line_width);
                        let cell = ((lpos.x / cell_w).floor() as i32, (lpos.y / cell_h).floor() as i32);
                        if !dense_zoom && !occupied_cells.insert(cell) {
                            continue;
                        }
                        let label = format!("#{}", seg.id);
                        let box_w = (label.len() as f32 * 6.0 + 5.0).max(18.0);
                        let bg_rect = Rect::from_center_size(lpos, Vec2::new(box_w, 13.0));
                        painter.line_segment([t0, t1], Stroke::new(1.0, pill_border));
                        painter.rect_filled(bg_rect, 2.5, pill_bg);
                        painter.rect_stroke(bg_rect, 2.5, Stroke::new(0.8, pill_border));
                        painter.text(
                            lpos,
                            egui::Align2::CENTER_CENTER,
                            label,
                            FontId::new(9.5, FontFamily::Monospace),
                            pill_text_col,
                        );
                    }
                }
                if self.show_grade_unlinks {
                    for &(u, v) in &self.grade_unlinks {
                        if let (Some(su), Some(sv)) = (self.segments.get(u), self.segments.get(v)) {
                            let m = Point2D::new(
                                0.5 * (su.midpoint().x + sv.midpoint().x),
                                0.5 * (su.midpoint().y + sv.midpoint().y),
                            );
                            let sm = self.world_to_screen(m, rect);
                            painter.circle_stroke(
                                sm,
                                5.5,
                                Stroke::new(1.6, Color32::from_rgb(235, 165, 15)),
                            );
                        }
                    }
                }
                if let Some(sd) = &self.step_depth_results {
                    for &oid in &sd.origin_ids {
                        if let Some(seg) = self.segments.get(oid) {
                            let sm = self.world_to_screen(seg.midpoint(), rect);
                            painter.circle_stroke(sm, 8.0, Stroke::new(2.2, sel_halo_col));
                            painter.circle_stroke(
                                sm,
                                12.0,
                                Stroke::new(1.8, Color32::from_rgb(25, 165, 235)),
                            );
                        }
                    }
                }
            }
            ActiveMapMode::AxialMap => {
                if let Some(ax) = &self.axial_results {
                    let mut occupied_cells: HashSet<(i32, i32)> = HashSet::new();
                    let dense_zoom = self.cam_zoom > 3.5;
                    let cell_w = if dense_zoom { 1.0 } else { 76.0 };
                    let cell_h = if dense_zoom { 1.0 } else { 30.0 };
                    for (idx, line) in ax.lines.iter().enumerate() {
                        let p0 = self.world_to_screen(line.p0, rect);
                        let p1 = self.world_to_screen(line.p1, rect);
                        let col = if self.show_network_colors {
                            let t = norm_t.get(idx).copied().unwrap_or(0.0);
                            let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
                            Color32::from_rgb(r, g, b)
                        } else {
                            neutral_wire_col
                        };
                        if self.selected_ids.contains(&idx) || self.hovered_feature_idx == Some(idx)
                        {
                            painter.line_segment(
                                [p0, p1],
                                Stroke::new(self.line_width + 3.5, sel_halo_col),
                            );
                        }
                        painter.line_segment([p0, p1], Stroke::new(self.line_width + 0.5, col));
                        if self.show_endpoints {
                            painter.circle_filled(p0, 2.8, node_dot_col);
                            painter.circle_filled(p1, 2.8, node_dot_col);
                        }
                        if self.show_segment_ids {
                            let mid = self.world_to_screen(line.midpoint(), rect);
                            if rect.contains(mid) && (dense_zoom || p0.distance(p1) >= 15.0) {
                                let (t0, t1, lpos) =
                                    offset_label_from_seg(p0, p1, mid, self.line_width + 0.5);
                                let cell = ((lpos.x / cell_w).floor() as i32, (lpos.y / cell_h).floor() as i32);
                                if dense_zoom || occupied_cells.insert(cell) {
                                    let label = format!("L{}", line.id);
                                    let box_w = (label.len() as f32 * 6.0 + 5.0).max(18.0);
                                    let bg_rect = Rect::from_center_size(lpos, Vec2::new(box_w, 13.0));
                                    painter.line_segment([t0, t1], Stroke::new(1.0, pill_border));
                                    painter.rect_filled(bg_rect, 2.5, pill_bg);
                                    painter.rect_stroke(bg_rect, 2.5, Stroke::new(0.8, pill_border));
                                    painter.text(
                                        lpos,
                                        egui::Align2::CENTER_CENTER,
                                        label,
                                        FontId::new(9.5, FontFamily::Monospace),
                                        pill_text_col,
                                    );
                                }
                            }
                        }
                    }
                }
            }
            ActiveMapMode::NaturalRoadsStrokes => {
                if let Some(nr) = &self.strokes_results {
                    let mut occupied_cells: HashSet<(i32, i32)> = HashSet::new();
                    let dense_zoom = self.cam_zoom > 3.5;
                    let cell_w = if dense_zoom { 1.0 } else { 76.0 };
                    let cell_h = if dense_zoom { 1.0 } else { 30.0 };
                    for (idx, st) in nr.strokes.iter().enumerate() {
                        let col = if self.show_network_colors {
                            let t = norm_t.get(idx).copied().unwrap_or(0.0);
                            let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
                            Color32::from_rgb(r, g, b)
                        } else {
                            neutral_wire_col
                        };
                        let is_hi = self.selected_ids.contains(&idx)
                            || self.hovered_feature_idx == Some(idx);
                        for w in st.coords.windows(2) {
                            let p0 = self.world_to_screen(w[0], rect);
                            let p1 = self.world_to_screen(w[1], rect);
                            if is_hi {
                                painter.line_segment(
                                    [p0, p1],
                                    Stroke::new(self.line_width + 3.5, sel_halo_col),
                                );
                            }
                            painter.line_segment([p0, p1], Stroke::new(self.line_width + 0.6, col));
                        }
                        if self.show_endpoints {
                            if let (Some(&f), Some(&l)) = (st.coords.first(), st.coords.last()) {
                                painter.circle_filled(self.world_to_screen(f, rect), 3.0, node_dot_col);
                                painter.circle_filled(self.world_to_screen(l, rect), 3.0, node_dot_col);
                            }
                        }
                        if self.show_segment_ids && st.coords.len() >= 2 {
                            let mid_idx = st.coords.len() / 2;
                            let mid_pt = st.coords[mid_idx];
                            let prev_pt = st.coords[mid_idx.saturating_sub(1)];
                            let p0 = self.world_to_screen(prev_pt, rect);
                            let p1 = self.world_to_screen(mid_pt, rect);
                            let mid = self.world_to_screen(mid_pt, rect);
                            if rect.contains(mid) {
                                let (t0, t1, lpos) =
                                    offset_label_from_seg(p0, p1, mid, self.line_width + 0.6);
                                let cell = ((lpos.x / cell_w).floor() as i32, (lpos.y / cell_h).floor() as i32);
                                if dense_zoom || occupied_cells.insert(cell) {
                                    let label = format!("S{}", st.stroke_id);
                                    let box_w = (label.len() as f32 * 6.0 + 5.0).max(18.0);
                                    let bg_rect = Rect::from_center_size(lpos, Vec2::new(box_w, 13.0));
                                    painter.line_segment([t0, t1], Stroke::new(1.0, pill_border));
                                    painter.rect_filled(bg_rect, 2.5, pill_bg);
                                    painter.rect_stroke(bg_rect, 2.5, Stroke::new(0.8, pill_border));
                                    painter.text(
                                        lpos,
                                        egui::Align2::CENTER_CENTER,
                                        label,
                                        FontId::new(9.5, FontFamily::Monospace),
                                        pill_text_col,
                                    );
                                }
                            }
                        }
                    }
                }
            }
            ActiveMapMode::VgaPointMap => {
                if let Some(vga) = &self.vga_results {
                    if self.show_vga_cells {
                        let half_px = ((vga.cell_size_m * self.cam_zoom * 0.46) as f32).max(1.8);
                        for (idx, pt) in vga.points.iter().enumerate() {
                            let sp = self.world_to_screen(Point2D::new(pt.x, pt.y), rect);
                            if !rect.contains(sp) {
                                continue;
                            }
                            let t = norm_t.get(idx).copied().unwrap_or(0.0);
                            let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
                            let col = Color32::from_rgb(r, g, b);
                            let cell_rect =
                                Rect::from_center_size(sp, Vec2::splat(half_px * 2.0));
                            painter.rect_filled(cell_rect, 0.5, col);
                            if self.selected_ids.contains(&idx)
                                || self.hovered_feature_idx == Some(idx)
                            {
                                painter.rect_stroke(
                                    cell_rect,
                                    0.5,
                                    Stroke::new(1.5, sel_halo_col),
                                );
                            }
                        }
                    }

                    // Optional VGA Inter-Visibility Graph Lines (LOS Edges)
                    if self.show_vga_los_edges {
                        let los_col = if self.dark_theme {
                            Color32::from_rgba_unmultiplied(110, 225, 255, 70)
                        } else {
                            Color32::from_rgba_unmultiplied(18, 95, 185, 80)
                        };
                        let mut roots: Vec<usize> = self.selected_ids.iter().copied().collect();
                        if let Some(h) = self.hovered_feature_idx {
                            if !roots.contains(&h) {
                                roots.push(h);
                            }
                        }
                        if !roots.is_empty() {
                            for u in roots {
                                if u + 1 < vga.row_offsets.len() {
                                    let p0 = self.world_to_screen(
                                        Point2D::new(vga.points[u].x, vga.points[u].y),
                                        rect,
                                    );
                                    let start = vga.row_offsets[u] as usize;
                                    let end = vga.row_offsets[u + 1] as usize;
                                    for &v in &vga.col_indices[start..end] {
                                        if let Some(pt_v) = vga.points.get(v as usize) {
                                            let p1 = self.world_to_screen(
                                                Point2D::new(pt_v.x, pt_v.y),
                                                rect,
                                            );
                                            painter.line_segment([p0, p1], Stroke::new(0.85, los_col));
                                        }
                                    }
                                }
                            }
                        } else {
                            let step_u = (vga.points.len() / 90).max(1);
                            for u in (0..vga.points.len()).step_by(step_u) {
                                if u + 1 < vga.row_offsets.len() {
                                    let p0 = self.world_to_screen(
                                        Point2D::new(vga.points[u].x, vga.points[u].y),
                                        rect,
                                    );
                                    let start = vga.row_offsets[u] as usize;
                                    let end = vga.row_offsets[u + 1] as usize;
                                    let step_e = ((end - start) / 14).max(1);
                                    for idx_e in (start..end).step_by(step_e) {
                                        let v = vga.col_indices[idx_e] as usize;
                                        if let Some(pt_v) = vga.points.get(v) {
                                            let p1 = self.world_to_screen(
                                                Point2D::new(pt_v.x, pt_v.y),
                                                rect,
                                            );
                                            painter.line_segment([p0, p1], Stroke::new(0.65, los_col));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // 2. Render Floorplan Walls / Architectural Boundaries (ONLY in SpatialVisibility domain)
        if self.project_domain == AnalysisDomain::SpatialVisibility
            && self.show_obstacles
            && self.wall_line_width > 0.0
        {
            let wall_col = if self.dark_theme {
                Color32::from_rgba_unmultiplied(220, 228, 240, 215)
            } else {
                Color32::from_rgba_unmultiplied(20, 28, 38, 235)
            };
            for poly in &self.obstacles {
                for w in poly.coords.windows(2) {
                    let p0 = self.world_to_screen(w[0], rect);
                    let p1 = self.world_to_screen(w[1], rect);
                    painter.line_segment([p0, p1], Stroke::new(self.wall_line_width, wall_col));
                }
            }
        }

        // 3. Render Cast 2D Analytical Benedikt Isovists (ONLY in SpatialVisibility domain)
        if self.project_domain == AnalysisDomain::SpatialVisibility
            && (self.show_isovists || self.show_isovist_rays || self.show_isovist_drift)
        {
            for iso in &self.cast_isovists {
                if iso.polygon.len() >= 3 {
                    let orig_s = self.world_to_screen(iso.origin, rect);
                    if self.show_isovists {
                        let fill_col = if self.dark_theme {
                            Color32::from_rgba_unmultiplied(45, 220, 255, 58)
                        } else {
                            Color32::from_rgba_unmultiplied(20, 140, 220, 52)
                        };
                        let mut mesh = egui::Mesh::default();
                        mesh.colored_vertex(orig_s, fill_col);
                        for &pt in &iso.polygon {
                            mesh.colored_vertex(self.world_to_screen(pt, rect), fill_col);
                        }
                        let n = iso.polygon.len() as u32;
                        for i in 0..n {
                            let next = (i + 1) % n;
                            mesh.add_triangle(0, 1 + i, 1 + next);
                        }
                        painter.add(egui::Shape::mesh(mesh));
                    }

                    if self.show_isovist_rays {
                        let ray_col = if self.dark_theme {
                            Color32::from_rgba_unmultiplied(80, 235, 255, 95)
                        } else {
                            Color32::from_rgba_unmultiplied(15, 110, 190, 115)
                        };
                        for &pt in &iso.polygon {
                            let ps = self.world_to_screen(pt, rect);
                            painter.line_segment([orig_s, ps], Stroke::new(0.85, ray_col));
                        }
                    }

                    if self.show_isovists {
                        let mut ring_pts: Vec<Pos2> = iso
                            .polygon
                            .iter()
                            .map(|&pt| self.world_to_screen(pt, rect))
                            .collect();
                        if let Some(&first) = ring_pts.first() {
                            ring_pts.push(first);
                        }
                        let border_col = if self.dark_theme {
                            Color32::from_rgb(55, 235, 255)
                        } else {
                            Color32::from_rgb(10, 105, 180)
                        };
                        painter.add(egui::Shape::line(ring_pts, Stroke::new(1.8, border_col)));
                    }

                    // Drift vector (origin -> centroid)
                    if self.show_isovist_drift {
                        let drift_s = self.world_to_screen(iso.drift_target, rect);
                        let drift_col = if self.dark_theme {
                            Color32::from_rgb(255, 220, 45)
                        } else {
                            Color32::from_rgb(215, 95, 10)
                        };
                        painter.line_segment([orig_s, drift_s], Stroke::new(2.2, drift_col));
                        painter.circle_filled(orig_s, 4.5, sel_halo_col);
                        painter.circle_filled(drift_s, 3.5, drift_col);
                    }
                }
            }
        }

        // 4. Render Gibson Ecological Visual Agents & Trails (ONLY in SpatialVisibility domain, with sub-step interpolation)
        if self.project_domain == AnalysisDomain::SpatialVisibility && self.show_agents_and_trails {
            if let Some(st) = &self.agent_state {
                if self.show_agent_history_trails {
                    let trail_col = if self.dark_theme {
                        Color32::from_rgba_unmultiplied(255, 235, 110, 75)
                    } else {
                        Color32::from_rgba_unmultiplied(195, 85, 15, 115)
                    };
                    for tr in st.finished_trails.iter().rev().take(65) {
                        for w in tr.windows(2) {
                            let p0 = self.world_to_screen(w[0], rect);
                            let p1 = self.world_to_screen(w[1], rect);
                            painter.line_segment(
                                [p0, p1],
                                Stroke::new((self.agent_trail_width * 0.78).max(0.6), trail_col),
                            );
                        }
                    }
                }
                if self.show_agent_active_trails {
                    let active_trail_col = if self.dark_theme {
                        Color32::from_rgba_unmultiplied(255, 255, 255, 175)
                    } else {
                        Color32::from_rgba_unmultiplied(15, 23, 42, 195)
                    };
                    for ag in &st.active_agents {
                        for w in ag.trail.windows(2) {
                            let p0 = self.world_to_screen(w[0], rect);
                            let p1 = self.world_to_screen(w[1], rect);
                            painter.line_segment(
                                [p0, p1],
                                Stroke::new(self.agent_trail_width, active_trail_col),
                            );
                        }
                    }
                }
                if self.show_agent_bodies {
                    let alpha = if self.is_playing_agents {
                        self.agent_step_accum.clamp(0.0, 1.0)
                    } else {
                        1.0
                    };
                    for ag in &st.active_agents {
                        let interp_world = Point2D::new(
                            ag.prev_pos.x + (ag.pos.x - ag.prev_pos.x) * alpha,
                            ag.prev_pos.y + (ag.pos.y - ag.prev_pos.y) * alpha,
                        );
                        let ap = self.world_to_screen(interp_world, rect);
                        let body_fill = if self.dark_theme {
                            Color32::WHITE
                        } else {
                            Color32::from_rgb(220, 38, 38)
                        };
                        let body_stroke = if self.dark_theme {
                            Color32::from_rgb(15, 23, 42)
                        } else {
                            Color32::WHITE
                        };
                        let head_tip = Pos2::new(
                            ap.x + (ag.dir.x as f32) * 7.5,
                            ap.y - (ag.dir.y as f32) * 7.5,
                        );
                        painter.line_segment([ap, head_tip], Stroke::new(1.6, body_fill));
                        painter.circle_filled(ap, 4.0, body_fill);
                        painter.circle_stroke(ap, 4.5, Stroke::new(1.2, body_stroke));
                    }
                }
            }
        }

        // 5. Floating Isovist Indicators Readout (Top-Right of Viewport when Isovist layers are active)
        if self.project_domain == AnalysisDomain::SpatialVisibility
            && (self.show_isovists || self.show_isovist_rays || self.show_isovist_drift)
        {
            if let Some(last_iso) = self.cast_isovists.last() {
                let panel_rect = Rect::from_min_size(
                    Pos2::new(rect.right() - 265.0, rect.top() + 12.0),
                    Vec2::new(253.0, 132.0),
                );
                painter.rect_filled(
                    panel_rect,
                    3.0,
                    Color32::from_rgba_unmultiplied(15, 20, 28, 225),
                );
                painter.rect_stroke(
                    panel_rect,
                    3.0,
                    Stroke::new(1.0, Color32::from_rgb(65, 195, 235)),
                );
                let lines = [
                    format!("ISOVIST INDICATORS (N={})", self.cast_isovists.len()),
                    format!(
                        "Origin      : ({:.2}, {:.2}) m",
                        last_iso.origin.x, last_iso.origin.y
                    ),
                    format!("Area        : {:.2} m²", last_iso.area),
                    format!("Perimeter   : {:.2} m", last_iso.perimeter),
                    format!("Compactness : {:.4}", last_iso.compactness),
                    format!("Occlusivity : {:.2} m", last_iso.occlusivity),
                    format!(
                        "Radial Min/Max: {:.2} / {:.2} m",
                        last_iso.min_radial, last_iso.max_radial
                    ),
                    format!(
                        "Drift Mag/Ang : {:.2}m @ {:.1}°",
                        last_iso.drift_magnitude, last_iso.drift_angle_deg
                    ),
                ];
                for (i, line) in lines.iter().enumerate() {
                    painter.text(
                        Pos2::new(
                            panel_rect.left() + 8.0,
                            panel_rect.top() + 8.0 + (i as f32) * 14.8,
                        ),
                        egui::Align2::LEFT_TOP,
                        line,
                        FontId::new(11.0, FontFamily::Monospace),
                        if i == 0 {
                            Color32::from_rgb(90, 220, 255)
                        } else {
                            Color32::from_rgb(225, 232, 240)
                        },
                    );
                }
            }
        }

        // 5b. Top-Left 5-Step Cleaner & Topology Diagnostic HUD (when Cleaner Diff or Intersection Nodes are toggled)
        if self.project_domain == AnalysisDomain::StreetNetwork
            && (self.show_cleaner_diff || self.show_endpoints)
        {
            if let Some(rep) = &self.cleaner_report {
                let hud_rect = Rect::from_min_size(
                    Pos2::new(rect.left() + 14.0, rect.top() + 12.0),
                    Vec2::new(690.0, 40.0),
                );
                painter.rect_filled(
                    hud_rect,
                    4.0,
                    Color32::from_rgba_unmultiplied(15, 20, 28, 228),
                );
                painter.rect_stroke(
                    hud_rect,
                    4.0,
                    Stroke::new(1.0, Color32::from_rgb(56, 189, 248)),
                );
                let line1 = format!(
                    "5-STEP TOPOLOGY CLEANER AUDIT  |  P90 Horizon: {:.2} m  |  Dangles: {} -> {} (-{} healed)  |  Junctions: {} -> {}",
                    rep.effective_extend_distance,
                    rep.raw_dangles_count,
                    rep.cleaned_dangles_count,
                    rep.raw_dangles_count.saturating_sub(rep.cleaned_dangles_count),
                    rep.raw_junctions_count,
                    rep.cleaned_junctions_count,
                );
                let line2 = format!(
                    "Ray Extensions: {}  |  Lateral T-Snaps: {}  |  Pruned Stubs: {}  |  Components: {} -> {}  |  Legend: Cyan = Welded Junction, Red Ring = Dead-End, Magenta = Healed Raw Dangle",
                    rep.ray_extensions_count,
                    rep.lateral_connectors_count,
                    rep.pruned_micro_stubs_count,
                    rep.raw_components_count,
                    rep.cleaned_components_count,
                );
                painter.text(
                    Pos2::new(hud_rect.left() + 8.0, hud_rect.top() + 5.0),
                    egui::Align2::LEFT_TOP,
                    line1,
                    FontId::new(10.5, FontFamily::Monospace),
                    Color32::from_rgb(125, 211, 252),
                );
                painter.text(
                    Pos2::new(hud_rect.left() + 8.0, hud_rect.top() + 21.0),
                    egui::Align2::LEFT_TOP,
                    line2,
                    FontId::new(9.8, FontFamily::Monospace),
                    Color32::from_rgb(226, 232, 240),
                );
            }
        }

        // 6. Bottom-Left Overlay: Classic UCL depthmapX 11-Stop Spectral Color Bar & Metric Scale Bar
        let stats = SummaryStats::from_slice(&vals);
        let leg_rect = Rect::from_min_size(
            Pos2::new(rect.left() + 14.0, rect.bottom() - 56.0),
            Vec2::new(310.0, 44.0),
        );
        painter.rect_filled(
            leg_rect,
            3.0,
            Color32::from_rgba_unmultiplied(15, 19, 26, 220),
        );
        painter.rect_stroke(
            leg_rect,
            3.0,
            Stroke::new(1.0, Color32::from_rgb(65, 78, 98)),
        );
        painter.text(
            Pos2::new(leg_rect.left() + 8.0, leg_rect.top() + 4.0),
            egui::Align2::LEFT_TOP,
            format!("{} ({})", self.active_attribute, self.color_stretch.as_str()),
            FontId::new(10.5, FontFamily::Proportional),
            Color32::from_rgb(220, 228, 240),
        );
        let bar_left = leg_rect.left() + 8.0;
        let bar_w = leg_rect.width() - 16.0;
        let bar_top = leg_rect.top() + 19.0;
        let steps = 44;
        let step_w = bar_w / (steps as f32);
        for i in 0..steps {
            let t = (i as f64) / ((steps - 1) as f64);
            let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
            let sub = Rect::from_min_size(
                Pos2::new(bar_left + (i as f32) * step_w, bar_top),
                Vec2::new(step_w + 0.8, 9.0),
            );
            painter.rect_filled(sub, 0.0, Color32::from_rgb(r, g, b));
        }
        painter.text(
            Pos2::new(bar_left, bar_top + 10.5),
            egui::Align2::LEFT_TOP,
            format!("{:.3}", stats.min),
            FontId::new(10.0, FontFamily::Monospace),
            Color32::from_rgb(190, 205, 225),
        );
        painter.text(
            Pos2::new(bar_left + bar_w * 0.5, bar_top + 10.5),
            egui::Align2::CENTER_TOP,
            format!("P50: {:.3}", stats.p50),
            FontId::new(10.0, FontFamily::Monospace),
            Color32::from_rgb(190, 205, 225),
        );
        painter.text(
            Pos2::new(bar_left + bar_w, bar_top + 10.5),
            egui::Align2::RIGHT_TOP,
            format!("{:.3}", stats.max),
            FontId::new(10.0, FontFamily::Monospace),
            Color32::from_rgb(190, 205, 225),
        );

        // Metric Scale Bar (Bottom-Right, adaptive from 0.5m architectural to 1000m urban)
        let target_px = 110.0;
        let raw_m = target_px / self.cam_zoom;
        let nice_m = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0]
            .into_iter()
            .min_by(|a, b| {
                (a - raw_m)
                    .abs()
                    .partial_cmp(&(b - raw_m).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(50.0);
        let scale_px = (nice_m * self.cam_zoom) as f32;
        let sb_right = rect.right() - 18.0;
        let sb_left = sb_right - scale_px;
        let sb_y = rect.bottom() - 22.0;
        let sb_col = self.heading_text_color();
        painter.line_segment(
            [Pos2::new(sb_left, sb_y), Pos2::new(sb_right, sb_y)],
            Stroke::new(2.2, sb_col),
        );
        painter.line_segment(
            [Pos2::new(sb_left, sb_y - 4.0), Pos2::new(sb_left, sb_y + 4.0)],
            Stroke::new(2.0, sb_col),
        );
        painter.line_segment(
            [Pos2::new(sb_right, sb_y - 4.0), Pos2::new(sb_right, sb_y + 4.0)],
            Stroke::new(2.0, sb_col),
        );
        let sb_label = if nice_m < 1.0 {
            format!("{:.1} m", nice_m)
        } else {
            format!("{:.0} m", nice_m)
        };
        painter.text(
            Pos2::new(0.5 * (sb_left + sb_right), sb_y - 14.0),
            egui::Align2::CENTER_TOP,
            sb_label,
            FontId::new(10.5, FontFamily::Monospace),
            sb_col,
        );

        // Hover Tooltip
        if let Some(h_idx) = self.hovered_feature_idx {
            let val = vals.get(h_idx).copied().unwrap_or(f64::NAN);
            response.on_hover_ui_at_pointer(|ui| {
                ui.label(
                    RichText::new(format!("Feature #{} ({})", h_idx, self.active_map_mode.label()))
                        .strong(),
                );
                ui.label(
                    RichText::new(format!("{}: {:.4}", self.active_attribute, val)).monospace(),
                );
                if self.active_map_mode == ActiveMapMode::SegmentMapAsa {
                    if let Some(seg) = self.segments.get(h_idx) {
                        ui.label(
                            RichText::new(format!(
                                "Length: {:.2} m | Azimuth: {:.1}° | Level: {:.0}",
                                seg.length_m, seg.azimuth_deg, seg.level
                            ))
                            .monospace()
                            .size(11.0),
                        );
                    }
                } else if self.active_map_mode == ActiveMapMode::NaturalRoadsStrokes {
                    if let Some(nr) = &self.strokes_results {
                        if let Some(st) = nr.strokes.get(h_idx) {
                            let sin_str = match st.sinuosity {
                                Some(s) => format!("{:.4}", s),
                                None => "None (Closed Loop)".to_string(),
                            };
                            ui.label(
                                RichText::new(format!(
                                    "Length: {:.2} m | Sinuosity: {}",
                                    st.length_m, sin_str
                                ))
                                .monospace()
                                .size(11.0),
                            );
                        }
                    }
                }
            });
        }
    }
}

impl SseWorkstationApp {
    fn render_scatter_plot_tab(&mut self, ui: &mut egui::Ui) {
        let accent = self.accent_text_color();
        let heading = self.heading_text_color();
        let attr_keys: Vec<String> = self
            .active_attributes_map()
            .map(|m| m.keys().cloned().collect())
            .unwrap_or_default();

        if !attr_keys.is_empty() {
            if !attr_keys.contains(&self.scatter_x_attr) {
                self.scatter_x_attr = if attr_keys.iter().any(|k| k == "Connectivity") {
                    "Connectivity".to_string()
                } else if attr_keys.iter().any(|k| k == "Stroke_Length") {
                    "Stroke_Length".to_string()
                } else {
                    attr_keys[0].clone()
                };
            }
            if !attr_keys.contains(&self.scatter_y_attr) {
                self.scatter_y_attr = if attr_keys.iter().any(|k| k == "NAIN_Rn") {
                    "NAIN_Rn".to_string()
                } else if attr_keys.iter().any(|k| k == "Integration_HH_n") {
                    "Integration_HH_n".to_string()
                } else if attr_keys.iter().any(|k| k == "Visual_Integration_HH") {
                    "Visual_Integration_HH".to_string()
                } else if attr_keys.iter().any(|k| k == "Stroke_Connectivity") {
                    "Stroke_Connectivity".to_string()
                } else {
                    attr_keys
                        .get(1)
                        .unwrap_or(&attr_keys[0])
                        .clone()
                };
            }
        }

        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("X-Axis Attribute:").strong().color(heading));
            egui::ComboBox::from_id_salt("scatter_x_combo")
                .selected_text(&self.scatter_x_attr)
                .show_ui(ui, |ui| {
                    for k in &attr_keys {
                        ui.selectable_value(&mut self.scatter_x_attr, k.clone(), k);
                    }
                });

            ui.label(RichText::new("Y-Axis Attribute:").strong().color(heading));
            egui::ComboBox::from_id_salt("scatter_y_combo")
                .selected_text(&self.scatter_y_attr)
                .show_ui(ui, |ui| {
                    for k in &attr_keys {
                        ui.selectable_value(&mut self.scatter_y_attr, k.clone(), k);
                    }
                });

            ui.separator();
            if ui
                .button("[Preset: Intelligibility (Connectivity vs NAIN_Rn)]")
                .clicked()
            {
                self.scatter_x_attr = "Connectivity".to_string();
                self.scatter_y_attr = "NAIN_Rn".to_string();
            }
            if ui
                .button("[Preset: Synergy (NAIN_R400m vs NAIN_Rn)]")
                .clicked()
            {
                self.scatter_x_attr = "NAIN_R400m".to_string();
                self.scatter_y_attr = "NAIN_Rn".to_string();
            }
        });

        let x_vals = self
            .active_attributes_map()
            .and_then(|m| m.get(&self.scatter_x_attr))
            .cloned()
            .unwrap_or_default();
        let y_vals = self
            .active_attributes_map()
            .and_then(|m| m.get(&self.scatter_y_attr))
            .cloned()
            .unwrap_or_default();
        let c_vals = self.active_attribute_values();
        let norm_t = self.compute_normalized_t_values(&c_vals);

        let (slope, intercept, pearson_r, r2) = ols_regression(&x_vals, &y_vals);
        let x_stats = SummaryStats::from_slice(&x_vals);
        let y_stats = SummaryStats::from_slice(&y_vals);

        ui.separator();
        ui.horizontal(|ui| {
            ui.label(
                RichText::new(format!(
                    "OLS Regression:  y = {:.5} * x + ({:.5})   |   R² = {:.4}   |   Pearson r = {:.4}   |   N = {}",
                    slope,
                    intercept,
                    r2,
                    pearson_r,
                    x_vals.len().min(y_vals.len())
                ))
                .monospace()
                .strong()
                .color(accent),
            );
        });

        let avail = ui.available_size();
        let (response, painter) = ui.allocate_painter(avail, Sense::click());
        let rect = response.rect;
        painter.rect_filled(
            rect,
            2.0,
            if self.dark_theme {
                Color32::from_rgb(12, 16, 22)
            } else {
                Color32::from_rgb(252, 252, 252)
            },
        );

        let pad_l = 65.0;
        let pad_r = 30.0;
        let pad_t = 28.0;
        let pad_b = 46.0;
        let plot_rect = Rect::from_min_max(
            Pos2::new(rect.left() + pad_l, rect.top() + pad_t),
            Pos2::new(rect.right() - pad_r, rect.bottom() - pad_b),
        );
        if plot_rect.width() <= 20.0 || plot_rect.height() <= 20.0 {
            return;
        }

        painter.rect_stroke(
            plot_rect,
            0.0,
            Stroke::new(1.2, Color32::from_rgb(90, 105, 130)),
        );

        let x_span = (x_stats.max - x_stats.min).max(1e-9);
        let y_span = (y_stats.max - y_stats.min).max(1e-9);

        let to_screen = |vx: f64, vy: f64| -> Pos2 {
            let tx = ((vx - x_stats.min) / x_span).clamp(0.0, 1.0) as f32;
            let ty = ((vy - y_stats.min) / y_span).clamp(0.0, 1.0) as f32;
            Pos2::new(
                plot_rect.left() + tx * plot_rect.width(),
                plot_rect.bottom() - ty * plot_rect.height(),
            )
        };

        // Grid ticks
        for k in 0..=4 {
            let frac = (k as f32) * 0.25;
            let gx = plot_rect.left() + frac * plot_rect.width();
            let gy = plot_rect.bottom() - frac * plot_rect.height();
            painter.line_segment(
                [Pos2::new(gx, plot_rect.top()), Pos2::new(gx, plot_rect.bottom())],
                Stroke::new(0.8, Color32::from_rgba_unmultiplied(80, 95, 120, 65)),
            );
            painter.line_segment(
                [Pos2::new(plot_rect.left(), gy), Pos2::new(plot_rect.right(), gy)],
                Stroke::new(0.8, Color32::from_rgba_unmultiplied(80, 95, 120, 65)),
            );
            let xv = x_stats.min + (frac as f64) * x_span;
            let yv = y_stats.min + (frac as f64) * y_span;
            painter.text(
                Pos2::new(gx, plot_rect.bottom() + 6.0),
                egui::Align2::CENTER_TOP,
                format!("{:.2}", xv),
                FontId::new(10.5, FontFamily::Monospace),
                heading,
            );
            painter.text(
                Pos2::new(plot_rect.left() - 6.0, gy),
                egui::Align2::RIGHT_CENTER,
                format!("{:.2}", yv),
                FontId::new(10.5, FontFamily::Monospace),
                heading,
            );
        }

        // Draw scatter points & handle click selection
        let n = x_vals.len().min(y_vals.len());
        let click_pos = if response.clicked() {
            response.interact_pointer_pos()
        } else {
            None
        };
        let mut clicked_idx = None;
        let mut best_dist = 9.0_f32;

        for i in 0..n {
            let vx = x_vals[i];
            let vy = y_vals[i];
            if !vx.is_finite() || !vy.is_finite() || vx <= -0.999999 || vy <= -0.999999 {
                continue;
            }
            let sp = to_screen(vx, vy);
            let t = norm_t.get(i).copied().unwrap_or(0.5);
            let [r, g, b] = depthmapx_color_rgb(t, self.color_palette);
            let is_sel = self.selected_ids.contains(&i);
            if is_sel {
                painter.circle_stroke(sp, 6.0, Stroke::new(2.0, heading));
            }
            painter.circle_filled(
                sp,
                if is_sel { 4.5 } else { 3.2 },
                Color32::from_rgb(r, g, b),
            );

            if let Some(cp) = click_pos {
                let d = cp.distance(sp);
                if d < best_dist {
                    best_dist = d;
                    clicked_idx = Some(i);
                }
            }
        }

        if let Some(idx) = clicked_idx {
            self.selected_ids.clear();
            self.selected_ids.insert(idx);
        }

        // Draw OLS Regression Line
        let y0 = slope * x_stats.min + intercept;
        let y1 = slope * x_stats.max + intercept;
        let p0 = to_screen(x_stats.min, y0);
        let p1 = to_screen(x_stats.max, y1);
        let reg_col = if self.dark_theme {
            Color32::from_rgb(255, 215, 55)
        } else {
            Color32::from_rgb(200, 45, 20)
        };
        painter.line_segment([p0, p1], Stroke::new(2.2, reg_col));

        // Axis labels
        painter.text(
            Pos2::new(plot_rect.center().x, rect.bottom() - 16.0),
            egui::Align2::CENTER_CENTER,
            format!("X: {}", self.scatter_x_attr),
            FontId::new(12.0, FontFamily::Proportional),
            heading,
        );
        painter.text(
            Pos2::new(plot_rect.left() + 8.0, plot_rect.top() - 14.0),
            egui::Align2::LEFT_CENTER,
            format!("Y: {}", self.scatter_y_attr),
            FontId::new(12.0, FontFamily::Proportional),
            heading,
        );
    }

    fn render_attribute_data_sheet_tab(&mut self, ui: &mut egui::Ui) {
        let heading = self.heading_text_color();
        let Some(attrs_map) = self.active_attributes_map().cloned() else {
            ui.label("No active attributes available.");
            return;
        };
        let col_names: Vec<String> = attrs_map.keys().cloned().collect();
        let row_count = col_names
            .first()
            .and_then(|k| attrs_map.get(k))
            .map(|v| v.len())
            .unwrap_or(0);

        ui.horizontal(|ui| {
            ui.label(
                RichText::new(format!(
                    "ATTRIBUTE DATA SHEET — {} ({} rows × {} columns)",
                    self.active_map_mode.label(),
                    row_count,
                    col_names.len()
                ))
                .strong()
                .color(heading),
            );
            ui.separator();
            ui.label(RichText::new("Click any column header to sort; click any row to highlight on Map.").size(12.0));
        });
        ui.separator();

        let mut row_indices: Vec<usize> = (0..row_count).collect();
        if let Some(sort_vec) = attrs_map.get(&self.table_sort_col) {
            let desc = self.table_sort_desc;
            row_indices.sort_by(|&a, &b| {
                let va = sort_vec.get(a).copied().unwrap_or(f64::NAN);
                let vb = sort_vec.get(b).copied().unwrap_or(f64::NAN);
                let cmp = va.partial_cmp(&vb).unwrap_or(std::cmp::Ordering::Equal);
                if desc {
                    cmp.reverse()
                } else {
                    cmp
                }
            });
        } else if self.table_sort_desc {
            row_indices.reverse();
        }

        let display_cols: Vec<String> = col_names.into_iter().take(12).collect();

        egui::ScrollArea::horizontal()
            .id_salt("datasheet_hscroll")
            .show(ui, |ui| {
                // Header Row
                ui.horizontal(|ui| {
                    let id_sel = self.table_sort_col == "id";
                    if ui
                        .selectable_label(
                            id_sel,
                            self.pill_text(format!("{:<7}", "ID"), id_sel).monospace(),
                        )
                        .clicked()
                    {
                        if self.table_sort_col == "id" {
                            self.table_sort_desc = !self.table_sort_desc;
                        } else {
                            self.table_sort_col = "id".to_string();
                            self.table_sort_desc = false;
                        }
                    }
                    for col in &display_cols {
                        let short = if col.len() > 14 { &col[..14] } else { col };
                        let hdr = format!("{:<14}", short);
                        let col_sel = &self.table_sort_col == col;
                        if ui
                            .selectable_label(col_sel, self.pill_text(hdr, col_sel).monospace())
                            .clicked()
                        {
                            if &self.table_sort_col == col {
                                self.table_sort_desc = !self.table_sort_desc;
                            } else {
                                self.table_sort_col = col.clone();
                                self.table_sort_desc = true;
                            }
                            self.active_attribute = col.clone();
                        }
                    }
                });
                ui.separator();

                let row_h = 19.0;
                egui::ScrollArea::vertical()
                    .id_salt("datasheet_vscroll")
                    .show_rows(ui, row_h, row_indices.len(), |ui, range| {
                        for r_pos in range {
                            let idx = row_indices[r_pos];
                            let is_sel = self.selected_ids.contains(&idx);
                            let mut line = format!("{:<7}", idx);
                            for col in &display_cols {
                                let v = attrs_map
                                    .get(col)
                                    .and_then(|vec| vec.get(idx))
                                    .copied()
                                    .unwrap_or(f64::NAN);
                                if v.is_finite() && v > -0.999999 {
                                    line.push_str(&format!(" {:<14.4}", v));
                                } else {
                                    line.push_str(&format!(" {:<14}", "None/NULL"));
                                }
                            }
                            if ui
                                .selectable_label(
                                    is_sel,
                                    self.pill_text(line, is_sel).monospace().size(11.5),
                                )
                                .clicked()
                            {
                                self.selected_ids.clear();
                                self.selected_ids.insert(idx);
                            }
                        }
                    });
            });
    }

    fn render_topology_audit_tab(&mut self, ui: &mut egui::Ui) {
        let heading = self.heading_text_color();
        let pass_col = if self.dark_theme {
            Color32::from_rgb(80, 230, 125)
        } else {
            Color32::from_rgb(16, 130, 60)
        };
        let fail_col = if self.dark_theme {
            Color32::from_rgb(255, 85, 85)
        } else {
            Color32::from_rgb(195, 25, 25)
        };

        egui::ScrollArea::vertical()
            .id_salt("topology_audit_scroll")
            .show(ui, |ui| {
                ui.label(
                    RichText::new("CSR DUAL GRAPH & TOPOLOGY DIAGNOSTICS")
                        .strong()
                        .color(heading),
                );
                let planar_lines: Vec<Polyline> = if !self.segments.is_empty() {
                    self.segments
                        .iter()
                        .map(|s| Polyline::new(vec![s.p0, s.p1]))
                        .collect()
                } else {
                    self.working_polylines.clone()
                };
                let comp_n = count_connected_components(&planar_lines);
                let junc_n = count_graph_junctions(&planar_lines);
                let dang_n = count_dangling_endpoints(&planar_lines);
                let mem_kb = self
                    .csr_graph
                    .as_ref()
                    .map(|g| g.memory_footprint_bytes() as f64 / 1024.0)
                    .unwrap_or(0.0);
                let first_seg_bearing = self
                    .segments
                    .first()
                    .map(|s| get_bearing(s.p0, s.p1).to_degrees())
                    .unwrap_or(0.0);
                let sample_turn = calculate_turn_deflection(0.0, std::f64::consts::FRAC_PI_2, 1024);

                egui::Grid::new("topo_diag_grid")
                    .num_columns(4)
                    .striped(true)
                    .spacing([18.0, 4.0])
                    .show(ui, |ui| {
                        ui.label(RichText::new("Planarized Segments:").monospace());
                        ui.label(RichText::new(format!("{}", self.segments.len())).monospace().strong());
                        ui.label(RichText::new("CSR Memory Footprint:").monospace());
                        ui.label(RichText::new(format!("{:.2} KB", mem_kb)).monospace().strong());
                        ui.end_row();

                        ui.label(RichText::new("Connected Components:").monospace());
                        ui.label(RichText::new(format!("{}", comp_n)).monospace());
                        ui.label(RichText::new("Graph Junctions (deg>=3):").monospace());
                        ui.label(RichText::new(format!("{}", junc_n)).monospace());
                        ui.end_row();

                        ui.label(RichText::new("Dangling Endpoints (deg==1):").monospace());
                        ui.label(RichText::new(format!("{}", dang_n)).monospace());
                        ui.label(RichText::new("Grade-Separated Unlinks:").monospace());
                        ui.label(RichText::new(format!("{}", self.grade_unlinks.len())).monospace());
                        ui.end_row();

                        ui.label(RichText::new("Ref Seg #0 Bearing:").monospace());
                        ui.label(RichText::new(format!("{:.2}°", first_seg_bearing)).monospace());
                        ui.label(RichText::new("90° Turn Tulip Weight:").monospace());
                        ui.label(RichText::new(format!("{:.4}", sample_turn)).monospace());
                        ui.end_row();
                    });

                ui.separator();
                ui.label(
                    RichText::new("5-STEP P90 ADAPTIVE WATERTIGHT CLEANER REPORT")
                        .strong()
                        .color(heading),
                );
                if let Some(rep) = &self.cleaner_report {
                    egui::Grid::new("cleaner_rep_grid")
                        .num_columns(4)
                        .striped(true)
                        .spacing([18.0, 4.0])
                        .show(ui, |ui| {
                            ui.label(RichText::new("Mode / P90 Horizon:").monospace());
                            ui.label(
                                RichText::new(format!(
                                    "{} ({:.2} m)",
                                    rep.mode, rep.effective_extend_distance
                                ))
                                .monospace(),
                            );
                            ui.label(RichText::new("Elapsed Time:").monospace());
                            ui.label(RichText::new(format!("{:.2} ms", rep.elapsed_ms)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Dangles (Raw -> Cleaned):").monospace());
                            ui.label(
                                RichText::new(format!(
                                    "{} -> {}",
                                    rep.raw_dangles_count, rep.cleaned_dangles_count
                                ))
                                .monospace()
                                .strong(),
                            );
                            ui.label(RichText::new("Junctions (Raw -> Cleaned):").monospace());
                            ui.label(
                                RichText::new(format!(
                                    "{} -> {}",
                                    rep.raw_junctions_count, rep.cleaned_junctions_count
                                ))
                                .monospace()
                                .strong(),
                            );
                            ui.end_row();

                            ui.label(RichText::new("Forward Ray Extensions:").monospace());
                            ui.label(RichText::new(format!("{}", rep.ray_extensions_count)).monospace());
                            ui.label(RichText::new("Lateral T-Connectors:").monospace());
                            ui.label(
                                RichText::new(format!("{}", rep.lateral_connectors_count)).monospace(),
                            );
                            ui.end_row();

                            ui.label(RichText::new("Pruned Micro-Stubs:").monospace());
                            ui.label(
                                RichText::new(format!("{}", rep.pruned_micro_stubs_count)).monospace(),
                            );
                            ui.label(RichText::new("Length Conservation Delta:").monospace());
                            ui.label(
                                RichText::new(format!("{:+.2}%", rep.length_delta_pct)).monospace(),
                            );
                            ui.end_row();
                        });
                } else {
                    if ui
                        .button(self.btn_text("[Run 5-Step P90 Adaptive Watertight Cleaner Now]"))
                        .clicked()
                    {
                        self.run_cleaner_5step();
                    }
                }

                ui.separator();
                ui.label(
                    RichText::new("MATHEMATICAL PARITY VERIFICATION SUITE (UCL depthmapX v0.8.0)")
                        .strong()
                        .color(heading),
                );
                egui::Grid::new("math_verif_grid")
                    .num_columns(5)
                    .striped(true)
                    .spacing([12.0, 5.0])
                    .show(ui, |ui| {
                        for row in &self.verification_rows {
                            ui.label(RichText::new(row.code).monospace().strong());
                            ui.label(RichText::new(row.title).monospace());
                            ui.label(RichText::new(&row.expected).monospace().size(11.0));
                            ui.label(RichText::new(&row.observed).monospace().size(11.0));
                            ui.label(
                                RichText::new(if row.passed { "[PASS]" } else { "[FAIL]" })
                                    .monospace()
                                    .strong()
                                    .color(if row.passed { pass_col } else { fail_col }),
                            );
                            ui.end_row();
                        }
                    });

                ui.separator();
                ui.label(
                    RichText::new("SESSION AUDIT LOG")
                        .strong()
                        .color(heading),
                );
                for entry in self.audit_log_entries.iter().rev().take(25) {
                    ui.label(RichText::new(entry).monospace().size(11.5));
                }
            });
    }

    fn render_modals(&mut self, ctx: &egui::Context) {
        let accent = self.accent_text_color();
        let heading = self.heading_text_color();
        let pass_col = if self.dark_theme {
            Color32::from_rgb(80, 230, 125)
        } else {
            Color32::from_rgb(16, 130, 60)
        };
        let fail_col = if self.dark_theme {
            Color32::from_rgb(255, 85, 85)
        } else {
            Color32::from_rgb(195, 25, 25)
        };

        // 1. OSM Overpass Downloader Modal (Interactive Slippy Map + Polygon/Rectangle/File Boundary)
        if self.show_overpass_modal {
            self.osm_tile_mgr.poll_loaded(ctx);
            let mut open = self.show_overpass_modal;
            let mut trigger_import_boundary = false;
            let mut trigger_fit_boundary = false;
            let mut trigger_search = false;
            let mut trigger_fetch = false;

            egui::Window::new("Download Live Street Network via OSM Overpass API")
                .open(&mut open)
                .resizable(true)
                .default_size([920.0, 650.0])
                .min_size([760.0, 460.0])
                .show(ctx, |ui| {
                    // Row 1: Reference Urban Presets + Basemap Selector (Zero API Key)
                    ui.horizontal_wrapped(|ui| {
                        ui.spacing_mut().interact_size.y = 23.0;
                        ui.label(RichText::new("Reference City:").strong().color(heading));
                        for (idx, name, s, w, n, e) in [
                            (0, "Bangkok (Sathorn/Silom)", 13.7200, 100.5200, 13.7320, 100.5350),
                            (1, "London (UCL Bloomsbury)", 51.5190, -0.1380, 51.5290, -0.1220),
                            (2, "Manhattan (Midtown Grid)", 40.7480, -73.9920, 40.7580, -73.9760),
                            (3, "Barcelona (Eixample)", 41.3880, 2.1550, 41.3980, 2.1720),
                        ] {
                            let is_sel = self.overpass_city_idx == idx && self.osm_polygon_lat_lon.is_empty();
                            if ui
                                .selectable_label(is_sel, self.pill_text(name, is_sel))
                                .clicked()
                            {
                                self.overpass_city_idx = idx;
                                self.overpass_south = s;
                                self.overpass_west = w;
                                self.overpass_north = n;
                                self.overpass_east = e;
                                self.osm_polygon_lat_lon.clear();
                                self.osm_polygon_closed = false;
                                self.osm_boundary_source = format!("Preset BBox ({})", name);
                                self.osm_search_status.clear();
                                trigger_fit_boundary = true;
                            }
                        }
                    });

                    ui.horizontal_wrapped(|ui| {
                        ui.spacing_mut().interact_size.y = 23.0;
                        ui.label(RichText::new("Basemap (No API Key):").strong().color(heading));
                        for bm in [
                            OsmBasemapStyle::OsmStandard,
                            OsmBasemapStyle::EsriSatellite,
                            OsmBasemapStyle::EsriHybrid,
                        ] {
                            let is_sel = self.osm_basemap == bm;
                            if ui
                                .selectable_label(is_sel, self.pill_text(bm.label(), is_sel))
                                .clicked()
                            {
                                self.osm_basemap = bm;
                            }
                        }
                    });

                    // Row 2: Boundary Drawing Tools & File Import (.gpkg / .dxf / .geojson)
                    ui.horizontal_wrapped(|ui| {
                        ui.spacing_mut().interact_size.y = 24.0;
                        ui.label(RichText::new("Boundary Tool:").strong().color(heading));
                        for (tool, lbl, tip) in [
                            (
                                OsmDrawTool::PanNavigate,
                                "[Pan / Navigate]",
                                "Drag map with left/right mouse button to pan; scroll wheel to zoom",
                            ),
                            (
                                OsmDrawTool::DrawRectangle,
                                "[Draw Rectangle BBox]",
                                "Left-click and drag on map to draw a rectangular bounding box (right-drag to pan)",
                            ),
                            (
                                OsmDrawTool::DrawPolygon,
                                "[Draw Polygon Boundary]",
                                "Left-click vertices on map to draw a custom polygon boundary; click first vertex or [Close Polygon] to finish",
                            ),
                        ] {
                            let is_sel = self.osm_draw_tool == tool;
                            if ui
                                .selectable_label(is_sel, self.pill_text(lbl, is_sel))
                                .on_hover_text(tip)
                                .clicked()
                            {
                                self.osm_draw_tool = tool;
                            }
                        }

                        if self.osm_polygon_lat_lon.len() >= 3 && !self.osm_polygon_closed {
                            if ui
                                .button(self.btn_text("[Close Polygon]"))
                                .on_hover_text("Close the active polygon boundary")
                                .clicked()
                            {
                                self.osm_polygon_closed = true;
                                self.osm_boundary_source = format!(
                                    "Drawn Polygon ({} vertices)",
                                    self.osm_polygon_lat_lon.len()
                                );
                            }
                        }
                        if !self.osm_polygon_lat_lon.is_empty() {
                            if ui
                                .button(self.btn_text("[Clear Polygon -> Use BBox]"))
                                .clicked()
                            {
                                self.osm_polygon_lat_lon.clear();
                                self.osm_polygon_closed = false;
                                self.osm_boundary_source = "Rectangle BBox".to_string();
                            }
                        }

                        ui.separator();
                        if ui
                            .button(self.btn_text("[Import Boundary (.gpkg / .dxf / .geojson)...]"))
                            .on_hover_text("Import polygon/polyline boundary from OGC GeoPackage (.gpkg), AutoCAD (.dxf), or GeoJSON (.geojson/.json)")
                            .clicked()
                        {
                            trigger_import_boundary = true;
                        }
                    });

                    ui.add_space(2.0);

                    // Interactive Slippy Map Viewport (Dynamically resizes both horizontally & vertically)
                    let bottom_reserved = if self.osm_search_status.is_empty() {
                        82.0
                    } else {
                        102.0
                    };
                    let map_w = ui.available_width().max(720.0);
                    let map_h = (ui.available_height() - bottom_reserved).max(240.0);
                    self.last_osm_map_size = (map_w as f64, map_h as f64);
                    let (map_rect, map_resp) =
                        ui.allocate_exact_size(Vec2::new(map_w, map_h), Sense::click_and_drag());

                    let z_int = (self.osm_map_zoom.round() as i32).clamp(2, 18);
                    let zf = z_int as f64;
                    let world_tiles = 1_i32 << z_int;

                    // Convert screen Pos2 <-> (lat, lon)
                    let center_wx = slippy_lon_to_world_px(self.osm_map_center_lon, zf);
                    let center_wy = slippy_lat_to_world_px(self.osm_map_center_lat, zf);
                    let screen_to_lat_lon = |pos: Pos2, c_wx: f64, c_wy: f64| -> (f64, f64) {
                        let wx = c_wx + (pos.x - map_rect.center().x) as f64;
                        let wy = c_wy + (pos.y - map_rect.center().y) as f64;
                        (
                            slippy_world_px_to_lat(wy, zf),
                            slippy_world_px_to_lon(wx, zf),
                        )
                    };
                    let lat_lon_to_screen = |lat: f64, lon: f64, c_wx: f64, c_wy: f64| -> Pos2 {
                        let wx = slippy_lon_to_world_px(lon, zf);
                        let wy = slippy_lat_to_world_px(lat, zf);
                        Pos2::new(
                            map_rect.center().x + (wx - c_wx) as f32,
                            map_rect.center().y + (wy - c_wy) as f32,
                        )
                    };

                    // Update live cursor (lat, lon) & gradual 1-level-per-notch mouse wheel zoom
                    if let Some(hover_pos) = map_resp.hover_pos() {
                        self.osm_cursor_lat_lon =
                            Some(screen_to_lat_lon(hover_pos, center_wx, center_wy));

                        let raw_scroll_y = ui.input(|i| i.raw_scroll_delta.y);
                        if raw_scroll_y.abs() > 0.1 {
                            self.osm_zoom_scroll_accum += (raw_scroll_y as f64) / 36.0;
                        }
                        let cooldown_ok = self
                            .last_osm_zoom_instant
                            .map_or(true, |t| t.elapsed() >= Duration::from_millis(95));
                        if self.osm_zoom_scroll_accum.abs() >= 0.75 && cooldown_ok {
                            let (anchor_lat, anchor_lon) =
                                screen_to_lat_lon(hover_pos, center_wx, center_wy);
                            let step = if self.osm_zoom_scroll_accum > 0.0 {
                                1.0
                            } else {
                                -1.0
                            };
                            self.osm_zoom_scroll_accum = 0.0;
                            self.last_osm_zoom_instant = Some(Instant::now());
                            let new_z = (z_int as f64 + step).clamp(2.0, 18.0);
                            if (new_z - zf).abs() > 0.1 {
                                self.osm_map_zoom = new_z;
                                let anchor_wx_new = slippy_lon_to_world_px(anchor_lon, new_z);
                                let anchor_wy_new = slippy_lat_to_world_px(anchor_lat, new_z);
                                let new_c_wx =
                                    anchor_wx_new - (hover_pos.x - map_rect.center().x) as f64;
                                let new_c_wy =
                                    anchor_wy_new - (hover_pos.y - map_rect.center().y) as f64;
                                self.osm_map_center_lon = slippy_world_px_to_lon(new_c_wx, new_z);
                                self.osm_map_center_lat = slippy_world_px_to_lat(new_c_wy, new_z);
                            }
                        }
                    } else {
                        self.osm_cursor_lat_lon = None;
                        self.osm_zoom_scroll_accum = 0.0;
                    }

                    // Recompute center world px after potential zoom
                    let z_int = (self.osm_map_zoom.round() as i32).clamp(2, 18);
                    let zf = z_int as f64;
                    let center_wx = slippy_lon_to_world_px(self.osm_map_center_lon, zf);
                    let center_wy = slippy_lat_to_world_px(self.osm_map_center_lat, zf);

                    // Handle map panning vs boundary drawing
                    let is_pan_drag = map_resp.dragged_by(egui::PointerButton::Secondary)
                        || map_resp.dragged_by(egui::PointerButton::Middle)
                        || (self.osm_draw_tool == OsmDrawTool::PanNavigate
                            && map_resp.dragged_by(egui::PointerButton::Primary));

                    if is_pan_drag {
                        let delta = map_resp.drag_delta();
                        let new_c_wx = center_wx - delta.x as f64;
                        let new_c_wy = center_wy - delta.y as f64;
                        self.osm_map_center_lon = slippy_world_px_to_lon(new_c_wx, zf);
                        self.osm_map_center_lat = slippy_world_px_to_lat(new_c_wy, zf);
                    } else if self.osm_draw_tool == OsmDrawTool::DrawRectangle {
                        if map_resp.drag_started_by(egui::PointerButton::Primary) {
                            if let Some(pos) = map_resp.interact_pointer_pos() {
                                let pt = screen_to_lat_lon(pos, center_wx, center_wy);
                                self.osm_rect_drag_start = Some(pt);
                                self.osm_polygon_lat_lon.clear();
                                self.osm_polygon_closed = false;
                            }
                        }
                        if map_resp.dragged_by(egui::PointerButton::Primary) {
                            if let (Some((lat0, lon0)), Some(pos)) =
                                (self.osm_rect_drag_start, map_resp.interact_pointer_pos())
                            {
                                let (lat1, lon1) = screen_to_lat_lon(pos, center_wx, center_wy);
                                self.overpass_south = lat0.min(lat1);
                                self.overpass_north = lat0.max(lat1);
                                self.overpass_west = lon0.min(lon1);
                                self.overpass_east = lon0.max(lon1);
                                self.osm_boundary_source =
                                    "Drawn Rectangle BBox".to_string();
                            }
                        }
                        if map_resp.drag_stopped() {
                            self.osm_rect_drag_start = None;
                        }
                    } else if self.osm_draw_tool == OsmDrawTool::DrawPolygon {
                        if map_resp.clicked_by(egui::PointerButton::Primary) {
                            if let Some(pos) = map_resp.interact_pointer_pos() {
                                let (lat, lon) = screen_to_lat_lon(pos, center_wx, center_wy);
                                if self.osm_polygon_closed {
                                    self.osm_polygon_lat_lon.clear();
                                    self.osm_polygon_closed = false;
                                }
                                // Check if clicking near first vertex to close polygon
                                let mut closed_now = false;
                                if self.osm_polygon_lat_lon.len() >= 3 {
                                    let (f_lat, f_lon) = self.osm_polygon_lat_lon[0];
                                    let f_scr = lat_lon_to_screen(f_lat, f_lon, center_wx, center_wy);
                                    if f_scr.distance(pos) <= 12.0 {
                                        self.osm_polygon_closed = true;
                                        closed_now = true;
                                    }
                                }
                                if !closed_now {
                                    self.osm_polygon_lat_lon.push((lat, lon));
                                }
                                if self.osm_polygon_lat_lon.len() >= 2 {
                                    let mut min_lat = f64::INFINITY;
                                    let mut max_lat = f64::NEG_INFINITY;
                                    let mut min_lon = f64::INFINITY;
                                    let mut max_lon = f64::NEG_INFINITY;
                                    for &(la, lo) in &self.osm_polygon_lat_lon {
                                        min_lat = min_lat.min(la);
                                        max_lat = max_lat.max(la);
                                        min_lon = min_lon.min(lo);
                                        max_lon = max_lon.max(lo);
                                    }
                                    self.overpass_south = min_lat;
                                    self.overpass_north = max_lat;
                                    self.overpass_west = min_lon;
                                    self.overpass_east = max_lon;
                                    self.osm_boundary_source = format!(
                                        "Drawn Polygon ({} vertices{})",
                                        self.osm_polygon_lat_lon.len(),
                                        if self.osm_polygon_closed { ", closed" } else { ", open" }
                                    );
                                }
                            }
                        }
                        if (map_resp.double_clicked()
                            || map_resp.clicked_by(egui::PointerButton::Secondary))
                            && self.osm_polygon_lat_lon.len() >= 3
                        {
                            self.osm_polygon_closed = true;
                            self.osm_boundary_source = format!(
                                "Drawn Polygon ({} vertices, closed)",
                                self.osm_polygon_lat_lon.len()
                            );
                        }
                    }

                    // Render Slippy Map Tiles inside clipped painter
                    let painter = ui.painter_at(map_rect);
                    painter.rect_filled(map_rect, 4.0, Color32::from_rgb(16, 22, 30));

                    let center_wx = slippy_lon_to_world_px(self.osm_map_center_lon, zf);
                    let center_wy = slippy_lat_to_world_px(self.osm_map_center_lat, zf);
                    let min_wx = center_wx - (map_rect.width() as f64) * 0.5;
                    let max_wx = center_wx + (map_rect.width() as f64) * 0.5;
                    let min_wy = center_wy - (map_rect.height() as f64) * 0.5;
                    let max_wy = center_wy + (map_rect.height() as f64) * 0.5;

                    let tx_start = (min_wx / 256.0).floor() as i32;
                    let tx_end = (max_wx / 256.0).floor() as i32;
                    let ty_start = ((min_wy / 256.0).floor() as i32).max(0);
                    let ty_end = ((max_wy / 256.0).floor() as i32).min(world_tiles - 1);

                    let uv_full = Rect::from_min_max(Pos2::new(0.0, 0.0), Pos2::new(1.0, 1.0));
                    for ty in ty_start..=ty_end {
                        for tx in tx_start..=tx_end {
                            let tile_x_scr =
                                map_rect.center().x + ((tx as f64 * 256.0) - center_wx) as f32;
                            let tile_y_scr =
                                map_rect.center().y + ((ty as f64 * 256.0) - center_wy) as f32;
                            let tile_rect = Rect::from_min_size(
                                Pos2::new(tile_x_scr, tile_y_scr),
                                Vec2::new(256.0, 256.0),
                            );
                            let tx_wrapped = tx.rem_euclid(world_tiles);
                            let key = SlippyTileKey {
                                style: self.osm_basemap,
                                z: z_int,
                                x: tx_wrapped,
                                y: ty,
                            };
                            if let Some(tex) = self.osm_tile_mgr.get_or_request(ctx, key) {
                                painter.image(tex.id(), tile_rect, uv_full, Color32::WHITE);
                            } else {
                                painter.rect_stroke(
                                    tile_rect,
                                    0.0,
                                    Stroke::new(0.5, Color32::from_rgb(45, 58, 76)),
                                );
                            }
                        }
                    }

                    // Render Active Boundary Overlay (Polygon or Rectangle BBox)
                    if self.osm_polygon_lat_lon.len() >= 2 {
                        let pts_scr: Vec<Pos2> = self
                            .osm_polygon_lat_lon
                            .iter()
                            .map(|&(la, lo)| lat_lon_to_screen(la, lo, center_wx, center_wy))
                            .collect();

                        // Also draw outer BBox dashed guide
                        let p_nw = lat_lon_to_screen(
                            self.overpass_north,
                            self.overpass_west,
                            center_wx,
                            center_wy,
                        );
                        let p_se = lat_lon_to_screen(
                            self.overpass_south,
                            self.overpass_east,
                            center_wx,
                            center_wy,
                        );
                        let bbox_rect = Rect::from_two_pos(p_nw, p_se);
                        painter.rect_stroke(
                            bbox_rect,
                            0.0,
                            Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 210, 80, 140)),
                        );

                        // Fill triangle fan around centroid for translucent polygon preview
                        if pts_scr.len() >= 3 {
                            let mut cx = 0.0_f32;
                            let mut cy = 0.0_f32;
                            for p in &pts_scr {
                                cx += p.x;
                                cy += p.y;
                            }
                            let c_pos = Pos2::new(
                                cx / (pts_scr.len() as f32),
                                cy / (pts_scr.len() as f32),
                            );
                            let fill_col = Color32::from_rgba_unmultiplied(0, 200, 255, 48);
                            for i in 0..pts_scr.len() {
                                let p0 = pts_scr[i];
                                let p1 = pts_scr[(i + 1) % pts_scr.len()];
                                painter.add(egui::Shape::convex_polygon(
                                    vec![c_pos, p0, p1],
                                    fill_col,
                                    Stroke::NONE,
                                ));
                            }
                        }

                        let stroke_col = if self.osm_polygon_closed {
                            Color32::from_rgb(0, 235, 255)
                        } else {
                            Color32::from_rgb(255, 205, 55)
                        };
                        for i in 0..(pts_scr.len() - 1) {
                            painter.line_segment(
                                [pts_scr[i], pts_scr[i + 1]],
                                Stroke::new(2.5, stroke_col),
                            );
                        }
                        if self.osm_polygon_closed && pts_scr.len() >= 3 {
                            painter.line_segment(
                                [pts_scr[pts_scr.len() - 1], pts_scr[0]],
                                Stroke::new(2.5, stroke_col),
                            );
                        } else if let Some(hover_pos) = map_resp.hover_pos() {
                            if self.osm_draw_tool == OsmDrawTool::DrawPolygon {
                                painter.line_segment(
                                    [pts_scr[pts_scr.len() - 1], hover_pos],
                                    Stroke::new(1.5, Color32::from_rgb(255, 220, 100)),
                                );
                            }
                        }

                        // Draw vertex handles (cap at 250 handles for huge imported polygons)
                        let step = (pts_scr.len() / 200).max(1);
                        for (vi, &p) in pts_scr.iter().enumerate().step_by(step) {
                            let rad = if vi == 0 { 5.0 } else { 3.5 };
                            let vcol = if vi == 0 {
                                Color32::from_rgb(80, 255, 140)
                            } else {
                                Color32::WHITE
                            };
                            painter.circle_filled(p, rad, vcol);
                            painter.circle_stroke(p, rad, Stroke::new(1.2, Color32::BLACK));
                        }
                    } else {
                        // Draw Rectangle BBox
                        let p_nw = lat_lon_to_screen(
                            self.overpass_north,
                            self.overpass_west,
                            center_wx,
                            center_wy,
                        );
                        let p_se = lat_lon_to_screen(
                            self.overpass_south,
                            self.overpass_east,
                            center_wx,
                            center_wy,
                        );
                        let bbox_rect = Rect::from_two_pos(p_nw, p_se);
                        painter.rect_filled(
                            bbox_rect,
                            0.0,
                            Color32::from_rgba_unmultiplied(0, 185, 255, 48),
                        );
                        painter.rect_stroke(
                            bbox_rect,
                            0.0,
                            Stroke::new(2.4, Color32::from_rgb(0, 225, 255)),
                        );
                        for corner in [
                            bbox_rect.left_top(),
                            bbox_rect.right_top(),
                            bbox_rect.left_bottom(),
                            bbox_rect.right_bottom(),
                        ] {
                            painter.circle_filled(corner, 4.0, Color32::WHITE);
                            painter.circle_stroke(
                                corner,
                                4.0,
                                Stroke::new(1.2, Color32::from_rgb(0, 120, 215)),
                            );
                        }
                    }

                    // Map Outer Border
                    painter.rect_stroke(
                        map_rect,
                        4.0,
                        Stroke::new(1.2, Color32::from_rgb(70, 88, 114)),
                    );

                    // Top-Left HUD Overlay Badge
                    let hud_tl = Rect::from_min_size(
                        map_rect.min + Vec2::new(8.0, 8.0),
                        Vec2::new(475.0, 26.0),
                    );
                    painter.rect_filled(
                        hud_tl,
                        4.0,
                        Color32::from_rgba_unmultiplied(12, 16, 22, 215),
                    );
                    let tool_hint = match self.osm_draw_tool {
                        OsmDrawTool::PanNavigate => "Mode: Pan/Navigate (Drag to pan, Scroll to zoom)",
                        OsmDrawTool::DrawRectangle => "Mode: Draw Rectangle (Left-drag box, Right-drag pan)",
                        OsmDrawTool::DrawPolygon => "Mode: Draw Polygon (Left-click vertices, Right-click/Dbl-click close)",
                    };
                    painter.text(
                        hud_tl.left_center() + Vec2::new(8.0, 0.0),
                        egui::Align2::LEFT_CENTER,
                        format!("{}  |  Zoom z={}", tool_hint, z_int),
                        FontId::proportional(11.5),
                        Color32::from_rgb(225, 235, 248),
                    );

                    // Bottom-Left Cursor & Boundary Status HUD
                    let hud_bl = Rect::from_min_size(
                        Pos2::new(map_rect.min.x + 8.0, map_rect.max.y - 30.0),
                        Vec2::new(460.0, 22.0),
                    );
                    painter.rect_filled(
                        hud_bl,
                        3.0,
                        Color32::from_rgba_unmultiplied(12, 16, 22, 215),
                    );
                    let cursor_txt = if let Some((cla, clo)) = self.osm_cursor_lat_lon {
                        format!("Cursor: {:.5}, {:.5}  |  {}", cla, clo, self.osm_boundary_source)
                    } else {
                        format!("Boundary: {}", self.osm_boundary_source)
                    };
                    painter.text(
                        hud_bl.left_center() + Vec2::new(8.0, 0.0),
                        egui::Align2::LEFT_CENTER,
                        cursor_txt,
                        FontId::proportional(11.0),
                        Color32::from_rgb(135, 210, 255),
                    );

                    // Bottom-Right Attribution Badge
                    let attr_str = self.osm_basemap.attribution();
                    let hud_br = Rect::from_min_size(
                        Pos2::new(map_rect.max.x - 365.0, map_rect.max.y - 28.0),
                        Vec2::new(357.0, 20.0),
                    );
                    painter.rect_filled(
                        hud_br,
                        3.0,
                        Color32::from_rgba_unmultiplied(12, 16, 22, 200),
                    );
                    painter.text(
                        hud_br.right_center() - Vec2::new(8.0, 0.0),
                        egui::Align2::RIGHT_CENTER,
                        attr_str,
                        FontId::proportional(10.5),
                        Color32::from_rgb(200, 210, 225),
                    );

                    ui.add_space(4.0);

                    // Row 3: Zoom Controls + Location / Lat,Lon Search Bar + Network Mode
                    ui.horizontal(|ui| {
                        ui.spacing_mut().interact_size.y = 24.0;
                        if ui.button(self.btn_text("[Zoom +]")).clicked() {
                            self.osm_map_zoom = (self.osm_map_zoom + 1.0).clamp(2.0, 18.0);
                        }
                        if ui.button(self.btn_text("[Zoom -]")).clicked() {
                            self.osm_map_zoom = (self.osm_map_zoom - 1.0).clamp(2.0, 18.0);
                        }
                        if ui
                            .button(self.btn_text("[Fit Map to Boundary]"))
                            .clicked()
                        {
                            trigger_fit_boundary = true;
                        }
                        ui.separator();
                        ui.label(RichText::new("Search:").strong().color(heading));
                        let search_w = (ui.available_width() - 215.0).max(160.0);
                        let search_resp = ui.add(
                            egui::TextEdit::singleline(&mut self.osm_search_query)
                                .desired_width(search_w)
                                .hint_text(
                                    "Lat, Lon (13.746, 100.534) or Location (e.g. Siam Square, Bangkok)",
                                ),
                        );
                        let enter_pressed =
                            search_resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                        if ui
                            .button(self.btn_text("[Search]"))
                            .on_hover_text("Search by 'lat, lon' coordinates or place/city name via OpenStreetMap Nominatim (Zero API Key)")
                            .clicked()
                            || enter_pressed
                        {
                            trigger_search = true;
                        }
                        ui.separator();
                        ui.label("Mode:");
                        egui::ComboBox::from_id_salt("overpass_mode_combo")
                            .selected_text(&self.overpass_mode)
                            .width(72.0)
                            .show_ui(ui, |ui| {
                                for m in ["walk", "drive", "all"] {
                                    ui.selectable_value(
                                        &mut self.overpass_mode,
                                        m.to_string(),
                                        m,
                                    );
                                }
                            });
                    });

                    if !self.osm_search_status.is_empty() {
                        ui.label(
                            RichText::new(&self.osm_search_status)
                                .size(11.0)
                                .color(accent),
                        );
                    }

                    ui.separator();
                    let is_job_running = self
                        .osm_pipeline_job
                        .as_ref()
                        .map_or(false, |j| j.rx.is_some());
                    let fetch_label = if is_job_running {
                        "[OSM Pipeline Running in Background... Click to View Progress]".to_string()
                    } else if self.osm_polygon_lat_lon.len() >= 3 {
                        format!(
                            "[Fetch Overpass Street Network inside Polygon ({} pts) -> Project UTM -> Clean -> Run ASA]",
                            self.osm_polygon_lat_lon.len()
                        )
                    } else {
                        "[Fetch Overpass Street Network inside Rectangle BBox -> Project UTM -> Clean -> Run ASA]"
                            .to_string()
                    };
                    if ui
                        .add_sized(
                            [ui.available_width(), 32.0],
                            egui::Button::new(self.btn_text(fetch_label).size(13.5)),
                        )
                        .clicked()
                    {
                        if is_job_running {
                            self.show_osm_pipeline_modal = true;
                        } else {
                            trigger_fetch = true;
                        }
                    }
                });

            self.show_overpass_modal = open;
            if trigger_import_boundary {
                self.dialog_import_osm_boundary_file();
            }
            if trigger_fit_boundary {
                let (vw, vh) = self.last_osm_map_size;
                self.fit_osm_map_to_current_boundary(vw.max(600.0), vh.max(260.0));
            }
            if trigger_search {
                self.run_osm_location_search();
            }
            if trigger_fetch {
                self.start_osm_pipeline_job();
            }
        }

        // 1B. Live OSM Pipeline Progress Popup Window (Fetch -> Filter/Clip -> Project UTM -> 5-Step Cleaner -> ASA)
        if self.show_osm_pipeline_modal {
            let mut open = self.show_osm_pipeline_modal;
            let mut request_cancel = false;
            let mut request_close = false;

            egui::Window::new("Live OSM Overpass -> UTM -> Cleaner -> ASA Pipeline")
                .open(&mut open)
                .resizable(true)
                .default_size([640.0, 410.0])
                .min_size([540.0, 340.0])
                .show(ctx, |ui| {
                    if let Some(job) = &self.osm_pipeline_job {
                        if let Ok(prog) = job.progress.lock() {
                            let elapsed_s = prog
                                .finished_elapsed_s
                                .unwrap_or_else(|| prog.started_at.elapsed().as_secs_f64());

                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new(format!("Boundary: {}", self.osm_boundary_source))
                                        .strong()
                                        .color(heading),
                                );
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        ui.label(
                                            RichText::new(format!("Elapsed: {:.2} s", elapsed_s))
                                                .monospace()
                                                .strong()
                                                .color(accent),
                                        );
                                    },
                                );
                            });

                            ui.add_space(4.0);
                            ui.add(
                                egui::ProgressBar::new((prog.progress_pct / 100.0).clamp(0.0, 1.0))
                                    .show_percentage()
                                    .animate(!prog.is_done)
                                    .text(prog.live_detail.clone()),
                            );
                            ui.add_space(6.0);

                            let stage_titles = [
                                "1. Fetch Live OSM Street Network",
                                "2. Filter Highway Mode & Clip Boundary",
                                "3. Project WGS84 (Lat, Lon) -> Metric UTM",
                                "4. 5-Step P90 Adaptive Watertight Cleaner",
                                "5. Build CSR Dual Graph & Solve ASA",
                            ];

                            egui::Grid::new("osm_pipeline_stages_grid")
                                .num_columns(3)
                                .striped(true)
                                .spacing([10.0, 6.0])
                                .show(ui, |ui| {
                                    for (idx, title) in stage_titles.iter().enumerate() {
                                        let st = &prog.stages[idx];
                                        let (badge, col, detail) = match st {
                                            OsmStageState::Pending => (
                                                "[ ] PENDING",
                                                Color32::from_rgb(135, 145, 160),
                                                "Waiting...".to_string(),
                                            ),
                                            OsmStageState::Running(msg) => (
                                                "[•] RUNNING",
                                                accent,
                                                msg.clone(),
                                            ),
                                            OsmStageState::Completed(msg) => (
                                                "[✓] DONE",
                                                pass_col,
                                                msg.clone(),
                                            ),
                                            OsmStageState::Failed(msg) => (
                                                "[!] ERROR",
                                                fail_col,
                                                msg.clone(),
                                            ),
                                        };
                                        ui.label(
                                            RichText::new(badge).monospace().strong().color(col),
                                        );
                                        ui.label(RichText::new(*title).strong().color(heading));
                                        ui.label(RichText::new(detail).size(11.5));
                                        ui.end_row();
                                    }
                                });

                            if let Some(err) = &prog.error_msg {
                                ui.add_space(4.0);
                                ui.label(
                                    RichText::new(format!("Error: {}", err))
                                        .strong()
                                        .color(fail_col),
                                );
                            }

                            ui.separator();
                            ui.label(
                                RichText::new("Live Pipeline Execution Log:")
                                    .strong()
                                    .color(heading),
                            );
                            egui::ScrollArea::vertical()
                                .id_salt("osm_pipeline_log_scroll")
                                .max_height(110.0)
                                .stick_to_bottom(true)
                                .show(ui, |ui| {
                                    for line in &prog.logs {
                                        ui.label(RichText::new(line).monospace().size(11.0));
                                    }
                                });

                            ui.separator();
                            if !prog.is_done {
                                if ui
                                    .add_sized(
                                        [ui.available_width(), 28.0],
                                        egui::Button::new(
                                            self.btn_text("[Cancel Pipeline]"),
                                        ),
                                    )
                                    .clicked()
                                {
                                    request_cancel = true;
                                }
                            } else if ui
                                .add_sized(
                                    [ui.available_width(), 28.0],
                                    egui::Button::new(
                                        self.btn_text("[Close Progress Window]"),
                                    ),
                                )
                                .clicked()
                            {
                                request_close = true;
                            }
                        }
                    } else {
                        ui.label("No active OSM pipeline job.");
                    }
                });

            if request_cancel {
                self.cancel_osm_pipeline_job();
            }
            if request_close {
                open = false;
            }
            self.show_osm_pipeline_modal = open;
        }

        // 2. 5-Step P90 Cleaner Configuration Modal
        if self.show_cleaner_modal {
            let mut open = self.show_cleaner_modal;
            egui::Window::new("5-Step P90 Adaptive Watertight Network Cleaner")
                .open(&mut open)
                .resizable(false)
                .show(ctx, |ui| {
                    egui::Grid::new("cleaner_cfg_modal_grid")
                        .num_columns(2)
                        .show(ui, |ui| {
                            ui.label("Horizon Mode:");
                            egui::ComboBox::from_id_salt("cleaner_mode_combo")
                                .selected_text(&self.cleaner_config.mode)
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(
                                        &mut self.cleaner_config.mode,
                                        "auto".to_string(),
                                        "auto (P90 Adaptive [6m..20m])",
                                    );
                                    ui.selectable_value(
                                        &mut self.cleaner_config.mode,
                                        "manual".to_string(),
                                        "manual (Fixed Horizon)",
                                    );
                                });
                            ui.end_row();
                            ui.label("Manual Extend Distance (m):");
                            ui.add(
                                egui::Slider::new(
                                    &mut self.cleaner_config.extend_distance_m,
                                    1.0..=35.0,
                                ),
                            );
                            ui.end_row();
                            ui.label("Collinear Turn Conservation (°):");
                            ui.add(
                                egui::Slider::new(
                                    &mut self.cleaner_config.max_turn_deflection_deg,
                                    2.0..=30.0,
                                ),
                            );
                            ui.end_row();
                            ui.label("Dead-End Stub Pruning (m):");
                            ui.add(
                                egui::Slider::new(
                                    &mut self.cleaner_config.dead_end_stub_len_m,
                                    0.0..=25.0,
                                ),
                            );
                            ui.end_row();
                            ui.label("Lateral T-Junction Fallback:");
                            ui.checkbox(&mut self.cleaner_config.lateral_fallback, "Enabled");
                            ui.end_row();
                        });
                    ui.separator();
                    if ui
                        .button(self.btn_text("[Execute 5-Step Watertight Cleaner & Re-Solve ASA]"))
                        .clicked()
                    {
                        self.run_cleaner_5step();
                    }
                });
            self.show_cleaner_modal = open;
        }

        // 3. Mathematical Reference, Clickable Literature Links & Parity Verification Modal
        if self.show_math_verification_modal {
            let mut open = self.show_math_verification_modal;
            egui::Window::new(
                "Mathematical Formulations & UCL depthmapX v0.8.0 Parity Verification",
            )
            .open(&mut open)
            .default_width(820.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("help_modal_scroll")
                    .max_height(620.0)
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new(
                                "Canonical Formulations Implemented in SSE Native Rust Engine (sse_core v1.1.0):",
                            )
                            .strong()
                            .color(heading),
                        );
                        ui.label(
                            RichText::new(
                                "• NACH (Hillier, Yang & Turner 2012): NACH_r = ln(Choice_r + 1) / ln(TotalDepth_r + 3)\n\
                                 • NAIN (Hillier, Yang & Turner 2012): NAIN_r = (NodeCount_r ^ 1.2) / (TotalDepth_r + 2)\n\
                                 • Hillier Diamond D_k (1984)        : D_k = 2 * (k * (log2((k + 2) / 3) - 1) + 1) / ((k - 1)*(k - 2))\n\
                                 • Axial RRA & Integration [HH]      : RA = 2 * (MD - 1) / (k - 2);  RRA = RA / D_k;  Int_HH = 1 / RRA\n\
                                 • Closed-Loop Sinuosity Guard       : sinuosity = None when chord_length <= 1e-6 m (solves osmnx#1410)\n\
                                 • 2D Benedikt Isovist Compactness   : C = 4 * pi * Area / (Perimeter ^ 2) in (0, 1]",
                            )
                            .monospace()
                            .size(11.5),
                        );
                        ui.separator();

                        ui.label(
                            RichText::new(
                                "Official Repositories & Primary Academic Literature (Click to Open in Browser):",
                            )
                            .strong()
                            .color(accent),
                        );
                        ui.add_space(3.0);
                        for (label, url) in [
                            (
                                "UCL depthmapX Official Repository & Regression Testdata (SpaceGroupUCL/depthmapX)",
                                "https://github.com/SpaceGroupUCL/depthmapX",
                            ),
                            (
                                "SSE — Space Syntax Engine Repository (napattan/sse-space-syntax)",
                                "https://github.com/napattan/sse-space-syntax",
                            ),
                            (
                                "Hillier & Hanson (1984) — The Social Logic of Space (Cambridge University Press: Axial RRA & Diamond D_k)",
                                "https://doi.org/10.1017/CBO9780511597237",
                            ),
                            (
                                "Benedikt (1979) — To Take Hold of Space: Isovists and Isovist Fields (Environment and Planning B)",
                                "https://doi.org/10.1068/b060047",
                            ),
                            (
                                "Turner, Doxa, O'Sullivan & Penn (2001) — From Isovists to Visibility Graphs: A Methodology for the Analysis of Architectural Space",
                                "https://doi.org/10.1068/b2684",
                            ),
                            (
                                "Turner & Penn (2002) — Encoding Natural Movement as an Agent-Based System (Gibson Ecological Visual Agents)",
                                "https://doi.org/10.1068/b12850",
                            ),
                            (
                                "Jiang & Claramunt (2004) — Topological Analysis of Urban Street Networks (Gestalt Natural Roads)",
                                "https://doi.org/10.1068/b306",
                            ),
                            (
                                "Turner (2007) — From Axial to Road-Centre Lines: A New Representation for Urban Network Analysis (Angular Segment Analysis)",
                                "https://doi.org/10.1068/b32067",
                            ),
                            (
                                "Hillier, Yang & Turner (2012) — Normalising Least Angle Choice in Depthmap (JOSS: NACH & NAIN)",
                                "https://joss.bartlett.ucl.ac.uk/journal/index.php/joss/article/view/141",
                            ),
                            (
                                "OSMnx Issue #1410 — Closed-Loop Zero-Chord Circuity Singularity (gboeing/osmnx#1410)",
                                "https://github.com/gboeing/osmnx/issues/1410",
                            ),
                        ] {
                            ui.hyperlink_to(label, url);
                        }

                        ui.separator();
                        ui.label(
                            RichText::new("Live Self-Verification Parity Suite:")
                                .strong()
                                .color(heading),
                        );
                        if ui
                            .button(self.btn_text("[Re-Run Analytical Self-Verification Checks]"))
                            .clicked()
                        {
                            self.run_self_verification_suite();
                        }
                        egui::Grid::new("modal_verif_grid")
                            .num_columns(4)
                            .striped(true)
                            .show(ui, |ui| {
                                for row in &self.verification_rows {
                                    ui.label(RichText::new(row.code).monospace().strong());
                                    ui.label(RichText::new(row.title).monospace());
                                    ui.label(RichText::new(&row.observed).monospace().size(11.0));
                                    ui.label(
                                        RichText::new(if row.passed { "PASS" } else { "FAIL" })
                                            .monospace()
                                            .strong()
                                            .color(if row.passed { pass_col } else { fail_col }),
                                    );
                                    ui.end_row();
                                }
                            });
                    });
            });
            self.show_math_verification_modal = open;
        }

        // 4. Publication Map Image (PNG) Export Modal
        if self.show_png_export_modal {
            let mut open = self.show_png_export_modal;
            let mut trigger_render = false;
            egui::Window::new("Export Publication Map Image (PNG)")
                .open(&mut open)
                .resizable(false)
                .default_width(540.0)
                .show(ctx, |ui| {
                    ui.label(
                        RichText::new("Resolution Preset & Dimensions:")
                            .strong()
                            .color(heading),
                    );
                    ui.horizontal_wrapped(|ui| {
                        for (w, h, lbl) in [
                            (1920, 1080, "1920 × 1080 (Full HD)"),
                            (2560, 1440, "2560 × 1440 (2K QHD)"),
                            (3840, 2160, "3840 × 2160 (4K Print)"),
                            (2400, 2400, "2400 × 2400 (Square Figure)"),
                        ] {
                            let is_sel = self.png_width == w && self.png_height == h;
                            if ui
                                .selectable_label(is_sel, self.pill_text(lbl, is_sel))
                                .clicked()
                            {
                                self.png_width = w;
                                self.png_height = h;
                            }
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Width (px):");
                        ui.add(egui::DragValue::new(&mut self.png_width).range(400..=7680).speed(10));
                        ui.add_space(12.0);
                        ui.label("Height (px):");
                        ui.add(
                            egui::DragValue::new(&mut self.png_height)
                                .range(400..=4320)
                                .speed(10),
                        );
                    });

                    ui.separator();
                    ui.label(
                        RichText::new("Background & Transparency:")
                            .strong()
                            .color(heading),
                    );
                    ui.checkbox(
                        &mut self.png_transparent_bg,
                        "Transparent Background (Alpha = 0)",
                    );
                    if !self.png_transparent_bg {
                        ui.horizontal_wrapped(|ui| {
                            ui.label("Background Color:");
                            ui.color_edit_button_srgb(&mut self.png_bg_rgb);
                            for (rgb, lbl) in [
                                ([255, 255, 255], "Pure White #FFFFFF"),
                                ([247, 248, 250], "Academic Paper #F7F8FA"),
                                ([14, 18, 24], "Dark Slate #0E1218"),
                                ([0, 0, 0], "Pure Black #000000"),
                            ] {
                                if ui.button(self.btn_text(lbl)).clicked() {
                                    self.png_bg_rgb = rgb;
                                }
                            }
                        });
                    }

                    ui.separator();
                    ui.label(
                        RichText::new("Stroke & Geometry Styling:")
                            .strong()
                            .color(heading),
                    );
                    egui::Grid::new("png_style_grid")
                        .num_columns(2)
                        .spacing([12.0, 6.0])
                        .show(ui, |ui| {
                            ui.label("Line / Segment Width (px):");
                            ui.add(egui::Slider::new(&mut self.png_line_width, 0.5..=15.0));
                            ui.end_row();

                            ui.label("Obstacle / Wall Width (px):");
                            ui.horizontal(|ui| {
                                ui.add(egui::Slider::new(&mut self.png_wall_width, 0.5..=10.0));
                                ui.color_edit_button_srgb(&mut self.png_wall_rgb);
                            });
                            ui.end_row();

                            ui.label("Canvas Margin (%):");
                            ui.add(egui::Slider::new(&mut self.png_margin_pct, 2.0..=20.0));
                            ui.end_row();
                        });

                    ui.separator();
                    ui.label(
                        RichText::new("Layer Overlays:")
                            .strong()
                            .color(heading),
                    );
                    ui.checkbox(
                        &mut self.png_draw_legend,
                        "Draw Spectral Color Scale Legend Bar",
                    );
                    ui.checkbox(
                        &mut self.png_draw_isovists,
                        "Draw Cast 2D Isovist Polygons & Rays",
                    );
                    ui.checkbox(
                        &mut self.png_draw_agents,
                        "Draw Active Gibson Visual Agent Trails",
                    );

                    ui.separator();
                    if ui
                        .add_sized(
                            [510.0, 32.0],
                            egui::Button::new(
                                self.btn_text("[Render & Save PNG File...]").size(13.5),
                            ),
                        )
                        .clicked()
                    {
                        trigger_render = true;
                    }
                });
            self.show_png_export_modal = open;
            if trigger_render {
                self.dialog_render_and_save_png();
            }
        }
    }

    fn poll_mcp_bridge_requests(&mut self, ctx: &egui::Context) {
        let mut pending = Vec::new();
        if let Some(rx) = &self.mcp_rx {
            while let Ok(req) = rx.try_recv() {
                pending.push(req);
            }
        }
        if pending.is_empty() {
            return;
        }
        for req in pending {
            let reply = self.handle_mcp_gui_command(&req.cmd, ctx);
            let _ = req.reply_tx.send(reply);
        }
        ctx.request_repaint();
    }

    fn handle_mcp_gui_command(
        &mut self,
        cmd: &serde_json::Value,
        ctx: &egui::Context,
    ) -> serde_json::Value {
        let action = cmd
            .get("action")
            .and_then(|v| v.as_str())
            .unwrap_or("get_state");
        let params = cmd
            .get("params")
            .cloned()
            .unwrap_or_else(|| serde_json::json!({}));

        match action {
            "get_state" => {
                let vga_cells = self.vga_results.as_ref().map_or(0, |v| v.points.len());
                let active_agents = self.agent_state.as_ref().map_or(0, |s| s.active_agents.len());
                serde_json::json!({
                    "ok": true,
                    "tab_title": self.tab_title,
                    "dataset_title": self.dataset_title,
                    "domain": self.project_domain.short_label(),
                    "active_map_mode": self.active_map_mode.label(),
                    "active_attribute": self.active_attribute,
                    "segments_count": self.segments.len(),
                    "vga_cells_count": vga_cells,
                    "active_agents_count": active_agents,
                    "is_playing_agents": self.is_playing_agents,
                    "cam_center": [self.cam_center.x, self.cam_center.y],
                    "cam_zoom": self.cam_zoom,
                    "dark_theme": self.dark_theme,
                    "show_network_colors": self.show_network_colors,
                    "show_segment_ids": self.show_segment_ids,
                    "show_endpoints": self.show_endpoints,
                    "show_cleaner_diff": self.show_cleaner_diff,
                    "status_message": self.status_message,
                })
            }
            "load_preset" => {
                let p_str = params
                    .get("preset")
                    .and_then(|v| v.as_str())
                    .unwrap_or("barnsbury_mif")
                    .to_lowercase();
                let preset = if p_str.contains("axial") || p_str.contains("extended") {
                    BenchmarkPreset::AxialStarDiamond
                } else if p_str.contains("gallery") {
                    BenchmarkPreset::ArchitecturalGalleryVga
                } else if p_str.contains("room") {
                    BenchmarkPreset::UclRoomsFloorplan
                } else if p_str.contains("loop") || p_str.contains("ring") {
                    BenchmarkPreset::RingRoadCulDeSacLoop
                } else {
                    BenchmarkPreset::UrbanSuperblockGrid
                };
                self.load_preset(preset);
                serde_json::json!({
                    "ok": true,
                    "loaded_preset": preset.label(),
                    "domain": self.project_domain.short_label(),
                    "segments": self.segments.len(),
                    "vga_cells": self.vga_results.as_ref().map_or(0, |v| v.points.len()),
                })
            }
            "load_file" => {
                let Some(path_str) = params.get("path").and_then(|v| v.as_str()) else {
                    return serde_json::json!({"ok": false, "error": "Missing 'path' in params"});
                };
                let dom_str = params
                    .get("domain")
                    .and_then(|v| v.as_str())
                    .unwrap_or("street")
                    .to_lowercase();
                let domain = if dom_str.contains("vga") || dom_str.contains("spatial") {
                    AnalysisDomain::SpatialVisibility
                } else {
                    AnalysisDomain::StreetNetwork
                };
                let path = PathBuf::from(path_str);
                match load_polylines_auto(&path) {
                    Ok(lines) => {
                        let fname = path
                            .file_name()
                            .and_then(|s| s.to_str())
                            .unwrap_or("Loaded Map")
                            .to_string();
                        self.load_custom_polylines(fname.clone(), path_str.to_string(), lines, domain);
                        serde_json::json!({
                            "ok": true,
                            "title": fname,
                            "domain": self.project_domain.short_label(),
                            "segments": self.segments.len(),
                            "vga_cells": self.vga_results.as_ref().map_or(0, |v| v.points.len()),
                        })
                    }
                    Err(e) => serde_json::json!({"ok": false, "error": e}),
                }
            }
            "set_attribute" => {
                if let Some(attr) = params.get("attribute").and_then(|v| v.as_str()) {
                    self.active_attribute = attr.to_string();
                }
                serde_json::json!({"ok": true, "active_attribute": self.active_attribute})
            }
            "set_overlays" => {
                if let Some(v) = params.get("show_network_colors").and_then(|v| v.as_bool()) {
                    self.show_network_colors = v;
                }
                if let Some(v) = params.get("show_segment_ids").and_then(|v| v.as_bool()) {
                    self.show_segment_ids = v;
                }
                if let Some(v) = params.get("show_endpoints").and_then(|v| v.as_bool()) {
                    self.show_endpoints = v;
                }
                if let Some(v) = params.get("show_cleaner_diff").and_then(|v| v.as_bool()) {
                    self.show_cleaner_diff = v;
                }
                if let Some(v) = params.get("show_vga_cells").and_then(|v| v.as_bool()) {
                    self.show_vga_cells = v;
                }
                if let Some(v) = params.get("show_obstacles").and_then(|v| v.as_bool()) {
                    self.show_obstacles = v;
                }
                if let Some(v) = params.get("show_agent_bodies").and_then(|v| v.as_bool()) {
                    self.show_agent_bodies = v;
                }
                if let Some(v) = params.get("show_agent_active_trails").and_then(|v| v.as_bool()) {
                    self.show_agent_active_trails = v;
                }
                if let Some(v) = params.get("show_isovists").and_then(|v| v.as_bool()) {
                    self.show_isovists = v;
                }
                if let Some(v) = params.get("dark_theme").and_then(|v| v.as_bool()) {
                    self.dark_theme = v;
                    self.apply_academic_theme(ctx);
                }
                serde_json::json!({"ok": true})
            }
            "run_cleaner" => {
                self.run_cleaner_5step();
                serde_json::json!({"ok": true, "segments": self.segments.len(), "status": self.status_message})
            }
            "run_asa" => {
                self.rebuild_segment_graph_and_solve_asa();
                self.active_map_mode = ActiveMapMode::SegmentMapAsa;
                serde_json::json!({"ok": true, "segments": self.segments.len(), "intelligibility_r2": self.asa_results.intelligibility_r2})
            }
            "run_axial" => {
                self.run_axial_solver();
                serde_json::json!({"ok": true, "status": self.status_message})
            }
            "run_natural_roads" => {
                self.run_natural_roads_solver();
                serde_json::json!({"ok": true, "status": self.status_message})
            }
            "create_vga_grid" => {
                if let Some(cs) = params.get("cell_size_m").and_then(|v| v.as_f64()) {
                    self.vga_cell_size_m = cs.clamp(0.05, 50.0);
                }
                let seed = match (
                    params.get("seed_x").and_then(|v| v.as_f64()),
                    params.get("seed_y").and_then(|v| v.as_f64()),
                ) {
                    (Some(x), Some(y)) => Some(Point2D::new(x, y)),
                    _ => None,
                };
                self.create_or_fill_vga_grid(seed);
                serde_json::json!({"ok": true, "vga_cells": self.vga_results.as_ref().map_or(0, |v| v.points.len())})
            }
            "run_vga" => {
                self.run_vga_solver_sync();
                serde_json::json!({"ok": true, "vga_cells": self.vga_results.as_ref().map_or(0, |v| v.points.len()), "elapsed_ms": self.last_solver_ms})
            }
            "place_agent" => {
                let x = params.get("x").and_then(|v| v.as_f64()).unwrap_or(15.0);
                let y = params.get("y").and_then(|v| v.as_f64()).unwrap_or(15.0);
                self.place_agents_at_world_point(Point2D::new(x, y));
                serde_json::json!({"ok": true, "status": self.status_message})
            }
            "play_agents" => {
                let play = params.get("play").and_then(|v| v.as_bool()).unwrap_or(true);
                if let Some(sps) = params.get("steps_per_sec").and_then(|v| v.as_f64()) {
                    self.agent_steps_per_sec = sps.clamp(0.2, 30.0);
                }
                self.show_agent_bodies = true;
                self.is_playing_agents = play;
                if play {
                    self.last_agent_frame_instant = Some(Instant::now());
                }
                serde_json::json!({"ok": true, "is_playing_agents": self.is_playing_agents})
            }
            "step_agents" => {
                let steps = params.get("steps").and_then(|v| v.as_u64()).unwrap_or(1) as usize;
                let allow_continuous =
                    !self.agent_manual_single_mode || self.agent_config.release_from_selected_only;
                if let (Some(vga), Some(st)) = (&self.vga_results, &mut self.agent_state) {
                    step_agent_simulation_with_mode(vga, st, &self.agent_config, steps, allow_continuous);
                }
                self.sync_agent_gate_counts_to_vga();
                serde_json::json!({"ok": true, "steps_advanced": steps})
            }
            "cast_isovist" => {
                let x = params.get("x").and_then(|v| v.as_f64()).unwrap_or(15.0);
                let y = params.get("y").and_then(|v| v.as_f64()).unwrap_or(15.0);
                let obs = if self.obstacles.is_empty() {
                    &self.working_polylines
                } else {
                    &self.obstacles
                };
                let iso = compute_isovist_at_point(Point2D::new(x, y), obs, self.vga_max_vis_m);
                let area = iso.area;
                let perim = iso.perimeter;
                let comp = iso.compactness;
                self.show_isovists = true;
                self.cast_isovists.push(iso);
                serde_json::json!({"ok": true, "area": area, "perimeter": perim, "compactness": comp})
            }
            "zoom_fit" => {
                self.needs_zoom_to_fit = true;
                serde_json::json!({"ok": true})
            }
            "export_png" => {
                let Some(path_str) = params.get("path").and_then(|v| v.as_str()) else {
                    return serde_json::json!({"ok": false, "error": "Missing 'path' in params"});
                };
                let path = PathBuf::from(path_str);
                match self.render_and_save_png_to_path(&path) {
                    Ok(()) => serde_json::json!({"ok": true, "path": path_str}),
                    Err(e) => serde_json::json!({"ok": false, "error": e}),
                }
            }
            other => serde_json::json!({"ok": false, "error": format!("Unknown GUI action '{}'", other)}),
        }
    }
}

impl eframe::App for SseWorkstationApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_vga_background_job(ctx);
        self.poll_osm_pipeline_job(ctx);
        self.poll_mcp_bridge_requests(ctx);

        // Global keyboard shortcuts (Ctrl+N, Ctrl+O, Ctrl+W)
        let (ctrl_n, ctrl_o, ctrl_w) = ctx.input(|i| {
            (
                i.modifiers.command && i.key_pressed(egui::Key::N),
                i.modifiers.command && i.key_pressed(egui::Key::O),
                i.modifiers.command && i.key_pressed(egui::Key::W),
            )
        });
        if ctrl_n {
            self.create_new_blank_project_tab();
        }
        if ctrl_o {
            self.dialog_open_vector_file();
        }
        if ctrl_w {
            let cur = self.active_project_idx;
            self.close_project_tab(cur);
        }

        egui::TopBottomPanel::top("sse_top_bar")
            .frame(
                egui::Frame::side_top_panel(&ctx.style())
                    .inner_margin(egui::Margin::symmetric(10.0, 5.0)),
            )
            .show(ctx, |ui| {
                self.render_top_menu_and_ribbon(ctx, ui);
            });

        egui::TopBottomPanel::bottom("sse_bottom_status")
            .exact_height(26.0)
            .show(ctx, |ui| {
                self.render_bottom_status_bar(ui);
            });

        egui::SidePanel::left("sse_left_dock")
            .default_width(305.0)
            .min_width(275.0)
            .max_width(420.0)
            .show(ctx, |ui| {
                self.render_left_dock(ui);
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.allocate_ui_with_layout(
                egui::vec2(ui.available_width(), 30.0),
                egui::Layout::left_to_right(egui::Align::Center),
                |ui| {
                    ui.spacing_mut().interact_size.y = 24.0;
                    ui.spacing_mut().button_padding = egui::vec2(8.0, 4.0);
                    for (tab, title) in [
                        (
                            WorkspaceTab::SpatialViewport,
                            "Spatial Map Viewport [2D CAD / VGA / Isovists / Agents]",
                        ),
                        (
                            WorkspaceTab::ScatterPlotRegression,
                            "Scatter Plot & OLS Regression [Intelligibility / Synergy R²]",
                        ),
                        (
                            WorkspaceTab::AttributeDataSheet,
                            "Attribute Data Sheet [Virtual Table]",
                        ),
                        (
                            WorkspaceTab::TopologyAuditLog,
                            "Topology, 5-Step Cleaner & Parity Audit",
                        ),
                    ] {
                        let is_active = self.active_tab == tab;
                        if ui
                            .selectable_label(is_active, self.pill_text(title, is_active))
                            .clicked()
                        {
                            self.active_tab = tab;
                        }
                    }
                },
            );
            self.bar_horizontal_divider(ui);

            match self.active_tab {
                WorkspaceTab::SpatialViewport => self.render_spatial_viewport(ctx, ui),
                WorkspaceTab::ScatterPlotRegression => self.render_scatter_plot_tab(ui),
                WorkspaceTab::AttributeDataSheet => self.render_attribute_data_sheet_tab(ui),
                WorkspaceTab::TopologyAuditLog => self.render_topology_audit_tab(ui),
            }
        });

        self.render_modals(ctx);

        // Open clicked hyperlinks in Windows default browser without console popup
        if let Some(open_url) = ctx.output_mut(|o| o.open_url.take()) {
            let _ = std::process::Command::new("rundll32")
                .args(["url.dll,FileProtocolHandler", &open_url.url])
                .spawn();
        }
    }
}

fn load_sse_window_icon() -> Option<std::sync::Arc<egui::IconData>> {
    const ICON_BYTES: &[u8] = include_bytes!("../../sse/plugins/qgis_plugin/icon.png");
    let img = image::load_from_memory(ICON_BYTES).ok()?.into_rgba8();
    let (width, height) = img.dimensions();
    Some(std::sync::Arc::new(egui::IconData {
        rgba: img.into_raw(),
        width,
        height,
    }))
}

fn main() -> eframe::Result<()> {
    let mut viewport = egui::ViewportBuilder::default()
        .with_inner_size([1520.0, 920.0])
        .with_min_inner_size([1024.0, 680.0])
        .with_title(
            "SSE — Space Syntax Engine v1.1.0 | Academic Spatial Network & Visibility Workstation (UCL depthmapX v0.8.0 Parity)",
        );
    if let Some(icon) = load_sse_window_icon() {
        viewport = viewport.with_icon(icon);
    }

    let native_options = eframe::NativeOptions {
        viewport,
        ..Default::default()
    };

    eframe::run_native(
        "SSE — Space Syntax Engine v1.1.0",
        native_options,
        Box::new(|cc| Ok(Box::new(SseWorkstationApp::new(cc)))),
    )
}
