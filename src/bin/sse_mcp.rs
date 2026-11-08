//! `sse-mcp` — Model Context Protocol (`MCP` JSON-RPC 2.0 `stdio`) Server & Live GUI Bridge
//! for **SSE — Space Syntax Engine (`v1.1.0`)**.
//!
//! Exposes 9 Space Syntax tools over dual-mode MCP `stdio` (supporting both newline-delimited
//! JSON-RPC and `Content-Length: N\r\n\r\n` framed JSON-RPC) plus a live TCP control bridge to
//! `sse-desktop.exe` on `127.0.0.1:9884`:
//! 1. `sse_status`
//! 2. `sse_fetch_and_clean_osm`
//! 3. `sse_clean_network`
//! 4. `sse_solve_asa`
//! 5. `sse_solve_axial_and_strokes`
//! 6. `sse_solve_vga`
//! 7. `sse_cast_isovist`
//! 8. `sse_simulate_visual_agents`
//! 9. `sse_gui_control`

use serde_json::{json, Value};
use sse_core::benchmarks::{load_benchmark, BenchmarkPreset};
use sse_core::cleaner::{
    clean_network_adaptive_watertight, count_connected_components, count_dangling_endpoints,
    count_graph_junctions, explode_to_2point_segments, is_prenoded_segment_map,
    planarize_and_node_pruned,
};
use sse_core::graph::detect_grade_unlinks;
use sse_core::io::{
    export_segments_csv, export_segments_dxf, export_segments_geojson, export_segments_mif_mid,
    export_vga_csv, fetch_osm_overpass_boundary_staged, geocode_osm_location, load_polylines_auto,
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
    generate_vga_grid, generate_vga_grid_flood_fill, solve_vga,
};
use sse_core::types::{
    AgentConfig, AgentMovementRule, AnalysisType, AsaConfig, CSRDualGraph, CleanerConfig, Point2D,
    Polyline, RadiusType, SegmentRecord, SummaryStats, WeightMode,
};
use std::collections::BTreeMap;
use std::env;
use std::io::{self, BufRead, BufReader, Write};
use std::net::{SocketAddr, TcpStream};
use std::path::{Path, PathBuf};
use std::time::Duration;

const GUI_BRIDGE_ADDR: &str = "127.0.0.1:9884";
const SERVER_NAME: &str = "sse-space-syntax";
const SERVER_VERSION: &str = "1.1.0";
const DEFAULT_PROTOCOL_VERSION: &str = "2024-11-05";

/// Sends a single JSON command line `{"action": ..., "params": ...}\n` to `sse-desktop.exe`
/// listening on `127.0.0.1:9884` and reads a single JSON response line.
fn send_gui_command(
    action: &str,
    params: Value,
    connect_timeout_ms: u64,
    io_timeout_ms: u64,
) -> Result<Value, String> {
    let addr: SocketAddr = GUI_BRIDGE_ADDR
        .parse()
        .map_err(|e| format!("Invalid GUI bridge address '{GUI_BRIDGE_ADDR}': {e}"))?;
    let mut stream =
        TcpStream::connect_timeout(&addr, Duration::from_millis(connect_timeout_ms.max(50)))
            .map_err(|e| {
                format!(
                    "Could not connect to live sse-desktop.exe GUI on {GUI_BRIDGE_ADDR}: {e}"
                )
            })?;

    let io_timeout = Some(Duration::from_millis(io_timeout_ms.max(250)));
    let _ = stream.set_read_timeout(io_timeout);
    let _ = stream.set_write_timeout(io_timeout);

    let req = json!({
        "action": action,
        "params": params,
    });
    let mut payload =
        serde_json::to_string(&req).map_err(|e| format!("JSON serialize error: {e}"))?;
    payload.push('\n');

    stream
        .write_all(payload.as_bytes())
        .and_then(|_| stream.flush())
        .map_err(|e| format!("Failed writing command '{action}' to GUI bridge: {e}"))?;

    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    reader
        .read_line(&mut line)
        .map_err(|e| format!("Failed reading response for '{action}' from GUI bridge: {e}"))?;

    let trimmed = line.trim();
    if trimmed.is_empty() {
        return Err(format!(
            "Empty response from GUI bridge for action '{action}'."
        ));
    }

    serde_json::from_str::<Value>(trimmed)
        .map_err(|e| format!("Invalid JSON response from GUI bridge: {e} (raw: {trimmed})"))
}

fn probe_gui_state() -> Option<Value> {
    send_gui_command("get_state", json!({}), 250, 1500).ok()
}

fn parse_preset_name(raw: &str) -> BenchmarkPreset {
    match raw.trim().to_ascii_lowercase().as_str() {
        "barnsbury_mif" | "urban" | "superblock" | "barnsbury" | "preset1" => {
            BenchmarkPreset::UrbanSuperblockGrid
        }
        "barnsbury_axial" | "axial" | "diamond" | "extended" | "preset2" => {
            BenchmarkPreset::AxialStarDiamond
        }
        "gallery_vga" | "gallery" | "national_gallery" | "preset3" => {
            BenchmarkPreset::ArchitecturalGalleryVga
        }
        "rooms_vga" | "rooms" | "ucl_rooms" | "preset4" => BenchmarkPreset::UclRoomsFloorplan,
        "loop_guard" | "ring" | "loop" | "cul_de_sac" | "preset5" => {
            BenchmarkPreset::RingRoadCulDeSacLoop
        }
        _ => BenchmarkPreset::UrbanSuperblockGrid,
    }
}

fn canonical_preset_id(preset: BenchmarkPreset) -> &'static str {
    match preset {
        BenchmarkPreset::UrbanSuperblockGrid => "barnsbury_mif",
        BenchmarkPreset::AxialStarDiamond => "barnsbury_axial",
        BenchmarkPreset::ArchitecturalGalleryVga => "gallery_vga",
        BenchmarkPreset::UclRoomsFloorplan => "rooms_vga",
        BenchmarkPreset::RingRoadCulDeSacLoop => "loop_guard",
    }
}

fn load_street_lines_from_args(
    args: &Value,
    default_preset: BenchmarkPreset,
) -> Result<(Vec<Polyline>, String, Option<BenchmarkPreset>), String> {
    if let Some(path_str) = args
        .get("input_path")
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
    {
        let p = Path::new(path_str);
        let lines = load_polylines_auto(p)?;
        return Ok((lines, format!("file:{path_str}"), None));
    }

    let preset = args
        .get("preset")
        .and_then(|v| v.as_str())
        .map(parse_preset_name)
        .unwrap_or(default_preset);
    let bm = load_benchmark(preset);
    Ok((
        bm.lines,
        format!("preset:{} ({})", canonical_preset_id(preset), bm.name),
        Some(preset),
    ))
}

fn load_floorplan_from_args(
    args: &Value,
    default_preset: BenchmarkPreset,
) -> Result<
    (
        Vec<Vec<Point2D>>,
        Vec<Polyline>,
        f64,
        String,
        Option<BenchmarkPreset>,
    ),
    String,
> {
    if let Some(path_str) = args
        .get("input_path")
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
    {
        let p = Path::new(path_str);
        let obstacles = load_polylines_auto(p)?;
        return Ok((
            Vec::new(),
            obstacles,
            0.30,
            format!("file:{path_str}"),
            None,
        ));
    }

    let preset = args
        .get("preset")
        .and_then(|v| v.as_str())
        .map(|s| match s.trim().to_ascii_lowercase().as_str() {
            "rooms" | "rooms_vga" | "ucl_rooms" => BenchmarkPreset::UclRoomsFloorplan,
            "gallery" | "gallery_vga" | "national_gallery" => {
                BenchmarkPreset::ArchitecturalGalleryVga
            }
            other => parse_preset_name(other),
        })
        .unwrap_or(default_preset);
    let bm = load_benchmark(preset);
    let obstacles = if bm.obstacles.is_empty() {
        bm.lines
    } else {
        bm.obstacles
    };
    Ok((
        bm.boundary_rings,
        obstacles,
        bm.default_vga_cell_m,
        format!("preset:{} ({})", canonical_preset_id(preset), bm.name),
        Some(preset),
    ))
}

fn parse_radii_from_args(args: &Value) -> Vec<f64> {
    let mut radii = Vec::new();
    if let Some(arr) = args.get("radii_m").and_then(|v| v.as_array()) {
        for item in arr {
            if let Some(f) = item.as_f64() {
                if f > 0.0 && f.is_finite() {
                    radii.push(f);
                } else if f.is_infinite() {
                    radii.push(f64::INFINITY);
                }
            } else if let Some(s) = item.as_str() {
                let t = s.trim().to_ascii_lowercase();
                if t == "n" || t == "rn" || t == "inf" || t == "global" {
                    radii.push(f64::INFINITY);
                } else if let Ok(v) = t.trim_end_matches('m').parse::<f64>() {
                    if v > 0.0 {
                        radii.push(v);
                    }
                }
            }
        }
    }
    if radii.is_empty() {
        radii = vec![400.0, 800.0, 1200.0, 2000.0, 5000.0, f64::INFINITY];
    } else if !radii.iter().any(|r| r.is_infinite()) {
        radii.push(f64::INFINITY);
    }
    radii
}

fn export_segments_by_path(
    out_path: &Path,
    segments: &[SegmentRecord],
    attributes: &BTreeMap<String, Vec<f64>>,
) -> Result<(), String> {
    let ext = out_path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("geojson")
        .to_ascii_lowercase();
    match ext.as_str() {
        "geojson" | "json" => export_segments_geojson(out_path, segments, attributes),
        "dxf" => export_segments_dxf(out_path, segments, attributes, None),
        "mif" => export_segments_mif_mid(out_path, segments, attributes),
        _ => export_segments_csv(out_path, segments, attributes),
    }
}

fn top_k_segments_for_attr(
    segments: &[SegmentRecord],
    attributes: &BTreeMap<String, Vec<f64>>,
    attr_name: &str,
    top_k: usize,
) -> Vec<Value> {
    let Some(col) = attributes.get(attr_name) else {
        return Vec::new();
    };
    let mut indexed: Vec<(usize, f64)> = col
        .iter()
        .copied()
        .enumerate()
        .filter(|&(_, v)| v.is_finite() && v > -0.999999)
        .collect();
    indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
    indexed
        .into_iter()
        .take(top_k.max(1))
        .filter_map(|(idx, val)| {
            let seg = segments.get(idx)?;
            let mid = seg.midpoint();
            Some(json!({
                "segment_id": seg.id,
                "value": (val * 1_000_000.0).round() / 1_000_000.0,
                "length_m": (seg.length_m * 100.0).round() / 100.0,
                "midpoint": [
                    (mid.x * 100.0).round() / 100.0,
                    (mid.y * 100.0).round() / 100.0
                ],
                "highway": seg.highway,
                "name": seg.name,
            }))
        })
        .collect()
}

fn parse_agent_movement_rule(raw: Option<&str>) -> (AgentMovementRule, &'static str) {
    let key = raw.unwrap_or("standard").trim().to_ascii_lowercase();
    let try_variant = |candidates: &[&str], fallback: AgentMovementRule| -> AgentMovementRule {
        for cand in candidates {
            if let Ok(rule) = serde_json::from_value::<AgentMovementRule>(Value::String(
                (*cand).to_string(),
            )) {
                return rule;
            }
        }
        fallback
    };

    match key.as_str() {
        "los_length" | "line_of_sight_length" | "los" | "sel_los" => (
            AgentMovementRule::LineOfSightLength,
            "los_length",
        ),
        "occ_length" | "occlusion_length" => (
            try_variant(
                &["OcclusionLength", "OccLength", "OccludingLength"],
                AgentMovementRule::LineOfSightLength,
            ),
            "occ_length",
        ),
        "occ_any" | "occlusion_any" => (
            try_variant(
                &["OcclusionAny", "OccAny", "AnyOccluding"],
                AgentMovementRule::Standard,
            ),
            "occ_any",
        ),
        "occ_group_45" | "occlusion_group_45" => (
            try_variant(
                &["OcclusionGroup45", "OccGroup45"],
                AgentMovementRule::LineOfSightLength,
            ),
            "occ_group_45",
        ),
        "occ_group_60" | "occlusion_group_60" => (
            try_variant(
                &["OcclusionGroup60", "OccGroup60"],
                AgentMovementRule::LineOfSightLength,
            ),
            "occ_group_60",
        ),
        "occ_furthest" | "occlusion_furthest" => (
            try_variant(
                &["OcclusionFurthest", "OccFurthest", "FurthestOccluding"],
                AgentMovementRule::LineOfSightLength,
            ),
            "occ_furthest",
        ),
        "bin_far_dist" | "bin_far_distance" => (
            try_variant(
                &["BinFarDistance", "BinFarDist"],
                AgentMovementRule::LineOfSightLength,
            ),
            "bin_far_dist",
        ),
        "bin_angle" => (
            try_variant(&["BinAngle"], AgentMovementRule::Standard),
            "bin_angle",
        ),
        "bin_far_dist_angle" | "bin_far_distance_angle" => (
            try_variant(
                &["BinFarDistanceAngle", "BinFarDistAngle"],
                AgentMovementRule::LineOfSightLength,
            ),
            "bin_far_dist_angle",
        ),
        "bin_memory" => (
            try_variant(&["BinMemory"], AgentMovementRule::Standard),
            "bin_memory",
        ),
        _ => (AgentMovementRule::Standard, "standard"),
    }
}

// ============================================================================
// Tool 1: sse_status
// ============================================================================
fn tool_sse_status() -> Result<Value, String> {
    let chain = vec![
        Polyline::new(vec![Point2D::new(0.0, 0.0), Point2D::new(100.0, 0.0)]),
        Polyline::new(vec![Point2D::new(100.0, 0.0), Point2D::new(200.0, 0.0)]),
        Polyline::new(vec![Point2D::new(200.0, 0.0), Point2D::new(300.0, 0.0)]),
    ];
    let (graph, _) = CSRDualGraph::from_polylines(&chain, 0, 0.0001);
    let asa = solve_angular_segment_analysis(
        &graph,
        &AsaConfig {
            radii: vec![f64::INFINITY],
            analysis_type: AnalysisType::Metric,
            radius_type: RadiusType::Metric,
            weight_mode: WeightMode::Unweighted,
            tulip_bins: 0,
            include_normalized: true,
            max_sources_per_radius: 5000,
        },
    );
    let ch = asa.attributes.get("Choice_Rn").cloned().unwrap_or_default();
    let md = asa
        .attributes
        .get("Mean_Depth_Rn")
        .cloned()
        .unwrap_or_default();
    let d5 = calculate_hillier_diamond_dn(5);
    let self_check_ok = ch.len() == 3
        && (ch[1] - 2.0).abs() < 1e-6
        && (md[1] - 100.0).abs() < 1e-6
        && (d5 - 0.35199368).abs() < 1e-6;

    let gui_state = probe_gui_state();

    Ok(json!({
        "engine": "SSE — Space Syntax Engine (UCL depthmapX v0.8.0 Native Rust Parity)",
        "version": SERVER_VERSION,
        "self_check_passed": self_check_ok,
        "self_check_invariants": {
            "benchmark_a_3seg_choice": ch,
            "benchmark_a_3seg_mean_depth": md,
            "hillier_diamond_d5": d5,
        },
        "gui_bridge": {
            "address": GUI_BRIDGE_ADDR,
            "live": gui_state.is_some(),
            "state": gui_state,
        },
        "presets": [
            {"id": "barnsbury_mif", "domain": "street", "name": BenchmarkPreset::UrbanSuperblockGrid.label()},
            {"id": "barnsbury_axial", "domain": "street", "name": BenchmarkPreset::AxialStarDiamond.label()},
            {"id": "gallery_vga", "domain": "vga", "name": BenchmarkPreset::ArchitecturalGalleryVga.label()},
            {"id": "rooms_vga", "domain": "vga", "name": BenchmarkPreset::UclRoomsFloorplan.label()},
            {"id": "loop_guard", "domain": "street", "name": BenchmarkPreset::RingRoadCulDeSacLoop.label()},
        ]
    }))
}

// ============================================================================
// Tool 2: sse_fetch_and_clean_osm
// ============================================================================
fn tool_sse_fetch_and_clean_osm(args: &Value) -> Result<Value, String> {
    let mode = args
        .get("mode")
        .and_then(|v| v.as_str())
        .unwrap_or("walk")
        .trim();
    let sync_to_gui = args
        .get("sync_to_gui")
        .and_then(|v| v.as_bool())
        .unwrap_or(true);

    let bbox_tuple = (
        args.get("south").and_then(|v| v.as_f64()),
        args.get("west").and_then(|v| v.as_f64()),
        args.get("north").and_then(|v| v.as_f64()),
        args.get("east").and_then(|v| v.as_f64()),
    );

    let (south, west, north, east, location_label, geocoded_meta) = match bbox_tuple {
        (Some(s), Some(w), Some(n), Some(e)) => (
            s,
            w,
            n,
            e,
            format!("BBox ({s:.5}, {w:.5}, {n:.5}, {e:.5})"),
            None,
        ),
        _ => {
            let query = args
                .get("query")
                .and_then(|v| v.as_str())
                .map(str::trim)
                .filter(|s| !s.is_empty())
                .ok_or_else(|| {
                    "Provide either 'query' (place name or 'lat, lon') or ('south', 'west', 'north', 'east') bounding box."
                        .to_string()
                })?;
            let geo = geocode_osm_location(query)?;
            let meta = json!({
                "query": query,
                "display_name": geo.display_name,
                "lat": geo.lat,
                "lon": geo.lon,
                "bbox": [geo.south, geo.west, geo.north, geo.east],
            });
            (
                geo.south,
                geo.west,
                geo.north,
                geo.east,
                geo.display_name,
                Some(meta),
            )
        }
    };

    let fetch_report = fetch_osm_overpass_boundary_staged(
        south,
        west,
        north,
        east,
        None,
        mode,
        None,
        |_, _, _| {},
    )?;

    let cleaner_cfg = CleanerConfig::default();
    let (cleaned_lines, cleaner_report) =
        clean_network_adaptive_watertight(&fetch_report.polylines, &cleaner_cfg);

    let radii = parse_radii_from_args(args);
    let (graph, segments) = CSRDualGraph::from_polylines(&cleaned_lines, 1024, 0.0001);
    let asa_cfg = AsaConfig {
        radii: radii.clone(),
        ..AsaConfig::default()
    };
    let asa_res = solve_angular_segment_analysis(&graph, &asa_cfg);

    let explicit_out: Option<PathBuf> = args
        .get("output_geojson")
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(PathBuf::from);

    let gui_live = probe_gui_state().is_some();
    let export_target: Option<PathBuf> = if let Some(ref p) = explicit_out {
        Some(p.clone())
    } else if sync_to_gui && gui_live {
        Some(env::temp_dir().join("sse_mcp_osm_live.geojson"))
    } else {
        None
    };

    if let Some(ref out_path) = export_target {
        export_segments_by_path(out_path, &segments, &asa_res.attributes)?;
    }

    let mut gui_sync_status = json!({ "attempted": false, "synced": false });
    if sync_to_gui && gui_live {
        if let Some(ref out_path) = export_target {
            let path_str = out_path.to_string_lossy().to_string();
            let load_resp = send_gui_command(
                "load_file",
                json!({ "path": path_str, "domain": "street" }),
                250,
                5000,
            );
            let asa_resp = send_gui_command("run_asa", json!({}), 250, 8000);
            let _ = send_gui_command("zoom_fit", json!({}), 250, 1500);
            gui_sync_status = json!({
                "attempted": true,
                "synced": load_resp.is_ok() && asa_resp.is_ok(),
                "loaded_path": path_str,
                "load_response": load_resp.ok(),
                "asa_response": asa_resp.ok(),
            });
        }
    }

    Ok(json!({
        "location": location_label,
        "geocoded": geocoded_meta,
        "bbox": {
            "south": south,
            "west": west,
            "north": north,
            "east": east,
        },
        "mode": mode,
        "osm_fetch": {
            "source_endpoint": fetch_report.source_endpoint,
            "raw_elements_count": fetch_report.raw_elements_count,
            "filtered_ways_count": fetch_report.filtered_ways_count,
            "utm_epsg": fetch_report.utm_epsg,
        },
        "cleaner_report": cleaner_report,
        "asa_summary": {
            "segments_count": segments.len(),
            "attributes_count": asa_res.attributes.len(),
            "intelligibility_r2": asa_res.intelligibility_r2,
            "intelligibility_integration_r2": asa_res.intelligibility_integration_r2,
            "synergy_r2": asa_res.synergy_r2,
            "elapsed_ms": asa_res.elapsed_ms,
            "top_nach_rn": top_k_segments_for_attr(&segments, &asa_res.attributes, "NACH_Rn", 8),
            "top_nain_rn": top_k_segments_for_attr(&segments, &asa_res.attributes, "NAIN_Rn", 8),
        },
        "output_geojson": explicit_out.map(|p| p.to_string_lossy().to_string()),
        "gui_sync": gui_sync_status,
    }))
}

// ============================================================================
// Tool 3: sse_clean_network
// ============================================================================
fn tool_sse_clean_network(args: &Value) -> Result<Value, String> {
    let (raw_lines, source_label, preset_opt) =
        load_street_lines_from_args(args, BenchmarkPreset::UrbanSuperblockGrid)?;

    let mut cfg = CleanerConfig::default();
    if let Some(m) = args.get("mode").and_then(|v| v.as_str()) {
        cfg.mode = m.trim().to_string();
    }
    if let Some(v) = args.get("extend_distance_m").and_then(|v| v.as_f64()) {
        cfg.extend_distance_m = v.max(0.5);
    }
    if let Some(v) = args.get("snap_distance_m").and_then(|v| v.as_f64()) {
        cfg.snap_distance_m = v.max(0.0);
    }
    if let Some(v) = args.get("min_stub_len_m").and_then(|v| v.as_f64()) {
        cfg.min_stub_len_m = v.max(0.0);
    }
    if let Some(v) = args.get("max_turn_deflection_deg").and_then(|v| v.as_f64()) {
        cfg.max_turn_deflection_deg = v.clamp(1.0, 89.0);
    }
    if let Some(v) = args.get("dead_end_stub_len_m").and_then(|v| v.as_f64()) {
        cfg.dead_end_stub_len_m = v.max(0.0);
    }
    if let Some(b) = args.get("lateral_fallback").and_then(|v| v.as_bool()) {
        cfg.lateral_fallback = b;
    }

    let (cleaned_lines, report) = clean_network_adaptive_watertight(&raw_lines, &cfg);
    let cleaned_segments = explode_to_2point_segments(&cleaned_lines);

    if let Some(out_str) = args
        .get("output_geojson")
        .or_else(|| args.get("output_path"))
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
    {
        export_segments_by_path(Path::new(out_str), &cleaned_segments, &BTreeMap::new())?;
    }

    let sync_to_gui = args
        .get("sync_to_gui")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let mut gui_synced = false;
    if sync_to_gui && probe_gui_state().is_some() {
        if let Some(preset) = preset_opt {
            let _ = send_gui_command(
                "load_preset",
                json!({ "preset": canonical_preset_id(preset) }),
                250,
                4000,
            );
        } else if let Some(in_path) = args.get("input_path").and_then(|v| v.as_str()) {
            let _ = send_gui_command(
                "load_file",
                json!({ "path": in_path, "domain": "street" }),
                250,
                4000,
            );
        }
        gui_synced = send_gui_command("run_cleaner", json!({}), 250, 6000).is_ok();
    }

    Ok(json!({
        "source": source_label,
        "raw_dangles_count": report.raw_dangles_count,
        "cleaned_dangles_count": report.cleaned_dangles_count,
        "raw_junctions_count": report.raw_junctions_count,
        "cleaned_junctions_count": report.cleaned_junctions_count,
        "effective_extend_distance": report.effective_extend_distance,
        "ray_extensions_count": report.ray_extensions_count,
        "lateral_connectors_count": report.lateral_connectors_count,
        "pruned_micro_stubs_count": report.pruned_micro_stubs_count,
        "remaining_micro_stubs_count": report.remaining_micro_stubs_count,
        "dead_end_stubs_removed_count": report.dead_end_stubs_removed_count,
        "raw_components_count": report.raw_components_count,
        "cleaned_components_count": report.cleaned_components_count,
        "raw_lines_count": report.raw_lines_count,
        "exploded_segments_count": report.exploded_segments_count,
        "cleaned_lines_count": report.cleaned_lines_count,
        "cleaned_2pt_segments_count": cleaned_segments.len(),
        "raw_total_length_m": report.raw_total_length_m,
        "cleaned_total_length_m": report.cleaned_total_length_m,
        "length_delta_pct": report.length_delta_pct,
        "circuity_avg": report.circuity_avg,
        "assertions_passed": report.assertions_passed,
        "elapsed_ms": report.elapsed_ms,
        "gui_synced": gui_synced,
    }))
}

// ============================================================================
// Tool 4: sse_solve_asa
// ============================================================================
fn tool_sse_solve_asa(args: &Value) -> Result<Value, String> {
    let (raw_lines, source_label, preset_opt) =
        load_street_lines_from_args(args, BenchmarkPreset::UrbanSuperblockGrid)?;

    let clean_first = args
        .get("clean_first")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let prune_whiskers = args
        .get("prune_whiskers")
        .and_then(|v| v.as_bool())
        .unwrap_or(true);
    let tulip_bins = args
        .get("tulip_bins")
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .unwrap_or(1024);
    let top_k = args
        .get("top_k")
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .unwrap_or(10)
        .clamp(1, 100);

    let working_lines = if clean_first {
        clean_network_adaptive_watertight(&raw_lines, &CleanerConfig::default()).0
    } else {
        raw_lines
    };

    let stub_ratio = if prune_whiskers { 0.40 } else { 0.0 };
    let noded_lines = if !clean_first && !prune_whiskers && is_prenoded_segment_map(&working_lines) {
        working_lines
    } else {
        planarize_and_node_pruned(&working_lines, stub_ratio)
    };
    let segments = explode_to_2point_segments(&noded_lines);
    let unlinks = detect_grade_unlinks(&segments);
    let angular_weight_ratio = args
        .get("angular_weight_ratio")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    let graph = CSRDualGraph::from_segments(&segments, &unlinks, tulip_bins, angular_weight_ratio);

    let connected_components = count_connected_components(&noded_lines);
    let junctions_deg3_plus = count_graph_junctions(&noded_lines);
    let dangles_deg1 = count_dangling_endpoints(&noded_lines);

    let radii = parse_radii_from_args(args);
    let analysis_type = match args
        .get("analysis_type")
        .and_then(|v| v.as_str())
        .unwrap_or("angular")
        .to_ascii_lowercase()
        .as_str()
    {
        "metric" => AnalysisType::Metric,
        "topological" | "topo" => AnalysisType::Topological,
        _ => AnalysisType::Angular,
    };
    let weight_mode = match args
        .get("weight_mode")
        .and_then(|v| v.as_str())
        .unwrap_or("segment_length")
        .to_ascii_lowercase()
        .as_str()
    {
        "unweighted" | "none" => WeightMode::Unweighted,
        _ => WeightMode::SegmentLength,
    };
    let max_sources_per_radius = args
        .get("max_sources_per_radius")
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .unwrap_or(0);

    let asa_cfg = AsaConfig {
        radii: radii.clone(),
        analysis_type,
        radius_type: RadiusType::Metric,
        weight_mode,
        tulip_bins,
        include_normalized: true,
        max_sources_per_radius,
    };

    let asa_res = solve_angular_segment_analysis(&graph, &asa_cfg);

    let origin_ids: Vec<usize> = args
        .get("origin_segment_ids")
        .and_then(|v| v.as_array())
        .map(|arr| {
            arr.iter()
                .filter_map(|x| x.as_u64().map(|u| u as usize))
                .filter(|&id| id < graph.num_nodes)
                .collect()
        })
        .filter(|v: &Vec<usize>| !v.is_empty())
        .unwrap_or_else(|| if graph.num_nodes > 0 { vec![0] } else { vec![] });

    let step_depth = solve_origin_step_depth(&graph, &origin_ids, f64::INFINITY, RadiusType::Metric);

    if let Some(out_str) = args
        .get("output_path")
        .or_else(|| args.get("output_geojson"))
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
    {
        export_segments_by_path(Path::new(out_str), &segments, &asa_res.attributes)?;
    }

    let mut summary_stats = serde_json::Map::new();
    for (col_name, values) in &asa_res.attributes {
        summary_stats.insert(col_name.clone(), json!(SummaryStats::from_slice(values)));
    }

    let sync_to_gui = args
        .get("sync_to_gui")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    if sync_to_gui && probe_gui_state().is_some() {
        if let Some(preset) = preset_opt {
            let _ = send_gui_command(
                "load_preset",
                json!({ "preset": canonical_preset_id(preset) }),
                250,
                4000,
            );
        }
        let _ = send_gui_command("run_asa", json!({}), 250, 6000);
    }

    Ok(json!({
        "source": source_label,
        "config": {
            "radii_m": radii,
            "tulip_bins": tulip_bins,
            "prune_whiskers": prune_whiskers,
            "clean_first": clean_first,
            "analysis_type": analysis_type.as_str(),
            "weight_mode": weight_mode.as_str(),
        },
        "planarized_topology": {
            "segments": segments.len(),
            "connected_components": connected_components,
            "junctions_deg3_plus": junctions_deg3_plus,
            "dangles_deg1": dangles_deg1,
            "grade_unlinks": unlinks.len(),
            "csr_memory_bytes": graph.memory_footprint_bytes(),
        },
        "intelligibility_r2": asa_res.intelligibility_r2,
        "intelligibility_integration_r2": asa_res.intelligibility_integration_r2,
        "synergy_r2": asa_res.synergy_r2,
        "origin_step_depth_summary": {
            "origin_ids": step_depth.origin_ids,
            "angular_step_depth": SummaryStats::from_slice(&step_depth.angular_step_depth),
            "metric_step_depth_m": SummaryStats::from_slice(&step_depth.metric_step_depth_m),
        },
        "summary_stats": Value::Object(summary_stats),
        "top_nach_rn": top_k_segments_for_attr(&segments, &asa_res.attributes, "NACH_Rn", top_k),
        "top_nain_rn": top_k_segments_for_attr(&segments, &asa_res.attributes, "NAIN_Rn", top_k),
        "elapsed_ms": asa_res.elapsed_ms,
    }))
}

// ============================================================================
// Tool 5: sse_solve_axial_and_strokes
// ============================================================================
fn tool_sse_solve_axial_and_strokes(args: &Value) -> Result<Value, String> {
    let (lines, source_label, preset_opt) =
        load_street_lines_from_args(args, BenchmarkPreset::AxialStarDiamond)?;

    let max_deflection_deg = args
        .get("max_deflection_deg")
        .and_then(|v| v.as_f64())
        .unwrap_or(45.0)
        .clamp(5.0, 90.0);
    let top_k = args
        .get("top_k")
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .unwrap_or(10)
        .clamp(1, 50);

    let axial = solve_axial_analysis(&lines, &[0, 3, 5], true);
    let strokes = solve_natural_roads(&lines, max_deflection_deg);

    if let Some(out_str) = args
        .get("output_path")
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
    {
        export_segments_by_path(Path::new(out_str), &axial.lines, &axial.attributes)?;
    }

    let mut axial_stats = serde_json::Map::new();
    for (k, col) in &axial.attributes {
        axial_stats.insert(k.clone(), json!(SummaryStats::from_slice(col)));
    }

    let closed_loop_strokes_count = strokes
        .strokes
        .iter()
        .filter(|s| s.sinuosity.is_none())
        .count();

    let mut sorted_strokes = strokes.strokes.clone();
    sorted_strokes.sort_by(|a, b| {
        b.length_m
            .partial_cmp(&a.length_m)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let top_strokes: Vec<Value> = sorted_strokes
        .iter()
        .take(top_k)
        .map(|s| {
            json!({
                "stroke_id": s.stroke_id,
                "length_m": (s.length_m * 100.0).round() / 100.0,
                "chord_length_m": (s.chord_length_m * 100.0).round() / 100.0,
                "seg_count": s.seg_count,
                "connectivity": s.connectivity,
                "sinuosity": s.sinuosity,
            })
        })
        .collect();

    let sync_to_gui = args
        .get("sync_to_gui")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    if sync_to_gui && probe_gui_state().is_some() {
        if let Some(preset) = preset_opt {
            let _ = send_gui_command(
                "load_preset",
                json!({ "preset": canonical_preset_id(preset) }),
                250,
                4000,
            );
        }
        let _ = send_gui_command("run_axial", json!({}), 250, 6000);
        let _ = send_gui_command("run_natural_roads", json!({}), 250, 6000);
    }

    Ok(json!({
        "source": source_label,
        "axial_analysis": {
            "lines_count": axial.lines.len(),
            "attributes_count": axial.attributes.len(),
            "attribute_names": axial.attributes.keys().collect::<Vec<_>>(),
            "intelligibility_r2": axial.intelligibility_r2,
            "summary_stats": Value::Object(axial_stats),
            "top_integration_hh_n": top_k_segments_for_attr(&axial.lines, &axial.attributes, "Integration_HH_n", top_k),
            "top_choice_n": top_k_segments_for_attr(&axial.lines, &axial.attributes, "Choice_n", top_k),
            "elapsed_ms": axial.elapsed_ms,
        },
        "natural_roads": {
            "max_deflection_deg": strokes.max_deflection_deg,
            "strokes_count": strokes.strokes.len(),
            "closed_loop_strokes_count": closed_loop_strokes_count,
            "circuity_avg": strokes.circuity_avg,
            "top_strokes_by_length": top_strokes,
            "elapsed_ms": strokes.elapsed_ms,
        }
    }))
}

// ============================================================================
// Tool 6: sse_solve_vga
// ============================================================================
fn tool_sse_solve_vga(args: &Value) -> Result<Value, String> {
    let (boundary_rings, obstacles, default_cell_m, source_label, preset_opt) =
        load_floorplan_from_args(args, BenchmarkPreset::ArchitecturalGalleryVga)?;

    let cell_size_m = args
        .get("cell_size_m")
        .and_then(|v| v.as_f64())
        .unwrap_or(default_cell_m)
        .max(0.04);
    let max_vis_m = args
        .get("max_vis_m")
        .and_then(|v| v.as_f64())
        .unwrap_or(100.0)
        .max(1.0);
    let include_isovist_metrics = args
        .get("include_isovist_metrics")
        .and_then(|v| v.as_bool())
        .unwrap_or(true);
    let top_k = args
        .get("top_k")
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .unwrap_or(10)
        .clamp(1, 50);

    let pts = if let (Some(sx), Some(sy)) = (
        args.get("seed_x").and_then(|v| v.as_f64()),
        args.get("seed_y").and_then(|v| v.as_f64()),
    ) {
        generate_vga_grid_flood_fill(&obstacles, Point2D::new(sx, sy), cell_size_m)
    } else {
        generate_vga_grid(&boundary_rings, &obstacles, cell_size_m, true)
    };

    if pts.is_empty() {
        return Err("VGA grid generation produced 0 walkable cells for this floorplan.".to_string());
    }

    let eff_cell_m = effective_vga_grid_spacing(&pts, cell_size_m);
    let lattice = build_walkable_vga_grid_lattice(&pts, &obstacles, eff_cell_m);
    let vga = solve_vga(
        &pts,
        &obstacles,
        eff_cell_m,
        max_vis_m,
        true,
        include_isovist_metrics,
    );

    if let Some(out_csv) = args
        .get("output_csv")
        .or_else(|| args.get("output_path"))
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
    {
        export_vga_csv(Path::new(out_csv), &vga)?;
    }

    let mut summary_stats = serde_json::Map::new();
    for (k, col) in &vga.attributes {
        summary_stats.insert(k.clone(), json!(SummaryStats::from_slice(col)));
    }

    let mut hotspots = Vec::new();
    if let Some(vi_col) = vga.attributes.get("Visual_Integration_HH") {
        let conn_col = vga.attributes.get("Connectivity");
        let mut indexed: Vec<(usize, f64)> = vi_col
            .iter()
            .copied()
            .enumerate()
            .filter(|&(_, v)| v.is_finite() && v > 0.0)
            .collect();
        indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        for (idx, val) in indexed.into_iter().take(top_k) {
            if let Some(pt) = vga.points.get(idx) {
                let conn = conn_col
                    .and_then(|c| c.get(idx))
                    .copied()
                    .unwrap_or(0.0);
                hotspots.push(json!({
                    "cell_id": pt.id,
                    "depthmap_ref": pt.depthmap_ref,
                    "x": (pt.x * 1000.0).round() / 1000.0,
                    "y": (pt.y * 1000.0).round() / 1000.0,
                    "grid_ij": [pt.grid_i, pt.grid_j],
                    "visual_integration_hh": (val * 10_000.0).round() / 10_000.0,
                    "connectivity": conn,
                }));
            }
        }
    }

    let sync_to_gui = args
        .get("sync_to_gui")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    if sync_to_gui && probe_gui_state().is_some() {
        if let Some(preset) = preset_opt {
            let _ = send_gui_command(
                "load_preset",
                json!({ "preset": canonical_preset_id(preset) }),
                250,
                4000,
            );
        }
        let _ = send_gui_command(
            "create_vga_grid",
            json!({ "cell_size_m": eff_cell_m }),
            250,
            4000,
        );
        let _ = send_gui_command("run_vga", json!({}), 250, 8000);
    }

    Ok(json!({
        "source": source_label,
        "cells_count": vga.points.len(),
        "requested_cell_size_m": cell_size_m,
        "effective_cell_size_m": eff_cell_m,
        "walkable_lattice_directed_edges": lattice.col_indices.len(),
        "visibility_directed_edges": vga.col_indices.len(),
        "attributes_count": vga.attributes.len(),
        "summary_stats": Value::Object(summary_stats),
        "top_visual_integration_hotspots": hotspots,
        "elapsed_ms": vga.elapsed_ms,
    }))
}

// ============================================================================
// Tool 7: sse_cast_isovist
// ============================================================================
fn tool_sse_cast_isovist(args: &Value) -> Result<Value, String> {
    let x = args
        .get("x")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| "Missing numeric parameter 'x' (meters).".to_string())?;
    let y = args
        .get("y")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| "Missing numeric parameter 'y' (meters).".to_string())?;
    let max_radius_m = args
        .get("max_radius_m")
        .and_then(|v| v.as_f64())
        .unwrap_or(100.0)
        .max(1.0);

    let (_, obstacles, _, source_label, _) =
        load_floorplan_from_args(args, BenchmarkPreset::ArchitecturalGalleryVga)?;

    let iso = compute_isovist_at_point(Point2D::new(x, y), &obstacles, max_radius_m);

    let sync_to_gui = args
        .get("sync_to_gui")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    if sync_to_gui && probe_gui_state().is_some() {
        let _ = send_gui_command("cast_isovist", json!({ "x": x, "y": y }), 250, 3000);
    }

    Ok(json!({
        "source": source_label,
        "origin": [iso.origin.x, iso.origin.y],
        "area": iso.area,
        "perimeter": iso.perimeter,
        "compactness": iso.compactness,
        "occlusivity": iso.occlusivity,
        "drift_magnitude": iso.drift_magnitude,
        "drift_angle_deg": iso.drift_angle_deg,
        "drift_target": [iso.drift_target.x, iso.drift_target.y],
        "min_radial": iso.min_radial,
        "max_radial": iso.max_radial,
        "polygon_vertices_count": iso.polygon.len(),
    }))
}

// ============================================================================
// Tool 8: sse_simulate_visual_agents
// ============================================================================
fn tool_sse_simulate_visual_agents(args: &Value) -> Result<Value, String> {
    let (boundary_rings, obstacles, default_cell_m, source_label, _) =
        load_floorplan_from_args(args, BenchmarkPreset::ArchitecturalGalleryVga)?;

    let cell_size_m = args
        .get("cell_size_m")
        .and_then(|v| v.as_f64())
        .unwrap_or(default_cell_m)
        .max(0.05);
    let pts = generate_vga_grid(&boundary_rings, &obstacles, cell_size_m, true);
    if pts.is_empty() {
        return Err("Floorplan produced 0 walkable VGA cells.".to_string());
    }
    let eff_cell_m = effective_vga_grid_spacing(&pts, cell_size_m);
    let vga = solve_vga(&pts, &obstacles, eff_cell_m, 100.0, false, false);

    let timesteps = args
        .get("timesteps")
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .unwrap_or(5000)
        .clamp(1, 100_000);
    let release_rate = args
        .get("release_rate")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.10)
        .max(0.0);
    let fov_bins = args
        .get("fov_bins")
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .unwrap_or(15)
        .clamp(1, 32);
    let steps_before_turn = args
        .get("steps_before_turn")
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .unwrap_or(3)
        .max(1);
    let agent_lifetime = args
        .get("agent_lifetime")
        .or_else(|| args.get("agent_life_steps"))
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .unwrap_or(1000)
        .max(1);
    let record_trails = args
        .get("record_trails")
        .and_then(|v| v.as_bool())
        .unwrap_or(true);
    let top_k = args
        .get("top_k")
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .unwrap_or(10)
        .clamp(1, 50);

    let (movement_rule, rule_id) =
        parse_agent_movement_rule(args.get("movement_rule").and_then(|v| v.as_str()));

    let mut release_cells = Vec::new();
    if let Some(rp_arr) = args.get("release_points").and_then(|v| v.as_array()) {
        for item in rp_arr {
            if let Some(pair) = item.as_array() {
                if pair.len() >= 2 {
                    if let (Some(rx), Some(ry)) = (pair[0].as_f64(), pair[1].as_f64()) {
                        let target = Point2D::new(rx, ry);
                        if let Some((best_idx, _)) = vga
                            .points
                            .iter()
                            .enumerate()
                            .filter(|&(u, _)| vga.row_offsets[u + 1] > vga.row_offsets[u])
                            .map(|(u, p)| (u, Point2D::new(p.x, p.y).dist(target)))
                            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
                        {
                            release_cells.push(best_idx);
                        }
                    }
                }
            }
        }
    }

    let has_manual_pins = !release_cells.is_empty();
    let cfg = AgentConfig {
        timesteps,
        release_rate,
        agent_life_steps: agent_lifetime,
        agent_fov_bins: fov_bins,
        agent_steps_before_turn: steps_before_turn,
        movement_rule,
        release_from_selected_only: has_manual_pins,
        record_trails,
        release_cells: release_cells.clone(),
        ..AgentConfig::default()
    };

    let sim_state = if has_manual_pins && release_rate <= 1e-9 {
        let mut st = init_agent_simulation(&vga, &cfg);
        for &cell_u in &release_cells {
            let _ = spawn_agents_at_cell(&vga, &mut st, &cfg, cell_u, 1);
        }
        step_agent_simulation_with_mode(&vga, &mut st, &cfg, timesteps, false);
        st
    } else {
        solve_agents_batch(&vga, &cfg)
    };

    let total_gate_counts: f64 = sim_state.gate_counts.iter().sum();
    let gate_stats = SummaryStats::from_slice(&sim_state.gate_counts);

    let mut indexed: Vec<(usize, f64)> = sim_state
        .gate_counts
        .iter()
        .copied()
        .enumerate()
        .filter(|&(_, g)| g > 0.0)
        .collect();
    indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    let top_cells: Vec<Value> = indexed
        .into_iter()
        .take(top_k)
        .filter_map(|(idx, g)| {
            let pt = vga.points.get(idx)?;
            Some(json!({
                "cell_id": pt.id,
                "depthmap_ref": pt.depthmap_ref,
                "x": (pt.x * 1000.0).round() / 1000.0,
                "y": (pt.y * 1000.0).round() / 1000.0,
                "grid_ij": [pt.grid_i, pt.grid_j],
                "gate_counts": g,
            }))
        })
        .collect();

    Ok(json!({
        "source": source_label,
        "config": {
            "timesteps": timesteps,
            "release_rate": release_rate,
            "fov_bins": fov_bins,
            "steps_before_turn": steps_before_turn,
            "agent_lifetime": agent_lifetime,
            "movement_rule_id": rule_id,
            "movement_rule_label": movement_rule.as_str(),
            "release_cells": release_cells,
        },
        "vga_cells_count": vga.points.len(),
        "effective_cell_size_m": eff_cell_m,
        "total_gate_counts": total_gate_counts,
        "active_agents_remaining": sim_state.active_agents.len(),
        "recorded_trails_count": sim_state.finished_trails.len(),
        "gate_counts_distribution": gate_stats,
        "top_movement_cells": top_cells,
    }))
}

// ============================================================================
// Tool 9: sse_gui_control
// ============================================================================
fn tool_sse_gui_control(args: &Value) -> Result<Value, String> {
    let action = args
        .get("action")
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .unwrap_or("get_state");
    let params = args
        .get("params")
        .cloned()
        .unwrap_or_else(|| json!({}));

    let resp = send_gui_command(action, params.clone(), 250, 10_000)?;
    Ok(json!({
        "bridge": GUI_BRIDGE_ADDR,
        "action": action,
        "params": params,
        "gui_response": resp,
    }))
}

// ============================================================================
// MCP Tool Registry & JSON Schemas
// ============================================================================
fn mcp_tools_list() -> Value {
    json!({
        "tools": [
            {
                "name": "sse_status",
                "description": "Check SSE — Space Syntax Engine version, run fast mathematical self-check (Benchmark A 3-segment chain + Hillier Diamond D_5), list built-in UCL depthmapX presets, and check if sse-desktop.exe GUI is live on 127.0.0.1:9884.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": false
                }
            },
            {
                "name": "sse_fetch_and_clean_osm",
                "description": "Geocode 'query' (place name or 'lat, lon') or use (south, west, north, east) WGS84 bbox + mode ('walk', 'drive', 'all'), fetch OpenStreetMap network, project to metric UTM, run 5-Step P90 Adaptive Watertight Cleaner + Angular Segment Analysis (ASA), optionally save .geojson, and sync to live sse-desktop.exe GUI.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "Place name or 'lat, lon' coordinate string (e.g. 'Siam Square, Bangkok' or '13.745, 100.533')."
                        },
                        "south": { "type": "number", "description": "Optional WGS84 south latitude." },
                        "west": { "type": "number", "description": "Optional WGS84 west longitude." },
                        "north": { "type": "number", "description": "Optional WGS84 north latitude." },
                        "east": { "type": "number", "description": "Optional WGS84 east longitude." },
                        "mode": {
                            "type": "string",
                            "enum": ["walk", "drive", "all"],
                            "default": "walk",
                            "description": "OSM street network filter mode."
                        },
                        "radii_m": {
                            "type": "array",
                            "items": { "type": "number" },
                            "description": "Metric horizon radii in meters (e.g. [400, 800, 1200, 2000, 5000])."
                        },
                        "output_geojson": {
                            "type": "string",
                            "description": "Optional file path to export cleaned & solved segments as GeoJSON."
                        },
                        "sync_to_gui": {
                            "type": "boolean",
                            "default": true,
                            "description": "Whether to push the fetched & solved network into the live sse-desktop.exe GUI."
                        }
                    }
                }
            },
            {
                "name": "sse_clean_network",
                "description": "Run the 5-Step P90 Adaptive Watertight Cleaner on an input vector file (input_path) or built-in preset (preset), returning the complete topology audit (raw/cleaned dangles, junctions, components, ray extensions, lateral connectors, pruned micro-stubs, and circuity_avg).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "input_path": { "type": "string", "description": "Path to .geojson, .dxf, .mif, .csv, or .osm file." },
                        "preset": {
                            "type": "string",
                            "enum": ["barnsbury_mif", "barnsbury_axial", "loop_guard", "gallery_vga", "rooms_vga"],
                            "default": "barnsbury_mif"
                        },
                        "mode": { "type": "string", "enum": ["auto", "manual"], "default": "auto" },
                        "extend_distance_m": { "type": "number", "default": 15.0 },
                        "snap_distance_m": { "type": "number", "default": 3.0 },
                        "min_stub_len_m": { "type": "number", "default": 0.5 },
                        "max_turn_deflection_deg": { "type": "number", "default": 15.0 },
                        "dead_end_stub_len_m": { "type": "number", "default": 0.0 },
                        "output_geojson": { "type": "string", "description": "Optional output path for cleaned network." },
                        "sync_to_gui": { "type": "boolean", "default": false }
                    }
                }
            },
            {
                "name": "sse_solve_asa",
                "description": "Solve multi-radius Angular Segment Analysis (ASA) on input_path or preset (radii_m, tulip_bins, prune_whiskers), returning summary stats, intelligibility_r2, synergy_r2, planarized topology (segments, connected_components, junctions_deg3_plus, dangles_deg1), and top-K segments by NACH_Rn and NAIN_Rn.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "input_path": { "type": "string", "description": "Optional path to input street network file." },
                        "preset": {
                            "type": "string",
                            "enum": ["barnsbury_mif", "barnsbury_axial", "loop_guard"],
                            "default": "barnsbury_mif"
                        },
                        "radii_m": {
                            "type": "array",
                            "items": { "type": "number" },
                            "description": "Metric horizon radii in meters (global Rn is always included)."
                        },
                        "tulip_bins": { "type": "integer", "default": 1024, "description": "1024 for depthmapX Tulip-1024, or 0 for continuous f64." },
                        "prune_whiskers": { "type": "boolean", "default": true },
                        "clean_first": { "type": "boolean", "default": false },
                        "top_k": { "type": "integer", "default": 10 },
                        "output_path": { "type": "string", "description": "Optional output path (.geojson, .csv, .dxf, .mif)." },
                        "sync_to_gui": { "type": "boolean", "default": false }
                    }
                }
            },
            {
                "name": "sse_solve_axial_and_strokes",
                "description": "Solve Hillier & Hanson 16-attribute Topological Axial Analysis and Jiang & Claramunt (2004) Gestalt Natural Roads (max_deflection_deg) with closed-loop sinuosity guard on input_path or preset.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "input_path": { "type": "string", "description": "Optional path to input network file." },
                        "preset": {
                            "type": "string",
                            "enum": ["barnsbury_axial", "barnsbury_mif", "loop_guard"],
                            "default": "barnsbury_axial"
                        },
                        "max_deflection_deg": { "type": "number", "default": 45.0 },
                        "top_k": { "type": "integer", "default": 10 },
                        "output_path": { "type": "string", "description": "Optional output path for axial attributes." },
                        "sync_to_gui": { "type": "boolean", "default": false }
                    }
                }
            },
            {
                "name": "sse_solve_vga",
                "description": "Solve Visibility Graph Analysis (VGA) on an architectural floorplan (input_path or preset: 'gallery' / 'rooms'), with cell_size_m, max_vis_m, include_isovist_metrics, returning summary stats and top Visual Integration [HH] hotspots.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "input_path": { "type": "string", "description": "Optional path to floorplan DXF/GeoJSON/MIF." },
                        "preset": {
                            "type": "string",
                            "enum": ["gallery", "rooms", "gallery_vga", "rooms_vga"],
                            "default": "gallery"
                        },
                        "cell_size_m": { "type": "number", "default": 0.30 },
                        "max_vis_m": { "type": "number", "default": 100.0 },
                        "include_isovist_metrics": { "type": "boolean", "default": true },
                        "seed_x": { "type": "number", "description": "Optional flood-fill seed X coordinate (m)." },
                        "seed_y": { "type": "number", "description": "Optional flood-fill seed Y coordinate (m)." },
                        "top_k": { "type": "integer", "default": 10 },
                        "output_csv": { "type": "string", "description": "Optional CSV output path for VGA PointMap." },
                        "sync_to_gui": { "type": "boolean", "default": false }
                    }
                }
            },
            {
                "name": "sse_cast_isovist",
                "description": "Cast an analytical 2D Benedikt (1979) corner-ray Isovist from (x, y) on input_path or preset ('gallery' / 'rooms'), returning area, perimeter, compactness, occlusivity, drift_magnitude, drift_angle_deg, min_radial, and max_radial.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "x": { "type": "number", "description": "Vantage point X coordinate in meters." },
                        "y": { "type": "number", "description": "Vantage point Y coordinate in meters." },
                        "input_path": { "type": "string", "description": "Optional floorplan file path." },
                        "preset": {
                            "type": "string",
                            "enum": ["gallery", "rooms", "gallery_vga", "rooms_vga"],
                            "default": "gallery"
                        },
                        "max_radius_m": { "type": "number", "default": 100.0 },
                        "sync_to_gui": { "type": "boolean", "default": false }
                    },
                    "required": ["x", "y"]
                }
            },
            {
                "name": "sse_simulate_visual_agents",
                "description": "Run UCL depthmapX Gibson Ecological Visual Agent simulation (Turner & Penn 2002) on input_path or preset ('gallery' / 'rooms'), supporting timesteps, release_rate, fov_bins (1..=32), steps_before_turn, agent_lifetime, movement_rule, and optional release_points [[x, y], ...], returning Gate_Counts distribution and top movement cells.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "input_path": { "type": "string", "description": "Optional floorplan file path." },
                        "preset": {
                            "type": "string",
                            "enum": ["gallery", "rooms", "gallery_vga", "rooms_vga"],
                            "default": "gallery"
                        },
                        "cell_size_m": { "type": "number", "default": 0.30 },
                        "timesteps": { "type": "integer", "default": 5000 },
                        "release_rate": { "type": "number", "default": 0.10 },
                        "fov_bins": { "type": "integer", "default": 15, "minimum": 1, "maximum": 32 },
                        "steps_before_turn": { "type": "integer", "default": 3 },
                        "agent_lifetime": { "type": "integer", "default": 1000 },
                        "movement_rule": {
                            "type": "string",
                            "enum": [
                                "standard",
                                "los_length",
                                "occ_length",
                                "occ_any",
                                "occ_group_45",
                                "occ_group_60",
                                "occ_furthest",
                                "bin_far_dist",
                                "bin_angle",
                                "bin_far_dist_angle",
                                "bin_memory"
                            ],
                            "default": "standard"
                        },
                        "release_points": {
                            "type": "array",
                            "items": {
                                "type": "array",
                                "items": { "type": "number" },
                                "minItems": 2,
                                "maxItems": 2
                            },
                            "description": "Optional release coordinates [[x, y], ...] in meters."
                        },
                        "top_k": { "type": "integer", "default": 10 }
                    }
                }
            },
            {
                "name": "sse_gui_control",
                "description": "Send a live command directly to the running sse-desktop.exe GUI on 127.0.0.1:9884 (action + params) to inspect state, load presets/files, toggle overlays, run solvers, place/play/step agents, cast isovists, zoom-to-fit, or export PNG screenshots.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": [
                                "get_state",
                                "load_preset",
                                "load_file",
                                "set_attribute",
                                "set_overlays",
                                "run_cleaner",
                                "run_asa",
                                "run_axial",
                                "run_natural_roads",
                                "create_vga_grid",
                                "run_vga",
                                "place_agent",
                                "play_agents",
                                "step_agents",
                                "cast_isovist",
                                "zoom_fit",
                                "export_png"
                            ]
                        },
                        "params": {
                            "type": "object",
                            "description": "Action parameters object forwarded to sse-desktop.exe."
                        }
                    },
                    "required": ["action"]
                }
            }
        ]
    })
}

fn dispatch_tool_call(tool_name: &str, args: &Value) -> Result<Value, String> {
    match tool_name {
        "sse_status" => tool_sse_status(),
        "sse_fetch_and_clean_osm" => tool_sse_fetch_and_clean_osm(args),
        "sse_clean_network" => tool_sse_clean_network(args),
        "sse_solve_asa" => tool_sse_solve_asa(args),
        "sse_solve_axial_and_strokes" => tool_sse_solve_axial_and_strokes(args),
        "sse_solve_vga" => tool_sse_solve_vga(args),
        "sse_cast_isovist" => tool_sse_cast_isovist(args),
        "sse_simulate_visual_agents" => tool_sse_simulate_visual_agents(args),
        "sse_gui_control" => tool_sse_gui_control(args),
        other => Err(format!("Unknown MCP tool '{other}'.")),
    }
}

fn handle_jsonrpc_message(msg: &Value) -> Option<Value> {
    let id = msg.get("id").cloned();
    let is_notification = id.is_none() || id.as_ref().is_some_and(Value::is_null);
    let method = msg.get("method").and_then(|v| v.as_str()).unwrap_or("");
    let params = msg.get("params").cloned().unwrap_or_else(|| json!({}));

    if is_notification {
        // Notifications (e.g. notifications/initialized, notifications/cancelled) must not emit a response
        return None;
    }

    let id_val = id.unwrap_or(Value::Null);

    match method {
        "initialize" => {
            let client_proto = params
                .get("protocolVersion")
                .and_then(|v| v.as_str())
                .unwrap_or(DEFAULT_PROTOCOL_VERSION);
            Some(json!({
                "jsonrpc": "2.0",
                "id": id_val,
                "result": {
                    "protocolVersion": client_proto,
                    "capabilities": {
                        "tools": {}
                    },
                    "serverInfo": {
                        "name": SERVER_NAME,
                        "version": SERVER_VERSION
                    }
                }
            }))
        }
        "ping" => Some(json!({
            "jsonrpc": "2.0",
            "id": id_val,
            "result": {}
        })),
        "tools/list" => Some(json!({
            "jsonrpc": "2.0",
            "id": id_val,
            "result": mcp_tools_list()
        })),
        "resources/list" => Some(json!({
            "jsonrpc": "2.0",
            "id": id_val,
            "result": { "resources": [] }
        })),
        "prompts/list" => Some(json!({
            "jsonrpc": "2.0",
            "id": id_val,
            "result": { "prompts": [] }
        })),
        "tools/call" => {
            let Some(tool_name) = params.get("name").and_then(|v| v.as_str()) else {
                return Some(json!({
                    "jsonrpc": "2.0",
                    "id": id_val,
                    "error": {
                        "code": -32602,
                        "message": "Missing 'name' in tools/call params"
                    }
                }));
            };
            let arguments = params
                .get("arguments")
                .cloned()
                .unwrap_or_else(|| json!({}));

            match dispatch_tool_call(tool_name, &arguments) {
                Ok(payload) => {
                    let text = serde_json::to_string_pretty(&payload)
                        .unwrap_or_else(|_| payload.to_string());
                    Some(json!({
                        "jsonrpc": "2.0",
                        "id": id_val,
                        "result": {
                            "content": [
                                {
                                    "type": "text",
                                    "text": text
                                }
                            ],
                            "isError": false
                        }
                    }))
                }
                Err(err_msg) => {
                    let err_json = json!({
                        "tool": tool_name,
                        "error": err_msg
                    });
                    Some(json!({
                        "jsonrpc": "2.0",
                        "id": id_val,
                        "result": {
                            "content": [
                                {
                                    "type": "text",
                                    "text": serde_json::to_string_pretty(&err_json).unwrap_or(err_msg)
                                }
                            ],
                            "isError": true
                        }
                    }))
                }
            }
        }
        other => Some(json!({
            "jsonrpc": "2.0",
            "id": id_val,
            "error": {
                "code": -32601,
                "message": format!("Method not found: {other}")
            }
        })),
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum FramingMode {
    LineDelimited,
    ContentLength,
}

fn read_next_rpc_message<R: BufRead>(
    reader: &mut R,
) -> io::Result<Option<(Value, FramingMode)>> {
    loop {
        let mut first_line = String::new();
        let n = reader.read_line(&mut first_line)?;
        if n == 0 {
            return Ok(None);
        }
        let trimmed = first_line.trim_start_matches('\u{feff}').trim();
        if trimmed.is_empty() {
            continue;
        }

        // Case 1: Content-Length header framing
        let lower = trimmed.to_ascii_lowercase();
        if lower.starts_with("content-length:") || lower.starts_with("content-type:") {
            let mut content_length: Option<usize> = None;
            if let Some(rest) = lower.strip_prefix("content-length:") {
                content_length = rest.trim().parse::<usize>().ok();
            }

            // Read remaining HTTP-style headers until blank line
            loop {
                let mut hdr_line = String::new();
                let hn = reader.read_line(&mut hdr_line)?;
                if hn == 0 {
                    return Ok(None);
                }
                let h_trim = hdr_line.trim();
                if h_trim.is_empty() {
                    break;
                }
                let h_low = h_trim.to_ascii_lowercase();
                if let Some(rest) = h_low.strip_prefix("content-length:") {
                    content_length = rest.trim().parse::<usize>().ok();
                }
            }

            let Some(len) = content_length else {
                continue;
            };
            let mut buf = vec![0u8; len];
            reader.read_exact(&mut buf)?;
            if let Ok(val) = serde_json::from_slice::<Value>(&buf) {
                return Ok(Some((val, FramingMode::ContentLength)));
            }
            continue;
        }

        // Case 2: Newline-delimited JSON-RPC
        if let Ok(val) = serde_json::from_str::<Value>(trimmed) {
            return Ok(Some((val, FramingMode::LineDelimited)));
        }
    }
}

fn write_rpc_response<W: Write>(
    writer: &mut W,
    resp: &Value,
    mode: FramingMode,
) -> io::Result<()> {
    let serialized = serde_json::to_string(resp)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    match mode {
        FramingMode::ContentLength => {
            write!(
                writer,
                "Content-Length: {}\r\n\r\n{}",
                serialized.len(),
                serialized
            )?;
        }
        FramingMode::LineDelimited => {
            writeln!(writer, "{serialized}")?;
        }
    }
    writer.flush()
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.iter().any(|a| a == "--version" || a == "-V") {
        println!("{SERVER_NAME} {SERVER_VERSION}");
        return;
    }
    if args.iter().any(|a| a == "--self-test") {
        match tool_sse_status() {
            Ok(v) => {
                println!(
                    "{}",
                    serde_json::to_string_pretty(&v).unwrap_or_else(|_| v.to_string())
                );
                return;
            }
            Err(e) => {
                eprintln!("Self-test error: {e}");
                std::process::exit(1);
            }
        }
    }

    let stdin = io::stdin();
    let mut reader = BufReader::new(stdin.lock());
    let stdout = io::stdout();
    let mut writer = stdout.lock();

    while let Ok(Some((msg, mode))) = read_next_rpc_message(&mut reader) {
        if let Some(batch) = msg.as_array() {
            let mut responses = Vec::new();
            for item in batch {
                if let Some(r) = handle_jsonrpc_message(item) {
                    responses.push(r);
                }
            }
            if !responses.is_empty() {
                let _ = write_rpc_response(&mut writer, &Value::Array(responses), mode);
            }
        } else if let Some(resp) = handle_jsonrpc_message(&msg) {
            let _ = write_rpc_response(&mut writer, &resp, mode);
        }
    }
}
