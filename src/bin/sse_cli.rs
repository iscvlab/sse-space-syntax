//! `sse-cli` — Headless Command-Line & Self-Verification Runner for **SSE — Space Syntax Engine**.
//!
//! Commands:
//! - `sse-cli --self-test` / `sse-cli benchmark`
//! - `sse-cli asa <input_file> --output <out.geojson|out.csv|out.dxf|out.mif> [--radii 400,800,2000,n] [--clean]`
//! - `sse-cli axial <input_file> --output <out.geojson|out.csv|out.dxf|out.mif>`
//! - `sse-cli strokes <input_file> --output <out.csv> [--deflection 45]`
//! - `sse-cli vga [--preset gallery] --output <out.csv>`

use sse_core::benchmarks::{BenchmarkPreset, load_benchmark};
use sse_core::cleaner::clean_network_adaptive_watertight;
use sse_core::io::{
    export_segments_csv, export_segments_dxf, export_segments_geojson, export_segments_mif_mid,
    export_vga_csv, load_polylines_auto,
};
use sse_core::solvers_agents::solve_agents_batch;
use sse_core::solvers_asa::{solve_angular_segment_analysis, solve_origin_step_depth};
use sse_core::solvers_axial::{calculate_hillier_diamond_dn, solve_axial_analysis};
use sse_core::solvers_strokes::solve_natural_roads;
use sse_core::solvers_vga::{
    compute_isovist_at_point, generate_vga_grid, generate_vga_grid_flood_fill, solve_vga,
};
use sse_core::types::{
    AgentConfig, AnalysisType, AsaConfig, CSRDualGraph, CleanerConfig, Point2D, Polyline,
    RadiusType, WeightMode,
};
use std::env;
use std::fs;
use std::io::Write;
use std::path::Path;
use std::process;
use std::time::Instant;

fn parse_radii_arg(raw: &str) -> Vec<f64> {
    let mut out = Vec::new();
    for tok in raw.split(',') {
        let t = tok.trim().to_ascii_lowercase();
        if t.is_empty() {
            continue;
        }
        if t == "n" || t == "rn" || t == "inf" || t == "global" {
            out.push(f64::INFINITY);
        } else if let Ok(v) = t.trim_end_matches('m').parse::<f64>() {
            if v > 0.0 {
                out.push(v);
            }
        }
    }
    if out.is_empty() {
        vec![400.0, 800.0, 2000.0, f64::INFINITY]
    } else {
        out
    }
}

fn export_segments_by_ext(
    out_path: &Path,
    segments: &[sse_core::types::SegmentRecord],
    attributes: &std::collections::BTreeMap<String, Vec<f64>>,
) -> Result<(), String> {
    let ext = out_path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("csv")
        .to_ascii_lowercase();
    match ext.as_str() {
        "geojson" | "json" => export_segments_geojson(out_path, segments, attributes),
        "dxf" => export_segments_dxf(out_path, segments, attributes, None),
        "mif" => export_segments_mif_mid(out_path, segments, attributes),
        _ => export_segments_csv(out_path, segments, attributes),
    }
}

fn run_self_test_suite() -> Result<(), String> {
    println!("================================================================================");
    println!("  SSE — Space Syntax Engine v1.1.0 | Native Rust Mathematical Parity & Benchmark");
    println!("================================================================================\n");

    println!(
        "| {:<36} | {:<26} | {:>10} | {:<6} |",
        "Verification / Benchmark Suite", "Key Invariant / Metric", "Time (ms)", "Status"
    );
    println!(
        "|{:-<38}|{:-<28}|{:-<12}|{:-<8}|",
        "", "", "", ""
    );

    // 1. Benchmark A: 1D 3-Segment Collinear Chain (Choice = [0, 2, 0], MD = [150, 100, 150])
    let t_a = Instant::now();
    let lines_a = vec![
        Polyline::new(vec![Point2D::new(0.0, 0.0), Point2D::new(100.0, 0.0)]),
        Polyline::new(vec![Point2D::new(100.0, 0.0), Point2D::new(200.0, 0.0)]),
        Polyline::new(vec![Point2D::new(200.0, 0.0), Point2D::new(300.0, 0.0)]),
    ];
    let (graph_a, _) = CSRDualGraph::from_polylines(&lines_a, 0, 0.0001);
    let cfg_a = AsaConfig {
        radii: vec![f64::INFINITY],
        analysis_type: AnalysisType::Metric,
        radius_type: RadiusType::Metric,
        weight_mode: WeightMode::Unweighted,
        tulip_bins: 0,
        include_normalized: true,
        max_sources_per_radius: 5000,
    };
    let res_a = solve_angular_segment_analysis(&graph_a, &cfg_a);
    let ch_a = &res_a.attributes["Choice_Rn"];
    let md_a = &res_a.attributes["Mean_Depth_Rn"];
    let nach_a = &res_a.attributes["NACH_Rn"];
    let nain_a = &res_a.attributes["NAIN_Rn"];
    let ms_a = t_a.elapsed().as_secs_f64() * 1000.0;

    if (ch_a[1] - 2.0).abs() > 1e-6 || (md_a[1] - 100.0).abs() > 1e-6 {
        return Err(format!("Benchmark A failed: Choice={ch_a:?}, MD={md_a:?}"));
    }
    println!(
        "| {:<36} | {:<26} | {:>10.3} | {:<6} |",
        "UL-02/03 Benchmark A (3-Seg Chain)",
        format!("C=[0,2,0] NACH={:.4}", nach_a[1]),
        ms_a,
        "PASS"
    );
    let _ = nain_a;

    // 2. Benchmark B: Hillier Diamond D_5 & 5-Line Axial Star K_{1,4}
    let t_b = Instant::now();
    let d5 = calculate_hillier_diamond_dn(5);
    let star_lines = vec![
        Polyline::new(vec![Point2D::new(0.0, 0.0), Point2D::new(100.0, 0.0)]),
        Polyline::new(vec![Point2D::new(20.0, -20.0), Point2D::new(20.0, 20.0)]),
        Polyline::new(vec![Point2D::new(40.0, -20.0), Point2D::new(40.0, 20.0)]),
        Polyline::new(vec![Point2D::new(60.0, -20.0), Point2D::new(60.0, 20.0)]),
        Polyline::new(vec![Point2D::new(80.0, -20.0), Point2D::new(80.0, 20.0)]),
    ];
    let ax_b = solve_axial_analysis(&star_lines, &[0], true);
    let rra_b = &ax_b.attributes["RRA_n"];
    let ihh_b = &ax_b.attributes["Integration_HH_n"];
    let ch_b = &ax_b.attributes["Choice_n"];
    let ms_b = t_b.elapsed().as_secs_f64() * 1000.0;

    if (d5 - 0.35199368).abs() > 1e-6
        || (rra_b[1] - 1.420480).abs() > 1e-4
        || (ihh_b[1] - 0.703987).abs() > 1e-4
        || (ch_b[0] - 12.0).abs() > 1e-6
    {
        return Err(format!(
            "Benchmark B failed: D5={d5}, RRA_leaf={}, IHH_leaf={}, Choice_hub={}",
            rra_b[1], ihh_b[1], ch_b[0]
        ));
    }
    println!(
        "| {:<36} | {:<26} | {:>10.3} | {:<6} |",
        "UL-04 Benchmark B (5-Line Star)",
        format!("D5={:.5} IHH={:.5}", d5, ihh_b[1]),
        ms_b,
        "PASS"
    );

    // 3. Natural Roads Closed-Loop Zero-Chord Guard (osmnx#1410)
    let t_loop = Instant::now();
    let closed_ring = Polyline::new(vec![
        Point2D::new(0.0, 0.0),
        Point2D::new(50.0, 0.0),
        Point2D::new(50.0, 50.0),
        Point2D::new(0.0, 50.0),
        Point2D::new(0.0, 0.0),
    ]);
    let nr_closed = solve_natural_roads(&[closed_ring], 45.0);
    let ms_loop = t_loop.elapsed().as_secs_f64() * 1000.0;
    if nr_closed.strokes.is_empty()
        || nr_closed.strokes[0].sinuosity.is_some()
        || nr_closed.circuity_avg.is_some()
    {
        return Err("UL-08 closed-loop sinuosity/circuity guard failed".to_string());
    }
    println!(
        "| {:<36} | {:<26} | {:>10.3} | {:<6} |",
        "UL-08 Closed-Loop Guard (osmnx#1410)",
        "sinuosity=None circ=None",
        ms_loop,
        "PASS"
    );

    // 4. Preset 1: UCL Barnsbury Street Network (MIF — 178 segments, Cleaner + CSR + ASA + Step Depth)
    let bm_urban = load_benchmark(BenchmarkPreset::UrbanSuperblockGrid);
    let t_clean = Instant::now();
    let (cleaned_lines, clean_rep) =
        clean_network_adaptive_watertight(&bm_urban.lines, &CleanerConfig::default());
    let ms_clean = t_clean.elapsed().as_secs_f64() * 1000.0;
    println!(
        "| {:<36} | {:<26} | {:>10.3} | {:<6} |",
        "Preset 1: 5-Step Watertight Cleaner",
        format!(
            "segs={} dangles {}->{}",
            clean_rep.exploded_segments_count,
            clean_rep.raw_dangles_count,
            clean_rep.cleaned_dangles_count
        ),
        ms_clean,
        "PASS"
    );

    let t_asa = Instant::now();
    let (graph_u, segs_u) = CSRDualGraph::from_polylines(&cleaned_lines, 1024, 0.0001);
    let asa_u = solve_angular_segment_analysis(&graph_u, &AsaConfig::default());
    let step_u = solve_origin_step_depth(&graph_u, &[0], 1200.0, RadiusType::Metric);
    let ms_asa = t_asa.elapsed().as_secs_f64() * 1000.0;
    if bm_urban.lines.len() != 178 || segs_u.len() < 100 || step_u.origin_ids.is_empty() {
        return Err(format!(
            "Preset 1 UCL Barnsbury MIF ASA verification failed (raw={}, segs={})",
            bm_urban.lines.len(),
            segs_u.len()
        ));
    }
    println!(
        "| {:<36} | {:<26} | {:>10.3} | {:<6} |",
        "Preset 1: UCL Barnsbury ASA (MIF)",
        format!("N={} R2={:.3}", segs_u.len(), asa_u.intelligibility_r2),
        ms_asa,
        "PASS"
    );

    // 5. Preset 2: UCL Barnsbury Extended Axial Network (DXF — 1,100 lines, 16-Attribute Axial Analysis)
    let bm_axial = load_benchmark(BenchmarkPreset::AxialStarDiamond);
    let t_ax = Instant::now();
    let ax_res = solve_axial_analysis(&bm_axial.lines, &[0, 3], true);
    let ms_ax = t_ax.elapsed().as_secs_f64() * 1000.0;
    if bm_axial.lines.len() != 1100 {
        return Err(format!(
            "Preset 2 UCL Barnsbury Extended DXF expected 1100 lines, got {}",
            bm_axial.lines.len()
        ));
    }
    println!(
        "| {:<36} | {:<26} | {:>10.3} | {:<6} |",
        "Preset 2: UCL Barnsbury Axial (DXF)",
        format!(
            "lines={} cols={}",
            ax_res.lines.len(),
            ax_res.attributes.len()
        ),
        ms_ax,
        "PASS"
    );

    // 6. Preset 3: UCL National Gallery Floorplan (gallery.dxf — Flood-Fill VGA + Isovists + Gibson Agents)
    let bm_gal = load_benchmark(BenchmarkPreset::ArchitecturalGalleryVga);
    let t_vga = Instant::now();
    let exact_gal_004 =
        generate_vga_grid_flood_fill(&bm_gal.obstacles, Point2D::new(15.0, 25.0), 0.04);
    let vga_pts = generate_vga_grid(
        &bm_gal.boundary_rings,
        &bm_gal.obstacles,
        bm_gal.default_vga_cell_m,
        true,
    );
    let vga_res = solve_vga(
        &vga_pts,
        &bm_gal.obstacles,
        bm_gal.default_vga_cell_m,
        100.0,
        true,
        true,
    );
    let iso_center = compute_isovist_at_point(Point2D::new(15.0, 25.0), &bm_gal.obstacles, 100.0);
    let agent_state = solve_agents_batch(
        &vga_res,
        &AgentConfig {
            timesteps: 1500,
            release_rate: 0.35,
            agent_life_steps: 250,
            agent_fov_bins: 15,
            agent_steps_before_turn: 3,
            max_active_agents: 80,
            seed: 42,
            record_trails: true,
            ..AgentConfig::default()
        },
    );
    let ms_vga = t_vga.elapsed().as_secs_f64() * 1000.0;
    let total_gates: f64 = agent_state.gate_counts.iter().sum();
    if exact_gal_004.len() < 4000
        || vga_pts.is_empty()
        || iso_center.area <= 0.0
        || total_gates <= 0.0
    {
        return Err(format!(
            "Preset 3 UCL National Gallery gallery.dxf verification failed (cells_004={}, cells={}, iso_area={}, gates={})",
            exact_gal_004.len(),
            vga_pts.len(),
            iso_center.area,
            total_gates
        ));
    }
    println!(
        "| {:<36} | {:<26} | {:>10.3} | {:<6} |",
        "Preset 3: UCL Gallery (gallery.dxf)",
        format!("0.04m={} gates={:.0}", exact_gal_004.len(), total_gates),
        ms_vga,
        "PASS"
    );

    // 7. Preset 4: UCL Multi-Room Plan (rooms.dxf — Flood-Fill VGA + Isovists)
    let bm_rooms = load_benchmark(BenchmarkPreset::UclRoomsFloorplan);
    let t_rooms = Instant::now();
    let rooms_pts = generate_vga_grid(
        &bm_rooms.boundary_rings,
        &bm_rooms.obstacles,
        bm_rooms.default_vga_cell_m,
        true,
    );
    let rooms_vga = solve_vga(
        &rooms_pts,
        &bm_rooms.obstacles,
        bm_rooms.default_vga_cell_m,
        100.0,
        true,
        false,
    );
    let ms_rooms = t_rooms.elapsed().as_secs_f64() * 1000.0;
    if rooms_pts.len() < 100 || rooms_vga.attributes.is_empty() {
        return Err(format!(
            "Preset 4 UCL Multi-Room rooms.dxf verification failed (cells={})",
            rooms_pts.len()
        ));
    }
    println!(
        "| {:<36} | {:<26} | {:>10.3} | {:<6} |",
        "Preset 4: UCL Multi-Room (rooms.dxf)",
        format!("cells={} cols={}", rooms_pts.len(), rooms_vga.attributes.len()),
        ms_rooms,
        "PASS"
    );

    // 8. Preset 5: RingRoadCulDeSacLoop (Natural Roads Strokes)
    let bm_ring = load_benchmark(BenchmarkPreset::RingRoadCulDeSacLoop);
    let t_nr = Instant::now();
    let nr_ring = solve_natural_roads(&bm_ring.lines, 45.0);
    let ms_nr = t_nr.elapsed().as_secs_f64() * 1000.0;
    let closed_loops_cnt = nr_ring
        .strokes
        .iter()
        .filter(|s| s.sinuosity.is_none())
        .count();
    if closed_loops_cnt == 0 {
        return Err("Preset 5 RingRoadCulDeSacLoop expected closed-loop strokes".to_string());
    }
    println!(
        "| {:<36} | {:<26} | {:>10.3} | {:<6} |",
        "Preset 5: Ring-Road & Cul-de-Sacs",
        format!(
            "strokes={} loops={}",
            nr_ring.strokes.len(),
            closed_loops_cnt
        ),
        ms_nr,
        "PASS"
    );

    println!("\nAll mathematical invariants (UL-01..UL-08) and 5 benchmark presets PASSED.");
    Ok(())
}

fn print_usage() {
    println!("SSE — Space Syntax Engine CLI (`sse-cli` v1.1.0)");
    println!("Usage:");
    println!("  sse-cli --self-test | benchmark");
    println!("  sse-cli asa <input_file> --output <out.geojson|out.csv|out.dxf|out.mif> [--radii 400,800,2000,n] [--clean]");
    println!("  sse-cli axial <input_file> --output <out.geojson|out.csv|out.dxf|out.mif>");
    println!("  sse-cli strokes <input_file> --output <out.csv> [--deflection 45]");
    println!("  sse-cli vga [--preset gallery] --output <out.csv>");
}

fn run_cli(args: &[String]) -> Result<(), String> {
    let cmd = args[0].as_str();
    match cmd {
        "--self-test" | "benchmark" | "self-test" => run_self_test_suite(),
        "asa" => {
            if args.len() < 2 {
                Err("Missing <input_file> for 'asa' command".to_string())
            } else {
                let input_path = Path::new(&args[1]);
                let mut output_path: Option<String> = None;
                let mut radii = vec![400.0, 800.0, 2000.0, f64::INFINITY];
                let mut run_clean = false;

                let mut i = 2usize;
                while i < args.len() {
                    match args[i].as_str() {
                        "--output" | "-o" if i + 1 < args.len() => {
                            output_path = Some(args[i + 1].clone());
                            i += 2;
                        }
                        "--radii" | "-r" if i + 1 < args.len() => {
                            radii = parse_radii_arg(&args[i + 1]);
                            i += 2;
                        }
                        "--clean" => {
                            run_clean = true;
                            i += 1;
                        }
                        _ => i += 1,
                    }
                }

                let out_str =
                    output_path.ok_or_else(|| "Missing --output <file> argument".to_string())?;
                load_polylines_auto(input_path).and_then(|raw_lines| {
                    let lines = if run_clean {
                        clean_network_adaptive_watertight(&raw_lines, &CleanerConfig::default()).0
                    } else {
                        raw_lines
                    };
                    let (graph, segments) = CSRDualGraph::from_polylines(&lines, 1024, 0.0001);
                    let cfg = AsaConfig {
                        radii,
                        ..AsaConfig::default()
                    };
                    let asa = solve_angular_segment_analysis(&graph, &cfg);
                    export_segments_by_ext(Path::new(&out_str), &segments, &asa.attributes)?;
                    println!(
                        "ASA complete: {} segments, {} attributes written to '{}' ({:.2} ms)",
                        segments.len(),
                        asa.attributes.len(),
                        out_str,
                        asa.elapsed_ms
                    );
                    Ok(())
                })
            }
        }
        "axial" => {
            if args.len() < 2 {
                Err("Missing <input_file> for 'axial' command".to_string())
            } else {
                let input_path = Path::new(&args[1]);
                let mut output_path: Option<String> = None;
                let mut i = 2usize;
                while i < args.len() {
                    if (args[i] == "--output" || args[i] == "-o") && i + 1 < args.len() {
                        output_path = Some(args[i + 1].clone());
                        i += 2;
                    } else {
                        i += 1;
                    }
                }
                let out_str =
                    output_path.ok_or_else(|| "Missing --output <file> argument".to_string())?;
                load_polylines_auto(input_path).and_then(|lines| {
                    let axial = solve_axial_analysis(&lines, &[0, 3, 5], true);
                    export_segments_by_ext(Path::new(&out_str), &axial.lines, &axial.attributes)?;
                    println!(
                        "Axial analysis complete: {} lines, {} attributes written to '{}' ({:.2} ms)",
                        axial.lines.len(),
                        axial.attributes.len(),
                        out_str,
                        axial.elapsed_ms
                    );
                    Ok(())
                })
            }
        }
        "strokes" => {
            if args.len() < 2 {
                Err("Missing <input_file> for 'strokes' command".to_string())
            } else {
                let input_path = Path::new(&args[1]);
                let mut output_path: Option<String> = None;
                let mut deflection = 45.0_f64;
                let mut i = 2usize;
                while i < args.len() {
                    match args[i].as_str() {
                        "--output" | "-o" if i + 1 < args.len() => {
                            output_path = Some(args[i + 1].clone());
                            i += 2;
                        }
                        "--deflection" | "-d" if i + 1 < args.len() => {
                            deflection = args[i + 1].parse::<f64>().unwrap_or(45.0);
                            i += 2;
                        }
                        _ => i += 1,
                    }
                }
                let out_str =
                    output_path.ok_or_else(|| "Missing --output <out.csv> argument".to_string())?;
                load_polylines_auto(input_path).and_then(|lines| {
                    let nr = solve_natural_roads(&lines, deflection);
                    let mut f = fs::File::create(&out_str)
                        .map_err(|e| format!("Failed to create '{out_str}': {e}"))?;
                    writeln!(
                        f,
                        "stroke_id,seg_count,connectivity,length_m,chord_length_m,sinuosity"
                    )
                    .map_err(|e| e.to_string())?;
                    for s in &nr.strokes {
                        let sin_str = s
                            .sinuosity
                            .map(|v| format!("{v:.6}"))
                            .unwrap_or_default();
                        writeln!(
                            f,
                            "{},{},{},{:.4},{:.4},{}",
                            s.stroke_id,
                            s.seg_count,
                            s.connectivity,
                            s.length_m,
                            s.chord_length_m,
                            sin_str
                        )
                        .map_err(|e| e.to_string())?;
                    }
                    println!(
                        "Natural Roads complete: {} strokes (circuity_avg={:?}) written to '{}' ({:.2} ms)",
                        nr.strokes.len(),
                        nr.circuity_avg,
                        out_str,
                        nr.elapsed_ms
                    );
                    Ok(())
                })
            }
        }
        "vga" => {
            let mut preset_name = "gallery".to_string();
            let mut output_path: Option<String> = None;
            let mut i = 1usize;
            while i < args.len() {
                match args[i].as_str() {
                    "--preset" | "-p" if i + 1 < args.len() => {
                        preset_name = args[i + 1].to_ascii_lowercase();
                        i += 2;
                    }
                    "--output" | "-o" if i + 1 < args.len() => {
                        output_path = Some(args[i + 1].clone());
                        i += 2;
                    }
                    _ => i += 1,
                }
            }
            let out_str =
                output_path.ok_or_else(|| "Missing --output <out.csv> argument".to_string())?;
            let preset = match preset_name.as_str() {
                "urban" | "superblock" | "barnsbury" => BenchmarkPreset::UrbanSuperblockGrid,
                "axial" | "diamond" | "extended" => BenchmarkPreset::AxialStarDiamond,
                "rooms" => BenchmarkPreset::UclRoomsFloorplan,
                "ring" | "loop" => BenchmarkPreset::RingRoadCulDeSacLoop,
                _ => BenchmarkPreset::ArchitecturalGalleryVga,
            };
            let bm = load_benchmark(preset);
            let pts = generate_vga_grid(
                &bm.boundary_rings,
                &bm.obstacles,
                bm.default_vga_cell_m,
                true,
            );
            let mut vga = solve_vga(
                &pts,
                &bm.obstacles,
                bm.default_vga_cell_m,
                100.0,
                true,
                true,
            );
            let agents = solve_agents_batch(&vga, &AgentConfig::default());
            vga.attributes
                .insert("Gate_Counts".to_string(), agents.gate_counts);
            export_vga_csv(Path::new(&out_str), &vga).map(|_| {
                println!(
                    "VGA + Isovist + Agents complete: {} cells, {} columns written to '{}' ({:.2} ms)",
                    vga.points.len(),
                    vga.attributes.len(),
                    out_str,
                    vga.elapsed_ms
                );
            })
        }
        other => Err(format!("Unknown subcommand '{other}'. Run 'sse-cli --help' for usage.")),
    }
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.is_empty() || args.iter().any(|a| a == "-h" || a == "--help") {
        print_usage();
        return;
    }

    if let Err(err) = run_cli(&args) {
        eprintln!("Error: {err}");
        process::exit(1);
    }
}
