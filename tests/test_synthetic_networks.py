# -*- coding: utf-8 -*-
"""
OpenDepthmap Test Suite: Synthetic Networks Verification
Validates mathematical correctness of:
1. CSRDualGraph turn deflection costs (Continuous & Tulip-1024)
2. ASA Choice & Integration formulas + Universal Normalized [0.0, 1.0] outputs
3. Analysis Type x Radius Type x Weighting Mode perturbation (UL-05, UL-06)
4. Grade separation overpass unlinking & dead-end stub pruning
5. Origin Step Depth & Detailed CSV + Summary R² diagnostics export
6. Hillier Axial D_n & Intelligibility R²
7. Natural Roads Gestalt stroke concatenation
8. VGA Line-of-sight & Isovists
"""

import math
import os
import sys
import tempfile
import numpy as np
from shapely.geometry import LineString, Point, Polygon

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SSE = os.path.join(_ROOT, "sse")
if os.path.isdir(_SSE) and _SSE not in sys.path:
    sys.path.insert(0, _SSE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from core import (  # noqa: E402
    calculate_turn_deflection,
    detect_grade_unlinks,
    CSRDualGraph,
    solve_angular_segment_analysis,
    solve_origin_step_depth,
    compute_network_summary_diagnostics,
    export_detailed_asa_tables,
    calculate_hillier_diamond_dn,
    concatenate_natural_road_strokes,
    compute_isovist_at_point,
    solve_vga_visibility_graph,
    planarize_and_node,
    explode_to_2point_segments,
    clean_network_adaptive_watertight,
    remove_dead_end_stubs,
    extract_grade_level_from_props,
)


def _build_3x3_grid_segments():
    """Helper returning planarized 2-point segments for a 3x3 orthogonal street grid."""
    lines = [
        LineString([(0, 0), (200, 0)]),
        LineString([(0, 100), (200, 100)]),
        LineString([(0, 200), (200, 200)]),
        LineString([(0, 0), (0, 200)]),
        LineString([(100, 0), (100, 200)]),
        LineString([(200, 0), (200, 200)]),
    ]
    noded = planarize_and_node(lines)
    return explode_to_2point_segments(noded)


def test_turn_deflection():
    """Validates Turner (2007) angular turn costs."""
    # Straight through: 0 rad -> cost 0
    assert abs(calculate_turn_deflection(0.0, math.pi)) < 1e-4
    # 90° right turn -> cost 1.0
    assert abs(calculate_turn_deflection(0.0, math.pi * 0.5) - 1.0) < 1e-4
    # U-turn / reversal -> cost 2.0
    assert abs(calculate_turn_deflection(0.0, 0.0) - 2.0) < 1e-4
    print("✓ test_turn_deflection passed")


def test_hillier_diamond():
    """Validates Hillier Diamond Dn benchmark values."""
    assert calculate_hillier_diamond_dn(1) == 1.0
    assert calculate_hillier_diamond_dn(2) == 1.0
    d10 = calculate_hillier_diamond_dn(10)
    assert 0.0 < d10 < 1.0
    d100 = calculate_hillier_diamond_dn(100)
    assert 0.0 < d100 < d10
    print("✓ test_hillier_diamond passed")


def test_grid_segment_analysis():
    """Validates 3x3 orthogonal street grid segment analysis."""
    segments = _build_3x3_grid_segments()
    assert len(segments) == 12  # 3 segments per direction * 2 + 3 * 2 = 12

    # Build CSR Dual Graph
    graph, _recs = CSRDualGraph.from_segments(segments)
    assert graph.num_nodes == 12

    # Run ASA (including universal [0.0, 1.0] normalized outputs)
    res = solve_angular_segment_analysis(
        graph,
        metric_radii=[150.0, float("inf")],
        include_normalized=True,
    )
    assert "connectivity" in res
    assert "angular_connectivity" in res
    assert "choice_r150" in res
    assert "nach_r150" in res
    assert "integration_rn" in res
    assert "mean_depth_rn" in res
    assert "nach_norm_r150" in res
    assert "nain_norm_r150" in res
    assert "nach_norm_rn" in res
    assert "nain_norm_rn" in res
    assert np.all((res["nach_norm_rn"] >= 0.0) & (res["nach_norm_rn"] <= 1.0))
    assert np.all((res["nain_norm_rn"] >= 0.0) & (res["nain_norm_rn"] <= 1.0))
    assert abs(float(np.max(res["nach_norm_rn"])) - 1.0) < 1e-4

    # UL-03 empty graph guard check
    empty_res = solve_angular_segment_analysis(CSRDualGraph(0), include_normalized=True)
    assert len(empty_res["nach_norm_rn"]) == 0
    assert len(empty_res["nain_norm_rn"]) == 0

    print("✓ test_grid_segment_analysis passed")


def test_analysis_radius_and_weighting_modes():
    """
    Test A (UL-05, UL-06): Validates Analysis Type x Radius Type x Weighting Mode perturbations.
    """
    # Use asymmetric grid with distinct segment lengths (60m, 140m, 80m, 120m)
    asym_lines = [
        LineString([(0, 0), (200, 0)]),
        LineString([(0, 80), (200, 80)]),
        LineString([(0, 200), (200, 200)]),
        LineString([(0, 0), (0, 200)]),
        LineString([(60, 0), (60, 200)]),
        LineString([(200, 0), (200, 200)]),
    ]
    segments = explode_to_2point_segments(planarize_and_node(asym_lines))
    graph, _ = CSRDualGraph.from_segments(segments)
    assert graph.num_nodes == 12

    # 1. Analysis Type perturbation: angular vs metric vs topological
    res_ang = solve_angular_segment_analysis(
        graph, metric_radii=[float("inf")], analysis_type="angular"
    )
    res_met = solve_angular_segment_analysis(
        graph, metric_radii=[float("inf")], analysis_type="metric"
    )
    res_top = solve_angular_segment_analysis(
        graph, metric_radii=[float("inf")], analysis_type="topological"
    )
    assert np.all(np.isfinite(res_ang["total_depth_rn"]))
    assert np.all(np.isfinite(res_met["total_depth_rn"]))
    assert np.all(np.isfinite(res_top["total_depth_rn"]))
    assert not np.allclose(res_ang["total_depth_rn"], res_met["total_depth_rn"])
    assert not np.allclose(res_ang["total_depth_rn"], res_top["total_depth_rn"])
    assert not np.allclose(res_met["total_depth_rn"], res_top["total_depth_rn"])

    # 2. Radius Type horizon pruning & monotonicity (td_r_small <= td_r_large)
    r_met = solve_angular_segment_analysis(
        graph, metric_radii=[100.0, 300.0], radius_type="metric"
    )
    assert np.all(r_met["total_depth_r100"] <= r_met["total_depth_r300"] + 1e-6)
    assert np.any(r_met["total_depth_r100"] < r_met["total_depth_r300"])

    r_ang = solve_angular_segment_analysis(
        graph, metric_radii=[1.0, 3.0], radius_type="angular"
    )
    assert np.all(r_ang["total_depth_r1"] <= r_ang["total_depth_r3"] + 1e-6)
    assert np.any(r_ang["total_depth_r1"] < r_ang["total_depth_r3"])

    r_top = solve_angular_segment_analysis(
        graph, metric_radii=[1.0, 3.0], radius_type="topological"
    )
    assert np.all(r_top["total_depth_r1"] <= r_top["total_depth_r3"] + 1e-6)
    assert np.any(r_top["total_depth_r1"] < r_top["total_depth_r3"])

    # 3. Weighting Mode perturbation: unweighted vs length vs custom
    custom_w = np.linspace(1.0, 12.0, graph.num_nodes)
    w_unw = solve_angular_segment_analysis(
        graph, metric_radii=[float("inf")], weight_mode="unweighted"
    )
    w_len = solve_angular_segment_analysis(
        graph, metric_radii=[float("inf")], weight_mode="length"
    )
    w_cus = solve_angular_segment_analysis(
        graph,
        metric_radii=[float("inf")],
        weight_mode="custom",
        custom_weights=custom_w,
    )
    assert not np.allclose(w_unw["total_depth_rn"], w_len["total_depth_rn"])
    assert not np.allclose(w_unw["total_depth_rn"], w_cus["total_depth_rn"])
    assert not np.allclose(w_len["total_depth_rn"], w_cus["total_depth_rn"])
    assert not np.allclose(w_unw["choice_rn"], w_cus["choice_rn"])
    print("✓ test_analysis_radius_and_weighting_modes passed")


def test_tulip_1024_vs_continuous_radians():
    """
    Test B: Validates Tulip 1024 angular binning vs continuous float64 radians.
    """
    # Cardinal directions match exactly
    assert abs(calculate_turn_deflection(0.0, math.pi, tulip_bins=1024)) < 1e-6
    assert abs(calculate_turn_deflection(0.0, math.pi * 0.5, tulip_bins=1024) - 1.0) < 1e-6

    # Oblique 17.3° turn has quantized step difference vs continuous float64
    b_oblique = math.pi - math.radians(17.3)
    cost_cont = calculate_turn_deflection(0.0, b_oblique, tulip_bins=0)
    cost_tulip = calculate_turn_deflection(0.0, b_oblique, tulip_bins=1024)
    assert abs(cost_cont - cost_tulip) > 1e-6
    assert abs(cost_cont - cost_tulip) < 0.01

    dx = 100.0 * math.sin(math.radians(17.3))
    dy = 100.0 * math.cos(math.radians(17.3))
    segs = [
        LineString([(0.0, 0.0), (0.0, 100.0)]),
        LineString([(0.0, 100.0), (dx, 100.0 + dy)]),
    ]
    g_cont, _ = CSRDualGraph.from_segments(segs, tulip_bins=0)
    g_tulip, _ = CSRDualGraph.from_segments(segs, tulip_bins=1024)
    assert abs(float(g_cont.angular_connectivity[0]) - float(g_tulip.angular_connectivity[0])) > 1e-6
    print("✓ test_tulip_1024_vs_continuous_radians passed")


def test_grade_separation_and_dead_end_stubs():
    """
    Test C: Validates Grade Separation (Bridge/Tunnel overpass unlink) and Dead-End Stub pruning.
    """
    assert extract_grade_level_from_props({"bridge": "yes"}) == 1.0
    assert extract_grade_level_from_props({"tunnel": "yes"}) == -1.0
    assert extract_grade_level_from_props({"layer": "2"}) == 2.0
    assert extract_grade_level_from_props({}) == 0.0

    # Two crossing streets: surface (level=0.0) and bridge overpass (level=1.0)
    s_surf = LineString([(0, 50), (100, 50)])
    s_bridge = LineString([(50, 0), (50, 100)])

    # 1a. Grade-separated cleaning: lines on different levels do NOT split or connect at (50, 50)
    cleaned_sep, _ = clean_network_adaptive_watertight(
        [s_surf, s_bridge],
        mode="none",
        line_levels=[0.0, 1.0],
        run_assertions=False,
    )
    segs_sep = explode_to_2point_segments(cleaned_sep)
    assert len(segs_sep) == 2
    g_sep, _ = CSRDualGraph.from_segments(segs_sep, segment_levels=[0.0, 1.0])
    assert np.all(g_sep.connectivity == 0)

    # 1b. Pre-noded 4-segment crossing with grade levels [0, 0, 1, 1] -> detect_grade_unlinks
    segs_4 = [
        LineString([(0, 50), (50, 50)]),
        LineString([(50, 50), (100, 50)]),
        LineString([(50, 0), (50, 50)]),
        LineString([(50, 50), (50, 100)]),
    ]
    unlinks = detect_grade_unlinks(segs_4, [0.0, 0.0, 1.0, 1.0])
    assert len(unlinks) == 4
    g_unlinked, _ = CSRDualGraph.from_segments(segs_4, segment_levels=[0.0, 0.0, 1.0, 1.0])
    # Each half only connects to its collinear partner on the same level (connectivity == 1)
    assert np.all(g_unlinked.connectivity == 1)

    # 1c. Same level [0.0, 0.0] -> splits into 4 connected segments (connectivity == 3)
    cleaned_flat, _ = clean_network_adaptive_watertight(
        [s_surf, s_bridge],
        mode="none",
        line_levels=[0.0, 0.0],
        run_assertions=False,
    )
    segs_flat = explode_to_2point_segments(cleaned_flat)
    assert len(segs_flat) == 4
    g_flat, _ = CSRDualGraph.from_segments(segs_flat, segment_levels=[0.0, 0.0, 0.0, 0.0])
    assert np.all(g_flat.connectivity == 3)

    # Bridge ramp touching down at surface T-junction stays connected
    t_junc_segs = [
        LineString([(0, 0), (50, 0)]),
        LineString([(50, 0), (100, 0)]),
        LineString([(50, 0), (50, 50)]),
    ]
    assert len(detect_grade_unlinks(t_junc_segs, [0.0, 0.0, 1.0])) == 0
    g_tj, _ = CSRDualGraph.from_segments(t_junc_segs, segment_levels=[0.0, 0.0, 1.0])
    assert list(solve_origin_step_depth(g_tj, [0])["topo_step_depth"]) == [0, 1, 1]

    # 2. Dead-End Stub Pruning: 3.0m driveway stub attached to a junction is pruned at 5.0m threshold
    grid_with_stub = [
        LineString([(0, 0), (100, 0)]),
        LineString([(100, 0), (200, 0)]),
        LineString([(100, 0), (100, 100)]),
        LineString([(100, 0), (100, -3.0)]),  # 3.0m dead-end stub at (100, 0) junction
    ]
    pruned = remove_dead_end_stubs(grid_with_stub, max_stub_length_m=5.0)
    assert len(pruned) == 3
    assert all(ln.length >= 50.0 for ln in pruned)
    print("✓ test_grade_separation_and_dead_end_stubs passed")


def test_origin_step_depth_and_csv_exports():
    """
    Test D: Validates Origin Step Depth and Detailed CSV + Summary R² Table Export.
    """
    segments = _build_3x3_grid_segments()
    graph, _ = CSRDualGraph.from_segments(segments)

    # 1. Origin Step Depth from segment 0
    step_res = solve_origin_step_depth(graph, origin_indices=[0])
    assert step_res["is_origin"][0] == 1
    assert int(np.sum(step_res["is_origin"])) == 1
    assert step_res["angular_step_depth"][0] == 0.0
    assert step_res["metric_step_depth_m"][0] == 0.0
    assert step_res["topo_step_depth"][0] == 0
    assert np.all(step_res["metric_step_depth_m"][1:] > 0.0)
    assert np.all(step_res["topo_step_depth"][1:] >= 1)

    # 2. Detailed CSV + Summary R² Table Export
    asa_res = solve_angular_segment_analysis(
        graph,
        metric_radii=[150.0, float("inf")],
        include_normalized=True,
    )
    diag_direct = compute_network_summary_diagnostics(graph, asa_res)
    assert "intelligibility_r2" in diag_direct
    assert "synergy_r2" in diag_direct

    with tempfile.TemporaryDirectory() as tmpdir:
        det_csv = os.path.join(tmpdir, "detailed_asa.csv")
        sum_csv = os.path.join(tmpdir, "summary_asa.csv")
        diag = export_detailed_asa_tables(
            graph,
            asa_res,
            detailed_csv_path=det_csv,
            summary_csv_path=sum_csv,
        )
        assert os.path.isfile(det_csv) and os.path.getsize(det_csv) > 0
        assert os.path.isfile(sum_csv) and os.path.getsize(sum_csv) > 0
        assert "intelligibility_r2" in diag
        assert "synergy_r2" in diag

        with open(det_csv, "r", encoding="utf-8") as f_det:
            det_text = f_det.read()
        assert "connectivity" in det_text
        assert "angular_connectivity" in det_text
        assert "mean_depth_rn" in det_text

        with open(sum_csv, "r", encoding="utf-8") as f_sum:
            sum_text = f_sum.read()
        assert "intelligibility_r2" in sum_text
        assert "synergy" in sum_text
    print("✓ test_origin_step_depth_and_csv_exports passed")


def test_natural_roads_strokes():
    """Validates Jiang-Claramunt stroke concatenation on dog-leg corridors."""
    l1 = LineString([(0, 0), (100, 0)])
    l2 = LineString([(100, 0), (200, 17.6)])  # ~10° turn
    l3 = LineString([(100, 0), (100, 100)])   # 90° cross street

    strokes = concatenate_natural_road_strokes([l1, l2, l3], max_deflection_deg=20.0)
    assert len(strokes) == 2
    stroke_lengths = sorted([s.length for s in strokes])
    assert abs(stroke_lengths[1] - (l1.length + l2.length)) < 0.1
    print("✓ test_natural_roads_strokes passed")


def test_isovist_and_vga():
    """Validates 360° isovist and visibility graph analysis."""
    walls = [
        LineString([(0, 0), (20, 0)]),
        LineString([(20, 0), (20, 20)]),
        LineString([(20, 20), (0, 20)]),
        LineString([(0, 20), (0, 0)]),
    ]
    iso = compute_isovist_at_point((10, 10), walls, max_radius_m=50.0, ray_count=72)
    assert iso["area"] > 350.0  # close to 400 m²
    assert abs(iso["drift_dist"]) < 1.0  # Centered isovist has near-zero drift

    pts = [Point(5, 5), Point(15, 5), Point(5, 15), Point(15, 15)]
    vga = solve_vga_visibility_graph(pts, walls, max_vision_m=30.0)
    assert np.all(vga["vga_connectivity"] == 3)  # fully connected clique

    # Verify Polygon obstacle interior exclusion + ring visibility occlusion
    from core import generate_vga_grid
    room_poly = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])
    pillar_poly = Polygon([(8, 8), (12, 8), (12, 12), (8, 12)])
    grid_pts = generate_vga_grid(room_poly, cell_size_m=2.0, obstacles=[pillar_poly])
    assert len(grid_pts) > 0
    for gp in grid_pts:
        assert not pillar_poly.contains(gp)
    vga_full = solve_vga_visibility_graph(
        grid_pts, [pillar_poly], max_vision_m=30.0, include_isovist=True
    )
    assert "vga_integration_hh" in vga_full
    assert "Isovist_Area" in vga_full
    print("✓ test_isovist_and_vga passed")


if __name__ == "__main__":
    test_turn_deflection()
    test_hillier_diamond()
    test_grid_segment_analysis()
    test_analysis_radius_and_weighting_modes()
    test_tulip_1024_vs_continuous_radians()
    test_grade_separation_and_dead_end_stubs()
    test_origin_step_depth_and_csv_exports()
    test_natural_roads_strokes()
    test_isovist_and_vga()
    print("\n✓ ALL SYNTHETIC TESTS PASSED MATHEMATICALLY!")
