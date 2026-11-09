"""
Unit tests for SSE Space Syntax Computational Engine.
Tests watertight geometry healing, angular dual graph building, metric Brandes solver,
Analysis Type x Radius Type x Weighting perturbations, Tulip-1024 binning,
Grade separation unlinking, Dead-end stub pruning, Origin Step Depth, and CSV/R² exports.
"""

import math
import os
import sys
import tempfile
import numpy as np
from shapely.geometry import LineString

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sse.core.snapping import snap_endpoints_rtree  # noqa: E402
from sse.core.topology import (  # noqa: E402
    planarize_and_node,
    build_angular_dual_graph,
    explode_to_2point_segments,
    turn_angle,
)
from sse.core.solver import solve_space_syntax_metrics  # noqa: E402
from sse.core.graph import (  # noqa: E402
    CSRDualGraph,
    calculate_turn_deflection,
    detect_grade_unlinks,
)
from sse.core.solvers_segment import (  # noqa: E402
    solve_angular_segment_analysis,
    solve_origin_step_depth,
    compute_network_summary_diagnostics,
    export_detailed_asa_tables,
)
from sse.core.cleaner import (  # noqa: E402
    clean_network_adaptive_watertight,
    remove_dead_end_stubs,
    extract_grade_level_from_props,
)


def test_endpoint_snapping():
    """Verify that gaps under tolerance are snapped together."""
    line1 = LineString([(0, 0), (100, 0)])
    line2 = LineString([(101.5, 0), (200, 0)])

    snapped = snap_endpoints_rtree([line1, line2], tolerance_m=2.0)
    assert len(snapped) == 2

    p1_end = snapped[0].coords[-1]
    p2_start = snapped[1].coords[0]
    dist = math.hypot(p1_end[0] - p2_start[0], p1_end[1] - p2_start[1])
    assert dist < 1e-4


def test_planarization_and_noding():
    """Verify that crossing lines are split at their intersection."""
    line_h = LineString([(-50, 0), (50, 0)])
    line_v = LineString([(0, -50), (0, 50)])

    noded = planarize_and_node([line_h, line_v])
    assert len(noded) == 4


def test_1d_collinear_analytical_toy_benchmark():
    """
    Verify 1D 3-segment collinear chain S0 -- S1 -- S2:
    - Zero angular deflection (turn_angle(pi/2, -pi/2) == 0.0 rad)
    - Central segment S1 carries maximum betweenness choice (S0 <-> S2 shortest path traverses S1)
    - Terminal segments S0 and S2 have zero interior shortest-path transit
    """
    s0 = LineString([(0, 0), (100, 0)])
    s1 = LineString([(100, 0), (200, 0)])
    s2 = LineString([(200, 0), (300, 0)])

    # Opposing outward bearings at shared vertex: East (+pi/2) vs West (-pi/2) -> 0.0 rad deflection
    assert abs(turn_angle(math.pi / 2.0, -math.pi / 2.0)) < 1e-12
    # Orthogonal turn: East (+pi/2) vs North (0.0) -> pi/2 rad deflection
    assert abs(turn_angle(math.pi / 2.0, 0.0) - (math.pi / 2.0)) < 1e-12

    G, _recs, stats = build_angular_dual_graph([s0, s1, s2])
    assert stats["num_segments"] == 3
    assert stats["num_graph_edges"] == 2

    res = solve_space_syntax_metrics(G, metric_radii=[500.0], include_global=True)
    # Central segment 1 must strictly dominate endpoints 0 and 2 in both Choice and Closeness
    assert res["choice_r500"][1] > res["choice_r500"].get(0, 0.0)
    assert res["choice_r500"][1] > res["choice_r500"].get(2, 0.0)
    assert res["nach_r500"][1] == 1.80
    assert res["nach_r500"][0] == 0.55
    assert res["nach_r500"][2] == 0.55
    assert res["integration_r500"][1] > res["integration_r500"][0]


def test_angular_dual_graph_and_solver():
    """Verify dual graph building and Space Syntax metric solver on a mini grid."""
    lines = [
        LineString([(0, 0), (200, 0)]),
        LineString([(0, 100), (200, 100)]),
        LineString([(0, 200), (200, 200)]),
        LineString([(0, 0), (0, 200)]),
        LineString([(100, 0), (100, 200)]),
        LineString([(200, 0), (200, 200)]),
    ]

    noded = planarize_and_node(lines)
    assert len(noded) > len(lines)

    G, _seg_records, stats = build_angular_dual_graph(noded)
    assert stats["num_segments"] == len(noded)
    assert stats["main_component_pct"] == 100.0

    results = solve_space_syntax_metrics(G, metric_radii=[150.0], include_global=True)

    assert "nach_r150" in results
    assert "nain_r150" in results
    assert "choice_r150" in results
    assert "nach_rn" in results
    assert "nain_rn" in results

    for nach_val in results["nach_r150"].values():
        assert not math.isnan(nach_val)
        assert nach_val > 0.0

    # Verify CSRDualGraph & solve_angular_segment_analysis + UL-03 empty graph guard
    segs = explode_to_2point_segments(noded)
    csr_graph, _ = CSRDualGraph.from_segments(segs)
    csr_res = solve_angular_segment_analysis(
        csr_graph, metric_radii=[150.0, float("inf")], include_normalized=True
    )
    assert "nach_norm_r150" in csr_res
    assert "nain_norm_rn" in csr_res
    empty_res = solve_angular_segment_analysis(CSRDualGraph(0), include_normalized=True)
    assert len(empty_res["nach_norm_rn"]) == 0


def test_analysis_radius_and_weighting_perturbation():
    """Test A (UL-05, UL-06): Analysis Type x Radius Type x Weighting perturbation."""
    asym_lines = [
        LineString([(0, 0), (200, 0)]),
        LineString([(0, 80), (200, 80)]),
        LineString([(0, 200), (200, 200)]),
        LineString([(0, 0), (0, 200)]),
        LineString([(60, 0), (60, 200)]),
        LineString([(200, 0), (200, 200)]),
    ]
    segs = explode_to_2point_segments(planarize_and_node(asym_lines))
    graph, _ = CSRDualGraph.from_segments(segs)

    res_ang = solve_angular_segment_analysis(
        graph, metric_radii=[float("inf")], analysis_type="angular"
    )
    res_met = solve_angular_segment_analysis(
        graph, metric_radii=[float("inf")], analysis_type="metric"
    )
    res_top = solve_angular_segment_analysis(
        graph, metric_radii=[float("inf")], analysis_type="topological"
    )
    assert not np.allclose(res_ang["total_depth_rn"], res_met["total_depth_rn"])
    assert not np.allclose(res_ang["total_depth_rn"], res_top["total_depth_rn"])
    assert not np.allclose(res_met["total_depth_rn"], res_top["total_depth_rn"])

    r_met = solve_angular_segment_analysis(
        graph, metric_radii=[100.0, 300.0], radius_type="metric"
    )
    assert np.all(r_met["total_depth_r100"] <= r_met["total_depth_r300"] + 1e-6)

    r_ang = solve_angular_segment_analysis(
        graph, metric_radii=[1.0, 3.0], radius_type="angular"
    )
    assert np.all(r_ang["total_depth_r1"] <= r_ang["total_depth_r3"] + 1e-6)

    r_top = solve_angular_segment_analysis(
        graph, metric_radii=[1.0, 3.0], radius_type="topological"
    )
    assert np.all(r_top["total_depth_r1"] <= r_top["total_depth_r3"] + 1e-6)

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


def test_tulip_1024_quantization():
    """Test B: Tulip 1024 bins vs continuous float64 radians."""
    assert abs(calculate_turn_deflection(0.0, math.pi, tulip_bins=1024)) < 1e-6
    assert abs(calculate_turn_deflection(0.0, math.pi * 0.5, tulip_bins=1024) - 1.0) < 1e-6

    b_oblique = math.pi - math.radians(17.3)
    c_cont = calculate_turn_deflection(0.0, b_oblique, tulip_bins=0)
    c_tulip = calculate_turn_deflection(0.0, b_oblique, tulip_bins=1024)
    assert abs(c_cont - c_tulip) > 1e-6
    assert abs(c_cont - c_tulip) < 0.01


def test_grade_separation_and_stub_pruning():
    """Test C: Grade separation overpass unlinking and dead-end stub pruning."""
    assert extract_grade_level_from_props({"bridge": "yes"}) == 1.0
    assert extract_grade_level_from_props({"tunnel": "yes"}) == -1.0

    s_surf = LineString([(0, 50), (100, 50)])
    s_bridge = LineString([(50, 0), (50, 100)])

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

    segs_4 = [
        LineString([(0, 50), (50, 50)]),
        LineString([(50, 50), (100, 50)]),
        LineString([(50, 0), (50, 50)]),
        LineString([(50, 50), (50, 100)]),
    ]
    unlinks = detect_grade_unlinks(segs_4, [0.0, 0.0, 1.0, 1.0])
    assert len(unlinks) == 4
    g_unlinked, _ = CSRDualGraph.from_segments(segs_4, segment_levels=[0.0, 0.0, 1.0, 1.0])
    assert np.all(g_unlinked.connectivity == 1)

    g_flat, _ = CSRDualGraph.from_segments(segs_4, segment_levels=[0.0, 0.0, 0.0, 0.0])
    assert np.all(g_flat.connectivity == 3)

    # Bridge ramp touching down at surface T-junction (level_counts = {0.0: 2, 1.0: 1}) stays connected
    t_junc_segs = [
        LineString([(0, 0), (50, 0)]),
        LineString([(50, 0), (100, 0)]),
        LineString([(50, 0), (50, 50)]),
    ]
    assert len(detect_grade_unlinks(t_junc_segs, [0.0, 0.0, 1.0])) == 0
    g_tj, _ = CSRDualGraph.from_segments(t_junc_segs, segment_levels=[0.0, 0.0, 1.0])
    assert list(solve_origin_step_depth(g_tj, [0])["topo_step_depth"]) == [0, 1, 1]

    grid_with_stub = [
        LineString([(0, 0), (100, 0)]),
        LineString([(100, 0), (200, 0)]),
        LineString([(100, 0), (100, 100)]),
        LineString([(100, 0), (100, -3.0)]),
    ]
    pruned = remove_dead_end_stubs(grid_with_stub, max_stub_length_m=5.0)
    assert len(pruned) == 3


def test_origin_step_depth_and_table_exports():
    """Test D: Origin Step Depth and Detailed CSV + Summary R² Table Export."""
    lines = [
        LineString([(0, 0), (200, 0)]),
        LineString([(0, 100), (200, 100)]),
        LineString([(0, 200), (200, 200)]),
        LineString([(0, 0), (0, 200)]),
        LineString([(100, 0), (100, 200)]),
        LineString([(200, 0), (200, 200)]),
    ]
    segs = explode_to_2point_segments(planarize_and_node(lines))
    graph, _ = CSRDualGraph.from_segments(segs)

    step_res = solve_origin_step_depth(graph, origin_indices=[0])
    assert step_res["is_origin"][0] == 1
    assert step_res["angular_step_depth"][0] == 0.0
    assert step_res["metric_step_depth_m"][0] == 0.0
    assert step_res["topo_step_depth"][0] == 0
    assert np.all(step_res["metric_step_depth_m"][1:] > 0.0)
    assert np.all(step_res["topo_step_depth"][1:] >= 1)

    asa_res = solve_angular_segment_analysis(
        graph, metric_radii=[150.0, float("inf")], include_normalized=True
    )
    diag = compute_network_summary_diagnostics(graph, asa_res)
    assert "intelligibility_r2" in diag
    assert "synergy_r2" in diag

    with tempfile.TemporaryDirectory() as tmpdir:
        det_csv = os.path.join(tmpdir, "detailed.csv")
        sum_csv = os.path.join(tmpdir, "summary.csv")
        export_detailed_asa_tables(
            graph, asa_res, detailed_csv_path=det_csv, summary_csv_path=sum_csv
        )
        assert os.path.isfile(det_csv) and os.path.getsize(det_csv) > 0
        assert os.path.isfile(sum_csv) and os.path.getsize(sum_csv) > 0
        with open(det_csv, "r", encoding="utf-8") as f_det:
            det_text = f_det.read()
        assert "connectivity" in det_text
        assert "angular_connectivity" in det_text
        assert "mean_depth_rn" in det_text
        with open(sum_csv, "r", encoding="utf-8") as f_sum:
            sum_text = f_sum.read()
        assert "intelligibility_r2" in sum_text
        assert "synergy" in sum_text


def test_depthmapx_tulip_1024_barnsbury_exact_parity():
    """Verify exact R² = 1.00000000 parity with UCL depthmapX v0.8.0 on Barnsbury MIF."""
    import csv
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    mif_path = os.path.join(root_dir, "examples", "ucl_depthmapx", "barnsbury_segment_lines.mif")
    mid_path = os.path.join(root_dir, "examples", "ucl_depthmapx", "barnsbury_segment_lines.mid")
    if not (os.path.isfile(mif_path) and os.path.isfile(mid_path)):
        mif_path = os.path.join(root_dir, "benchmarks", "data", "barnsbury_segment_lines.mif")
        mid_path = os.path.join(root_dir, "benchmarks", "data", "barnsbury_segment_lines.mid")
    assert os.path.isfile(mif_path) and os.path.isfile(mid_path)

    with open(mif_path, "r", encoding="utf-8") as fm:
        lines_txt = fm.read().splitlines()
    cols = []
    in_cols = False
    segs = []
    for ln in lines_txt:
        t = ln.strip()
        if t.upper().startswith("COLUMNS"):
            in_cols = True
            continue
        if in_cols:
            if t.upper().startswith("DATA"):
                in_cols = False
                continue
            parts = t.split()
            if parts:
                cols.append(parts[0])
        elif t.upper().startswith("LINE "):
            p = t.split()
            segs.append(LineString([(float(p[1]), float(p[2])), (float(p[3]), float(p[4]))]))

    with open(mid_path, "r", encoding="utf-8") as fd:
        rows = list(csv.reader(fd))
    col_idx = {c: i for i, c in enumerate(cols)}
    ref_ch_unw = np.array([float(r[col_idx["T1024_Choice"]]) for r in rows])
    ref_ig_unw = np.array([float(r[col_idx["T1024_Integration"]]) for r in rows])
    ref_td_unw = np.array([float(r[col_idx["T1024_Total_Depth"]]) for r in rows])
    ref_nc = np.array([float(r[col_idx["T1024_Node_Count"]]) for r in rows])
    ref_ch_wgt = np.array([float(r[col_idx["T1024_Choice_Segment_Length_Wgt_"]]) for r in rows])
    ref_ig_wgt = np.array([float(r[col_idx["T1024_Integration_Segment_Length_Wgt_"]]) for r in rows])
    ref_td_wgt = np.array([float(r[col_idx["T1024_Total_Depth_Segment_Length_Wgt_"]]) for r in rows])

    g_tulip, _ = CSRDualGraph.from_segments(segs, tulip_bins=1024)
    res_unw = solve_angular_segment_analysis(
        g_tulip,
        metric_radii=[float("inf")],
        analysis_type="angular",
        radius_type="metric",
        weight_mode="unweighted"
    )
    res_wgt = solve_angular_segment_analysis(
        g_tulip,
        metric_radii=[float("inf")],
        analysis_type="angular",
        radius_type="metric",
        weight_mode="length"
    )
    assert float(np.corrcoef(ref_ch_unw, res_unw["choice_rn"])[0, 1] ** 2) > 0.99999999
    assert float(np.corrcoef(ref_ig_unw, res_unw["integration_rn"])[0, 1] ** 2) > 0.99999999
    assert float(np.corrcoef(ref_td_unw, res_unw["total_depth_rn"])[0, 1] ** 2) > 0.99999999
    assert np.allclose(ref_nc, res_unw["node_count_rn"], atol=1e-6)
    assert np.allclose(ref_ch_unw, res_unw["choice_rn"], atol=1e-5)

    assert float(np.corrcoef(ref_ch_wgt, res_wgt["choice_rn"])[0, 1] ** 2) > 0.99999999
    assert float(np.corrcoef(ref_ig_wgt, res_wgt["integration_rn"])[0, 1] ** 2) > 0.99999999
    assert float(np.corrcoef(ref_td_wgt, res_wgt["total_depth_rn"])[0, 1] ** 2) > 0.99999999


def test_osm_pipeline_and_prenoded_guard_parity():
    """Verify QGIS Plugin OSM Overpass parser, UTM projection, grade separation, P90 cleaner, and FALS-08 guard."""
    from sse.core import (
        build_overpass_highway_query,
        parse_overpass_json_to_features,
        resolve_universal_metric_crs_pyproj,
        extract_grade_level_from_props,
        clean_network_adaptive_watertight,
        is_prenoded_segment_map,
        explode_to_2point_segments,
    )

    q_walk = build_overpass_highway_query((100.595, 13.700, 100.610, 13.715), network_mode="walk")
    assert 'way["highway"]' in q_walk and "out body geom;" in q_walk

    mock_overpass = {
        "elements": [
            {"type": "node", "id": 1, "lon": 100.6000, "lat": 13.7050},
            {"type": "node", "id": 2, "lon": 100.6010, "lat": 13.7050},
            {"type": "node", "id": 3, "lon": 100.6005, "lat": 13.7045},
            {"type": "node", "id": 4, "lon": 100.6005, "lat": 13.7055},
            {
                "type": "way",
                "id": 100,
                "nodes": [1, 2],
                "tags": {"highway": "residential", "name": "Sukhumvit 77"},
            },
            {
                "type": "way",
                "id": 101,
                "nodes": [3, 4],
                "tags": {"highway": "primary", "bridge": "yes", "layer": "1"},
            },
        ]
    }
    feats = parse_overpass_json_to_features(mock_overpass, network_mode="all")
    assert len(feats) == 2
    assert extract_grade_level_from_props(feats[0]["properties"]) == 0.0
    assert extract_grade_level_from_props(feats[1]["properties"]) == 1.0

    needs_reproj, target_crs, audit_msg = resolve_universal_metric_crs_pyproj(
        "EPSG:4326", (100.6000, 13.7045, 100.6010, 13.7055)
    )
    assert needs_reproj is True
    assert target_crs == "EPSG:32647"
    assert "32647" in audit_msg

    # Test 5-step P90 cleaner and FALS-08 pre-noded guard
    raw_lines = [
        LineString([(0.0, 0.0), (100.0, 0.0)]),
        LineString([(50.0, -40.0), (50.0, -2.5)]),  # 2.5m undershoot gap toward y=0
        LineString([(50.0, 0.0), (50.0, 40.0)]),
    ]
    cleaned, report = clean_network_adaptive_watertight(raw_lines, mode="auto")
    assert report["assertions_passed"] is True
    segs = explode_to_2point_segments(cleaned)
    assert is_prenoded_segment_map(segs) is True
    assert len(explode_to_2point_segments(segs)) == len(segs)


def test_qgis_portal_bandit_and_qt6_scoped_enums_guard():
    """Verify URL scheme validation (B310), zero try-except-continue (B112), and Qt6 scoped Qgs* enums."""
    import ast
    import pathlib
    import pytest
    from sse.core.osm_downloader import _validate_http_url, fetch_osm_network

    assert _validate_http_url("https://overpass-api.de/api/interpreter") == "https://overpass-api.de/api/interpreter"
    for bad_url in ("file:///etc/passwd", "ftp://overpass-api.de/api", "custom://host/path", ""):
        with pytest.raises(ValueError):
            _validate_http_url(bad_url)
    with pytest.raises(ValueError):
        fetch_osm_network((100.59, 13.70, 100.60, 13.71), custom_endpoint="file:///etc/passwd")

    plugin_dir = pathlib.Path(__file__).resolve().parent.parent / "sse" / "plugins" / "qgis_plugin"
    forbidden_unscoped = {
        ("QgsWkbTypes", "LineString"),
        ("QgsWkbTypes", "NoGeometry"),
        ("QgsProcessing", "TypeVectorLine"),
        ("QgsProcessing", "TypeVectorPolygon"),
        ("QgsProcessing", "TypeVectorAnyGeometry"),
        ("QgsProcessing", "TypeVector"),
        ("QgsProcessingParameterNumber", "Double"),
        ("QgsProcessingParameterField", "Numeric"),
        ("QgsFeatureSink", "FastInsert"),
    }
    for py_file in plugin_dir.rglob("*.py"):
        tree = ast.parse(py_file.read_text(encoding="utf-8"), filename=str(py_file))
        for parent in ast.walk(tree):
            if isinstance(parent, ast.ExceptHandler):
                is_broad = parent.type is None or (
                    isinstance(parent.type, ast.Name)
                    and parent.type.id in ("Exception", "BaseException")
                )
                assert not (
                    is_broad and len(parent.body) == 1 and isinstance(parent.body[0], ast.Continue)
                ), f"Bandit B112 try_except_continue found in {py_file}:{parent.lineno}"
            for node in ast.iter_child_nodes(parent):
                if (
                    isinstance(node, ast.Attribute)
                    and isinstance(node.value, ast.Name)
                    and not isinstance(parent, ast.Attribute)
                ):
                    assert (node.value.id, node.attr) not in forbidden_unscoped, (
                        f"Unscoped Qt6 enum {node.value.id}.{node.attr} in {py_file}:{node.lineno}"
                    )
