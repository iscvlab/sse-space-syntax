# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core Computational Engine
High-performance Space Syntax analysis modules:
- ProjectionEngine & Universal Metric CRS Guard (CRS auto-detection & batch reprojection)
- OSM Overpass Downloader (native Overpass QL street network fetcher & mode filter)
- Preprocessing (noise filtering, watertight snapping, dual-carriageway rungs)
- Topology (O(N log N) planarization, collinear merge, 2-point explosion)
- CSRDualGraph (Compressed Sparse Row dual graph)
- Solvers (ASA Segment Analysis, Origin Step Depth, Hillier Axial Map, Natural Roads strokes, VGA & Isovists)
"""

from .projection import (
    ProjectionEngine,
    WEB_MERCATOR_AUTHIDS,
    ensure_qgis_metric_crs,
    get_utm_epsg_for_lon_lat,
    resolve_universal_metric_crs_pyproj,
    transform_qgs_geometry,
)
from .osm_downloader import (
    OSM_NETWORK_MODES,
    OVERPASS_ENDPOINTS,
    build_overpass_highway_query,
    fetch_osm_network,
    parse_overpass_json_to_features,
)
from .cleaner import (
    filter_noise_features,
    extract_grade_level_from_props,
    snap_endpoints_watertight,
    synthesize_dual_carriageway_rungs,
    clean_network_adaptive_watertight,
    collinear_join_with_turn_conservation,
    remove_dead_end_stubs,
    angle_between_vectors,
    count_connected_components,
    count_graph_junctions,
    count_dangling_endpoints
)
from .topology import (
    is_prenoded_segment_map,
    planarize_and_node,
    collinear_merge_preserving_junctions,
    explode_to_2point_segments
)
from .graph import CSRDualGraph, get_bearing, calculate_turn_deflection, detect_grade_unlinks
from .solvers_segment import (
    solve_angular_segment_analysis,
    solve_brandes_single_radius,
    solve_origin_step_depth,
    compute_network_summary_diagnostics,
    export_detailed_asa_tables
)
from .solvers_axial import solve_axial_analysis, calculate_hillier_diamond_dn
from .solvers_natural_roads import concatenate_natural_road_strokes
from .utils import parse_radii

__all__ = [
    "ProjectionEngine",
    "WEB_MERCATOR_AUTHIDS",
    "get_utm_epsg_for_lon_lat",
    "resolve_universal_metric_crs_pyproj",
    "ensure_qgis_metric_crs",
    "transform_qgs_geometry",
    "OVERPASS_ENDPOINTS",
    "OSM_NETWORK_MODES",
    "build_overpass_highway_query",
    "parse_overpass_json_to_features",
    "fetch_osm_network",
    "filter_noise_features",
    "extract_grade_level_from_props",
    "snap_endpoints_watertight",
    "synthesize_dual_carriageway_rungs",
    "clean_network_adaptive_watertight",
    "collinear_join_with_turn_conservation",
    "remove_dead_end_stubs",
    "angle_between_vectors",
    "count_connected_components",
    "count_graph_junctions",
    "count_dangling_endpoints",
    "is_prenoded_segment_map",
    "planarize_and_node",
    "collinear_merge_preserving_junctions",
    "explode_to_2point_segments",
    "CSRDualGraph",
    "get_bearing",
    "calculate_turn_deflection",
    "detect_grade_unlinks",
    "solve_angular_segment_analysis",
    "solve_brandes_single_radius",
    "solve_origin_step_depth",
    "compute_network_summary_diagnostics",
    "export_detailed_asa_tables",
    "solve_axial_analysis",
    "calculate_hillier_diamond_dn",
    "concatenate_natural_road_strokes",
    "parse_radii"
]
