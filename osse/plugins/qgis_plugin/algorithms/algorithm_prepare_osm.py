# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS Algorithm: Prepare OSM Network for Space Syntax
Complete spatial preprocessing pipeline:
- Direct OpenStreetMap Overpass API street network download (All / Vehicular / Pedestrian)
- Optional boundary polygon clipping & Universal 4-Case Metric CRS Guard (Local UTM meters)
- Watertight endpoint snapping, dual-carriageway rungs, planarization,
  grade-separation awareness, dead-end stub pruning, collinear merge,
  2-point explosion, and Network Cleaning Audit Summary Table export.
"""

import os
import sys
from typing import Any, List, Optional, Tuple

from qgis.core import (
    QgsCoordinateReferenceSystem,
    QgsCoordinateTransform,
    QgsFeature,
    QgsFeatureSink,
    QgsField,
    QgsFields,
    QgsGeometry,
    QgsPointXY,
    QgsProcessing,
    QgsProcessingAlgorithm,
    QgsProcessingParameterBoolean,
    QgsProcessingParameterEnum,
    QgsProcessingParameterExtent,
    QgsProcessingParameterFeatureSink,
    QgsProcessingParameterFeatureSource,
    QgsProcessingParameterNumber,
    QgsProcessingParameterString,
    QgsWkbTypes,
)
from qgis.PyQt.QtCore import QVariant
from qgis.PyQt.QtGui import QIcon
from shapely import wkt as shapely_wkt
from shapely.geometry import LineString
from shapely.ops import unary_union

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
DEV_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
for p in [PLUGIN_ROOT, DEV_ROOT]:
    if p not in sys.path:
        sys.path.insert(0, p)

from core import (  # noqa: E402
    clean_network_adaptive_watertight,
    collinear_merge_preserving_junctions,
    count_connected_components,
    count_dangling_endpoints,
    count_graph_junctions,
    explode_to_2point_segments,
    extract_grade_level_from_props,
    planarize_and_node,
    remove_dead_end_stubs,
    snap_endpoints_watertight,
    synthesize_dual_carriageway_rungs,
)

try:
    from core import (  # noqa: E402
        ensure_qgis_metric_crs,
        fetch_osm_network,
        transform_qgs_geometry,
    )
except ImportError:
    ensure_qgis_metric_crs = None
    fetch_osm_network = None
    transform_qgs_geometry = None

try:
    from core.cleaner import DEFAULT_NOISE_HIGHWAYS  # noqa: E402
except ImportError:
    DEFAULT_NOISE_HIGHWAYS = {
        "services",
        "rest_area",
        "construction",
        "elevator",
        "abandoned",
        "proposed",
    }


def _extract_clipped_linestrings(geom: Any) -> List[LineString]:
    if geom is None or geom.is_empty:
        return []
    gtype = geom.geom_type
    if gtype == "LineString":
        return [geom] if len(geom.coords) >= 2 and geom.length > 0.001 else []
    if gtype in ("MultiLineString", "GeometryCollection"):
        parts: List[LineString] = []
        for sub in geom.geoms:
            parts.extend(_extract_clipped_linestrings(sub))
        return parts
    return []


def _read_boundary_union(
    poly_source: Any,
    target_crs: QgsCoordinateReferenceSystem,
    project_instance: Any,
) -> Optional[Any]:
    if poly_source is None:
        return None
    src_crs = poly_source.sourceCrs()
    xform = None
    if src_crs is not None and src_crs.isValid() and src_crs != target_crs:
        xform = QgsCoordinateTransform(src_crs, target_crs, project_instance)
    polys = []
    for pfeat in poly_source.getFeatures():
        pgeom = pfeat.geometry()
        if pgeom is None or pgeom.isNull() or pgeom.isEmpty():
            continue
        if xform is not None:
            if transform_qgs_geometry is not None:
                pgeom = transform_qgs_geometry(pgeom, xform)
            else:
                pgeom.transform(xform)
        try:
            sp = shapely_wkt.loads(pgeom.asWkt())
            if sp is not None and not sp.is_empty:
                polys.append(sp)
        except Exception:
            continue
    if not polys:
        return None
    return unary_union(polys)


class SSEPrepareOSMAlgorithm(QgsProcessingAlgorithm):
    INPUT = "INPUT"
    DOWNLOAD_OSM = "DOWNLOAD_OSM"
    OSM_EXTENT = "OSM_EXTENT"
    OSM_BOUNDARY_LAYER = "OSM_BOUNDARY_LAYER"
    OSM_NETWORK_MODE = "OSM_NETWORK_MODE"
    CUSTOM_OVERPASS_URL = "CUSTOM_OVERPASS_URL"
    CLEAN_MODE = "CLEAN_MODE"
    EXTEND_DISTANCE = "EXTEND_DISTANCE"
    PRUNE_STUBS = "PRUNE_STUBS"
    MIN_STUB_LEN = "MIN_STUB_LEN"
    DEAD_END_STUB_LEN = "DEAD_END_STUB_LEN"
    GRADE_SEPARATION = "GRADE_SEPARATION"
    PRESERVE_TURNS = "PRESERVE_TURNS"
    RUNGS = "RUNGS"
    EXPORT_TABLE = "EXPORT_TABLE"
    OUTPUT = "OUTPUT"
    OUTPUT_CSV = "OUTPUT_CSV"

    def name(self):
        return "prepare_osm_network"

    def displayName(self):
        return "Prepare OSM Network for Space Syntax"

    def group(self):
        return "Network Cleansing"

    def groupId(self):
        return "cleansing"

    def icon(self):
        icon_path = os.path.join(PLUGIN_ROOT, "icon.png")
        return QIcon(icon_path) if os.path.exists(icon_path) else super().icon()

    def shortHelpString(self):
        return (
            "<b>Prepare OSM Network for Space Syntax "
            "(Enhanced 5-Step Pipeline + Overpass Downloader)</b><br>"
            "Downloads directly from OpenStreetMap (Overpass API) or cleans "
            "an existing GIS centerline layer into a topologically watertight, "
            "metric-projected (Local UTM) Space Syntax segment network:<br><br>"
            "0. <b>Direct OSM Download &amp; Universal CRS Guard</b>: "
            "Fetches walkable/drivable streets from Overpass API (no API key) "
            "or auto-reprojects EPSG:4326 / EPSG:3857 / Feet layers to "
            "Local UTM meters.<br>"
            "1. <b>Auto Mode (Recommended)</b>: Computes empirical P90 gap "
            "distribution to adaptively bridge floating ramps and slip roads "
            "without hardcoding.<br>"
            "2. <b>Manual Mode</b>: Custom raycasting threshold (meters) via "
            "user-defined SpinBox.<br>"
            "3. <b>Grade Separation Guard</b>: Respects bridge/tunnel/layer "
            "attributes so overpasses do not falsely split or merge with "
            "underpasses.<br>"
            "4. <b>Micro-Stub &amp; Dead-End Pruning</b>: Purges dangling "
            "overshoot stubs (&lt; 0.5m) and optional short dead-end spurs.<br>"
            "5. <b>Turn Conservation &amp; 2-Point Explosion</b>: Preserves "
            "sharp corners (&gt; 15°) and explodes into 2-point LINE segments "
            "with an optional Network Cleaning Audit Summary Table."
        )

    def initAlgorithm(self, config=None):
        self.addParameter(
            QgsProcessingParameterFeatureSource(
                self.INPUT,
                "Raw Input Line Layer "
                "[optional when downloading from OpenStreetMap below]",
                [QgsProcessing.TypeVectorLine],
                optional=True,
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.DOWNLOAD_OSM,
                "Download Street Network Directly from OpenStreetMap "
                "(Overpass API — No API Key Required)",
                defaultValue=False,
            )
        )
        self.addParameter(
            QgsProcessingParameterExtent(
                self.OSM_EXTENT,
                "OSM Download Extent (Map Canvas / Layer / Draw on Canvas)",
                optional=True,
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSource(
                self.OSM_BOUNDARY_LAYER,
                "Optional Boundary Polygon Layer "
                "(Clip OSM Network to Exact Polygon)",
                [QgsProcessing.TypeVectorPolygon],
                optional=True,
            )
        )
        self.addParameter(
            QgsProcessingParameterEnum(
                self.OSM_NETWORK_MODE,
                "OSM Street Network Filter Preset",
                options=[
                    "All Public Walkable & Drivable Streets "
                    "(Space Syntax Standard)",
                    "Vehicular / Drivable Roads Only (Macro / Axial Standard)",
                    "Pedestrian & Active Mobility Only (Micro / Park / Campus)",
                ],
                defaultValue=0,
                optional=True,
            )
        )
        self.addParameter(
            QgsProcessingParameterString(
                self.CUSTOM_OVERPASS_URL,
                "Custom Overpass Server URL "
                "[optional — leave blank for free public servers]",
                defaultValue="",
                optional=True,
            )
        )
        self.addParameter(
            QgsProcessingParameterEnum(
                self.CLEAN_MODE,
                "Network Cleaning Mode",
                options=[
                    "Auto (Empirical P90 Gap Analysis — Recommended)",
                    "Manual (User-defined Extension Threshold)",
                    "Legacy (Fixed 3.0m Snapping)",
                ],
                defaultValue=0,
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.EXTEND_DISTANCE,
                "Manual Extension Distance (meters) [Manual Mode only]",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=15.0,
                minValue=1.0,
                maxValue=50.0,
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.PRUNE_STUBS,
                "Prune Dangling Micro-Stubs (< 0.5m)",
                defaultValue=True,
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.MIN_STUB_LEN,
                "Micro-Stub Pruning Threshold (meters)",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=0.5,
                minValue=0.1,
                maxValue=5.0,
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.DEAD_END_STUB_LEN,
                "Remove Dead-End Stubs Shorter Than (meters, 0 = disabled)",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=0.0,
                minValue=0.0,
                maxValue=500.0,
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.GRADE_SEPARATION,
                "Respect Bridge / Tunnel Grade Separation "
                "(layer / bridge / tunnel)",
                defaultValue=True,
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.PRESERVE_TURNS,
                "Turn Conservation (Preserve Sharp Corners > 15°)",
                defaultValue=True,
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.RUNGS,
                "Generate Dual-Carriageway Pedestrian Cross-Rungs",
                defaultValue=True,
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.EXPORT_TABLE,
                "Generate Network Cleaning Audit Summary Table Layer in QGIS",
                defaultValue=True,
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT,
                "Cleansed & Noded Space Syntax Segments",
                type=QgsProcessing.TypeVectorLine,
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT_CSV,
                "Network Cleaning Audit Summary Table",
                type=QgsProcessing.TypeVector,
                optional=True,
                createByDefault=True,
            )
        )

    def _resolve_osm_bbox_and_clip(
        self,
        parameters,
        context,
        source,
        poly_source,
    ) -> Tuple[Tuple[float, float, float, float], Optional[Any]]:
        wgs84_crs = QgsCoordinateReferenceSystem("EPSG:4326")
        clip_polygon_wgs84 = _read_boundary_union(
            poly_source, wgs84_crs, context.project()
        )
        if clip_polygon_wgs84 is not None and not clip_polygon_wgs84.is_empty:
            b = clip_polygon_wgs84.bounds
            return (float(b[0]), float(b[1]), float(b[2]), float(b[3])), clip_polygon_wgs84

        raw_ext = parameters.get(self.OSM_EXTENT)
        if raw_ext is not None and str(raw_ext).strip() != "":
            rect = self.parameterAsExtent(
                parameters, self.OSM_EXTENT, context, wgs84_crs
            )
            if (
                rect is not None
                and not rect.isEmpty()
                and rect.width() > 0.0
                and rect.height() > 0.0
            ):
                return (
                    float(rect.xMinimum()),
                    float(rect.yMinimum()),
                    float(rect.xMaximum()),
                    float(rect.yMaximum()),
                ), None

        if source is not None:
            src_ext = source.sourceExtent()
            if src_ext is not None and not src_ext.isEmpty():
                src_crs = source.sourceCrs()
                if src_crs is not None and src_crs.isValid() and src_crs != wgs84_crs:
                    xf = QgsCoordinateTransform(
                        src_crs, wgs84_crs, context.project()
                    )
                    src_ext = xf.transformBoundingBox(src_ext)
                return (
                    float(src_ext.xMinimum()),
                    float(src_ext.yMinimum()),
                    float(src_ext.xMaximum()),
                    float(src_ext.yMaximum()),
                ), None

        raise RuntimeError(
            "Please provide either an Input Line Layer or an "
            "OSM Download Extent / Boundary Polygon."
        )

    def processAlgorithm(self, parameters, context, feedback):
        source = self.parameterAsSource(parameters, self.INPUT, context)
        download_osm = self.parameterAsBoolean(
            parameters, self.DOWNLOAD_OSM, context
        )
        poly_source = self.parameterAsSource(
            parameters, self.OSM_BOUNDARY_LAYER, context
        )
        osm_mode_idx = self.parameterAsEnum(
            parameters, self.OSM_NETWORK_MODE, context
        )
        custom_url = self.parameterAsString(
            parameters, self.CUSTOM_OVERPASS_URL, context
        ).strip()
        export_table = self.parameterAsBoolean(
            parameters, self.EXPORT_TABLE, context
        )

        mode_idx = self.parameterAsEnum(parameters, self.CLEAN_MODE, context)
        clean_mode_str = (
            "auto"
            if mode_idx == 0
            else ("manual" if mode_idx == 1 else "legacy")
        )
        extend_m = self.parameterAsDouble(
            parameters, self.EXTEND_DISTANCE, context
        )
        prune_stubs = self.parameterAsBoolean(
            parameters, self.PRUNE_STUBS, context
        )
        min_stub = (
            self.parameterAsDouble(parameters, self.MIN_STUB_LEN, context)
            if prune_stubs
            else 0.0
        )
        dead_end_stub_len = self.parameterAsDouble(
            parameters, self.DEAD_END_STUB_LEN, context
        )
        grade_sep = self.parameterAsBoolean(
            parameters, self.GRADE_SEPARATION, context
        )
        preserve_turns = self.parameterAsBoolean(
            parameters, self.PRESERVE_TURNS, context
        )
        add_rungs = self.parameterAsBoolean(parameters, self.RUNGS, context)

        raw_lines: List[LineString] = []
        raw_levels: List[float] = []

        if download_osm or source is None:
            # Path A: Download from OpenStreetMap Overpass API
            bbox_wgs84, clip_polygon_wgs84 = self._resolve_osm_bbox_and_clip(
                parameters, context, source, poly_source
            )
            mode_map = {0: "all", 1: "vehicular", 2: "pedestrian"}
            net_mode_str = mode_map.get(osm_mode_idx, "all")

            if fetch_osm_network is None:
                raise RuntimeError(
                    "OSM downloader (fetch_osm_network) is not available."
                )

            feedback.setProgressText(
                f"Downloading OSM network (mode='{net_mode_str}') "
                f"from Overpass API for bbox={bbox_wgs84}..."
            )
            fetched_lines, props_list, target_epsg, _ = fetch_osm_network(
                bbox_wgs84=bbox_wgs84,
                network_mode=net_mode_str,
                custom_endpoint=custom_url,
                clip_polygon_wgs84=clip_polygon_wgs84,
                reproject_to_utm=True,
            )
            out_crs = QgsCoordinateReferenceSystem(f"EPSG:{target_epsg}")
            raw_lines = list(fetched_lines)
            raw_levels = [
                extract_grade_level_from_props(p) if grade_sep else 0.0
                for p in props_list
            ]
            source_mode_str = f"osm_overpass_{net_mode_str}"
            feedback.setProgressText(
                f"Downloaded {len(raw_lines):,} OSM ways "
                f"(auto-projected to EPSG:{target_epsg})."
            )
        else:
            # Path B: Existing QGIS Vector Line Layer + Universal Metric CRS Guard
            if ensure_qgis_metric_crs is not None:
                out_crs, xform, crs_reason = ensure_qgis_metric_crs(
                    source.sourceCrs(),
                    source.sourceExtent(),
                    context.project(),
                    feedback,
                )
            else:
                out_crs = source.sourceCrs()
                xform = None
                crs_reason = "Native CRS retained"

            clip_poly_metric = _read_boundary_union(
                poly_source, out_crs, context.project()
            )

            field_names = [f.name() for f in source.fields()]
            has_highway_col = any(fn.lower() == "highway" for fn in field_names)

            for feat in source.getFeatures():
                geom = feat.geometry()
                if geom is None or geom.isNull() or geom.isEmpty():
                    continue
                props = {fn: feat[fn] for fn in field_names}

                if has_highway_col:
                    hw_raw = props.get("highway")
                    acc_raw = props.get("access")
                    hw_val = (
                        str(hw_raw).strip().lower()
                        if hw_raw is not None
                        else ""
                    )
                    acc_val = (
                        str(acc_raw).strip().lower()
                        if acc_raw is not None
                        else ""
                    )
                    if hw_val in DEFAULT_NOISE_HIGHWAYS or acc_val in (
                        "private",
                        "no",
                    ):
                        continue

                if xform is not None:
                    if transform_qgs_geometry is not None:
                        geom = transform_qgs_geometry(geom, xform)
                    else:
                        geom.transform(xform)

                lvl = (
                    extract_grade_level_from_props(props)
                    if grade_sep
                    else 0.0
                )

                feat_lines: List[LineString] = []
                if geom.isMultipart():
                    for part in geom.asMultiPolyline():
                        if len(part) >= 2:
                            feat_lines.append(
                                LineString([(p.x(), p.y()) for p in part])
                            )
                else:
                    polyline = geom.asPolyline()
                    if len(polyline) >= 2:
                        feat_lines.append(
                            LineString([(p.x(), p.y()) for p in polyline])
                        )

                for ln in feat_lines:
                    if clip_poly_metric is not None and not clip_poly_metric.is_empty:
                        clipped = ln.intersection(clip_poly_metric)
                        for cl_part in _extract_clipped_linestrings(clipped):
                            raw_lines.append(cl_part)
                            raw_levels.append(lvl)
                    else:
                        raw_lines.append(ln)
                        raw_levels.append(lvl)

            source_mode_str = "vector_layer"
            feedback.setProgressText(
                f"Loaded {len(raw_lines):,} input lines ({crs_reason})."
            )

        raw_input_lines_count = len(raw_lines)

        if clean_mode_str in ["auto", "manual"]:
            feedback.setProgressText(
                f"Executing Enhanced 5-Step Cleaner "
                f"({clean_mode_str.upper()} mode)..."
            )
            raw_lines, report = clean_network_adaptive_watertight(
                raw_lines,
                mode=clean_mode_str,
                extend_distance=extend_m,
                min_stub_len=min_stub,
                max_turn_deflection=15.0 if preserve_turns else 180.0,
                lateral_fallback=True,
                run_assertions=False,
                dead_end_stub_len=dead_end_stub_len,
                line_levels=raw_levels if grade_sep else None,
            )
            eff_d = report.get("effective_extend_distance", 0)
            n_rays = report.get("ray_extensions_count", 0)
            n_lat = report.get("lateral_connectors_count", 0)
            n_stubs = report.get("pruned_micro_stubs_count", 0)
            d_pct = report.get("length_delta_pct", 0)
            feedback.setProgressText(
                f"Cleaner complete: effective search={eff_d}m, rays={n_rays}, "
                f"lateral={n_lat}, stubs pruned={n_stubs}, "
                f"length delta={d_pct:+.2f}%"
            )
        else:
            # Legacy mode
            feedback.setProgressText("Applying legacy 3.0m snapping...")
            init_comp = count_connected_components(raw_lines)
            init_junc = count_graph_junctions(raw_lines)
            init_dang = count_dangling_endpoints(raw_lines)
            init_len = sum(ln.length for ln in raw_lines)

            raw_lines = snap_endpoints_watertight(raw_lines, snap_distance_m=3.0)
            raw_lines = planarize_and_node(raw_lines)
            raw_lines = collinear_merge_preserving_junctions(raw_lines)
            if dead_end_stub_len > 0.0:
                raw_lines = remove_dead_end_stubs(
                    raw_lines, max_stub_length_m=dead_end_stub_len
                )

            fin_comp = count_connected_components(raw_lines)
            fin_junc = count_graph_junctions(raw_lines)
            fin_dang = count_dangling_endpoints(raw_lines)
            fin_len = sum(ln.length for ln in raw_lines)
            d_pct = (
                ((fin_len - init_len) / init_len * 100.0)
                if init_len > 0.0
                else 0.0
            )
            report = {
                "effective_extend_distance": 3.0,
                "ray_extensions_count": 0,
                "lateral_connectors_count": 0,
                "pruned_micro_stubs_count": 0,
                "length_delta_pct": round(d_pct, 2),
                "raw_components_count": init_comp,
                "cleaned_components_count": fin_comp,
                "raw_junctions_count": init_junc,
                "cleaned_junctions_count": fin_junc,
                "raw_dangles_count": init_dang,
                "cleaned_dangles_count": fin_dang,
            }

        if add_rungs:
            feedback.setProgressText(
                "Synthesizing dual-carriageway cross-rungs..."
            )
            rungs = synthesize_dual_carriageway_rungs(raw_lines)
            raw_lines.extend(rungs)

        feedback.setProgressText(
            "Exploding to 2-point Space Syntax LINE segments..."
        )
        segments = explode_to_2point_segments(raw_lines)
        feedback.setProgressText(
            f"Resulting cleansed network: {len(segments):,} segments."
        )

        fields = QgsFields()
        fields.append(QgsField("seg_id", QVariant.Int))
        fields.append(QgsField("length_m", QVariant.Double))

        (sink, dest_id) = self.parameterAsSink(
            parameters,
            self.OUTPUT,
            context,
            fields,
            QgsWkbTypes.LineString,
            out_crs,
        )

        if sink is not None:
            for i, s in enumerate(segments):
                coords = list(s.coords)
                qgs_geom = QgsGeometry.fromPolylineXY(
                    [
                        QgsPointXY(coords[0][0], coords[0][1]),
                        QgsPointXY(coords[-1][0], coords[-1][1]),
                    ]
                )
                feat = QgsFeature(fields)
                feat.setGeometry(qgs_geom)
                feat.setAttributes([i, float(round(s.length, 2))])
                sink.addFeature(feat, QgsFeatureSink.FastInsert)

        out_dict = {self.OUTPUT: dest_id}

        if export_table:
            csv_fields = QgsFields()
            csv_fields.append(QgsField("metric", QVariant.String))
            csv_fields.append(QgsField("value", QVariant.String))
            (csv_sink, csv_dest_id) = self.parameterAsSink(
                parameters,
                self.OUTPUT_CSV,
                context,
                csv_fields,
                QgsWkbTypes.NoGeometry,
                QgsCoordinateReferenceSystem(),
            )
            if csv_sink is not None:
                audit_rows = [
                    ("source_mode", str(source_mode_str)),
                    ("output_crs", str(out_crs.authid() or "EPSG:32647")),
                    ("raw_input_lines", str(raw_input_lines_count)),
                    ("cleaned_2pt_segments", str(len(segments))),
                    ("clean_mode", str(clean_mode_str)),
                    (
                        "effective_extend_distance_m",
                        str(report.get("effective_extend_distance", 0.0)),
                    ),
                    (
                        "ray_extensions_count",
                        str(report.get("ray_extensions_count", 0)),
                    ),
                    (
                        "lateral_connectors_count",
                        str(report.get("lateral_connectors_count", 0)),
                    ),
                    (
                        "pruned_micro_stubs_count",
                        str(report.get("pruned_micro_stubs_count", 0)),
                    ),
                    (
                        "length_delta_pct",
                        str(report.get("length_delta_pct", 0.0)),
                    ),
                    (
                        "initial_components",
                        str(report.get("raw_components_count", 0)),
                    ),
                    (
                        "final_components",
                        str(report.get("cleaned_components_count", 0)),
                    ),
                    (
                        "initial_junctions",
                        str(report.get("raw_junctions_count", 0)),
                    ),
                    (
                        "final_junctions",
                        str(report.get("cleaned_junctions_count", 0)),
                    ),
                    (
                        "initial_dangles",
                        str(report.get("raw_dangles_count", 0)),
                    ),
                    (
                        "final_dangles",
                        str(report.get("cleaned_dangles_count", 0)),
                    ),
                ]
                for m_key, m_val in audit_rows:
                    c_feat = QgsFeature(csv_fields)
                    c_feat.setAttributes([m_key, m_val])
                    csv_sink.addFeature(c_feat, QgsFeatureSink.FastInsert)
            if csv_dest_id:
                out_dict[self.OUTPUT_CSV] = csv_dest_id

        return out_dict

    def createInstance(self):
        return SSEPrepareOSMAlgorithm()


OpenDepthmapPrepareOSMAlgorithm = SSEPrepareOSMAlgorithm
