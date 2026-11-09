# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS Algorithm: Angular Segment Analysis (ASA)
Computes multi-radius Choice, Integration, NACH, NAIN, Total Depth, Mean Depth,
Node Count, Entropy, Control, and Controllability across Angular, Metric, or
Topological analysis modes with Grade Separation and Tulip-1024 / Exact IEEE-754.
"""

import math
import os
import sys
from typing import List, Optional

import numpy as np
from shapely.geometry import LineString
from shapely.strtree import STRtree

from qgis.core import (
    QgsProcessing,
    QgsProcessingAlgorithm,
    QgsProcessingParameterFeatureSource,
    QgsProcessingParameterString,
    QgsProcessingParameterNumber,
    QgsProcessingParameterBoolean,
    QgsProcessingParameterEnum,
    QgsProcessingParameterField,
    QgsProcessingParameterFeatureSink,
    QgsProcessingUtils,
    QgsFeatureSink,
    QgsFields,
    QgsField,
    QgsGeometry,
    QgsPointXY,
    QgsFeature,
    QgsWkbTypes,
    QgsCoordinateReferenceSystem
)
from qgis.PyQt.QtCore import QVariant, QTimer
from qgis.PyQt.QtGui import QIcon

# Ensure SSE root is in path
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
DEV_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
for p in [PLUGIN_ROOT, DEV_ROOT]:
    if p not in sys.path:
        sys.path.insert(0, p)

from core import (  # noqa: E402
    clean_network_adaptive_watertight,
    ensure_qgis_metric_crs,
    extract_grade_level_from_props,
    is_prenoded_segment_map,
    remove_dead_end_stubs,
    snap_endpoints_watertight,
    synthesize_dual_carriageway_rungs,
    transform_qgs_geometry,
    planarize_and_node,
    collinear_merge_preserving_junctions,
    explode_to_2point_segments,
    CSRDualGraph,
    solve_angular_segment_analysis,
    export_detailed_asa_tables,
    parse_radii
)
from styles.qml_templates import apply_space_syntax_style_to_layer  # noqa: E402


class SSEASAAlgorithm(QgsProcessingAlgorithm):
    INPUT = "INPUT"
    RADII = "RADII"
    ANALYSIS_TYPE = "ANALYSIS_TYPE"
    RADIUS_TYPE = "RADIUS_TYPE"
    WEIGHT_MODE = "WEIGHT_MODE"
    WEIGHT_FIELD = "WEIGHT_FIELD"
    TULIP_MODE = "TULIP_MODE"
    GRADE_SEPARATION = "GRADE_SEPARATION"
    GRADE_FIELD = "GRADE_FIELD"
    DEAD_END_STUB_LEN = "DEAD_END_STUB_LEN"
    CLEAN_MODE = "CLEAN_MODE"
    EXTEND_DISTANCE = "EXTEND_DISTANCE"
    PRUNE_STUBS = "PRUNE_STUBS"
    SNAP_TOLERANCE = "SNAP_TOLERANCE"
    DUAL_CARRIAGEWAY = "DUAL_CARRIAGEWAY"
    NORMALIZE_OUTPUT = "NORMALIZE_OUTPUT"
    EXPORT_TABLE = "EXPORT_TABLE"
    APPLY_SYMBOLOGY = "APPLY_SYMBOLOGY"
    OPEN_SCATTER_PLOT = "OPEN_SCATTER_PLOT"
    OUTPUT = "OUTPUT"
    OUTPUT_CSV = "OUTPUT_CSV"
    OUTPUT_SUMMARY_CSV = "OUTPUT_SUMMARY_CSV"

    def __init__(self):
        super().__init__()
        self._dest_id = None
        self._apply_symbology = True
        self._style_attr = ""
        self._style_values = None
        self._open_scatter = False

    def name(self):
        return "angular_segment_analysis"

    def displayName(self):
        return "Angular Segment Analysis (ASA Pro)"

    def group(self):
        return "Segment Analysis"

    def groupId(self):
        return "segment"

    def icon(self):
        icon_path = os.path.join(PLUGIN_ROOT, "icon.png")
        return QIcon(icon_path) if os.path.exists(icon_path) else super().icon()

    def shortHelpString(self):
        return (
            "<b>SSE (Space Syntax Engine) Angular Segment Analysis (ASA Pro — Universal Network Edition)</b><br>"
            "Computes multi-scale Space Syntax segment centrality with full "
            "UCL depthmapX v0.8.0 parity and Dual-Mode 5-Step Network Cleaner:<br><br>"
            "<b>Analysis & Routing Modes:</b><br>"
            "• <b>Analysis Type</b>: Angular (Turner 2007), Metric (Euclidean), "
            "or Topological (Turn Steps).<br>"
            "• <b>Radius Type</b>: Metric (m), Angular (cumulative rad), or "
            "Topological steps.<br>"
            "• <b>Weighting</b>: Segment Length, Unweighted (1.0), or Custom Field.<br>"
            "• <b>Precision</b>: Exact IEEE-754 Continuous Radians or Tulip 1024 Bins.<br>"
            "• <b>Grade Separation</b>: Automatically respects bridge/tunnel/layer "
            "elevations to prevent false overpass-underpass intersections.<br>"
            "• <b>Diagnostics</b>: Computes Intelligibility R² & Synergy R² and "
            "exports full CSV attribute tables."
        )

    def initAlgorithm(self, config=None):
        self.addParameter(
            QgsProcessingParameterFeatureSource(
                self.INPUT,
                "Input Street Network Layer",
                [QgsProcessing.SourceType.TypeVectorLine]
            )
        )
        self.addParameter(
            QgsProcessingParameterString(
                self.RADII,
                "Radii Suite (comma-separated; 'n' for global)",
                defaultValue="n, 10000, 5000, 2000, 1200, 800, 400"
            )
        )
        self.addParameter(
            QgsProcessingParameterEnum(
                self.ANALYSIS_TYPE,
                "Segment Analysis Type",
                options=[
                    "Angular (Least Angle — Turner 2007)",
                    "Metric (Shortest Euclidean Distance)",
                    "Topological (Fewest Turns / Steps)"
                ],
                defaultValue=0
            )
        )
        self.addParameter(
            QgsProcessingParameterEnum(
                self.RADIUS_TYPE,
                "Radius Horizon Type",
                options=[
                    "Metric Distance (meters)",
                    "Angular Cumulative Turns",
                    "Topological Step Count"
                ],
                defaultValue=0
            )
        )
        self.addParameter(
            QgsProcessingParameterEnum(
                self.WEIGHT_MODE,
                "Segment Weighting Mode",
                options=[
                    "Segment Length Weighted",
                    "Unweighted (Uniform = 1.0)",
                    "Custom Attribute Field"
                ],
                defaultValue=0
            )
        )
        self.addParameter(
            QgsProcessingParameterField(
                self.WEIGHT_FIELD,
                "Custom Weight Attribute Field [Custom Weight Mode]",
                parentLayerParameterName=self.INPUT,
                type=QgsProcessingParameterField.DataType.Numeric,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterEnum(
                self.TULIP_MODE,
                "Angular Precision Mode",
                options=[
                    "Exact Continuous Radians (IEEE-754 Float64 — Recommended)",
                    "Tulip 1024 Bins (UCL depthmapX Compatibility)"
                ],
                defaultValue=0
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.GRADE_SEPARATION,
                "Respect Bridge / Tunnel Grade Separation (layer / bridge / tunnel)",
                defaultValue=True
            )
        )
        self.addParameter(
            QgsProcessingParameterField(
                self.GRADE_FIELD,
                "Grade Level / Layer Field (Optional — auto-detects layer/bridge/tunnel if blank)",
                parentLayerParameterName=self.INPUT,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.DEAD_END_STUB_LEN,
                "Remove Dead-End Stubs Shorter Than (meters, 0 = disabled)",
                type=QgsProcessingParameterNumber.Type.Double,
                defaultValue=0.0,
                minValue=0.0,
                maxValue=500.0
            )
        )
        self.addParameter(
            QgsProcessingParameterEnum(
                self.CLEAN_MODE,
                "Network Cleaning Mode (Dual-Mode Contract)",
                options=[
                    "Auto (Empirical P90 Gap Analysis — Recommended)",
                    "Manual (User-defined Extension Distance)",
                    "Legacy (Simple STRtree Snapping)",
                    "None (Skip Cleaning — Pre-noded Network)"
                ],
                defaultValue=0
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.EXTEND_DISTANCE,
                "Manual Extension Distance (meters) [Manual Mode]",
                type=QgsProcessingParameterNumber.Type.Double,
                defaultValue=15.0,
                minValue=1.0,
                maxValue=50.0
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.PRUNE_STUBS,
                "Prune Dangling Micro-Stubs (< 0.5m)",
                defaultValue=True
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.SNAP_TOLERANCE,
                "Legacy Watertight Snapping Distance (meters) [Legacy Mode]",
                type=QgsProcessingParameterNumber.Type.Double,
                defaultValue=3.0,
                minValue=0.0,
                maxValue=50.0
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.DUAL_CARRIAGEWAY,
                "Synthesize Dual-Carriageway Pedestrian Cross-Rungs",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.NORMALIZE_OUTPUT,
                "Include Universal Normalized [0-1] Scores (nach_norm_*, nain_norm_*)",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.EXPORT_TABLE,
                "Generate Detailed Attribute & Summary R² Table Layers in QGIS",
                defaultValue=True
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.APPLY_SYMBOLOGY,
                "Apply Classic Space Syntax Blue-to-Red Color Ramp Automatically",
                defaultValue=True
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.OPEN_SCATTER_PLOT,
                "Open Interactive Scatter Plot (Intelligibility / Synergy R²) After Running",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT,
                "Analyzed Segment Network",
                type=QgsProcessing.SourceType.TypeVectorLine
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT_CSV,
                "Detailed Segment Attribute Table",
                type=QgsProcessing.SourceType.TypeVector,
                optional=True,
                createByDefault=True
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT_SUMMARY_CSV,
                "Network Summary & Intelligibility / Synergy R² Diagnostics Table",
                type=QgsProcessing.SourceType.TypeVector,
                optional=True,
                createByDefault=True
            )
        )

    def processAlgorithm(self, parameters, context, feedback):
        source = self.parameterAsSource(parameters, self.INPUT, context)
        if source is None:
            raise RuntimeError("Invalid input layer.")

        out_crs, xform, _crs_reason = ensure_qgis_metric_crs(
            source.sourceCrs(),
            source.sourceExtent(),
            context.project(),
            feedback
        )

        radii_str = self.parameterAsString(parameters, self.RADII, context)
        radii_list = parse_radii(radii_str)

        a_idx = self.parameterAsEnum(parameters, self.ANALYSIS_TYPE, context)
        analysis_types = ["angular", "metric", "topological"]
        analysis_type_str = (
            analysis_types[a_idx] if 0 <= a_idx < 3 else "angular"
        )

        r_idx = self.parameterAsEnum(parameters, self.RADIUS_TYPE, context)
        radius_types = ["metric", "angular", "topological"]
        radius_type_str = radius_types[r_idx] if 0 <= r_idx < 3 else "metric"

        w_idx = self.parameterAsEnum(parameters, self.WEIGHT_MODE, context)
        weight_modes = ["length", "unweighted", "custom"]
        weight_mode_str = weight_modes[w_idx] if 0 <= w_idx < 3 else "length"
        weight_field = self.parameterAsString(
            parameters, self.WEIGHT_FIELD, context
        )

        tulip_idx = self.parameterAsEnum(parameters, self.TULIP_MODE, context)
        tulip_bins = 1024 if tulip_idx == 1 else 0

        grade_sep = self.parameterAsBoolean(
            parameters, self.GRADE_SEPARATION, context
        )
        grade_field = self.parameterAsString(
            parameters, self.GRADE_FIELD, context
        )
        dead_end_stub_len = self.parameterAsDouble(
            parameters, self.DEAD_END_STUB_LEN, context
        )

        mode_idx = self.parameterAsEnum(parameters, self.CLEAN_MODE, context)
        clean_modes = ["auto", "manual", "legacy", "none"]
        clean_mode_str = (
            clean_modes[mode_idx] if 0 <= mode_idx <= 3 else "auto"
        )
        extend_m = self.parameterAsDouble(
            parameters, self.EXTEND_DISTANCE, context
        )
        prune_stubs = self.parameterAsBoolean(
            parameters, self.PRUNE_STUBS, context
        )
        snap_dist = self.parameterAsDouble(
            parameters, self.SNAP_TOLERANCE, context
        )
        add_rungs = self.parameterAsBoolean(
            parameters, self.DUAL_CARRIAGEWAY, context
        )
        include_norm = self.parameterAsBoolean(
            parameters, self.NORMALIZE_OUTPUT, context
        )
        export_table = self.parameterAsBoolean(
            parameters, self.EXPORT_TABLE, context
        )
        apply_symbology = self.parameterAsBoolean(
            parameters, self.APPLY_SYMBOLOGY, context
        )
        open_scatter = self.parameterAsBoolean(
            parameters, self.OPEN_SCATTER_PLOT, context
        )
        raw_csv_param = parameters.get(self.OUTPUT_CSV)
        raw_sum_param = parameters.get(self.OUTPUT_SUMMARY_CSV)
        csv_path = (
            raw_csv_param
            if isinstance(raw_csv_param, str) and raw_csv_param.lower().endswith(".csv")
            else ""
        )
        summary_csv_path = (
            raw_sum_param
            if isinstance(raw_sum_param, str) and raw_sum_param.lower().endswith(".csv")
            else ""
        )

        feedback.setProgressText(
            "Reading vector geometries and attributes from input layer..."
        )
        raw_lines: List[LineString] = []
        raw_levels: List[float] = []
        raw_weights: List[float] = []

        field_names = [f.name() for f in source.fields()]
        total_features = source.featureCount()
        for idx, feat in enumerate(source.getFeatures()):
            if feedback.isCanceled():
                return {}
            geom = feat.geometry()
            if geom is None or geom.isNull() or geom.isEmpty():
                continue
            if xform is not None:
                geom = transform_qgs_geometry(geom, xform)

            props = {fn: feat[fn] for fn in field_names}
            lvl = (
                extract_grade_level_from_props(props, grade_field or "")
                if grade_sep
                else 0.0
            )
            w_val = 1.0
            if weight_mode_str == "custom" and weight_field in props:
                try:
                    val_raw = props[weight_field]
                    if val_raw is not None:
                        w_f = float(val_raw)
                        if math.isfinite(w_f) and w_f >= 0.0:
                            w_val = w_f
                except (ValueError, TypeError):
                    w_val = 1.0

            if geom.isMultipart():
                for part in geom.asMultiPolyline():
                    if len(part) >= 2:
                        raw_lines.append(
                            LineString([(p.x(), p.y()) for p in part])
                        )
                        raw_levels.append(lvl)
                        raw_weights.append(w_val)
            else:
                polyline = geom.asPolyline()
                if len(polyline) >= 2:
                    raw_lines.append(
                        LineString([(p.x(), p.y()) for p in polyline])
                    )
                    raw_levels.append(lvl)
                    raw_weights.append(w_val)

            if idx % 5000 == 0:
                feedback.setProgress(
                    int((idx / max(total_features, 1)) * 20.0)
                )

        if not raw_lines:
            raise RuntimeError("No valid line geometries found in input layer.")

        orig_lines = list(raw_lines)
        orig_levels = list(raw_levels)
        orig_weights = list(raw_weights)
        orig_tree = STRtree(orig_lines) if (grade_sep or weight_mode_str == "custom") else None

        feedback.setProgressText(
            f"Loaded {len(raw_lines):,} raw lines. "
            f"Preprocessing topology ({clean_mode_str.upper()})..."
        )

        if is_prenoded_segment_map(raw_lines) and dead_end_stub_len <= 0.0:
            feedback.setProgressText(
                "Input layer is already a planarized 2-point segment map (FALS-08 guard active) "
                "— preserving 1-to-1 segment topology."
            )
        elif clean_mode_str in ["auto", "manual"]:
            raw_lines, report = clean_network_adaptive_watertight(
                raw_lines,
                mode=clean_mode_str,
                extend_distance=extend_m,
                min_stub_len=0.5 if prune_stubs else 0.0,
                max_turn_deflection=15.0,
                lateral_fallback=True,
                run_assertions=False,
                dead_end_stub_len=dead_end_stub_len,
                line_levels=raw_levels if grade_sep else None
            )
            eff_d = report.get("effective_extend_distance", 0)
            n_rays = report.get("ray_extensions_count", 0)
            n_lat = report.get("lateral_connectors_count", 0)
            feedback.setProgressText(
                f"Enhanced Cleaner complete: search={eff_d}m, "
                f"rays={n_rays}, lateral={n_lat}"
            )
        elif clean_mode_str == "legacy":
            if snap_dist > 0.0:
                feedback.setProgressText(
                    f"Applying legacy STRtree watertight snapping ({snap_dist}m)..."
                )
                raw_lines = snap_endpoints_watertight(
                    raw_lines, snap_distance_m=snap_dist
                )
            feedback.setProgressText(
                "Planarizing crossings and noding intersections..."
            )
            raw_lines = planarize_and_node(raw_lines)
            feedback.setProgressText("Merging degree-2 collinear segments...")
            raw_lines = collinear_merge_preserving_junctions(raw_lines)
            if dead_end_stub_len > 0.0:
                raw_lines = remove_dead_end_stubs(
                    raw_lines, max_stub_length_m=dead_end_stub_len
                )
        else:
            if dead_end_stub_len > 0.0:
                raw_lines = remove_dead_end_stubs(
                    raw_lines, max_stub_length_m=dead_end_stub_len
                )

        if add_rungs:
            feedback.setProgressText("Synthesizing dual-carriageway rungs...")
            rungs = synthesize_dual_carriageway_rungs(raw_lines)
            raw_lines.extend(rungs)

        segments = explode_to_2point_segments(raw_lines)
        n_segs = len(segments)

        seg_levels: Optional[List[float]] = None
        custom_weights_arr: Optional[np.ndarray] = None
        if orig_tree is not None and n_segs > 0:
            nearest_idxs = [
                int(orig_tree.nearest(seg.interpolate(0.5, normalized=True)))
                for seg in segments
            ]
            if grade_sep:
                seg_levels = [orig_levels[k] for k in nearest_idxs]
            if weight_mode_str == "custom":
                custom_weights_arr = np.array(
                    [orig_weights[k] for k in nearest_idxs], dtype=np.float64
                )

        feedback.setProgress(30)
        feedback.setProgressText(
            f"Constructing CSR Dual Graph across {n_segs:,} segments "
            f"(tulip_bins={tulip_bins}, grade_separation={grade_sep})..."
        )
        graph, _ = CSRDualGraph.from_segments(
            segments,
            tulip_bins=tulip_bins,
            segment_levels=seg_levels
        )

        feedback.setProgress(40)
        feedback.setProgressText(
            f"Solving Segment Analysis ({analysis_type_str.upper()}, "
            f"radius_type={radius_type_str}) for {len(radii_list)} radii..."
        )

        def solver_callback(msg, pct):
            feedback.setProgressText(msg)
            feedback.setProgress(40 + int(pct * 0.45))

        asa_results = solve_angular_segment_analysis(
            graph,
            metric_radii=radii_list,
            progress_callback=solver_callback,
            include_normalized=include_norm,
            analysis_type=analysis_type_str,
            radius_type=radius_type_str,
            weight_mode=weight_mode_str,
            custom_weights=custom_weights_arr
        )

        diag = export_detailed_asa_tables(
            graph,
            asa_results,
            detailed_csv_path=csv_path if (csv_path and csv_path.strip()) else "",
            summary_csv_path=(
                summary_csv_path
                if (summary_csv_path and summary_csv_path.strip())
                else ""
            )
        )
        intell = diag.get("intelligibility_r2", 0.0)
        synergy = diag.get("synergy_r2", {})
        if isinstance(intell, dict):
            i_str = ", ".join(f"{k}={v:.4f}" for k, v in intell.items())
        else:
            i_str = f"{float(intell):.4f}"
        s_str = (
            ", ".join(f"{k}={v:.4f}" for k, v in synergy.items())
            if isinstance(synergy, dict) and synergy
            else "N/A"
        )
        feedback.setProgressText(
            f"Diagnostics — Intelligibility R²: {i_str} | "
            f"Synergy R²: [{s_str}]"
        )

        feedback.setProgress(85)
        feedback.setProgressText("Writing analyzed features to output sink...")

        # Create output fields
        fields = QgsFields()
        fields.append(QgsField("seg_id", QVariant.Int))
        fields.append(QgsField("length_m", QVariant.Double))

        for col_name in asa_results.keys():
            if col_name == "connectivity" or "node_count" in col_name:
                fields.append(QgsField(col_name, QVariant.Int))
            else:
                fields.append(QgsField(col_name, QVariant.Double))

        (sink, dest_id) = self.parameterAsSink(
            parameters,
            self.OUTPUT,
            context,
            fields,
            QgsWkbTypes.Type.LineString,
            out_crs
        )

        table_sink = None
        table_dest_id = csv_path if csv_path else None
        if export_table and raw_csv_param is not None and not csv_path:
            (table_sink, table_dest_id) = self.parameterAsSink(
                parameters,
                self.OUTPUT_CSV,
                context,
                fields,
                QgsWkbTypes.Type.NoGeometry,
                QgsCoordinateReferenceSystem()
            )

        for i in range(n_segs):
            if feedback.isCanceled():
                return {}
            line = segments[i]
            coords = list(line.coords)
            qgs_geom = QgsGeometry.fromPolylineXY([
                QgsPointXY(coords[0][0], coords[0][1]),
                QgsPointXY(coords[-1][0], coords[-1][1])
            ])

            attrs = [int(i), float(round(graph.segment_lengths[i], 2))]
            for col in asa_results.keys():
                val = asa_results[col][i]
                if col == "connectivity" or "node_count" in col:
                    attrs.append(int(val))
                else:
                    fval = float(val)
                    attrs.append(fval if math.isfinite(fval) else -1.0)

            if sink is not None:
                feat = QgsFeature(fields)
                feat.setGeometry(qgs_geom)
                feat.setAttributes(attrs)
                sink.addFeature(feat, QgsFeatureSink.Flag.FastInsert)

            if table_sink is not None:
                t_feat = QgsFeature(fields)
                t_feat.setAttributes(attrs)
                table_sink.addFeature(t_feat, QgsFeatureSink.Flag.FastInsert)

            if i % 10000 == 0:
                feedback.setProgress(85 + int((i / max(n_segs, 1)) * 15.0))

        sum_dest_id = summary_csv_path if summary_csv_path else None
        if export_table and raw_sum_param is not None and not summary_csv_path:
            sum_fields = QgsFields()
            sum_fields.append(QgsField("metric", QVariant.String))
            for st_col in ("min", "max", "mean", "std", "p10", "p50", "p90"):
                sum_fields.append(QgsField(st_col, QVariant.Double))
            (sum_sink, sum_dest_id) = self.parameterAsSink(
                parameters,
                self.OUTPUT_SUMMARY_CSV,
                context,
                sum_fields,
                QgsWkbTypes.Type.NoGeometry,
                QgsCoordinateReferenceSystem()
            )
            if sum_sink is not None:
                for m_name, st in diag.get("metrics", {}).items():
                    sf = QgsFeature(sum_fields)
                    sf.setAttributes([
                        m_name,
                        float(st["min"]), float(st["max"]), float(st["mean"]),
                        float(st["std"]), float(st["p10"]), float(st["p50"]), float(st["p90"])
                    ])
                    sum_sink.addFeature(sf, QgsFeatureSink.Flag.FastInsert)
                for r2_key in ("intelligibility_r2", "intelligibility_integration_r2"):
                    r2_v = float(diag.get(r2_key, 0.0))
                    sf = QgsFeature(sum_fields)
                    sf.setAttributes([r2_key, r2_v, r2_v, r2_v, 0.0, 0.0, 0.0, 0.0])
                    sum_sink.addFeature(sf, QgsFeatureSink.Flag.FastInsert)
                for r_lbl, syn_v in (diag.get("synergy_r2") or {}).items():
                    sv = float(syn_v)
                    sf = QgsFeature(sum_fields)
                    sf.setAttributes([f"synergy_{r_lbl}_r2", sv, sv, sv, 0.0, 0.0, 0.0, 0.0])
                    sum_sink.addFeature(sf, QgsFeatureSink.Flag.FastInsert)

        style_attr = ""
        if "NACH_Rn" in asa_results:
            style_attr = "NACH_Rn"
        elif "nach_rn" in asa_results:
            style_attr = "nach_rn"
        else:
            for k in asa_results.keys():
                if k.startswith(("NACH_", "Choice_", "Int_")) or k.lower().startswith(
                    ("nach_", "choice_", "int_", "integration_")
                ):
                    style_attr = k
                    break

        self._dest_id = dest_id
        self._apply_symbology = apply_symbology
        self._style_attr = style_attr
        self._style_values = (
            np.asarray(asa_results[style_attr], dtype=np.float64)
            if style_attr and style_attr in asa_results
            else None
        )
        self._open_scatter = open_scatter

        feedback.setProgress(100)
        feedback.setProgressText("Analysis finished successfully!")

        out_dict = {self.OUTPUT: dest_id}
        if table_dest_id:
            out_dict[self.OUTPUT_CSV] = table_dest_id
        if sum_dest_id:
            out_dict[self.OUTPUT_SUMMARY_CSV] = sum_dest_id
        return out_dict

    def postProcessAlgorithm(self, context, feedback):
        layer = None
        if getattr(self, "_dest_id", None):
            layer = QgsProcessingUtils.mapLayerFromString(
                self._dest_id, context
            )
        if getattr(self, "_apply_symbology", True) and layer is not None and layer.isValid():
            apply_space_syntax_style_to_layer(
                layer,
                attr_name=getattr(self, "_style_attr", ""),
                values=getattr(self, "_style_values", None),
                palette_name="classic_rainbow",
                line_width_mm=0.45
            )
        if getattr(self, "_open_scatter", False):
            self._schedule_scatter_dialog(layer)
        return {}

    def _schedule_scatter_dialog(self, layer=None):
        try:
            from qgis.utils import iface
            from ..dialog_scatter import SSEScatterDialog
            if iface is not None:
                def _open():
                    dlg = SSEScatterDialog(iface)
                    dlg.populate_layers(
                        preferred_layer=layer,
                        preferred_x="NAIN_R400",
                        preferred_y="NAIN_Rn"
                    )
                    dlg.show()
                    dlg.raise_()
                QTimer.singleShot(400, _open)
        except (ImportError, RuntimeError, AttributeError):
            pass

    def createInstance(self):
        return SSEASAAlgorithm()


OpenDepthmapASAAlgorithm = SSEASAAlgorithm
