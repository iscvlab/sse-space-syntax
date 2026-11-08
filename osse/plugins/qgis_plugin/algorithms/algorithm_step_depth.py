# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS Algorithm: Origin Step Depth Analysis
Computes multi-source Angular, Metric, and Topological Step Depth from selected
origin segments or an external Origin Points/Lines layer.
"""

import math
import os
import sys
from typing import List, Optional

import numpy as np
import shapely
from shapely.geometry import LineString, Point
from shapely.strtree import STRtree

from qgis.core import (
    QgsProcessing,
    QgsProcessingAlgorithm,
    QgsProcessingParameterBoolean,
    QgsProcessingParameterDefinition,
    QgsProcessingParameterFeatureSource,
    QgsProcessingParameterString,
    QgsProcessingParameterNumber,
    QgsProcessingParameterEnum,
    QgsProcessingParameterFeatureSink,
    QgsProcessingUtils,
    QgsFeatureSink,
    QgsFields,
    QgsField,
    QgsGeometry,
    QgsPointXY,
    QgsFeature,
    QgsWkbTypes,
    QgsCoordinateReferenceSystem,
    QgsCoordinateTransform
)
from qgis.PyQt.QtCore import QVariant
from qgis.PyQt.QtGui import QIcon

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
    snap_endpoints_watertight,
    transform_qgs_geometry,
    explode_to_2point_segments,
    CSRDualGraph,
    solve_origin_step_depth,
    export_detailed_asa_tables
)
from styles.qml_templates import apply_space_syntax_style_to_layer  # noqa: E402


class SSEStepDepthAlgorithm(QgsProcessingAlgorithm):
    INPUT = "INPUT"
    ORIGIN_LAYER = "ORIGIN_LAYER"
    ORIGIN_POINTS = "ORIGIN_POINTS"
    ORIGIN_IDS = "ORIGIN_IDS"
    ORIGIN_SNAP_TOLERANCE = "ORIGIN_SNAP_TOLERANCE"
    MAX_RADIUS = "MAX_RADIUS"
    RADIUS_TYPE = "RADIUS_TYPE"
    TULIP_MODE = "TULIP_MODE"
    CLEAN_NETWORK = "CLEAN_NETWORK"
    SNAP_TOLERANCE = "SNAP_TOLERANCE"
    RESPECT_GRADE_SEPARATION = "RESPECT_GRADE_SEPARATION"
    EXPORT_TABLE = "EXPORT_TABLE"
    APPLY_SYMBOLOGY = "APPLY_SYMBOLOGY"
    OUTPUT = "OUTPUT"
    OUTPUT_CSV = "OUTPUT_CSV"

    def __init__(self):
        super().__init__()
        self._dest_id = None
        self._apply_symbology = True
        self._style_attr = ""
        self._style_values = None

    def name(self):
        return "origin_step_depth"

    def displayName(self):
        return (
            "Origin Step Depth Analysis (Angular / Metric / Topological)"
        )

    def group(self):
        return "Segment Analysis"

    def groupId(self):
        return "segment"

    def icon(self):
        icon_path = os.path.join(PLUGIN_ROOT, "icon.png")
        return QIcon(icon_path) if os.path.exists(icon_path) else super().icon()

    def shortHelpString(self):
        return (
            "<b>SSE (Space Syntax Engine) Origin Step Depth Analysis</b><br>"
            "Calculates exact multi-source shortest-path Step Depth from one "
            "or more origin segments (or an Origin Points/Lines layer) across "
            "the street network.<br><br>"
            "<b>Output Columns:</b><br>"
            "• <b>is_origin</b>: 1 if the segment is an origin source, else 0.<br>"
            "• <b>nearest_origin_id</b>: Segment ID of the closest origin.<br>"
            "• <b>angular_step_depth</b>: Cumulative angular turns from origin.<br>"
            "• <b>metric_step_depth_m</b>: Shortest metric distance (m) from origin.<br>"
            "• <b>topo_step_depth</b>: Topological step count (turns) from origin."
        )

    def initAlgorithm(self, config=None):
        self.addParameter(
            QgsProcessingParameterFeatureSource(
                self.INPUT,
                "Input Street Network Layer",
                [QgsProcessing.TypeVectorLine]
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSource(
                self.ORIGIN_LAYER,
                "Origin Points or Lines Layer (Optional — overrides Origin IDs)",
                [QgsProcessing.TypeVectorAnyGeometry],
                optional=True
            )
        )
        param_origin_pts = QgsProcessingParameterFeatureSource(
            self.ORIGIN_POINTS,
            "Origin Points Layer (Script Alias for ORIGIN_LAYER)",
            [QgsProcessing.TypeVectorAnyGeometry],
            optional=True
        )
        try:
            flag_hidden = getattr(
                QgsProcessingParameterDefinition,
                "FlagHidden",
                QgsProcessingParameterDefinition.Flag.FlagHidden
            )
            param_origin_pts.setFlags(param_origin_pts.flags() | flag_hidden)
        except Exception:  # nosec B110
            pass
        self.addParameter(param_origin_pts)
        self.addParameter(
            QgsProcessingParameterString(
                self.ORIGIN_IDS,
                "Origin Segment IDs (comma-separated, used if no Origin Layer)",
                defaultValue="0",
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.ORIGIN_SNAP_TOLERANCE,
                "Origin Layer Snap Tolerance (meters)",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=50.0,
                minValue=0.1,
                maxValue=5000.0
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.MAX_RADIUS,
                "Maximum Search Horizon (0 = Unlimited Global)",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=0.0,
                minValue=0.0
            )
        )
        self.addParameter(
            QgsProcessingParameterEnum(
                self.RADIUS_TYPE,
                "Search Horizon / Primary Routing Type",
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
                self.TULIP_MODE,
                "Angular Precision Mode",
                options=[
                    "Exact Continuous Radians (IEEE-754 Float64)",
                    "Tulip 1024 Bins (UCL depthmapX Compatibility)"
                ],
                defaultValue=0
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.CLEAN_NETWORK,
                "Clean & Node Street Network Before Step Depth Analysis",
                defaultValue=False,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.SNAP_TOLERANCE,
                "Network Endpoint Snap Tolerance (meters) [Clean Network]",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=1.5,
                minValue=0.0,
                maxValue=50.0,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.RESPECT_GRADE_SEPARATION,
                "Respect Bridge / Tunnel Grade Separation (layer/bridge/tunnel)",
                defaultValue=True,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.EXPORT_TABLE,
                "Generate Detailed Step Depth Table Layer in QGIS",
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
            QgsProcessingParameterFeatureSink(
                self.OUTPUT,
                "Step Depth Segment Network",
                type=QgsProcessing.TypeVectorLine
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT_CSV,
                "Detailed Step Depth Table",
                type=QgsProcessing.TypeVector,
                optional=True,
                createByDefault=True
            )
        )

    def processAlgorithm(self, parameters, context, feedback):
        source = self.parameterAsSource(parameters, self.INPUT, context)
        if source is None:
            raise RuntimeError("Invalid input street network layer.")

        out_crs, xform, _crs_reason = ensure_qgis_metric_crs(
            source.sourceCrs(),
            source.sourceExtent(),
            context.project(),
            feedback
        )

        origin_source = self.parameterAsSource(
            parameters, self.ORIGIN_LAYER, context
        )
        if origin_source is None and "ORIGIN_POINTS" in parameters:
            origin_source = self.parameterAsSource(
                parameters, "ORIGIN_POINTS", context
            )

        origin_ids_str = self.parameterAsString(
            parameters, self.ORIGIN_IDS, context
        )
        snap_tol = self.parameterAsDouble(
            parameters, self.ORIGIN_SNAP_TOLERANCE, context
        )
        max_r = self.parameterAsDouble(parameters, self.MAX_RADIUS, context)
        r_type_idx = self.parameterAsEnum(parameters, self.RADIUS_TYPE, context)
        tulip_idx = self.parameterAsEnum(parameters, self.TULIP_MODE, context)
        clean_net = self.parameterAsBoolean(
            parameters, self.CLEAN_NETWORK, context
        )
        net_snap_tol = self.parameterAsDouble(
            parameters, self.SNAP_TOLERANCE, context
        )
        respect_grade = (
            self.parameterAsBoolean(
                parameters, self.RESPECT_GRADE_SEPARATION, context
            )
            if self.RESPECT_GRADE_SEPARATION in parameters
            else True
        )
        export_table = self.parameterAsBoolean(
            parameters, self.EXPORT_TABLE, context
        )
        apply_symbology = self.parameterAsBoolean(
            parameters, self.APPLY_SYMBOLOGY, context
        )
        raw_csv_param = parameters.get(self.OUTPUT_CSV)
        csv_path = (
            raw_csv_param
            if isinstance(raw_csv_param, str) and raw_csv_param.lower().endswith(".csv")
            else ""
        )

        r_types = ["metric", "angular", "topological"]
        r_type_str = r_types[r_type_idx] if 0 <= r_type_idx < 3 else "metric"

        field_names = [f.name() for f in source.fields()]
        raw_lines: List[LineString] = []
        raw_levels: List[float] = []
        for feat in source.getFeatures():
            if feedback.isCanceled():
                return {}
            geom = feat.geometry()
            if geom is None or geom.isNull() or geom.isEmpty():
                continue
            if xform is not None:
                geom = transform_qgs_geometry(geom, xform)
            props = {fn: feat[fn] for fn in field_names}
            lvl = (
                extract_grade_level_from_props(props)
                if respect_grade
                else 0.0
            )
            if geom.isMultipart():
                for part in geom.asMultiPolyline():
                    if len(part) >= 2:
                        raw_lines.append(
                            LineString([(p.x(), p.y()) for p in part])
                        )
                        raw_levels.append(lvl)
            else:
                polyline = geom.asPolyline()
                if len(polyline) >= 2:
                    raw_lines.append(
                        LineString([(p.x(), p.y()) for p in polyline])
                    )
                    raw_levels.append(lvl)

        if not raw_lines:
            raise RuntimeError("No valid line geometries found in input layer.")

        orig_lines = list(raw_lines)
        orig_levels = list(raw_levels)
        orig_tree = STRtree(orig_lines) if respect_grade else None

        if clean_net:
            feedback.setProgressText(
                f"Cleaning street network (snap_tolerance={net_snap_tol}m, "
                f"grade_separation={respect_grade})..."
            )
            if net_snap_tol > 0.0:
                raw_lines = snap_endpoints_watertight(
                    raw_lines, snap_distance_m=net_snap_tol
                )
            raw_lines, _ = clean_network_adaptive_watertight(
                raw_lines,
                mode="manual" if net_snap_tol > 0.0 else "auto",
                extend_distance=max(net_snap_tol, 1.0),
                run_assertions=False,
                line_levels=raw_levels if respect_grade else None
            )

        segments = explode_to_2point_segments(raw_lines)
        n_segs = len(segments)
        feedback.setProgressText(
            f"Prepared {n_segs:,} 2-point segments for Origin Step Depth."
        )

        seg_levels: Optional[List[float]] = None
        if respect_grade and orig_tree is not None and n_segs > 0:
            seg_levels = [
                orig_levels[
                    int(
                        orig_tree.nearest(
                            seg.interpolate(0.5, normalized=True)
                        )
                    )
                ]
                for seg in segments
            ]

        # Determine origin_indices
        origin_indices: List[int] = []
        if origin_source is not None and origin_source.featureCount() > 0:
            origin_xform = None
            if origin_source.sourceCrs() != out_crs:
                origin_xform = QgsCoordinateTransform(
                    origin_source.sourceCrs(), out_crs, context.project()
                )
            tree = STRtree(segments)
            matched_set = set()
            for ofeat in origin_source.getFeatures():
                ogeom = ofeat.geometry()
                if ogeom is None or ogeom.isNull() or ogeom.isEmpty():
                    continue
                if origin_xform is not None:
                    ogeom = transform_qgs_geometry(ogeom, origin_xform)
                try:
                    shp_geom = shapely.from_wkt(ogeom.asWkt())
                except Exception:
                    c = ogeom.centroid().asPoint()
                    shp_geom = Point(c.x(), c.y())

                cand_idxs = tree.query(shp_geom.buffer(snap_tol))
                found_within = False
                for c_idx in cand_idxs:
                    idx_int = int(c_idx)
                    if shp_geom.distance(segments[idx_int]) <= snap_tol:
                        matched_set.add(idx_int)
                        found_within = True
                if not found_within:
                    nearest_idx = tree.nearest(shp_geom)
                    if nearest_idx is not None:
                        matched_set.add(int(nearest_idx))
            origin_indices = sorted(matched_set)

        if not origin_indices:
            parsed_ids: List[int] = []
            if origin_ids_str:
                for tok in origin_ids_str.split(","):
                    tok = tok.strip()
                    if tok:
                        try:
                            val = int(tok)
                            if 0 <= val < n_segs:
                                parsed_ids.append(val)
                        except ValueError:
                            continue
            origin_indices = sorted(set(parsed_ids)) if parsed_ids else [0]

        feedback.setProgressText(
            f"Constructing CSR Dual Graph (origins={len(origin_indices)}, "
            f"tulip_bins={1024 if tulip_idx == 1 else 0}, "
            f"grade_separation={respect_grade})..."
        )
        graph, _ = CSRDualGraph.from_segments(
            segments,
            tulip_bins=1024 if tulip_idx == 1 else 0,
            segment_levels=seg_levels
        )

        feedback.setProgressText(
            f"Solving Origin Step Depth (mode={r_type_str}, "
            f"max_radius={'inf' if max_r <= 0 else max_r})..."
        )
        step_results = solve_origin_step_depth(
            graph,
            origin_indices,
            max_radius=float("inf") if max_r <= 0 else max_r,
            radius_type=r_type_str
        )

        if csv_path and csv_path.strip():
            feedback.setProgressText(
                f"Exporting detailed Step Depth CSV to {csv_path}..."
            )
            export_detailed_asa_tables(
                graph,
                step_results,
                detailed_csv_path=csv_path
            )

        fields = QgsFields()
        fields.append(QgsField("seg_id", QVariant.Int))
        fields.append(QgsField("length_m", QVariant.Double))
        fields.append(QgsField("is_origin", QVariant.Int))
        fields.append(QgsField("nearest_origin_id", QVariant.Int))
        fields.append(QgsField("angular_step_depth", QVariant.Double))
        fields.append(QgsField("metric_step_depth_m", QVariant.Double))
        fields.append(QgsField("topo_step_depth", QVariant.Int))

        (sink, dest_id) = self.parameterAsSink(
            parameters,
            self.OUTPUT,
            context,
            fields,
            QgsWkbTypes.LineString,
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
                QgsWkbTypes.NoGeometry,
                QgsCoordinateReferenceSystem()
            )

        is_orig_arr = step_results["is_origin"]
        near_id_arr = step_results["nearest_origin_id"]
        ang_arr = step_results["angular_step_depth"]
        met_arr = step_results["metric_step_depth_m"]
        top_arr = step_results["topo_step_depth"]

        for i in range(n_segs):
            if feedback.isCanceled():
                return {}
            line = segments[i]
            coords = list(line.coords)
            qgs_geom = QgsGeometry.fromPolylineXY([
                QgsPointXY(coords[0][0], coords[0][1]),
                QgsPointXY(coords[-1][0], coords[-1][1])
            ])

            ang_val = float(ang_arr[i])
            met_val = float(met_arr[i])
            attrs = [
                int(i),
                float(round(graph.segment_lengths[i], 2)),
                int(is_orig_arr[i]),
                int(near_id_arr[i]),
                ang_val if math.isfinite(ang_val) else -1.0,
                met_val if math.isfinite(met_val) else -1.0,
                int(top_arr[i])
            ]
            if sink is not None:
                feat = QgsFeature(fields)
                feat.setGeometry(qgs_geom)
                feat.setAttributes(attrs)
                sink.addFeature(feat, QgsFeatureSink.FastInsert)
            if table_sink is not None:
                t_feat = QgsFeature(fields)
                t_feat.setAttributes(attrs)
                table_sink.addFeature(t_feat, QgsFeatureSink.FastInsert)

        if r_type_str == "metric":
            style_attr = (
                "step_depth_metric"
                if "step_depth_metric" in step_results
                else "metric_step_depth_m"
            )
        elif r_type_str == "angular":
            style_attr = (
                "step_depth_ang"
                if "step_depth_ang" in step_results
                else "angular_step_depth"
            )
        else:
            style_attr = (
                "step_depth_topo"
                if "step_depth_topo" in step_results
                else "topo_step_depth"
            )

        self._dest_id = dest_id
        self._apply_symbology = apply_symbology
        self._style_attr = style_attr
        self._style_values = (
            np.asarray(step_results[style_attr], dtype=np.float64)
            if style_attr in step_results
            else None
        )

        out_dict = {self.OUTPUT: dest_id}
        if table_dest_id:
            out_dict[self.OUTPUT_CSV] = table_dest_id
        return out_dict

    def postProcessAlgorithm(self, context, feedback):
        if getattr(self, "_apply_symbology", True) and getattr(self, "_dest_id", None):
            layer = QgsProcessingUtils.mapLayerFromString(
                self._dest_id, context
            )
            if layer is not None and layer.isValid():
                apply_space_syntax_style_to_layer(
                    layer,
                    attr_name=getattr(self, "_style_attr", ""),
                    values=getattr(self, "_style_values", None),
                    palette_name="inverted_rainbow",
                    line_width_mm=0.50,
                    ignore_negative=True
                )
        return {}

    def createInstance(self):
        return SSEStepDepthAlgorithm()


OpenDepthmapStepDepthAlgorithm = SSEStepDepthAlgorithm
