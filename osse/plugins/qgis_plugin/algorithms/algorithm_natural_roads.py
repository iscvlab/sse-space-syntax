# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS Algorithm: Natural Roads (Gestalt Stroke Concatenation)
Concatenates street segments into perceptual continuous strokes based on
deflection angle threshold.
"""

import csv
import math
import os
import sys
from typing import List, Optional

import numpy as np
from qgis.core import (
    QgsProcessing,
    QgsProcessingAlgorithm,
    QgsProcessingParameterBoolean,
    QgsProcessingParameterFeatureSource,
    QgsProcessingParameterNumber,
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
from qgis.PyQt.QtCore import QVariant
from qgis.PyQt.QtGui import QIcon
from shapely.geometry import LineString
from shapely.strtree import STRtree

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
DEV_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
for p in [PLUGIN_ROOT, DEV_ROOT]:
    if p not in sys.path:
        sys.path.insert(0, p)

from core import (  # noqa: E402
    clean_network_adaptive_watertight,
    concatenate_natural_road_strokes,
    ensure_qgis_metric_crs,
    snap_endpoints_watertight,
    transform_qgs_geometry
)
from styles.qml_templates import apply_space_syntax_style_to_layer  # noqa: E402


class SSENaturalRoadsAlgorithm(QgsProcessingAlgorithm):
    INPUT = "INPUT"
    ANGLE_THRESHOLD = "ANGLE_THRESHOLD"
    CLEAN_NETWORK = "CLEAN_NETWORK"
    SNAP_TOLERANCE = "SNAP_TOLERANCE"
    EXPORT_TABLE = "EXPORT_TABLE"
    APPLY_SYMBOLOGY = "APPLY_SYMBOLOGY"
    OUTPUT = "OUTPUT"
    OUTPUT_CSV = "OUTPUT_CSV"

    def __init__(self):
        super().__init__()
        self._dest_id = None
        self._apply_symbology = True
        self._style_attr = "length_m"
        self._style_values = None
        self._stroke_lengths = None

    def name(self):
        return "natural_roads_strokes"

    def displayName(self):
        return "Natural Roads (Stroke Concatenation)"

    def icon(self):
        icon_path = os.path.join(PLUGIN_ROOT, "icon.png")
        return QIcon(icon_path) if os.path.exists(icon_path) else super().icon()

    def group(self):
        return "Stroke Analysis"

    def groupId(self):
        return "strokes"

    def shortHelpString(self):
        return (
            "<b>Natural Roads / Stroke Concatenation "
            "(Jiang & Claramunt 2004)</b><br>"
            "Concatenates fragmented street segments across junctions into "
            "perceptual movement corridors.<br><br>"
            "<b>Angle Threshold:</b> Maximum angular deflection (degrees) "
            "allowed across junctions to consider segments part of the same "
            "stroke (default: 20°).<br><br>"
            "<b>Outputs:</b><br>"
            "• <b>stroke_id</b>: Unique stroke identifier.<br>"
            "• <b>length_m</b>: Total concatenated stroke length (m).<br>"
            "• <b>connectivity</b>: Number of intersecting strokes.<br>"
            "• <b>sinuosity</b>: Ratio of stroke length to straight chord."
        )

    def initAlgorithm(self, config=None):
        self.addParameter(
            QgsProcessingParameterFeatureSource(
                self.INPUT,
                "Input Street Segments",
                [QgsProcessing.TypeVectorLine]
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.ANGLE_THRESHOLD,
                "Maximum Deflection Angle Threshold (Degrees)",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=20.0,
                minValue=1.0,
                maxValue=90.0
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.CLEAN_NETWORK,
                "Clean & Node Un-Noded Street Network Before Concatenation",
                defaultValue=False,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.SNAP_TOLERANCE,
                "Endpoint Snap Tolerance (meters) [when Clean Network enabled]",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=1.5,
                minValue=0.0,
                maxValue=50.0,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.EXPORT_TABLE,
                "Generate Detailed Stroke Attribute Table Layer in QGIS",
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
                "Concatenated Natural Road Strokes",
                type=QgsProcessing.TypeVectorLine
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT_CSV,
                "Detailed Stroke Attribute Table",
                type=QgsProcessing.TypeVector,
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

        angle_deg = self.parameterAsDouble(
            parameters, self.ANGLE_THRESHOLD, context
        )
        clean_net = self.parameterAsBoolean(
            parameters, self.CLEAN_NETWORK, context
        )
        snap_tol = self.parameterAsDouble(
            parameters, self.SNAP_TOLERANCE, context
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

        raw_lines: List[LineString] = []
        for feat in source.getFeatures():
            if feedback.isCanceled():
                return {}
            geom = feat.geometry()
            if geom is None or geom.isNull() or geom.isEmpty():
                continue
            if xform is not None:
                geom = transform_qgs_geometry(geom, xform)
            if geom.isMultipart():
                for part in geom.asMultiPolyline():
                    if len(part) >= 2:
                        raw_lines.append(
                            LineString([(p.x(), p.y()) for p in part])
                        )
            else:
                polyline = geom.asPolyline()
                if len(polyline) >= 2:
                    raw_lines.append(
                        LineString([(p.x(), p.y()) for p in polyline])
                    )

        if not raw_lines:
            raise RuntimeError("No valid line geometries found in input layer.")

        if clean_net:
            feedback.setProgressText(
                f"Cleaning and noding street network "
                f"(snap_tolerance={snap_tol}m)..."
            )
            if snap_tol > 0.0:
                raw_lines = snap_endpoints_watertight(
                    raw_lines, snap_distance_m=snap_tol
                )
            raw_lines, _ = clean_network_adaptive_watertight(
                raw_lines,
                mode="manual" if snap_tol > 0.0 else "auto",
                extend_distance=max(snap_tol, 1.0),
                run_assertions=False
            )

        feedback.setProgressText(
            f"Concatenating {len(raw_lines):,} segments into strokes "
            f"(theta <= {angle_deg}°)..."
        )
        strokes = concatenate_natural_road_strokes(
            raw_lines, max_deflection_deg=angle_deg
        )
        feedback.setProgressText(
            f"Synthesized {len(strokes):,} natural road strokes."
        )

        stroke_conn: List[int] = [0] * len(strokes)
        stroke_sinuosity: List[Optional[float]] = [None] * len(strokes)
        if len(strokes) > 0:
            tree = STRtree(strokes)
            q_idx, _ = tree.query(strokes, predicate="intersects")
            conn_counts = np.maximum(
                0, np.bincount(q_idx, minlength=len(strokes)) - 1
            )
            for i, s in enumerate(strokes):
                stroke_conn[i] = int(conn_counts[i])
                coords = list(s.coords)
                chord_len = math.hypot(
                    coords[-1][0] - coords[0][0],
                    coords[-1][1] - coords[0][1]
                )
                if chord_len <= 1e-6:
                    stroke_sinuosity[i] = None
                else:
                    stroke_sinuosity[i] = float(
                        round(float(s.length) / chord_len, 4)
                    )

        if csv_path and csv_path.strip():
            os.makedirs(
                os.path.dirname(os.path.abspath(csv_path)), exist_ok=True
            )
            with open(csv_path, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                writer.writerow(
                    ["stroke_id", "length_m", "connectivity", "sinuosity"]
                )
                for i, s in enumerate(strokes):
                    writer.writerow([
                        i,
                        round(float(s.length), 2),
                        stroke_conn[i],
                        "" if stroke_sinuosity[i] is None else round(stroke_sinuosity[i], 3)
                    ])

        fields = QgsFields()
        fields.append(QgsField("stroke_id", QVariant.Int))
        fields.append(QgsField("length_m", QVariant.Double))
        fields.append(QgsField("connectivity", QVariant.Int))
        fields.append(QgsField("sinuosity", QVariant.Double))

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

        for i, s in enumerate(strokes):
            if feedback.isCanceled():
                return {}
            coords = list(s.coords)
            qgs_geom = QgsGeometry.fromPolylineXY([
                QgsPointXY(p[0], p[1]) for p in coords
            ])
            sin_val = (
                None
                if stroke_sinuosity[i] is None
                else float(round(stroke_sinuosity[i], 3))
            )
            attrs = [
                int(i),
                float(round(s.length, 2)),
                int(stroke_conn[i]),
                sin_val
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

        self._dest_id = dest_id
        self._apply_symbology = apply_symbology
        self._style_attr = "length_m"
        self._stroke_lengths = np.array(
            [float(s.length) for s in strokes], dtype=np.float64
        )
        self._style_values = self._stroke_lengths

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
                    attr_name=getattr(self, "_style_attr", "length_m"),
                    values=getattr(self, "_style_values", None),
                    palette_name="classic_rainbow",
                    line_width_mm=0.50
                )
        return {}

    def createInstance(self):
        return SSENaturalRoadsAlgorithm()


OpenDepthmapNaturalRoadsAlgorithm = SSENaturalRoadsAlgorithm
