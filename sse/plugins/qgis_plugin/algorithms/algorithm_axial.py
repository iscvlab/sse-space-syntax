# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS Algorithm: Topological Axial Analysis
Computes Hillier Axial Map metrics: Connectivity, Mean Depth, RA, RRA,
Rn, R3, Choice, Entropy, and Intelligibility R² with native QGIS Table Layer output.
"""

import csv
import os
import sys
from typing import List

import numpy as np
from qgis.core import (
    QgsProcessing,
    QgsProcessingAlgorithm,
    QgsProcessingParameterFeatureSource,
    QgsProcessingParameterString,
    QgsProcessingParameterBoolean,
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
from shapely.geometry import LineString

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
DEV_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
for p in [PLUGIN_ROOT, DEV_ROOT]:
    if p not in sys.path:
        sys.path.insert(0, p)

from core import (  # noqa: E402
    ensure_qgis_metric_crs,
    solve_axial_analysis,
    transform_qgs_geometry
)
from styles.qml_templates import apply_space_syntax_style_to_layer  # noqa: E402


class SSEAxialAlgorithm(QgsProcessingAlgorithm):
    INPUT = "INPUT"
    RADII = "RADII"
    INCLUDE_CHOICE = "INCLUDE_CHOICE"
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
        return "axial_analysis"

    def displayName(self):
        return "Topological Axial Analysis (Hillier / depthmapX)"

    def group(self):
        return "Axial Analysis"

    def icon(self):
        icon_path = os.path.join(PLUGIN_ROOT, "icon.png")
        return QIcon(icon_path) if os.path.exists(icon_path) else super().icon()

    def groupId(self):
        return "axial"

    def shortHelpString(self):
        return (
            "<b>Topological Axial Analysis (Full depthmapX v0.8.0 Parity)</b><br>"
            "Calculates full topological step-depth measures across axial "
            "sightlines.<br><br>"
            "<b>Outputs (for each radius e.g. Global n, R3, R5):</b><br>"
            "• <b>Connectivity, Control, Controllability</b><br>"
            "• <b>Integration [HH], Integration [P-value], Integration [Tekl]</b><br>"
            "• <b>Choice & Choice [Norm]</b> (Topological Betweenness)<br>"
            "• <b>RA, RRA, RA [Penn], Mean Depth, Total Depth, Node Count</b><br>"
            "• <b>Entropy, Relativised Entropy, Harmonic Mean Depth, Intensity</b><br>"
            "• <b>QGIS Table Layers</b>: Loads Detailed Attribute Table & Summary R² "
            "Diagnostics Table directly into the QGIS Layers panel."
        )

    def initAlgorithm(self, config=None):
        self.addParameter(
            QgsProcessingParameterFeatureSource(
                self.INPUT,
                "Input Axial Lines Layer",
                [QgsProcessing.SourceType.TypeVectorLine]
            )
        )
        self.addParameter(
            QgsProcessingParameterString(
                self.RADII,
                "Radius / list of radii (topological steps, use 'n' for Global)",
                defaultValue="n, 3, 5"
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.INCLUDE_CHOICE,
                "Include choice (betweenness)",
                defaultValue=True
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
                "Open Interactive Scatter Plot (Connectivity vs Integration R²) After Running",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT,
                "Analyzed Axial Map",
                type=QgsProcessing.SourceType.TypeVectorLine
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT_CSV,
                "Detailed Attribute Table",
                type=QgsProcessing.SourceType.TypeVector,
                optional=True,
                createByDefault=True
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT_SUMMARY_CSV,
                "Summary & Intelligibility R² Diagnostics Table",
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

        radii_str = (
            self.parameterAsString(parameters, self.RADII, context) or "n, 3, 5"
        )
        include_choice = self.parameterAsBoolean(
            parameters, self.INCLUDE_CHOICE, context
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
        radii_list = [t.strip() for t in radii_str.split(",") if t.strip()]

        raw_lines: List[LineString] = []
        for feat in source.getFeatures():
            geom = feat.geometry()
            if geom is None or geom.isNull() or geom.isEmpty():
                continue
            if xform is not None:
                geom = transform_qgs_geometry(geom, xform)
            if geom.isMultipart():
                polylines = geom.asMultiPolyline()
                for pl in polylines:
                    if len(pl) >= 2:
                        raw_lines.append(
                            LineString([(p.x(), p.y()) for p in pl])
                        )
            else:
                polyline = geom.asPolyline()
                if len(polyline) >= 2:
                    raw_lines.append(
                        LineString([(p.x(), p.y()) for p in polyline])
                    )

        feedback.setProgressText(
            f"Solving axial topology across {len(raw_lines):,} axial lines "
            f"(radii={radii_str})..."
        )
        results, intelligibility_r2 = solve_axial_analysis(
            raw_lines,
            radii=radii_list,
            include_choice=include_choice
        )
        feedback.setProgressText(
            f"Intelligibility R²: {intelligibility_r2:.4f}"
        )

        col_names = list(results.keys())

        fields = QgsFields()
        fields.append(QgsField("line_id", QVariant.Int))
        fields.append(QgsField("length_m", QVariant.Double))
        for col in col_names:
            is_int = col in ("Depthmap_Ref", "ax_connectivity")
            fields.append(
                QgsField(col, QVariant.Int if is_int else QVariant.Double)
            )

        (sink, dest_id) = self.parameterAsSink(
            parameters,
            self.OUTPUT,
            context,
            fields,
            QgsWkbTypes.Type.LineString,
            out_crs
        )

        # Optional Detailed Attribute Table Layer Sink (NoGeometry)
        table_sink = None
        table_dest_id = None
        raw_csv_param = parameters.get(self.OUTPUT_CSV)
        if export_table and raw_csv_param is not None:
            if isinstance(raw_csv_param, str) and raw_csv_param.lower().endswith(".csv"):
                os.makedirs(
                    os.path.dirname(os.path.abspath(raw_csv_param)), exist_ok=True
                )
                with open(raw_csv_param, "w", newline="", encoding="utf-8") as fh:
                    writer = csv.writer(fh)
                    writer.writerow(["line_id", "length_m"] + col_names)
                    for i, line in enumerate(raw_lines):
                        row = [i + 1, round(float(line.length), 3)] + [
                            int(results[c][i])
                            if c in ("Depthmap_Ref", "ax_connectivity")
                            else float(results[c][i])
                            for c in col_names
                        ]
                        writer.writerow(row)
                table_dest_id = raw_csv_param
            else:
                (table_sink, table_dest_id) = self.parameterAsSink(
                    parameters,
                    self.OUTPUT_CSV,
                    context,
                    fields,
                    QgsWkbTypes.Type.NoGeometry,
                    QgsCoordinateReferenceSystem()
                )

        for i, line in enumerate(raw_lines):
            coords = list(line.coords)
            qgs_geom = QgsGeometry.fromPolylineXY(
                [QgsPointXY(c[0], c[1]) for c in coords]
            )
            attrs = [i + 1, round(float(line.length), 3)] + [
                int(results[c][i])
                if c in ("Depthmap_Ref", "ax_connectivity")
                else float(results[c][i])
                for c in col_names
            ]
            if sink is not None:
                feat = QgsFeature(fields)
                feat.setGeometry(qgs_geom)
                feat.setAttributes(attrs)
                sink.addFeature(feat, QgsFeatureSink.Flag.FastInsert)

            if table_sink is not None:
                t_feat = QgsFeature(fields)
                t_feat.setAttributes(attrs)
                table_sink.addFeature(t_feat, QgsFeatureSink.Flag.FastInsert)

        # Optional Summary & Intelligibility R² Diagnostics Table Layer Sink
        sum_dest_id = None
        if export_table and parameters.get(self.OUTPUT_SUMMARY_CSV) is not None:
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
                for col in col_names:
                    arr = np.asarray(results[col], dtype=np.float64)
                    valid = arr[np.isfinite(arr) & (arr >= 0.0)]
                    if len(valid) > 0:
                        s_vals = [
                            col,
                            round(float(np.min(valid)), 4),
                            round(float(np.max(valid)), 4),
                            round(float(np.mean(valid)), 4),
                            round(float(np.std(valid)), 4),
                            round(float(np.percentile(valid, 10)), 4),
                            round(float(np.percentile(valid, 50)), 4),
                            round(float(np.percentile(valid, 90)), 4),
                        ]
                    else:
                        s_vals = [col, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    sf = QgsFeature(sum_fields)
                    sf.setAttributes(s_vals)
                    sum_sink.addFeature(sf, QgsFeatureSink.Flag.FastInsert)

                r2_feat = QgsFeature(sum_fields)
                r2_val = round(float(intelligibility_r2), 4)
                r2_feat.setAttributes([
                    "intelligibility_r2 (Connectivity vs Integration_HH)",
                    r2_val, r2_val, r2_val, 0.0, 0.0, 0.0, 0.0
                ])
                sum_sink.addFeature(r2_feat, QgsFeatureSink.Flag.FastInsert)

        style_attr = ""
        for cand in ("Int_HH_Rn", "Integration_HH_", "ax_integration_rn"):
            if cand in results:
                style_attr = cand
                break
        if not style_attr:
            for pfx in (
                "Int_HH_", "Integration_HH", "Connectivity", "ax_connectivity"
            ):
                found = next(
                    (k for k in results.keys() if k.startswith(pfx)), ""
                )
                if found:
                    style_attr = found
                    break

        self._dest_id = dest_id
        self._apply_symbology = apply_symbology
        self._style_attr = style_attr
        self._style_values = (
            np.asarray(results[style_attr], dtype=np.float64)
            if style_attr and style_attr in results
            else None
        )
        self._open_scatter = open_scatter

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
                line_width_mm=0.50
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
                        preferred_x="Connectivity",
                        preferred_y="Int_HH_Rn"
                    )
                    dlg.show()
                    dlg.raise_()
                QTimer.singleShot(400, _open)
        except (ImportError, RuntimeError, AttributeError):
            pass

    def createInstance(self):
        return SSEAxialAlgorithm()


OpenDepthmapAxialAlgorithm = SSEAxialAlgorithm
