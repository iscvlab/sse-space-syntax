# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS Algorithm: Visibility Graph Analysis (VGA) & Isovists
Discretizes boundary polygon into walkable grid and computes Visual Integration,
Control, and Clustering.
"""

import csv
import math
import os
import sys
from typing import Any, List
import numpy as np
import shapely

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
    QgsCoordinateReferenceSystem,
    QgsCoordinateTransform
)
from qgis.PyQt.QtCore import QVariant
from qgis.PyQt.QtGui import QIcon
from shapely.geometry import Polygon, LineString

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
DEV_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
for p in [PLUGIN_ROOT, DEV_ROOT]:
    if p not in sys.path:
        sys.path.insert(0, p)

from core import (  # noqa: E402
    ensure_qgis_metric_crs,
    generate_vga_grid,
    solve_agent_analysis,
    solve_vga_visibility_graph,
    transform_qgs_geometry
)
from styles.qml_templates import apply_space_syntax_style_to_layer  # noqa: E402


class SSEVGAAlgorithm(QgsProcessingAlgorithm):
    BOUNDARY = "BOUNDARY"
    OBSTACLES = "OBSTACLES"
    GRID_SPACING = "GRID_SPACING"
    MAX_VISION = "MAX_VISION"
    RUN_AGENTS = "RUN_AGENTS"
    AGENT_TIMESTEPS = "AGENT_TIMESTEPS"
    AGENT_FOV_BINS = "AGENT_FOV_BINS"
    AGENT_STEPS_TURN = "AGENT_STEPS_TURN"
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
        return "visibility_graph_analysis"

    def displayName(self):
        return "Visibility Graph Analysis (VGA)"

    def group(self):
        return "Visibility Analysis"

    def groupId(self):
        return "vga"

    def icon(self):
        icon_path = os.path.join(PLUGIN_ROOT, "icon.png")
        return QIcon(icon_path) if os.path.exists(icon_path) else super().icon()

    def shortHelpString(self):
        return (
            "<b>Visibility Graph Analysis (Turner et al. 2001)</b><br>"
            "Calculates isovist-based line-of-sight visual fields inside "
            "spatial boundaries.<br><br>"
            "<b>Outputs:</b><br>"
            "• <b>vga_connectivity</b>: Number of visible grid points.<br>"
            "• <b>vga_mean_depth</b>: Average visual steps.<br>"
            "• <b>vga_integration_hh</b>: Visual Integration (Hillier & Hanson).<br>"
            "• <b>vga_control</b>: Visual Control.<br>"
            "• <b>vga_clustering_coef</b>: Visual Clustering Coefficient."
        )

    def initAlgorithm(self, config=None):
        self.addParameter(
            QgsProcessingParameterFeatureSource(
                self.BOUNDARY,
                "Walkable Boundary Polygon (Optional if Obstacles provided)",
                [QgsProcessing.TypeVectorPolygon],
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSource(
                self.OBSTACLES,
                "Obstacle Walls (Optional Lines/Polygons)",
                [QgsProcessing.TypeVectorLine, QgsProcessing.TypeVectorPolygon],
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.GRID_SPACING,
                "Grid Cell Spacing (meters)",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=10.0,
                minValue=0.5,
                maxValue=100.0,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.MAX_VISION,
                "Maximum Vision Horizon (meters, 0 = Unlimited)",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=150.0,
                minValue=0.0,
                maxValue=10000.0,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.RUN_AGENTS,
                "Run Gibson Ecological Visual Agent Simulation (Turner & Penn 2002 Gate_Counts)",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.AGENT_TIMESTEPS,
                "Agent Simulation Timesteps (Ticks) [Agent Mode]",
                type=QgsProcessingParameterNumber.Integer,
                defaultValue=3000,
                minValue=100,
                maxValue=50000,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.AGENT_FOV_BINS,
                "Agent Forward Field-of-View Bins (15 = ~170 deg) [Agent Mode]",
                type=QgsProcessingParameterNumber.Integer,
                defaultValue=15,
                minValue=1,
                maxValue=32,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.AGENT_STEPS_TURN,
                "Agent Steps Before Turn Decision [Agent Mode]",
                type=QgsProcessingParameterNumber.Integer,
                defaultValue=3,
                minValue=1,
                maxValue=20,
                optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.EXPORT_TABLE,
                "Generate Detailed Attribute Table Layer in QGIS",
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
                "VGA Grid Analysis Points",
                type=QgsProcessing.TypeVectorPoint
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT_CSV,
                "Detailed VGA Attribute Table",
                type=QgsProcessing.TypeVector,
                optional=True,
                createByDefault=True
            )
        )

    def processAlgorithm(self, parameters, context, feedback):
        boundary_source = self.parameterAsSource(
            parameters, self.BOUNDARY, context
        )
        obs_source = self.parameterAsSource(
            parameters, self.OBSTACLES, context
        )
        if boundary_source is None and obs_source is None:
            raise RuntimeError(
                "Either a Walkable Boundary Polygon or Obstacle Walls "
                "layer must be provided."
            )

        primary_src = obs_source if obs_source is not None else boundary_source
        out_crs, obs_xform, _crs_reason = ensure_qgis_metric_crs(
            primary_src.sourceCrs(),
            primary_src.sourceExtent(),
            context.project(),
            feedback
        )
        bound_xform = None
        if boundary_source is not None:
            if obs_source is None:
                bound_xform = obs_xform
            elif boundary_source.sourceCrs() != out_crs:
                bound_xform = QgsCoordinateTransform(
                    boundary_source.sourceCrs(), out_crs, context.project()
                )

        grid_step = self.parameterAsDouble(
            parameters, self.GRID_SPACING, context
        )
        max_vis = self.parameterAsDouble(
            parameters, self.MAX_VISION, context
        )
        run_agents = self.parameterAsBoolean(
            parameters, self.RUN_AGENTS, context
        )
        agent_timesteps = self.parameterAsInt(
            parameters, self.AGENT_TIMESTEPS, context
        )
        agent_fov_bins = self.parameterAsInt(
            parameters, self.AGENT_FOV_BINS, context
        )
        agent_steps_turn = self.parameterAsInt(
            parameters, self.AGENT_STEPS_TURN, context
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

        # Collect obstacles (both LineGeometry and PolygonGeometry)
        obstacle_lines: List[LineString] = []
        obstacle_geoms: List[Any] = []
        if obs_source is not None:
            for feat in obs_source.getFeatures():
                if feedback.isCanceled():
                    return {}
                g = feat.geometry()
                if g is None or g.isNull() or g.isEmpty():
                    continue
                if obs_xform is not None:
                    g = transform_qgs_geometry(g, obs_xform)
                if g.type() == QgsWkbTypes.GeometryType.LineGeometry:
                    polylines = (
                        g.asMultiPolyline()
                        if g.isMultipart()
                        else [g.asPolyline()]
                    )
                    for pl in polylines:
                        if len(pl) >= 2:
                            ln = LineString([(pt.x(), pt.y()) for pt in pl])
                            if ln.length > 1e-9:
                                obstacle_lines.append(ln)
                                obstacle_geoms.append(ln)
                elif g.type() == QgsWkbTypes.GeometryType.PolygonGeometry:
                    polygons = (
                        g.asMultiPolygon()
                        if g.isMultipart()
                        else [g.asPolygon()]
                    )
                    for poly_rings in polygons:
                        if not poly_rings or len(poly_rings[0]) < 3:
                            continue
                        for ring in poly_rings:
                            if len(ring) >= 2:
                                r_ln = LineString(
                                    [(pt.x(), pt.y()) for pt in ring]
                                )
                                if r_ln.length > 1e-9:
                                    obstacle_lines.append(r_ln)
                        ext_coords = [
                            (pt.x(), pt.y()) for pt in poly_rings[0]
                        ]
                        hole_coords = [
                            [(pt.x(), pt.y()) for pt in r]
                            for r in poly_rings[1:]
                            if len(r) >= 3
                        ]
                        obstacle_geoms.append(
                            Polygon(ext_coords, hole_coords)
                        )

        # Merge boundary polygons (or derive envelope from obstacles)
        poly_list = []
        if boundary_source is not None:
            for feat in boundary_source.getFeatures():
                if feedback.isCanceled():
                    return {}
                g = feat.geometry()
                if g is None or g.isNull() or g.isEmpty():
                    continue
                if bound_xform is not None:
                    g = transform_qgs_geometry(g, bound_xform)
                polygons = (
                    g.asMultiPolygon()
                    if g.isMultipart()
                    else [g.asPolygon()]
                )
                for p in polygons:
                    if p and len(p[0]) >= 3:
                        ext = [(pt.x(), pt.y()) for pt in p[0]]
                        holes = [
                            [(pt.x(), pt.y()) for pt in r]
                            for r in p[1:]
                            if len(r) >= 3
                        ]
                        poly_list.append(Polygon(ext, holes))

        if poly_list:
            combined_poly = shapely.unary_union(poly_list)
        elif obstacle_geoms:
            obs_union_env = shapely.unary_union(obstacle_geoms)
            combined_poly = obs_union_env.envelope.buffer(grid_step * 2.0)
        else:
            raise RuntimeError(
                "No valid geometries found in boundary or obstacle layers."
            )

        # Non-blocking resource guard (UL-08 MAX_CELLS protection)
        minx, miny, maxx, maxy = combined_poly.bounds
        span_x = max(maxx - minx, 0.0)
        span_y = max(maxy - miny, 0.0)
        est_cells = (span_x / max(grid_step, 1e-6)) * (
            span_y / max(grid_step, 1e-6)
        )
        if est_cells > 25000.0:
            old_step = grid_step
            grid_step = round(
                max(grid_step, math.sqrt((span_x * span_y) / 4500.0)), 2
            )
            feedback.pushWarning(
                f"Estimated VGA grid ({int(est_cells):,} cells) exceeds "
                f"25,000-cell safety limit. Auto-coarsening GRID_SPACING "
                f"from {old_step}m to {grid_step}m (<= 5,000 points)."
            )

        feedback.setProgressText(
            f"Generating grid points ({grid_step}m spacing)..."
        )
        grid_pts = generate_vga_grid(
            combined_poly,
            cell_size_m=grid_step,
            obstacles=obstacle_geoms,
            depthmapx_align=True
        )
        if len(grid_pts) > 15000:
            old_step = grid_step
            grid_step = round(
                grid_step * math.sqrt(len(grid_pts) / 4500.0), 2
            )
            feedback.pushWarning(
                f"Generated VGA grid ({len(grid_pts):,} points) exceeds "
                f"15,000-point safety limit. Auto-coarsening GRID_SPACING "
                f"from {old_step}m to {grid_step}m (<= 5,000 points)."
            )
            grid_pts = generate_vga_grid(
                combined_poly,
                cell_size_m=grid_step,
                obstacles=obstacle_geoms,
                depthmapx_align=True
            )

        feedback.setProgressText(
            f"Generated {len(grid_pts):,} walkable grid points."
        )

        feedback.setProgressText(
            "Solving visibility graph (LOS), Through Vision & Isovists..."
        )
        max_vis_param = max_vis if max_vis > 0 else None
        vga_results = solve_vga_visibility_graph(
            grid_pts,
            obstacle_lines,
            max_vision_m=max_vis_param,
            include_isovist=True,
            include_through_vision=True,
            cell_size_m=grid_step
        )

        if run_agents and len(grid_pts) > 0:
            feedback.setProgressText(
                "Running Gibson Ecological Visual Agent Simulation "
                f"({agent_timesteps} ticks, FOV={agent_fov_bins} bins)..."
            )
            ag_res = solve_agent_analysis(
                grid_pts,
                obstacle_lines,
                timesteps=max(100, int(agent_timesteps or 3000)),
                release_rate=0.1,
                fov_bins=max(1, min(32, int(agent_fov_bins or 15))),
                steps_before_turn=max(1, int(agent_steps_turn or 3)),
                cell_size_m=grid_step
            )
            vga_results["Gate_Counts"] = ag_res["Gate_Counts"]

        attr_cols = [
            c for c in vga_results.keys() if not c.startswith("_")
        ]
        if csv_path and csv_path.strip():
            os.makedirs(
                os.path.dirname(os.path.abspath(csv_path)), exist_ok=True
            )
            with open(csv_path, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                writer.writerow(["pt_id", "x", "y"] + attr_cols)
                for i, pt in enumerate(grid_pts):
                    row = [i, round(float(pt.x), 3), round(float(pt.y), 3)] + [
                        int(vga_results[c][i])
                        if c in ("Depthmap_Ref", "vga_connectivity")
                        else float(vga_results[c][i])
                        for c in attr_cols
                    ]
                    writer.writerow(row)

        fields = QgsFields()
        fields.append(QgsField("pt_id", QVariant.Int))
        for col in attr_cols:
            is_int = col in ("Depthmap_Ref", "vga_connectivity")
            fields.append(
                QgsField(col, QVariant.Int if is_int else QVariant.Double)
            )

        (sink, dest_id) = self.parameterAsSink(
            parameters,
            self.OUTPUT,
            context,
            fields,
            QgsWkbTypes.Point,
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

        for i, pt in enumerate(grid_pts):
            qgs_geom = QgsGeometry.fromPointXY(QgsPointXY(pt.x, pt.y))
            attrs = [i] + [
                int(vga_results[c][i])
                if c in ("Depthmap_Ref", "vga_connectivity")
                else float(vga_results[c][i])
                for c in attr_cols
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

        style_attr = (
            "v_int_HH"
            if "v_int_HH" in vga_results
            else (
                "vga_integration_hh"
                if "vga_integration_hh" in vga_results
                else "vga_connectivity"
            )
        )
        self._dest_id = dest_id
        self._apply_symbology = apply_symbology
        self._style_attr = style_attr
        self._style_values = (
            np.asarray(vga_results[style_attr], dtype=np.float64)
            if style_attr in vga_results
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
                    palette_name="classic_rainbow",
                    point_size_mm=2.4
                )
        return {}

    def createInstance(self):
        return SSEVGAAlgorithm()


OpenDepthmapVGAAlgorithm = SSEVGAAlgorithm
