# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS GUI Dialog: Enhanced 4-Step Network Cleaner (Dual-Mode Contract)
Provides:
- Direct OpenStreetMap Overpass API street network download (All / Vehicular / Pedestrian)
- Universal 4-Case Metric CRS Guard (auto-reprojects EPSG:4326, EPSG:3857, Feet -> Local UTM)
- Interactive RadioButton toggle (Auto Empirical P90 vs Manual SpinBox 15.0m),
  Micro-Stub Pruning checkbox, Dead-End Stub Pruning SpinBox, Grade Separation checkbox,
  Turn Conservation checkbox, and live topology + CRS audit reporting.
"""

import os
import sys
from typing import List

from qgis.core import (
    QgsCoordinateReferenceSystem,
    QgsCoordinateTransform,
    QgsFeature,
    QgsField,
    QgsFields,
    QgsGeometry,
    QgsPointXY,
    QgsProject,
    QgsVectorLayer,
    QgsWkbTypes,
)
from qgis.PyQt.QtCore import QVariant
from qgis.PyQt.QtGui import QIcon
from qgis.PyQt.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QTextEdit,
    QVBoxLayout,
)
from shapely.geometry import LineString

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
DEV_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
for p in [CURRENT_DIR, DEV_ROOT]:
    if p not in sys.path:
        sys.path.insert(0, p)

from core import (  # noqa: E402
    clean_network_adaptive_watertight,
    explode_to_2point_segments,
    extract_grade_level_from_props,
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


class SSECleanerDialog(QDialog):
    """
    Interactive Dual-Mode Network Cleaner Dialog for QGIS 3/4.
    Implements Invariant 7 (Dual-Mode Interface Contract) + Direct OSM Download & CRS Guard:
    - Input Source: Existing Vector Line Layer OR Direct OpenStreetMap Overpass Download.
    - Auto Mode (RadioButton): Adaptive empirical P90 gap analysis [6.0m, 20.0m].
    - Manual Mode (RadioButton + QDoubleSpinBox): Custom extension distance
      (default 15.0m, range 1.0–50.0m, step 0.5m).
    - Micro-Stub Pruning Checkbox (< 0.5m), Dead-End Stub Length SpinBox,
      Grade Separation Checkbox, & Turn Conservation Checkbox (Δθ ≤ 15°).
    """

    def __init__(self, iface, parent=None):
        super().__init__(parent or (iface.mainWindow() if iface else None))
        self.iface = iface
        self.setWindowTitle(
            "SSE (Space Syntax Engine) — Enhanced 5-Step Network Cleaner"
        )
        icon_path = os.path.join(CURRENT_DIR, "icon.png")
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))
        self.setMinimumWidth(540)
        self._init_ui()
        self.populate_layers()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        # 1. Input Street Network Layer & Direct OSM Overpass Download
        grp_input = QGroupBox("1. Input Street Network Layer")
        lay_input_grp = QVBoxLayout(grp_input)

        lay_input = QHBoxLayout()
        lay_input.addWidget(QLabel("Vector Line Layer:"))
        self.cmb_layer = QComboBox()
        lay_input.addWidget(self.cmb_layer, 1)
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.populate_layers)
        lay_input.addWidget(self.btn_refresh)
        lay_input_grp.addLayout(lay_input)

        lay_osm = QHBoxLayout()
        self.chk_download_osm = QCheckBox(
            "Download Directly from OpenStreetMap "
            "(Current Map Canvas Extent — No API Key)"
        )
        self.chk_download_osm.setChecked(False)
        lay_osm.addWidget(self.chk_download_osm)

        self.cmb_osm_mode = QComboBox()
        self.cmb_osm_mode.addItem("All Walkable & Drivable Streets", "all")
        self.cmb_osm_mode.addItem("Vehicular Only", "vehicular")
        self.cmb_osm_mode.addItem("Pedestrian Only", "pedestrian")
        self.cmb_osm_mode.setEnabled(False)
        lay_osm.addWidget(self.cmb_osm_mode)
        lay_input_grp.addLayout(lay_osm)

        self.chk_download_osm.toggled.connect(self._on_osm_toggled)
        layout.addWidget(grp_input)

        # 2. Dual-Mode Extension Contract (Auto vs Manual SpinBox)
        grp_mode = QGroupBox("2. Raycasting Extension Mode (Dual-Mode Contract)")
        lay_mode = QVBoxLayout(grp_mode)

        self.rb_auto = QRadioButton(
            "Auto Mode — Adaptive Empirical P90 Gap Distribution "
            "[6.0m–20.0m] (Recommended)"
        )
        self.rb_auto.setChecked(True)
        lay_mode.addWidget(self.rb_auto)

        lay_manual_row = QHBoxLayout()
        self.rb_manual = QRadioButton(
            "Manual Mode — Custom Extension Distance:"
        )
        lay_manual_row.addWidget(self.rb_manual)

        self.spin_extend = QDoubleSpinBox()
        self.spin_extend.setRange(1.0, 50.0)
        self.spin_extend.setSingleStep(0.5)
        self.spin_extend.setDecimals(1)
        self.spin_extend.setValue(15.0)
        self.spin_extend.setSuffix(" m")
        self.spin_extend.setEnabled(False)
        lay_manual_row.addWidget(self.spin_extend)
        lay_manual_row.addStretch()
        lay_mode.addLayout(lay_manual_row)

        self.rb_auto.toggled.connect(self._on_mode_toggled)
        self.rb_manual.toggled.connect(self._on_mode_toggled)
        layout.addWidget(grp_mode)

        # 3. Topological Guards & Pruning Options
        grp_opts = QGroupBox("3. Topological Conservation & Pruning Guards")
        lay_opts = QVBoxLayout(grp_opts)

        lay_stub_row = QHBoxLayout()
        self.chk_prune_stubs = QCheckBox(
            "Prune Dangling Micro-Stubs (Overshoot Trim):"
        )
        self.chk_prune_stubs.setChecked(True)
        lay_stub_row.addWidget(self.chk_prune_stubs)

        self.spin_stub = QDoubleSpinBox()
        self.spin_stub.setRange(0.1, 5.0)
        self.spin_stub.setSingleStep(0.1)
        self.spin_stub.setDecimals(2)
        self.spin_stub.setValue(0.50)
        self.spin_stub.setSuffix(" m")
        lay_stub_row.addWidget(self.spin_stub)
        lay_stub_row.addStretch()
        lay_opts.addLayout(lay_stub_row)
        self.chk_prune_stubs.toggled.connect(self.spin_stub.setEnabled)

        lay_dead_row = QHBoxLayout()
        lay_dead_row.addWidget(
            QLabel("Remove Dead-End Stubs Shorter Than (0 = disabled):")
        )
        self.spin_dead_end = QDoubleSpinBox()
        self.spin_dead_end.setRange(0.0, 500.0)
        self.spin_dead_end.setSingleStep(1.0)
        self.spin_dead_end.setDecimals(1)
        self.spin_dead_end.setValue(0.0)
        self.spin_dead_end.setSuffix(" m")
        lay_dead_row.addWidget(self.spin_dead_end)
        lay_dead_row.addStretch()
        lay_opts.addLayout(lay_dead_row)

        self.chk_grade_sep = QCheckBox(
            "Respect Bridge / Tunnel Grade Separation (layer / bridge / tunnel)"
        )
        self.chk_grade_sep.setChecked(True)
        lay_opts.addWidget(self.chk_grade_sep)

        self.chk_preserve_turns = QCheckBox(
            "Turn Conservation — Preserve Sharp Corners (Δθ > 15°) "
            "for Space Syntax Angular Depth"
        )
        self.chk_preserve_turns.setChecked(True)
        lay_opts.addWidget(self.chk_preserve_turns)

        self.chk_rungs = QCheckBox(
            "Synthesize Dual-Carriageway Transverse Pedestrian Cross-Rungs"
        )
        self.chk_rungs.setChecked(False)
        lay_opts.addWidget(self.chk_rungs)

        self.chk_explode = QCheckBox(
            "Explode Cleaned Corridors to 2-Point Space Syntax LINE Segments"
        )
        self.chk_explode.setChecked(True)
        lay_opts.addWidget(self.chk_explode)

        layout.addWidget(grp_opts)

        # 4. Post-Flight Audit Log
        grp_log = QGroupBox("4. Post-Flight Topological Audit Report")
        lay_log = QVBoxLayout(grp_log)
        self.txt_report = QTextEdit()
        self.txt_report.setReadOnly(True)
        self.txt_report.setMinimumHeight(140)
        self.txt_report.setPlaceholderText(
            "Step 0–5 topological & CRS verification metrics will appear here "
            "after execution..."
        )
        lay_log.addWidget(self.txt_report)
        layout.addWidget(grp_log)

        # 5. Action Buttons
        lay_btns = QHBoxLayout()
        lay_btns.addStretch()
        self.btn_run = QPushButton("Run Enhanced 4-Step Cleaner")
        self.btn_run.setDefault(True)
        self.btn_run.clicked.connect(self.run_cleaner)
        lay_btns.addWidget(self.btn_run)

        self.btn_close = QPushButton("Close")
        self.btn_close.clicked.connect(self.close)
        lay_btns.addWidget(self.btn_close)
        layout.addLayout(lay_btns)

    def _on_osm_toggled(self, checked: bool):
        self.cmb_osm_mode.setEnabled(checked)
        self.cmb_layer.setEnabled(not checked)
        self.btn_refresh.setEnabled(not checked)

    def _on_mode_toggled(self):
        self.spin_extend.setEnabled(self.rb_manual.isChecked())

    def populate_layers(self):
        self.cmb_layer.clear()
        for lyr_id, lyr in QgsProject.instance().mapLayers().items():
            if (
                isinstance(lyr, QgsVectorLayer)
                and lyr.geometryType() == QgsWkbTypes.GeometryType.LineGeometry
            ):
                self.cmb_layer.addItem(
                    f"{lyr.name()} ({lyr.featureCount():,} features)", lyr_id
                )

    def get_current_config(self) -> dict:
        """Returns the active UI parameter configuration dictionary."""
        prune = bool(self.chk_prune_stubs.isChecked())
        preserve = bool(self.chk_preserve_turns.isChecked())
        mode_map = {0: "all", 1: "vehicular", 2: "pedestrian"}
        osm_mode = str(
            self.cmb_osm_mode.currentData()
            or mode_map.get(self.cmb_osm_mode.currentIndex(), "all")
        )
        return {
            "mode": "auto" if self.rb_auto.isChecked() else "manual",
            "extend_distance": float(self.spin_extend.value()),
            "prune_stubs": prune,
            "min_stub_len": float(self.spin_stub.value()) if prune else 0.0,
            "dead_end_stub_len": float(self.spin_dead_end.value()),
            "grade_separation": bool(self.chk_grade_sep.isChecked()),
            "preserve_turns": preserve,
            "max_turn_deflection": 15.0 if preserve else 180.0,
            "rungs": bool(self.chk_rungs.isChecked()),
            "explode": bool(self.chk_explode.isChecked()),
            "download_osm": bool(self.chk_download_osm.isChecked()),
            "osm_network_mode": osm_mode,
        }

    def run_cleaner(self):
        cfg = self.get_current_config()
        raw_lines: List[LineString] = []
        raw_levels: List[float] = []

        if self.chk_download_osm.isChecked():
            if not self.iface or not self.iface.mapCanvas():
                QMessageBox.warning(
                    self,
                    "SSE — Space Syntax Engine",
                    "QGIS Map Canvas is not available for OSM extent download.",
                )
                return
            if fetch_osm_network is None:
                QMessageBox.warning(
                    self,
                    "SSE — Space Syntax Engine",
                    "OSM Overpass downloader (fetch_osm_network) is not available.",
                )
                return

            canvas = self.iface.mapCanvas()
            canvas_ext = canvas.extent()
            canvas_crs = canvas.mapSettings().destinationCrs()
            wgs84_crs = QgsCoordinateReferenceSystem("EPSG:4326")

            if canvas_crs is not None and canvas_crs.isValid() and canvas_crs != wgs84_crs:
                xf_wgs = QgsCoordinateTransform(
                    canvas_crs, wgs84_crs, QgsProject.instance()
                )
                wgs_ext = xf_wgs.transformBoundingBox(canvas_ext)
            else:
                wgs_ext = canvas_ext

            bbox_wgs84 = (
                float(wgs_ext.xMinimum()),
                float(wgs_ext.yMinimum()),
                float(wgs_ext.xMaximum()),
                float(wgs_ext.yMaximum()),
            )
            net_mode = cfg["osm_network_mode"]
            try:
                fetched_lines, props_list, target_epsg, _ = fetch_osm_network(
                    bbox_wgs84=bbox_wgs84,
                    network_mode=net_mode,
                    reproject_to_utm=True,
                )
            except Exception as exc:
                QMessageBox.warning(
                    self,
                    "SSE — Space Syntax Engine",
                    f"Failed to download OSM street network:\n{exc}",
                )
                return

            raw_lines = list(fetched_lines)
            raw_levels = [
                extract_grade_level_from_props(p)
                if cfg["grade_separation"]
                else 0.0
                for p in props_list
            ]
            crs_auth = f"EPSG:{target_epsg}"
            base_name = f"OSM_{net_mode.capitalize()}"
            crs_status_line = (
                f"✓ CRS Guard: OpenStreetMap Overpass ({net_mode}) "
                f"→ Auto-reprojected to Local UTM {crs_auth} (meters)"
            )
        else:
            lyr_id = self.cmb_layer.currentData()
            if not lyr_id:
                QMessageBox.warning(
                    self,
                    "SSE — Space Syntax Engine",
                    "Please select a valid input line layer.",
                )
                return

            source_lyr = QgsProject.instance().mapLayer(lyr_id)
            if not source_lyr:
                QMessageBox.warning(
                    self,
                    "SSE — Space Syntax Engine",
                    "Selected layer is no longer valid.",
                )
                return

            if ensure_qgis_metric_crs is not None:
                out_crs, xform, crs_reason = ensure_qgis_metric_crs(
                    source_lyr.crs(),
                    source_lyr.extent(),
                    QgsProject.instance(),
                )
            else:
                out_crs = source_lyr.crs()
                xform = None
                crs_reason = f"Native CRS ({out_crs.authid()}) retained"

            crs_auth = out_crs.authid() or "EPSG:32647"
            base_name = source_lyr.name()
            crs_status_line = f"✓ CRS Guard: {crs_reason}"

            field_names = [f.name() for f in source_lyr.fields()]
            has_highway_col = any(
                fn.lower() == "highway" for fn in field_names
            )

            for feat in source_lyr.getFeatures():
                geom = feat.geometry()
                if not geom or geom.isNull() or geom.isEmpty():
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
                    if cfg["grade_separation"]
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
                    pl = geom.asPolyline()
                    if len(pl) >= 2:
                        raw_lines.append(
                            LineString([(p.x(), p.y()) for p in pl])
                        )
                        raw_levels.append(lvl)

        if not raw_lines:
            QMessageBox.warning(
                self,
                "SSE — Space Syntax Engine",
                "No valid line geometries found in input source.",
            )
            return

        cleaned_lines, report = clean_network_adaptive_watertight(
            raw_lines,
            mode=cfg["mode"],
            extend_distance=cfg["extend_distance"],
            min_stub_len=cfg["min_stub_len"],
            max_turn_deflection=cfg["max_turn_deflection"],
            lateral_fallback=True,
            run_assertions=False,
            dead_end_stub_len=cfg["dead_end_stub_len"],
            line_levels=raw_levels if cfg["grade_separation"] else None,
        )

        if cfg["rungs"]:
            rungs = synthesize_dual_carriageway_rungs(cleaned_lines)
            cleaned_lines.extend(rungs)

        out_geoms = (
            explode_to_2point_segments(cleaned_lines)
            if cfg["explode"]
            else cleaned_lines
        )

        # Create memory output layer in active project using metric CRS
        eff_dist = report["effective_extend_distance"]
        out_name = (
            f"{base_name} "
            f"[Cleaned {cfg['mode'].upper()} {eff_dist}m]"
        )
        mem_lyr = QgsVectorLayer(
            f"LineString?crs={crs_auth}", out_name, "memory"
        )
        pr = mem_lyr.dataProvider()
        fields = QgsFields()
        fields.append(QgsField("seg_id", QVariant.Int))
        fields.append(QgsField("length_m", QVariant.Double))
        pr.addAttributes(fields)
        mem_lyr.updateFields()

        feats = []
        for idx, line in enumerate(out_geoms):
            coords = list(line.coords)
            f = QgsFeature(mem_lyr.fields())
            f.setGeometry(
                QgsGeometry.fromPolylineXY(
                    [QgsPointXY(pt[0], pt[1]) for pt in coords]
                )
            )
            f.setAttributes([idx, float(round(line.length, 2))])
            feats.append(f)

        pr.addFeatures(feats)
        mem_lyr.updateExtents()
        QgsProject.instance().addMapLayer(mem_lyr)

        summary_lines = [
            crs_status_line,
            (
                f"✓ Mode: {report['mode'].upper()} "
                f"(Effective Search Ceiling: {eff_dist} m)"
            ),
            (
                f"• Input Lines: {report['raw_lines_count']:,} → "
                f"Exploded 2-pt Segments: {report['exploded_segments_count']:,}"
            ),
            (
                f"• Dangling Endpoints: {report['raw_dangles_count']:,} → "
                f"{report['cleaned_dangles_count']:,}"
            ),
            (
                f"• Ray Extensions Created: {report['ray_extensions_count']:,} | "
                f"Lateral Connectors: {report['lateral_connectors_count']:,}"
            ),
            (
                f"• Micro-Stubs Pruned (< {cfg['min_stub_len']}m): "
                f"{report['pruned_micro_stubs_count']:,} "
                f"(Remaining: {report['remaining_micro_stubs_count']})"
            ),
            (
                f"• Graph Junctions (degree ≥ 3): "
                f"{report['raw_junctions_count']:,} → "
                f"{report['cleaned_junctions_count']:,}"
            ),
            (
                f"• Connected Components: {report['raw_components_count']:,} → "
                f"{report['cleaned_components_count']:,}"
            ),
            (
                f"• Total Length Delta: {report['length_delta_pct']:+.2f}% | "
                f"Assertions Passed: {report['assertions_passed']}"
            ),
            (
                f"✓ Added Output Layer: '{out_name}' "
                f"({len(out_geoms):,} features, CRS={crs_auth})"
            ),
        ]
        self.txt_report.setPlainText("\n".join(summary_lines))


OpenDepthmapCleanerDialog = SSECleanerDialog
