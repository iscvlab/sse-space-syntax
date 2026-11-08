# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Interactive Space Syntax Scatter Plot & R² Explorer (Qt5 / Qt6 Ready)
Provides depthmapX-style interactive bivariate scatter plots (e.g., Connectivity vs
Global Integration [HH] / NAIN for Intelligibility R², or Local vs Global NAIN for
Synergy R²) with OLS linear regression, Spectral/Turbo color ramp, and live
canvas feature selection linking.
"""

import math
import os
from typing import List, Optional, Tuple
import numpy as np

from qgis.core import (
    QgsProject,
    QgsVectorLayer,
    QgsWkbTypes
)
from qgis.PyQt.QtCore import Qt, QRectF, QPointF
from qgis.PyQt.QtGui import (
    QPainter,
    QPen,
    QColor,
    QBrush,
    QFont,
    QIcon
)
from qgis.PyQt.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QComboBox,
    QPushButton,
    QWidget,
    QSizePolicy,
    QFileDialog
)


def _space_syntax_color(t: float, selected: bool = False) -> QColor:
    """Maps normalized [0, 1] value to classic Space Syntax Blue-Cyan-Green-Yellow-Red ramp."""
    if selected:
        return QColor(255, 0, 255, 240)
    u = max(0.0, min(1.0, float(t)))
    if u < 0.25:
        k = u / 0.25
        r, g, b = 43, int(60 + 140 * k), 215
    elif u < 0.50:
        k = (u - 0.25) / 0.25
        r, g, b = int(43 + 80 * k), int(200 + 30 * k), int(215 - 155 * k)
    elif u < 0.75:
        k = (u - 0.50) / 0.25
        r, g, b = int(123 + 127 * k), int(230 - 40 * k), int(60 - 40 * k)
    else:
        k = (u - 0.75) / 0.25
        r, g, b = int(250 - 30 * k), int(190 - 155 * k), int(20 + 10 * k)
    return QColor(r, g, b, 195)


class ScatterCanvasWidget(QWidget):
    """Custom QPainter scatter plot canvas with rubber-band selection linking to QGIS."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(560, 400)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self.x_vals: np.ndarray = np.empty(0, dtype=np.float64)
        self.y_vals: np.ndarray = np.empty(0, dtype=np.float64)
        self.fids: List[int] = []
        self.selected_fids: set = set()
        self.x_label: str = "X"
        self.y_label: str = "Y"
        self.slope: float = 0.0
        self.intercept: float = 0.0
        self.r2: float = 0.0
        self.pearson_r: float = 0.0
        self._drag_start: Optional[QPointF] = None
        self._drag_curr: Optional[QPointF] = None
        self.on_selection_changed = None

    def set_data(
        self,
        x_vals: np.ndarray,
        y_vals: np.ndarray,
        fids: List[int],
        x_label: str,
        y_label: str
    ):
        self.x_vals = np.asarray(x_vals, dtype=np.float64)
        self.y_vals = np.asarray(y_vals, dtype=np.float64)
        self.fids = list(fids)
        self.selected_fids = set()
        self.x_label = x_label
        self.y_label = y_label

        if len(self.x_vals) >= 2 and float(np.std(self.x_vals)) > 1e-12 and float(np.std(self.y_vals)) > 1e-12:
            self.slope, self.intercept = [
                float(v) for v in np.polyfit(self.x_vals, self.y_vals, 1)
            ]
            corr = float(np.corrcoef(self.x_vals, self.y_vals)[0, 1])
            self.pearson_r = 0.0 if math.isnan(corr) else corr
            self.r2 = self.pearson_r * self.pearson_r
        else:
            self.slope = 0.0
            self.intercept = float(np.mean(self.y_vals)) if len(self.y_vals) > 0 else 0.0
            self.pearson_r = 0.0
            self.r2 = 0.0
        self.update()

    def _plot_rect(self) -> QRectF:
        w = float(self.width())
        h = float(self.height())
        return QRectF(72.0, 28.0, max(100.0, w - 96.0), max(100.0, h - 76.0))

    def _bounds(self) -> Tuple[float, float, float, float]:
        if len(self.x_vals) == 0:
            return 0.0, 1.0, 0.0, 1.0
        xmin, xmax = float(np.min(self.x_vals)), float(np.max(self.x_vals))
        ymin, ymax = float(np.min(self.y_vals)), float(np.max(self.y_vals))
        dx = (xmax - xmin) if xmax > xmin else 1.0
        dy = (ymax - ymin) if ymax > ymin else 1.0
        return xmin - 0.04 * dx, xmax + 0.04 * dx, ymin - 0.04 * dy, ymax + 0.04 * dy

    def _to_screen(
        self,
        x: float,
        y: float,
        rect: QRectF,
        bnds: Tuple[float, float, float, float]
    ) -> Tuple[float, float]:
        xmin, xmax, ymin, ymax = bnds
        sx = rect.left() + ((x - xmin) / max(xmax - xmin, 1e-12)) * rect.width()
        sy = rect.bottom() - ((y - ymin) / max(ymax - ymin, 1e-12)) * rect.height()
        return sx, sy

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.fillRect(self.rect(), QColor(250, 250, 250))

        rect = self._plot_rect()
        painter.fillRect(rect, QColor(255, 255, 255))
        painter.setPen(QPen(QColor(215, 220, 228), 1.0))
        painter.drawRect(rect)

        if len(self.x_vals) == 0:
            painter.setPen(QColor(110, 118, 129))
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "No valid numeric pairs to plot.")
            return

        bnds = self._bounds()
        xmin, xmax, ymin, ymax = bnds

        # Draw grid & tick labels
        font_tick = QFont("Segoe UI", 8)
        painter.setFont(font_tick)
        for i in range(6):
            frac = i / 5.0
            gx = rect.left() + frac * rect.width()
            gy = rect.bottom() - frac * rect.height()
            painter.setPen(QPen(QColor(238, 241, 245), 1.0))
            painter.drawLine(QPointF(gx, rect.top()), QPointF(gx, rect.bottom()))
            painter.drawLine(QPointF(rect.left(), gy), QPointF(rect.right(), gy))

            vx = xmin + frac * (xmax - xmin)
            vy = ymin + frac * (ymax - ymin)
            painter.setPen(QColor(80, 90, 105))
            painter.drawText(
                QRectF(gx - 30.0, rect.bottom() + 4.0, 60.0, 18.0),
                Qt.AlignmentFlag.AlignCenter,
                f"{vx:.2g}" if abs(vx) < 100 else f"{vx:.0f}"
            )
            painter.drawText(
                QRectF(4.0, gy - 9.0, 62.0, 18.0),
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                f"{vy:.3g}" if abs(vy) < 100 else f"{vy:.0f}"
            )

        # Draw axis labels
        font_axis = QFont("Segoe UI", 9, QFont.Weight.Bold)
        painter.setFont(font_axis)
        painter.setPen(QColor(15, 23, 42))
        painter.drawText(
            QRectF(rect.left(), rect.bottom() + 24.0, rect.width(), 20.0),
            Qt.AlignmentFlag.AlignCenter,
            f"{self.x_label}   (Drag box on chart to select segments on map)"
        )
        painter.drawText(
            QRectF(rect.left(), 4.0, rect.width(), 20.0),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            f"Y: {self.y_label}   |   R² = {self.r2:.4f} (r = {self.pearson_r:+.4f}, N = {len(self.x_vals):,})"
        )

        # Draw scatter points (downsample display if > 8,000 points for smooth 60fps UI)
        n_pts = len(self.x_vals)
        step = max(1, n_pts // 8000)
        y_min_v = float(np.min(self.y_vals))
        y_span_v = max(float(np.max(self.y_vals)) - y_min_v, 1e-12)

        painter.setPen(Qt.PenStyle.NoPen)
        for idx in range(0, n_pts, step):
            fid = self.fids[idx]
            is_sel = fid in self.selected_fids
            sx, sy = self._to_screen(float(self.x_vals[idx]), float(self.y_vals[idx]), rect, bnds)
            t_col = (float(self.y_vals[idx]) - y_min_v) / y_span_v
            painter.setBrush(QBrush(_space_syntax_color(t_col, selected=is_sel)))
            rad = 4.5 if is_sel else 2.8
            painter.drawEllipse(QPointF(sx, sy), rad, rad)

        # Draw OLS Linear Regression trendline
        y_x0 = self.slope * xmin + self.intercept
        y_x1 = self.slope * xmax + self.intercept
        lx0, ly0 = self._to_screen(xmin, y_x0, rect, bnds)
        lx1, ly1 = self._to_screen(xmax, y_x1, rect, bnds)
        painter.setClipRect(rect)
        pen_reg = QPen(QColor(15, 23, 42), 2.2, Qt.PenStyle.DashLine)
        painter.setPen(pen_reg)
        painter.drawLine(QPointF(lx0, ly0), QPointF(lx1, ly1))
        painter.setClipping(False)

        # Draw rubber-band selection box if dragging
        if self._drag_start is not None and self._drag_curr is not None:
            sel_rect = QRectF(self._drag_start, self._drag_curr).normalized()
            painter.setPen(QPen(QColor(29, 78, 216), 1.5, Qt.PenStyle.SolidLine))
            painter.setBrush(QBrush(QColor(29, 78, 216, 40)))
            painter.drawRect(sel_rect)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = QPointF(event.pos())
            self._drag_curr = QPointF(event.pos())
            self.update()

    def mouseMoveEvent(self, event):
        if self._drag_start is not None:
            self._drag_curr = QPointF(event.pos())
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self._drag_start is not None:
            end_pt = QPointF(event.pos())
            sel_rect = QRectF(self._drag_start, end_pt).normalized()
            self._drag_start = None
            self._drag_curr = None

            if len(self.x_vals) > 0:
                rect = self._plot_rect()
                bnds = self._bounds()
                chosen = set()
                if sel_rect.width() < 5.0 and sel_rect.height() < 5.0:
                    # Single-click nearest point within 8px
                    cx, cy = end_pt.x(), end_pt.y()
                    best_d = 64.0
                    best_fid = None
                    for i in range(len(self.x_vals)):
                        sx, sy = self._to_screen(float(self.x_vals[i]), float(self.y_vals[i]), rect, bnds)
                        d2 = (sx - cx) ** 2 + (sy - cy) ** 2
                        if d2 < best_d:
                            best_d = d2
                            best_fid = self.fids[i]
                    if best_fid is not None:
                        chosen.add(best_fid)
                else:
                    for i in range(len(self.x_vals)):
                        sx, sy = self._to_screen(float(self.x_vals[i]), float(self.y_vals[i]), rect, bnds)
                        if sel_rect.contains(QPointF(sx, sy)):
                            chosen.add(self.fids[i])

                self.selected_fids = chosen
                if self.on_selection_changed is not None:
                    self.on_selection_changed(list(chosen))
            self.update()


class SSEScatterDialog(QDialog):
    """Interactive Space Syntax R² Scatter Plot Explorer linked to QGIS layers."""

    def __init__(self, iface=None, parent=None):
        super().__init__(parent or (iface.mainWindow() if iface else None))
        self.iface = iface
        self.setWindowTitle("SSE — Space Syntax Scatter Plot (Intelligibility & Synergy R²)")
        icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icon.png")
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))
        self.resize(760, 560)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        # Top control bar
        top_row = QHBoxLayout()
        top_row.addWidget(QLabel("Layer:"))
        self.cmb_layer = QComboBox()
        self.cmb_layer.setMinimumWidth(200)
        self.cmb_layer.currentIndexChanged.connect(self._on_layer_changed)
        top_row.addWidget(self.cmb_layer, 2)

        top_row.addWidget(QLabel("X Axis:"))
        self.cmb_x = QComboBox()
        self.cmb_x.currentIndexChanged.connect(self.refresh_plot)
        top_row.addWidget(self.cmb_x, 1)

        top_row.addWidget(QLabel("Y Axis:"))
        self.cmb_y = QComboBox()
        self.cmb_y.currentIndexChanged.connect(self.refresh_plot)
        top_row.addWidget(self.cmb_y, 1)

        btn_export = QPushButton("Save Plot PNG...")
        btn_export.clicked.connect(self._export_png)
        top_row.addWidget(btn_export)
        layout.addLayout(top_row)

        # Stats summary banner
        self.lbl_stats = QLabel("Select a Space Syntax layer and X/Y numeric fields.")
        self.lbl_stats.setStyleSheet(
            "background: #f1f5f9; color: #0f172a; padding: 6px 10px; "
            "border-radius: 4px; font-weight: bold;"
        )
        layout.addWidget(self.lbl_stats)

        # Scatter canvas
        self.canvas = ScatterCanvasWidget(self)
        self.canvas.on_selection_changed = self._select_on_map
        layout.addWidget(self.canvas, 1)

    def populate_layers(
        self,
        preferred_layer: Optional[QgsVectorLayer] = None,
        preferred_x: str = "",
        preferred_y: str = ""
    ):
        self.cmb_layer.blockSignals(True)
        self.cmb_layer.clear()
        layers = [
            lyr for lyr in QgsProject.instance().mapLayers().values()
            if isinstance(lyr, QgsVectorLayer) and lyr.featureCount() > 0
        ]
        sel_idx = 0
        for idx, lyr in enumerate(layers):
            self.cmb_layer.addItem(lyr.name(), lyr.id())
            if preferred_layer is not None and lyr.id() == preferred_layer.id():
                sel_idx = idx
            elif preferred_layer is None and (
                "SSE" in lyr.name()
                or "OpenDepthmap" in lyr.name()
                or "Analyzed" in lyr.name()
                or "Axial" in lyr.name()
            ):
                sel_idx = idx
        self.cmb_layer.setCurrentIndex(sel_idx)
        self.cmb_layer.blockSignals(False)
        self._on_layer_changed(preferred_x=preferred_x, preferred_y=preferred_y)

    def _current_layer(self) -> Optional[QgsVectorLayer]:
        lyr_id = self.cmb_layer.currentData()
        if not lyr_id:
            return None
        return QgsProject.instance().mapLayer(lyr_id)

    def _on_layer_changed(self, _idx=0, preferred_x: str = "", preferred_y: str = ""):
        lyr = self._current_layer()
        self.cmb_x.blockSignals(True)
        self.cmb_y.blockSignals(True)
        self.cmb_x.clear()
        self.cmb_y.clear()
        if lyr is not None:
            num_fields = [
                f.name() for f in lyr.fields()
                if f.isNumeric() and f.name() not in ("fid", "seg_id", "line_id", "cell_id", "Depthmap_Ref")
            ]
            for fn in num_fields:
                self.cmb_x.addItem(fn)
                self.cmb_y.addItem(fn)

            def _match_field(target: str, candidates: Tuple[str, ...]) -> str:
                if target:
                    if target in num_fields:
                        return target
                    t_low = target.lower()
                    for fn in num_fields:
                        if fn.lower() == t_low or fn.lower().startswith(t_low):
                            return fn
                for cand in candidates:
                    c_low = cand.lower()
                    for fn in num_fields:
                        if fn.lower() == c_low or fn.lower().startswith(c_low):
                            return fn
                return ""

            x_target = _match_field(
                preferred_x,
                ("NAIN_R400", "NAIN_R800", "Int_R400", "Int_R800", "Int_HH_R3", "Connectivity", "vga_connectivity"),
            )
            y_target = _match_field(
                preferred_y,
                ("NAIN_Rn", "Int_Rn", "Int_HH_Rn", "Integration_HH", "v_int_HH", "vga_integration_hh"),
            )
            if x_target and x_target in num_fields:
                self.cmb_x.setCurrentText(x_target)
            if y_target and y_target in num_fields and y_target != x_target:
                self.cmb_y.setCurrentText(y_target)
            elif len(num_fields) >= 2:
                self.cmb_y.setCurrentIndex(1)

        self.cmb_x.blockSignals(False)
        self.cmb_y.blockSignals(False)
        self.refresh_plot()

    def refresh_plot(self):
        lyr = self._current_layer()
        fx = self.cmb_x.currentText()
        fy = self.cmb_y.currentText()
        if lyr is None or not fx or not fy:
            self.canvas.set_data(np.empty(0), np.empty(0), [], "X", "Y")
            return

        xs, ys, fids = [], [], []
        for feat in lyr.getFeatures():
            vx = feat[fx]
            vy = feat[fy]
            if vx is None or vy is None:
                continue
            try:
                fvx = float(vx)
                fvy = float(vy)
                if math.isfinite(fvx) and math.isfinite(fvy) and fvx >= 0.0 and fvy >= 0.0:
                    xs.append(fvx)
                    ys.append(fvy)
                    fids.append(feat.id())
            except (ValueError, TypeError):
                continue

        self.canvas.set_data(
            np.asarray(xs, dtype=np.float64),
            np.asarray(ys, dtype=np.float64),
            fids,
            fx,
            fy
        )
        self.lbl_stats.setText(
            f"OLS Regression: {fy} = {self.canvas.slope:.4f} × {fx} + {self.canvas.intercept:.4f}   |   "
            f"R² = {self.canvas.r2:.4f}   |   Pearson r = {self.canvas.pearson_r:+.4f}   |   "
            f"Valid Features: {len(fids):,}"
        )

    def _select_on_map(self, fids: List[int]):
        lyr = self._current_layer()
        if lyr is not None and lyr.geometryType() != QgsWkbTypes.GeometryType.NullGeometry:
            lyr.selectByIds(fids)
            if self.iface and fids:
                self.iface.mapCanvas().refresh()

    def _export_png(self):
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Space Syntax Scatter Plot PNG",
            "space_syntax_scatter_r2.png",
            "PNG Images (*.png)"
        )
        if path:
            pix = self.canvas.grab()
            pix.save(path, "PNG")


OpenDepthmapScatterDialog = SSEScatterDialog
