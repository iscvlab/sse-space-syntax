# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Styles: QGIS QML Style Generator & DepthmapX Palette Suite
Generates valid QGIS graduated line and point style XMLs (.qml) embedding:
1. Classic DepthmapX Rainbow (Indigo -> Blue -> Cyan -> Green -> Yellow -> Orange -> Red)
2. Inverted Rainbow
3. Swiss Modern High-Contrast (Thesis Cartographic Standard)
4. Thermal / Heatmap
5. Greyscale
6. Viridis
"""

import os
import tempfile
from typing import Any, List, Optional
import numpy as np


PALETTES = {
    "classic_rainbow": [
        (37, 99, 235, 255),    # Blue
        (6, 182, 212, 255),    # Cyan
        (16, 185, 129, 255),   # Green
        (245, 158, 11, 255),   # Amber/Yellow
        (239, 68, 68, 255),    # Red
    ],
    "inverted_rainbow": [
        (239, 68, 68, 255),    # Red
        (245, 158, 11, 255),   # Amber/Yellow
        (16, 185, 129, 255),   # Green
        (6, 182, 212, 255),    # Cyan
        (37, 99, 235, 255),    # Blue
    ],
    "swiss": [
        (37, 99, 235, 240),    # #2563EB Segregated / Quiet
        (6, 182, 212, 240),    # #06B6D4 Neighborhood
        (16, 185, 129, 240),   # #10B981 Sub-Arterial
        (245, 158, 11, 255),   # #F59E0B Primary Through
        (239, 68, 68, 255),    # #EF4444 Super-Arterial Spines
    ],
    "thermal": [
        (40, 20, 80, 255),     # Dark Indigo
        (120, 28, 129, 255),   # Purple
        (217, 33, 90, 255),    # Crimson
        (245, 130, 32, 255),   # Orange
        (254, 210, 50, 255),   # Bright Yellow
    ],
    "greyscale": [
        (220, 220, 225, 255),  # Light grey
        (175, 175, 180, 255),
        (120, 120, 125, 255),
        (70, 70, 75, 255),
        (20, 20, 25, 255),     # Charcoal Black
    ],
    "viridis": [
        (68, 1, 84, 255),      # Dark Violet
        (59, 82, 139, 255),    # Blue
        (33, 145, 140, 255),   # Teal
        (94, 201, 98, 255),    # Light Green
        (253, 231, 37, 255),   # Yellow
    ]
}


def _ensure_strictly_increasing_breaks(breaks: List[float], num_classes: int) -> List[float]:
    if not breaks or len(breaks) != num_classes + 1:
        return list(np.linspace(0.0, 1.0, num_classes + 1))
    b = [float(x) for x in breaks]
    if b[-1] <= b[0]:
        base = b[0]
        return [base + i * 0.1 for i in range(num_classes + 1)]
    # Check if strictly increasing
    is_strict = all(b[i + 1] > b[i] for i in range(len(b) - 1))
    if is_strict:
        return b
    # Fallback to linear spacing across [min, max] if quantiles collapsed
    return [float(x) for x in np.linspace(b[0], b[-1], num_classes + 1)]


def generate_qml_style(
    attr_name: str = "nach_r800",
    values: Optional[np.ndarray] = None,
    palette_name: str = "swiss",
    line_width_mm: float = 0.35,
    custom_breaks: Optional[List[float]] = None,
    geom_type: str = "line",
    point_size_mm: float = 2.2,
    ignore_negative: bool = False
) -> str:
    """
    Generates QGIS 3.x XML graduated style (.qml) for line or point vectors.
    Computes natural quantile or custom breaks with finite and non-negative guards.
    """
    palette = PALETTES.get(palette_name, PALETTES["swiss"])
    num_classes = len(palette)
    gtype = str(geom_type or "line").strip().lower()

    if custom_breaks and len(custom_breaks) == num_classes + 1:
        breaks = _ensure_strictly_increasing_breaks(list(custom_breaks), num_classes)
    elif values is not None and len(values) > 0:
        arr = np.asarray(values, dtype=np.float64)
        valid_vals = arr[np.isfinite(arr)]
        if ignore_negative:
            valid_vals = valid_vals[valid_vals >= 0.0]
        if len(valid_vals) > 0:
            qs = np.linspace(0, 100, num_classes + 1)
            raw_breaks = list(np.percentile(valid_vals, qs))
            breaks = _ensure_strictly_increasing_breaks(raw_breaks, num_classes)
        else:
            breaks = list(np.linspace(0.0, 1.0, num_classes + 1))
    else:
        # Default breaks suitable for NACH / normalized values
        breaks = [0.0, 0.70, 0.90, 1.15, 1.40, 2.0]

    ranges_xml = []
    symbols_xml = []

    for i in range(num_classes):
        low = breaks[i]
        high = breaks[i + 1]
        color = palette[i]
        color_str = f"{color[0]},{color[1]},{color[2]},{color[3]}"
        label = f"{low:.2f} - {high:.2f}"

        ranges_xml.append(
            f'      <range symbol="{i}" lower="{low:.4f}" upper="{high:.4f}" label="{label}" render="true"/>'
        )

        if gtype == "point":
            symbols_xml.append(f"""      <symbol type="marker" name="{i}" alpha="1.0" clip_to_extent="1" force_rhr="0">
        <layer class="SimpleMarker" locked="0" enabled="1" pass="{i}">
          <Option type="Map">
            <Option type="QString" name="color" value="{color_str}"/>
            <Option type="QString" name="name" value="circle"/>
            <Option type="QString" name="outline_style" value="no"/>
            <Option type="QString" name="size" value="{point_size_mm:.2f}"/>
            <Option type="QString" name="size_unit" value="MM"/>
          </Option>
        </layer>
      </symbol>""")
        else:
            symbols_xml.append(f"""      <symbol type="line" name="{i}" alpha="1.0" clip_to_extent="1" force_rhr="0">
        <layer class="SimpleLine" locked="0" enabled="1" pass="{i}">
          <Option type="Map">
            <Option type="QString" name="line_color" value="{color_str}"/>
            <Option type="QString" name="line_style" value="solid"/>
            <Option type="QString" name="line_width" value="{line_width_mm:.2f}"/>
            <Option type="QString" name="line_width_unit" value="MM"/>
          </Option>
        </layer>
      </symbol>""")

    ranges_str = "\n".join(ranges_xml)
    symbols_str = "\n".join(symbols_xml)

    qgis_hdr = (
        '<qgis styleCategories="AllStyleCategories" version="3.34.0" '
        'hasScaleBasedVisibilityFlag="0" minScale="1e+08" maxScale="0">'
    )
    rend_hdr = (
        f'<renderer-v2 type="graduatedSymbol" attr="{attr_name}" '
        'graduatedMethod="GraduatedColor" symbollevels="1" enableorderby="0" forceraster="0">'
    )
    qml_content = f"""<!DOCTYPE qgis PUBLIC 'http://mrcc.com/qgis.dtd' 'SYSTEM'>
{qgis_hdr}
  {rend_hdr}
    <ranges>
{ranges_str}
    </ranges>
    <symbols>
{symbols_str}
    </symbols>
  </renderer-v2>
  <blendMode>0</blendMode>
  <featureBlendMode>0</featureBlendMode>
  <layerOpacity>1</layerOpacity>
</qgis>
"""
    return qml_content


def _auto_detect_space_syntax_field(layer: Any) -> str:
    fields = layer.fields()
    field_names = [f.name() for f in fields]
    if not field_names:
        return ""

    exact_priority = (
        "NACH_Rn",
        "Int_HH_Rn",
        "v_int_HH",
        "step_depth_metric",
        "step_depth_ang",
        "step_depth_topo",
        "length_m",
    )
    for cand in exact_priority:
        if cand in field_names:
            return cand

    for prefix in ("NACH_", "Int_HH_"):
        for fname in field_names:
            if fname.startswith(prefix):
                return fname

    for f in fields:
        if f.isNumeric() and f.name().lower() not in ("id", "fid", "osm_id", "seg_id", "stroke_id", "cell_id"):
            return f.name()

    return field_names[0]


def apply_space_syntax_style_to_layer(
    layer: Any,
    attr_name: str = "",
    values: Optional[np.ndarray] = None,
    palette_name: str = "classic_rainbow",
    line_width_mm: float = 0.45,
    point_size_mm: float = 2.2,
    ignore_negative: bool = False
) -> bool:
    """
    Applies a graduated Space Syntax QML style directly to a live QgsVectorLayer
    and triggers a canvas repaint.
    """
    if layer is None or not hasattr(layer, "isValid") or not layer.isValid():
        return False

    target_attr = str(attr_name or "").strip()
    if not target_attr or layer.fields().indexFromName(target_attr) < 0:
        target_attr = _auto_detect_space_syntax_field(layer)
    if not target_attr:
        return False

    if values is None:
        extracted = []
        idx = layer.fields().indexFromName(target_attr)
        if idx >= 0:
            for feat in layer.getFeatures():
                val = feat.attribute(idx)
                if val is not None:
                    try:
                        fval = float(val)
                        if np.isfinite(fval):
                            extracted.append(fval)
                    except (ValueError, TypeError):
                        pass
        if extracted:
            values = np.asarray(extracted, dtype=np.float64)

    geom_type = "point" if int(layer.geometryType()) == 0 else "line"
    if target_attr.startswith("step_depth"):
        ignore_negative = True

    qml_xml = generate_qml_style(
        attr_name=target_attr,
        values=values,
        palette_name=palette_name,
        line_width_mm=line_width_mm,
        geom_type=geom_type,
        point_size_mm=point_size_mm,
        ignore_negative=ignore_negative,
    )

    tmp_path = None
    try:
        fd, tmp_path = tempfile.mkstemp(suffix=".qml", prefix="sse_style_")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(qml_xml)
        layer.loadNamedStyle(tmp_path)
        layer.triggerRepaint()
        return True
    finally:
        if tmp_path and os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except OSError:
                pass
