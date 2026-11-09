# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) IO: AutoCAD DXF (AC1018 / R2000+) Handler
High-performance reader and writer for AutoCAD DXF drawing files via ezdxf.
Extracts LINE, LWPOLYLINE, and POLYLINE entities with layer attributes.
"""

import os
from typing import List, Dict, Any, Tuple, Optional
import ezdxf
from shapely.geometry import LineString


def read_dxf_lines(dxf_path: str) -> Tuple[List[LineString], List[Dict[str, Any]]]:
    """
    Reads LINE, LWPOLYLINE, and POLYLINE geometries from a DXF file.
    Returns:
        (shapely_linestrings, list_of_attribute_dicts)
    """
    if not os.path.exists(dxf_path):
        raise FileNotFoundError(f"DXF file not found: {dxf_path}")

    doc = ezdxf.readfile(dxf_path)
    try:
        msp = doc.modelspace()
    except Exception:
        msp = doc.entitydb.values()

    lines: List[LineString] = []
    metadata: List[Dict[str, Any]] = []

    for entity in msp:
        dxftype = entity.dxftype()
        layer = entity.dxf.layer if hasattr(entity.dxf, "layer") else "0"
        color = entity.dxf.color if hasattr(entity.dxf, "color") else 256

        if dxftype == "LINE":
            p1 = (float(entity.dxf.start.x), float(entity.dxf.start.y))
            p2 = (float(entity.dxf.end.x), float(entity.dxf.end.y))
            if p1 != p2:
                ls = LineString([p1, p2])
                if ls.length > 0.01:
                    lines.append(ls)
                    metadata.append({"layer": layer, "color": color, "type": "LINE"})

        elif dxftype == "LWPOLYLINE":
            pts = [(float(p[0]), float(p[1])) for p in entity.get_points(format="xy")]
            if len(pts) >= 2:
                ls = LineString(pts)
                if ls.length > 0.01:
                    lines.append(ls)
                    metadata.append({"layer": layer, "color": color, "type": "LWPOLYLINE"})

        elif dxftype == "POLYLINE":
            pts = [(float(v.dxf.location.x), float(v.dxf.location.y)) for v in entity.vertices]
            if len(pts) >= 2:
                ls = LineString(pts)
                if ls.length > 0.01:
                    lines.append(ls)
                    metadata.append({"layer": layer, "color": color, "type": "POLYLINE"})

    return lines, metadata


def write_dxf_lines(
    lines: List[LineString],
    output_dxf_path: str,
    attributes: Optional[List[Dict[str, Any]]] = None,
    layer_name: str = "OPENDEPTHMAP"
):
    """
    Writes a list of LineStrings to an AutoCAD R2000 DXF file.
    """
    os.makedirs(os.path.dirname(os.path.abspath(output_dxf_path)), exist_ok=True)
    doc = ezdxf.new("R2000")
    msp = doc.modelspace()

    for i, line in enumerate(lines):
        coords = list(line.coords)
        curr_layer = layer_name
        if attributes and i < len(attributes):
            curr_layer = str(attributes[i].get("layer", layer_name))

        if len(coords) == 2:
            msp.add_line(coords[0], coords[1], dxfattribs={"layer": curr_layer})
        elif len(coords) > 2:
            msp.add_lwpolyline(coords, dxfattribs={"layer": curr_layer})

    doc.saveas(output_dxf_path)
