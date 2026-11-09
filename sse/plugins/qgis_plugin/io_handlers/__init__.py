# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) IO Package
Unified import surface for geospatial input/output formats.
"""

from .geopackage import read_geopackage, write_geopackage
from .mif_io import read_mif_lines, write_mif_lines
from .geojson_io import read_geojson, write_geojson

try:
    from .dxf_io import read_dxf_lines, write_dxf_lines
except ImportError:
    read_dxf_lines = None
    write_dxf_lines = None

__all__ = [
    "read_geopackage",
    "write_geopackage",
    "read_dxf_lines",
    "write_dxf_lines",
    "read_mif_lines",
    "write_mif_lines",
    "read_geojson",
    "write_geojson"
]
