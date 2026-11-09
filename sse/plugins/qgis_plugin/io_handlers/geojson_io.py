# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) IO: GeoJSON Handler
Reads and writes GeoJSON FeatureCollections for web interchange and local visualization.
"""

import os
import geopandas as gpd


def read_geojson(geojson_path: str) -> gpd.GeoDataFrame:
    """Reads a GeoJSON file into a GeoDataFrame."""
    if not os.path.exists(geojson_path):
        raise FileNotFoundError(f"GeoJSON file not found: {geojson_path}")
    return gpd.read_file(geojson_path, driver="GeoJSON")


def write_geojson(
    gdf: gpd.GeoDataFrame,
    output_path: str
):
    """Writes a GeoDataFrame to a GeoJSON file."""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    gdf.to_file(output_path, driver="GeoJSON")
