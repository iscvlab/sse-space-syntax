# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) IO: High-Speed GeoPackage (.gpkg) Handler
Reads and writes OGC GeoPackage files via pyogrio/geopandas.
Supports embedding QGIS .qml style tables directly into the GeoPackage SQLite database.
"""

import os
import sqlite3
from typing import Optional
import geopandas as gpd


def read_geopackage(gpkg_path: str, layer_name: str = None) -> gpd.GeoDataFrame:
    """Reads a GeoPackage layer into a GeoDataFrame."""
    if not os.path.exists(gpkg_path):
        raise FileNotFoundError(f"GeoPackage not found: {gpkg_path}")
    return gpd.read_file(gpkg_path, layer=layer_name, engine="pyogrio")


def write_geopackage(
    gdf: gpd.GeoDataFrame,
    gpkg_path: str,
    layer_name: str = "space_syntax_results",
    style_qml_path: Optional[str] = None
):
    """
    Writes a GeoDataFrame to a GeoPackage file.
    If style_qml_path is provided, embeds the QGIS style into the layer_styles table.
    """
    os.makedirs(os.path.dirname(os.path.abspath(gpkg_path)), exist_ok=True)
    gdf.to_file(gpkg_path, layer=layer_name, driver="GPKG", engine="pyogrio")

    # Embed QGIS style into GeoPackage layer_styles table if provided
    if style_qml_path and os.path.exists(style_qml_path):
        try:
            with open(style_qml_path, "r", encoding="utf-8") as f:
                style_qml_content = f.read()

            conn = sqlite3.connect(gpkg_path)
            cursor = conn.cursor()

            # Ensure layer_styles table exists
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS layer_styles (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    f_table_catalog TEXT,
                    f_table_schema TEXT,
                    f_table_name TEXT,
                    f_geometry_column TEXT,
                    styleName TEXT,
                    styleQML TEXT,
                    styleSLD TEXT,
                    useAsDefault BOOLEAN,
                    description TEXT,
                    owner TEXT,
                    ui TEXT,
                    update_time DATETIME DEFAULT CURRENT_TIMESTAMP
                );
            """)

            cursor.execute("""
                INSERT OR REPLACE INTO layer_styles
                (f_table_name, f_geometry_column, styleName, styleQML, useAsDefault, description)
                VALUES (?, ?, ?, ?, 1, 'OpenDepthmap Default Swiss Style');
            """, (layer_name, "geom", "OpenDepthmap", style_qml_content))

            conn.commit()
            conn.close()
        except Exception as e:
            print(f"[Warning] Could not embed QML style into GeoPackage: {e}")
