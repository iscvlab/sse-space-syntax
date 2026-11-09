"""
GeoJSON Reader and Atomic Writer
Reads and writes standard GeoJSON FeatureCollections.
Enforces atomic writes and UTF-8 encoding for cloud sync safety.
"""

import os
import json
from typing import List, Dict, Any
from shapely.geometry import shape


def read_geojson(filepath: str) -> List[Dict[str, Any]]:
    """
    Reads a GeoJSON file and returns a list of features with Shapely geometries.
    Each feature dict contains:
        {"geometry": LineString, "properties": Dict[str, Any]}
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"GeoJSON file not found: {filepath}")

    with open(filepath, "r", encoding="utf-8") as f:
        data = json.load(f)

    features = []
    raw_feats = data.get("features", [])
    for rf in raw_feats:
        geom_json = rf.get("geometry")
        if not geom_json:
            continue
        g = shape(geom_json)
        if g.is_empty:
            continue
        features.append({
            "geometry": g,
            "properties": rf.get("properties", {})
        })

    return features


def write_geojson(
    features: List[Dict[str, Any]],
    out_path: str,
    crs_name: str = "urn:ogc:def:crs:OGC:1.3:CRS84",
    layer_name: str = "space_syntax_layer",
    coord_precision: int = 6
) -> None:
    """
    Writes features with Shapely geometries to a GeoJSON file atomically.
    """
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    feat_list = []
    for f in features:
        geom = f["geometry"]
        coords = list(geom.coords)
        rounded_coords = [[round(c[0], coord_precision), round(c[1], coord_precision)] for c in coords]

        feat_list.append({
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": rounded_coords
            },
            "properties": f.get("properties", {})
        })

    collection = {
        "type": "FeatureCollection",
        "name": layer_name,
        "crs": {
            "type": "name",
            "properties": {"name": crs_name}
        },
        "features": feat_list
    }

    temp_path = out_path + ".tmp"
    with open(temp_path, "w", encoding="utf-8") as f:
        json.dump(collection, f, ensure_ascii=False)

    os.replace(temp_path, out_path)
