# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core: Native OpenStreetMap Overpass Network Downloader
Zero-dependency (stdlib urllib + json + shapely) street network fetcher with:
1. Multi-endpoint failover across official Overpass API mirrors.
2. Three standardized Space Syntax network modes ('all', 'vehicular', 'pedestrian').
3. Automatic noise & private-access filtering and optional polygon clipping.
4. Automatic WGS84 -> Local UTM metric CRS reprojection via ProjectionEngine.
"""

import json
import socket
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict, List, Optional, Set, Tuple

from shapely.geometry import LineString

from .cleaner import DEFAULT_NOISE_HIGHWAYS, filter_noise_features
from .projection import ProjectionEngine, get_utm_epsg_for_lon_lat

OVERPASS_ENDPOINTS: Tuple[str, ...] = (
    "https://overpass.openstreetmap.fr/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass-api.de/api/interpreter",
    "https://lz4.overpass-api.de/api/interpreter",
)

OSM_NETWORK_MODES: Tuple[str, ...] = ("all", "vehicular", "pedestrian")

HIGHWAY_WHITELIST_ALL: Set[str] = {
    "primary",
    "secondary",
    "tertiary",
    "unclassified",
    "residential",
    "living_street",
    "pedestrian",
    "footway",
    "path",
    "steps",
    "cycleway",
    "service",
    "trunk",
    "motorway",
    "motorway_link",
    "trunk_link",
    "primary_link",
    "secondary_link",
    "tertiary_link",
    "track",
    "road",
}

HIGHWAY_WHITELIST_VEHICULAR: Set[str] = {
    "motorway",
    "trunk",
    "primary",
    "secondary",
    "tertiary",
    "unclassified",
    "residential",
    "living_street",
    "service",
    "motorway_link",
    "trunk_link",
    "primary_link",
    "secondary_link",
    "tertiary_link",
    "road",
}

HIGHWAY_WHITELIST_PEDESTRIAN: Set[str] = {
    "pedestrian",
    "footway",
    "path",
    "steps",
    "living_street",
    "cycleway",
    "residential",
    "unclassified",
    "track",
    "service",
}

EXCLUDED_ACCESS_ALL: Set[str] = {"private", "no", "customers", "military"}
EXCLUDED_ACCESS_RESTRICTED: Set[str] = {"private", "no"}


def _normalize_mode(network_mode: str) -> str:
    mode = str(network_mode or "all").strip().lower()
    if mode not in OSM_NETWORK_MODES:
        return "all"
    return mode


def _validate_and_clamp_bbox(bbox_wgs84: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    if not bbox_wgs84 or len(bbox_wgs84) != 4:
        raise ValueError("bbox_wgs84 must be a 4-tuple (min_lon, min_lat, max_lon, max_lat) in WGS84 degrees.")
    min_lon = max(-180.0, min(180.0, float(bbox_wgs84[0])))
    min_lat = max(-90.0, min(90.0, float(bbox_wgs84[1])))
    max_lon = max(-180.0, min(180.0, float(bbox_wgs84[2])))
    max_lat = max(-90.0, min(90.0, float(bbox_wgs84[3])))
    if min_lon >= max_lon or min_lat >= max_lat:
        raise ValueError(
            f"Invalid WGS84 bounding box ({min_lon}, {min_lat}, {max_lon}, {max_lat}): "
            "min_lon < max_lon and min_lat < max_lat required."
        )
    return (min_lon, min_lat, max_lon, max_lat)


def build_overpass_highway_query(
    bbox_wgs84: Tuple[float, float, float, float],
    network_mode: str = "all",
    timeout_s: int = 60
) -> str:
    """
    Builds a compact Overpass QL query string for highway ways within bbox_wgs84.
    Note: Overpass QL bounding box order is (south, west, north, east) = (min_lat, min_lon, max_lat, max_lon).
    """
    _ = _normalize_mode(network_mode)
    min_lon, min_lat, max_lon, max_lat = _validate_and_clamp_bbox(bbox_wgs84)
    timeout_int = max(5, int(timeout_s))
    return (
        f'[out:json][timeout:{timeout_int}];'
        f'(way["highway"]["area"!="yes"]["indoor"!="yes"]'
        f'({min_lat:.7f},{min_lon:.7f},{max_lat:.7f},{max_lon:.7f}););'
        f'out body geom;'
    )


def _passes_mode_filter(tags: Dict[str, Any], mode: str) -> bool:
    if not tags:
        return False
    hw = str(tags.get("highway", "") or "").strip().lower()
    if not hw or hw in DEFAULT_NOISE_HIGHWAYS:
        return False

    area_val = str(tags.get("area", "") or "").strip().lower()
    indoor_val = str(tags.get("indoor", "") or "").strip().lower()
    if area_val == "yes" or indoor_val == "yes":
        return False

    access_val = str(tags.get("access", "") or "").strip().lower()

    if mode == "vehicular":
        if hw not in HIGHWAY_WHITELIST_VEHICULAR:
            return False
        if access_val in EXCLUDED_ACCESS_RESTRICTED:
            return False
        mv_val = str(tags.get("motor_vehicle", "") or "").strip().lower()
        if mv_val == "no":
            return False
        return True

    if mode == "pedestrian":
        if hw not in HIGHWAY_WHITELIST_PEDESTRIAN:
            return False
        if access_val in EXCLUDED_ACCESS_RESTRICTED:
            return False
        foot_val = str(tags.get("foot", "") or "").strip().lower()
        if foot_val == "no":
            return False
        return True

    # Default 'all' mode (Space Syntax Walkable + Drivable Standard)
    if hw not in HIGHWAY_WHITELIST_ALL:
        return False
    if access_val in EXCLUDED_ACCESS_ALL:
        return False
    return True


def _extract_linestrings(geom) -> List[LineString]:
    if geom is None or geom.is_empty:
        return []
    gtype = geom.geom_type
    if gtype == "LineString":
        return [geom] if len(geom.coords) >= 2 and geom.length > 0.0 else []
    if gtype in ("MultiLineString", "GeometryCollection"):
        parts: List[LineString] = []
        for sub in geom.geoms:
            parts.extend(_extract_linestrings(sub))
        return parts
    return []


def parse_overpass_json_to_features(
    data: Dict[str, Any],
    network_mode: str = "all",
    clip_polygon_wgs84: Optional[Any] = None
) -> List[Dict[str, Any]]:
    """
    Parses Overpass API or Official OSM API v0.6 JSON response dictionary
    into filtered LineString features with standardized Space Syntax attributes.
    """
    if not isinstance(data, dict):
        return []

    mode = _normalize_mode(network_mode)
    elements = data.get("elements", [])
    raw_features: List[Dict[str, Any]] = []

    # Build node coordinate lookup for Official OSM API v0.6 map.json format
    node_map: Dict[int, Tuple[float, float]] = {}
    for el in elements:
        if isinstance(el, dict) and el.get("type") == "node":
            if "id" in el and "lon" in el and "lat" in el:
                node_map[int(el["id"])] = (float(el["lon"]), float(el["lat"]))

    seen_way_ids: Set[int] = set()
    for el in elements:
        if not isinstance(el, dict) or el.get("type") != "way":
            continue
        way_id = int(el.get("id", 0))
        if way_id and way_id in seen_way_ids:
            continue
        tags = el.get("tags") or {}
        if not _passes_mode_filter(tags, mode):
            continue

        coords: List[Tuple[float, float]] = []
        geom_pts = el.get("geometry")
        if geom_pts and len(geom_pts) >= 2:
            for pt in geom_pts:
                if isinstance(pt, dict) and "lon" in pt and "lat" in pt:
                    coords.append((float(pt["lon"]), float(pt["lat"])))
        else:
            node_ids = el.get("nodes") or []
            for nid in node_ids:
                pt_xy = node_map.get(int(nid))
                if pt_xy is not None:
                    coords.append(pt_xy)

        if len(coords) < 2:
            continue
        if way_id:
            seen_way_ids.add(way_id)

        line = LineString(coords)
        if line.is_empty or line.length <= 0.0:
            continue

        props = {
            "osm_id": way_id,
            "highway": str(tags.get("highway", "")),
            "name": str(tags.get("name", "")),
            "oneway": str(tags.get("oneway", "")),
            "bridge": str(tags.get("bridge", "")),
            "tunnel": str(tags.get("tunnel", "")),
            "layer": str(tags.get("layer", "0")),
            "surface": str(tags.get("surface", "")),
        }

        if clip_polygon_wgs84 is not None and not clip_polygon_wgs84.is_empty:
            clipped = line.intersection(clip_polygon_wgs84)
            for part in _extract_linestrings(clipped):
                raw_features.append({"geometry": part, "properties": dict(props)})
        else:
            raw_features.append({"geometry": line, "properties": props})

    return filter_noise_features(raw_features, DEFAULT_NOISE_HIGHWAYS)


def _fetch_from_osm_v06_cdn(
    clamped_bbox: Tuple[float, float, float, float],
    timeout_s: int = 12
) -> Optional[Dict[str, Any]]:
    """
    Fastly CDN-backed Official OpenStreetMap API v0.6 JSON fetcher.
    Supports automatic tiling for moderately sized extents and responds in ~1-2s globally.
    """
    min_lon, min_lat, max_lon, max_lat = clamped_bbox
    dx = max_lon - min_lon
    dy = max_lat - min_lat
    if dx <= 0.0 or dy <= 0.0 or (dx * dy) > 0.04:
        return None

    nx = max(1, int(dx / 0.045) + 1)
    ny = max(1, int(dy / 0.045) + 1)
    if nx * ny > 9:
        return None

    all_elements: List[Dict[str, Any]] = []
    step_x = dx / nx
    step_y = dy / ny
    req_timeout = max(4, min(int(timeout_s), 12))

    for ix in range(nx):
        for iy in range(ny):
            x0 = min_lon + ix * step_x
            x1 = min_lon + (ix + 1) * step_x
            y0 = min_lat + iy * step_y
            y1 = min_lat + (iy + 1) * step_y
            url = (
                "https://api.openstreetmap.org/api/0.6/map.json"
                f"?bbox={x0:.7f},{y0:.7f},{x1:.7f},{y1:.7f}"
            )
            req = urllib.request.Request(
                url,
                headers={
                    "User-Agent": "SSE-QGIS-Plugin/1.1",
                    "Accept": "application/json",
                },
            )
            with urllib.request.urlopen(req, timeout=req_timeout) as resp:
                raw_bytes = resp.read()
                tile_data = json.loads(raw_bytes.decode("utf-8"))
                all_elements.extend(tile_data.get("elements", []))

    return {"elements": all_elements}


def fetch_osm_network(
    bbox_wgs84: Tuple[float, float, float, float],
    network_mode: str = "all",
    custom_endpoint: str = "",
    timeout_s: int = 60,
    clip_polygon_wgs84: Optional[Any] = None,
    reproject_to_utm: bool = True
) -> Tuple[List[LineString], List[Dict[str, Any]], int, Dict[str, Any]]:
    """
    Fetches OpenStreetMap street network within bbox_wgs84 from Official OSM Fastly CDN
    or Overpass API mirrors, filters ways by network_mode, optionally clips to
    clip_polygon_wgs84, and reprojects geometries to Local UTM metric CRS.

    Returns:
        (lines, props_list, target_epsg, meta_report)
    """
    mode = _normalize_mode(network_mode)
    clamped_bbox = _validate_and_clamp_bbox(bbox_wgs84)
    query = build_overpass_highway_query(
        clamped_bbox, network_mode=mode, timeout_s=timeout_s
    )

    last_error: Optional[Exception] = None
    payload: Optional[Dict[str, Any]] = None
    endpoint_used = ""

    # Tier 1: If no custom Overpass server is forced, try Fastly CDN Official OSM API v0.6 first
    if not (custom_endpoint and str(custom_endpoint).strip()):
        try:
            cdn_payload = _fetch_from_osm_v06_cdn(clamped_bbox, timeout_s=timeout_s)
            if cdn_payload is not None:
                payload = cdn_payload
                endpoint_used = "https://api.openstreetmap.org/api/0.6/map.json"
        except Exception as exc:
            last_error = exc

    # Tier 2: Overpass API endpoints (or custom endpoint when specified)
    if payload is None:
        endpoints: List[str] = []
        if custom_endpoint and str(custom_endpoint).strip():
            endpoints.append(str(custom_endpoint).strip())
        for ep in OVERPASS_ENDPOINTS:
            if ep not in endpoints:
                endpoints.append(ep)

        headers = {
            "User-Agent": "SSE-QGIS-Plugin/1.1",
            "Content-Type": "application/x-www-form-urlencoded; charset=UTF-8",
        }
        per_ep_timeout = max(5, min(int(timeout_s), 15))

        post_body = urllib.parse.urlencode({"data": query}).encode("utf-8")
        is_real_urlopen = (
            getattr(urllib.request.urlopen, "__module__", "") == "urllib.request"
        )

        for ep in endpoints:
            try:
                if is_real_urlopen:
                    parsed_ep = urllib.parse.urlparse(ep)
                    if parsed_ep.hostname:
                        port = parsed_ep.port or (
                            443 if parsed_ep.scheme == "https" else 80
                        )
                        sock = socket.create_connection(
                            (parsed_ep.hostname, port), timeout=2.5
                        )
                        sock.close()
                req = urllib.request.Request(
                    ep,
                    data=post_body,
                    headers=headers,
                )
                with urllib.request.urlopen(req, timeout=per_ep_timeout) as resp:
                    raw_bytes = resp.read()
                    payload = json.loads(raw_bytes.decode("utf-8"))
                    endpoint_used = ep
                    break
            except Exception as exc:
                last_error = exc
                continue

    if payload is None:
        raise RuntimeError(
            f"All OSM/Overpass endpoints failed for bbox {clamped_bbox}. "
            f"Last error: {last_error}"
        )

    features = parse_overpass_json_to_features(
        payload,
        network_mode=mode,
        clip_polygon_wgs84=clip_polygon_wgs84,
    )

    mid_lon = (clamped_bbox[0] + clamped_bbox[2]) * 0.5
    mid_lat = (clamped_bbox[1] + clamped_bbox[3]) * 0.5
    utm_epsg = get_utm_epsg_for_lon_lat(mid_lon, mid_lat)
    target_epsg = utm_epsg if reproject_to_utm else 4326

    lines: List[LineString] = []
    props_list: List[Dict[str, Any]] = []

    if reproject_to_utm:
        proj_engine = ProjectionEngine("EPSG:4326", utm_epsg)
        for feat in features:
            geom_m = proj_engine.to_metric(feat["geometry"])
            if geom_m is not None and not geom_m.is_empty and len(geom_m.coords) >= 2:
                lines.append(geom_m)
                props_list.append(feat["properties"])
    else:
        for feat in features:
            lines.append(feat["geometry"])
            props_list.append(feat["properties"])

    meta_report = {
        "fetched_ways_count": len(lines),
        "target_epsg": target_epsg,
        "endpoint_used": endpoint_used,
        "network_mode": mode,
        "bbox_wgs84": clamped_bbox,
    }
    return lines, props_list, target_epsg, meta_report
