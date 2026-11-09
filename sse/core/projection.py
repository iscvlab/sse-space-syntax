# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core: Projection & Geodesy Engine
Provides automated CRS detection, UTM zone calculation, Universal 4-Case Metric CRS Guard,
and high-speed batch metric reprojection for both standalone Python (pyproj) and PyQGIS.
"""

import math
from typing import Any, List, Optional, Tuple, Union
import pyproj
from shapely.ops import transform

WEB_MERCATOR_AUTHIDS = {"EPSG:3857", "EPSG:900913", "EPSG:3785", "OSGEO:41001"}


def get_utm_epsg_for_lon_lat(lon: float, lat: float) -> int:
    """
    Computes standard WGS84 UTM Zone EPSG code (32601-32660 for North, 32701-32760 for South).
    Example: Bangkok (lon=100.6, lat=13.7) -> EPSG:32647 (UTM Zone 47N).
    """
    zone = max(1, min(60, int(math.floor((float(lon) + 180.0) / 6.0) + 1)))
    if float(lat) >= 0.0:
        return 32600 + zone
    else:
        return 32700 + zone


def _extract_centroid_from_bounds(sample_bounds: Optional[Tuple[float, ...]]) -> Tuple[float, float]:
    if not sample_bounds or len(sample_bounds) < 4:
        return (100.6, 13.7)
    min_x, min_y, max_x, max_y = (
        float(sample_bounds[0]),
        float(sample_bounds[1]),
        float(sample_bounds[2]),
        float(sample_bounds[3]),
    )
    if not (math.isfinite(min_x) and math.isfinite(min_y) and math.isfinite(max_x) and math.isfinite(max_y)):
        return (100.6, 13.7)
    return ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5)


def resolve_universal_metric_crs_pyproj(
    crs_input: Any,
    sample_bounds: Optional[Tuple[float, float, float, float]] = None,
    fallback_epsg: int = 32647
) -> Tuple[bool, str, str]:
    """
    Universal 4-Case Metric CRS Guard (Pure-Python + pyproj).
    Inspects crs_input and sample_bounds (minx, miny, maxx, maxy) and resolves whether
    reprojection is required so all Space Syntax radii and segment lengths are in true planar meters:

    Case 1: True Planar Metric CRS (UTM, National Grids in meters) -> Retained 100%.
    Case 2: Geographic Degree CRS (EPSG:4326, EPSG:4269, etc.) -> Auto-reprojected to Local UTM.
    Case 3: Web Mercator (EPSG:3857 / Pseudo-Mercator) -> Auto-reprojected to Local UTM
            (eliminating 1/cos(lat) scale distortion).
    Case 4: Non-Metric Projected CRS (Feet / US Survey Feet / Yards) -> Auto-reprojected to Local UTM.

    Returns:
        (needs_reproject: bool, target_crs_str: str, reason: str)
    """
    mid_x, mid_y = _extract_centroid_from_bounds(sample_bounds)

    if crs_input is None or str(crs_input).strip() == "":
        if -180.0 <= mid_x <= 180.0 and -90.0 <= mid_y <= 90.0 and (abs(mid_x) > 0.001 or abs(mid_y) > 0.001):
            utm_epsg = get_utm_epsg_for_lon_lat(mid_x, mid_y)
            return (
                True,
                f"EPSG:{utm_epsg}",
                f"Geographic CRS (EPSG:4326 assumed) -> Auto-reprojected to Local UTM EPSG:{utm_epsg} (meters)",
            )
        return (
            False,
            f"EPSG:{fallback_epsg}",
            "Unreferenced local planar metric coordinates retained 100%",
        )

    crs_str_raw = f"EPSG:{crs_input}" if isinstance(crs_input, int) else str(crs_input).strip()
    crs_upper = crs_str_raw.upper()

    try:
        crs = pyproj.CRS.from_user_input(crs_str_raw)
    except Exception:
        if -180.0 <= mid_x <= 180.0 and -90.0 <= mid_y <= 90.0 and (abs(mid_x) > 0.001 or abs(mid_y) > 0.001):
            utm_epsg = get_utm_epsg_for_lon_lat(mid_x, mid_y)
            return (
                True,
                f"EPSG:{utm_epsg}",
                f"Geographic CRS ({crs_str_raw}) -> Auto-reprojected to Local UTM EPSG:{utm_epsg} (meters)",
            )
        return (False, crs_str_raw, f"Native Planar Metric CRS ({crs_str_raw}) retained 100%")

    auth = crs.to_authority()
    src_str = f"{auth[0]}:{auth[1]}" if auth else crs_str_raw
    src_upper = src_str.upper()

    axis_info = crs.axis_info
    unit_name = axis_info[0].unit_name.lower() if axis_info else "metre"

    # Helper to compute Local UTM EPSG from centroid in `crs`
    def _centroid_to_utm() -> int:
        try:
            if src_upper == "EPSG:4326":
                lon, lat = mid_x, mid_y
            else:
                xform = pyproj.Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
                lon, lat = xform.transform(mid_x, mid_y)
            if not (math.isfinite(lon) and math.isfinite(lat)):
                return int(fallback_epsg)
            return get_utm_epsg_for_lon_lat(lon, lat)
        except Exception:
            return int(fallback_epsg)

    # Case 2: Geographic Degree CRS
    if crs.is_geographic or "degree" in unit_name:
        utm_epsg = _centroid_to_utm()
        return (
            True,
            f"EPSG:{utm_epsg}",
            f"Geographic CRS ({src_str}) -> Auto-reprojected to Local UTM EPSG:{utm_epsg} (meters)",
        )

    # Case 3: Web Mercator (EPSG:3857 / Pseudo-Mercator)
    crs_name_lower = (crs.name or "").lower()
    is_web_mercator = (
        src_upper in WEB_MERCATOR_AUTHIDS
        or crs_upper in WEB_MERCATOR_AUTHIDS
        or "pseudo-mercator" in crs_name_lower
        or "web mercator" in crs_name_lower
    )
    if not is_web_mercator and crs.coordinate_operation is not None:
        op_name = (crs.coordinate_operation.method_name or "").lower()
        if "pseudo mercator" in op_name or "pseudo-mercator" in op_name or "web mercator" in op_name:
            is_web_mercator = True

    if is_web_mercator:
        utm_epsg = _centroid_to_utm()
        return (
            True,
            f"EPSG:{utm_epsg}",
            f"Web Mercator ({src_str}) -> Auto-reprojected to Local UTM "
            f"EPSG:{utm_epsg} (eliminating 1/cos(lat) distortion)",
        )

    # Case 1: True Planar Metric CRS
    if unit_name in ("metre", "meter", "m", "meters", "metres"):
        return (
            False,
            src_str,
            f"Native Planar Metric CRS ({src_str}) retained 100%",
        )

    # Case 4: Non-Metric Projected CRS (Feet / US Survey Feet / Yards / Links)
    utm_epsg = _centroid_to_utm()
    return (
        True,
        f"EPSG:{utm_epsg}",
        f"Non-Metric Projected CRS ({src_str}, unit={unit_name}) -> "
        f"Auto-reprojected to Local UTM EPSG:{utm_epsg} (meters)",
    )


def ensure_qgis_metric_crs(
    qgs_crs: Any,
    extent_rect: Any = None,
    project_instance: Any = None,
    feedback: Any = None
) -> Tuple[Any, Any, str]:
    """
    Universal 4-Case Metric CRS Guard for PyQGIS Vector Layers & Processing Algorithms.
    Safely inspects a QgsCoordinateReferenceSystem and QgsRectangle extent and returns
    (target_qgs_crs, qgs_coordinate_transform_or_None, reason_str).

    - Case 1 (Planar Metric CRS, e.g. UTM, BNG, national meter grids): retained untouched (xform=None).
    - Case 2 (Geographic Degrees, e.g. EPSG:4326): auto-reprojected to Local UTM EPSG:{utm_epsg}.
    - Case 3 (Web Mercator EPSG:3857): auto-reprojected to Local UTM EPSG:{utm_epsg}.
    - Case 4 (Non-Meter Projected CRS in Feet/Yards): auto-reprojected to Local UTM EPSG:{utm_epsg}.
    """
    from qgis.core import (
        QgsCoordinateReferenceSystem,
        QgsCoordinateTransform,
        QgsPointXY,
        QgsProject,
        QgsUnitTypes,
    )

    mid_x, mid_y = 100.6, 13.7
    if extent_rect is not None:
        try:
            if not extent_rect.isEmpty():
                ex_x = (extent_rect.xMinimum() + extent_rect.xMaximum()) * 0.5
                ex_y = (extent_rect.yMinimum() + extent_rect.yMaximum()) * 0.5
                if math.isfinite(ex_x) and math.isfinite(ex_y):
                    mid_x, mid_y = ex_x, ex_y
        except (AttributeError, TypeError, ValueError):
            pass

    proj_inst = project_instance if project_instance is not None else QgsProject.instance()
    proj_ctx = proj_inst.transformContext() if proj_inst is not None else QgsCoordinateTransform()

    if qgs_crs is None or not qgs_crs.isValid():
        if -180.0 <= mid_x <= 180.0 and -90.0 <= mid_y <= 90.0 and (abs(mid_x) > 0.001 or abs(mid_y) > 0.001):
            utm_epsg = get_utm_epsg_for_lon_lat(mid_x, mid_y)
            wgs84_crs = QgsCoordinateReferenceSystem("EPSG:4326")
            target_crs = QgsCoordinateReferenceSystem(f"EPSG:{utm_epsg}")
            xform = QgsCoordinateTransform(wgs84_crs, target_crs, proj_ctx)
            reason = (
                "Unreferenced degree coordinates -> Auto-reprojected from "
                f"EPSG:4326 to Local UTM EPSG:{utm_epsg} (meters)"
            )
            if feedback is not None and hasattr(feedback, "setProgressText"):
                feedback.setProgressText(f"[Universal CRS Guard] {reason}")
            return (target_crs, xform, reason)
        reason = "Unreferenced local planar metric coordinates retained"
        return (qgs_crs, None, reason)

    authid = (qgs_crs.authid() or "").upper()
    is_geog = qgs_crs.isGeographic()
    map_units = qgs_crs.mapUnits()
    proj_str = ""
    try:
        proj_str = (qgs_crs.toProj() or "").lower()
    except Exception:
        proj_str = ""

    is_web_merc = (
        authid in WEB_MERCATOR_AUTHIDS
        or "+proj=merc" in proj_str
        or "+proj=webmerc" in proj_str
        or "pseudo-mercator" in (qgs_crs.description() or "").lower()
    )

    # Case 1: Native Planar Metric CRS
    if (
        not is_geog
        and map_units == QgsUnitTypes.DistanceUnit.DistanceMeters
        and not is_web_merc
    ):
        reason = f"Native Planar Metric CRS ({authid or qgs_crs.description()}) retained"
        return (qgs_crs, None, reason)

    # Cases 2, 3, 4: Transform centroid to WGS84 (EPSG:4326) to determine Local UTM Zone
    wgs84_crs = QgsCoordinateReferenceSystem("EPSG:4326")
    lon, lat = mid_x, mid_y
    if authid != "EPSG:4326":
        try:
            to_wgs84 = QgsCoordinateTransform(qgs_crs, wgs84_crs, proj_ctx)
            pt_wgs = to_wgs84.transform(QgsPointXY(mid_x, mid_y))
            if math.isfinite(pt_wgs.x()) and math.isfinite(pt_wgs.y()):
                lon, lat = pt_wgs.x(), pt_wgs.y()
        except Exception:
            lon, lat = 100.6, 13.7

    utm_epsg = get_utm_epsg_for_lon_lat(lon, lat)
    target_crs = QgsCoordinateReferenceSystem(f"EPSG:{utm_epsg}")
    xform = QgsCoordinateTransform(qgs_crs, target_crs, proj_ctx)

    if is_geog:
        reason = f"Geographic CRS ({authid or 'Degrees'}) -> Auto-reprojected to Local UTM EPSG:{utm_epsg} (meters)"
    elif is_web_merc:
        reason = (
            f"Web Mercator ({authid or 'EPSG:3857'}) -> Auto-reprojected to Local UTM EPSG:{utm_epsg} "
            "(eliminating 1/cos(lat) distortion)"
        )
    else:
        try:
            unit_label = QgsUnitTypes.toString(map_units)
        except Exception:
            unit_label = str(map_units)
        reason = (
            f"Non-Metric Projected CRS ({authid or qgs_crs.description()}, unit={unit_label}) "
            f"-> Auto-reprojected to Local UTM EPSG:{utm_epsg} (meters)"
        )

    if feedback is not None and hasattr(feedback, "setProgressText"):
        feedback.setProgressText(f"[Universal CRS Guard] {reason}")

    return (target_crs, xform, reason)


def transform_qgs_geometry(geom: Any, xform: Any) -> Any:
    """
    In-place safe coordinate transformation helper for QgsGeometry when xform is active.
    """
    if xform is not None and geom is not None and not geom.isEmpty():
        geom.transform(xform)
    return geom


class ProjectionEngine:
    def __init__(self, src_crs: Union[str, int] = "EPSG:4326", target_crs: Union[str, int] = None):
        self.src_crs_str = f"EPSG:{src_crs}" if isinstance(src_crs, int) else str(src_crs)
        self.target_crs_str = (
            f"EPSG:{target_crs}" if isinstance(target_crs, int)
            else (str(target_crs) if target_crs else None)
        )
        self.transformer_forward = None
        self.transformer_backward = None

        if self.target_crs_str:
            self._init_transformers(self.src_crs_str, self.target_crs_str)

    def _init_transformers(self, src: str, dst: str):
        self.src_crs_str = src
        self.target_crs_str = dst
        self.transformer_forward = pyproj.Transformer.from_crs(src, dst, always_xy=True)
        self.transformer_backward = pyproj.Transformer.from_crs(dst, src, always_xy=True)

    def auto_configure_from_geometries(self, geoms: List[Any], fallback_epsg: int = 32647):
        """
        If target CRS is not specified, probes geometry coordinates and source CRS
        using the Universal 4-Case Metric CRS Guard.
        """
        if not geoms:
            self._init_transformers("EPSG:4326", f"EPSG:{fallback_epsg}")
            return

        sample = geoms[0]
        bounds = sample.bounds  # minx, miny, maxx, maxy
        mid_x = (bounds[0] + bounds[2]) * 0.5
        mid_y = (bounds[1] + bounds[3]) * 0.5

        if self.src_crs_str and self.src_crs_str.upper() != "EPSG:4326":
            needs_reproj, target_crs_resolved, _ = resolve_universal_metric_crs_pyproj(
                self.src_crs_str, bounds, fallback_epsg=fallback_epsg
            )
            if needs_reproj:
                self._init_transformers(self.src_crs_str, target_crs_resolved)
            else:
                self._init_transformers(target_crs_resolved, target_crs_resolved)
            return

        # Check if coordinates look like longitude / latitude
        if -180.0 <= mid_x <= 180.0 and -90.0 <= mid_y <= 90.0:
            utm_epsg = get_utm_epsg_for_lon_lat(mid_x, mid_y)
            self._init_transformers("EPSG:4326", f"EPSG:{utm_epsg}")
        else:
            # Already projected metric coordinates
            target = self.target_crs_str or f"EPSG:{fallback_epsg}"
            self._init_transformers(target, target)

    def to_metric(self, geom: Any) -> Any:
        """Reprojects geometry from source CRS to metric target CRS."""
        if not self.transformer_forward or self.src_crs_str == self.target_crs_str:
            return geom
        return transform(self.transformer_forward.transform, geom)

    def to_wgs84(self, geom: Any) -> Any:
        """Reprojects geometry from metric target CRS back to WGS84."""
        if not self.transformer_backward:
            return geom
        if self.target_crs_str == "EPSG:4326":
            return geom
        return transform(self.transformer_backward.transform, geom)
