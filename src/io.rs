//! Spatial I/O, WGS84 → UTM Metric Projection, and Multi-Mirror OSM Overpass Downloader.
//!
//! Supports GeoJSON, CSV (WKT / endpoint columns), AutoCAD DXF (`AC1018`), MapInfo `MIF/MID`,
//! OpenStreetMap `.osm` XML, and live Overpass API JSON fetching via `ureq`.

use crate::types::{ColorPalette, Point2D, Polyline, SegmentRecord, SummaryStats, VgaResults, depthmapx_color_rgb};
use serde_json::{Value, json};
use std::collections::{BTreeMap, HashMap};
use std::fs;
use std::io::Write;
use std::path::Path;
use std::time::Duration;

/// Projects a single WGS84 `(lon_deg, lat_deg)` coordinate to UTM `(easting_m, northing_m)`
/// in the specified UTM `zone` (`1..=60`) and hemisphere (`north`).
#[inline]
pub fn wgs84_lon_lat_to_utm(lon_deg: f64, lat_deg: f64, zone: u32, north: bool) -> Point2D {
    const A: f64 = 6_378_137.0;
    const F: f64 = 1.0 / 298.257_223_563;
    const K0: f64 = 0.9996;
    let e2 = F * (2.0 - F);
    let ep2 = e2 / (1.0 - e2);

    let lat = lat_deg.clamp(-84.5, 84.5).to_radians();
    let lon = lon_deg.to_radians();
    let lon0 = (((zone as f64) - 1.0) * 6.0 - 180.0 + 3.0).to_radians();

    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let tan_lat = lat.tan();

    let n = A / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let t = tan_lat * tan_lat;
    let c = ep2 * cos_lat * cos_lat;
    let a_term = cos_lat * (lon - lon0);

    let e4 = e2 * e2;
    let e6 = e4 * e2;
    let m = A
        * ((1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * lat
            - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * (2.0 * lat).sin()
            + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * (4.0 * lat).sin()
            - (35.0 * e6 / 3072.0) * (6.0 * lat).sin());

    let a2 = a_term * a_term;
    let a3 = a2 * a_term;
    let a4 = a2 * a2;
    let a5 = a4 * a_term;
    let a6 = a4 * a2;

    let easting = 500_000.0
        + K0 * n
            * (a_term
                + (1.0 - t + c) * a3 / 6.0
                + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2) * a5 / 120.0);

    let mut northing = K0
        * (m + n
            * tan_lat
            * (a2 / 2.0
                + (5.0 - t + 9.0 * c + 4.0 * c * c) * a4 / 24.0
                + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * a6 / 720.0));

    if !north {
        northing += 10_000_000.0;
    }

    Point2D::new(easting, northing)
}

/// Detects if `lines` are in WGS84 geographic coordinates (`|x| <= 180.0 && |y| <= 85.0` with
/// geographic degree span) and projects them in-place to metric UTM (`EPSG:326xx` / `327xx`).
pub fn project_wgs84_to_utm_if_needed(lines: &mut [Polyline]) -> Option<u32> {
    if lines.is_empty() {
        return None;
    }
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut count = 0usize;

    for line in lines.iter() {
        for p in &line.coords {
            if !p.x.is_finite() || !p.y.is_finite() {
                return None;
            }
            if p.x.abs() > 180.0 || p.y.abs() > 85.0 {
                return None;
            }
            min_x = min_x.min(p.x);
            min_y = min_y.min(p.y);
            max_x = max_x.max(p.x);
            max_y = max_y.max(p.y);
            count += 1;
        }
    }

    if count == 0 {
        return None;
    }

    // Distinguish geographic lon/lat degrees (span <= 10 deg) from synthetic local metric grids (e.g. 0..100m)
    let span_x = max_x - min_x;
    let span_y = max_y - min_y;
    if span_x > 10.0 || span_y > 10.0 {
        return None;
    }
    // If coordinates start near (0,0) and span > 5.0m, treat as local planar coordinates
    if min_x >= -1.0 && min_y >= -1.0 && (span_x > 4.0 || span_y > 4.0) && max_x <= 180.0 {
        return None;
    }

    let mean_lon = 0.5 * (min_x + max_x);
    let mean_lat = 0.5 * (min_y + max_y);
    let zone = (((mean_lon + 180.0) / 6.0).floor() as i32 + 1).clamp(1, 60) as u32;
    let north = mean_lat >= 0.0;
    let epsg = if north { 32600 + zone } else { 32700 + zone };

    for line in lines.iter_mut() {
        for p in &mut line.coords {
            *p = wgs84_lon_lat_to_utm(p.x, p.y, zone, north);
        }
    }

    Some(epsg)
}

fn extract_props_meta(props: Option<&Value>) -> (f64, String, String) {
    let Some(obj) = props.and_then(|v| v.as_object()) else {
        return (0.0, String::new(), String::new());
    };

    let highway = obj
        .get("highway")
        .and_then(|v| v.as_str())
        .unwrap_or("")
        .to_string();
    let name = obj
        .get("name")
        .and_then(|v| v.as_str())
        .unwrap_or("")
        .to_string();

    let mut level = 0.0;
    if let Some(lv) = obj.get("layer").or_else(|| obj.get("level")) {
        if let Some(f) = lv.as_f64() {
            level = f;
        } else if let Some(s) = lv.as_str() {
            if let Ok(f) = s.trim().parse::<f64>() {
                level = f;
            }
        }
    }
    if level == 0.0 {
        if let Some(br) = obj.get("bridge").and_then(|v| v.as_str()) {
            let b = br.trim().to_ascii_lowercase();
            if !b.is_empty() && b != "no" && b != "false" && b != "0" {
                level = 1.0;
            }
        }
        if let Some(tu) = obj.get("tunnel").and_then(|v| v.as_str()) {
            let t = tu.trim().to_ascii_lowercase();
            if !t.is_empty() && t != "no" && t != "false" && t != "0" {
                level = -1.0;
            }
        }
    }

    (level, highway, name)
}

fn parse_coord_pair(v: &Value) -> Option<Point2D> {
    let arr = v.as_array()?;
    if arr.len() < 2 {
        return None;
    }
    let x = arr[0].as_f64()?;
    let y = arr[1].as_f64()?;
    if x.is_finite() && y.is_finite() {
        Some(Point2D::new(x, y))
    } else {
        None
    }
}

fn collect_geojson_geometry(
    geom: &Value,
    level: f64,
    highway: &str,
    name: &str,
    out: &mut Vec<Polyline>,
) {
    let Some(gtype) = geom.get("type").and_then(|v| v.as_str()) else {
        return;
    };
    match gtype {
        "LineString" => {
            if let Some(coords_arr) = geom.get("coordinates").and_then(|v| v.as_array()) {
                let pts: Vec<Point2D> = coords_arr.iter().filter_map(parse_coord_pair).collect();
                if pts.len() >= 2 {
                    out.push(Polyline::new(pts).with_meta(level, highway, name));
                }
            }
        }
        "MultiLineString" | "Polygon" => {
            if let Some(rings) = geom.get("coordinates").and_then(|v| v.as_array()) {
                for ring in rings {
                    if let Some(coords_arr) = ring.as_array() {
                        let pts: Vec<Point2D> =
                            coords_arr.iter().filter_map(parse_coord_pair).collect();
                        if pts.len() >= 2 {
                            out.push(Polyline::new(pts).with_meta(level, highway, name));
                        }
                    }
                }
            }
        }
        "MultiPolygon" => {
            if let Some(polys) = geom.get("coordinates").and_then(|v| v.as_array()) {
                for poly in polys {
                    if let Some(rings) = poly.as_array() {
                        for ring in rings {
                            if let Some(coords_arr) = ring.as_array() {
                                let pts: Vec<Point2D> =
                                    coords_arr.iter().filter_map(parse_coord_pair).collect();
                                if pts.len() >= 2 {
                                    out.push(Polyline::new(pts).with_meta(level, highway, name));
                                }
                            }
                        }
                    }
                }
            }
        }
        "GeometryCollection" => {
            if let Some(geoms) = geom.get("geometries").and_then(|v| v.as_array()) {
                for sub in geoms {
                    collect_geojson_geometry(sub, level, highway, name, out);
                }
            }
        }
        _ => {}
    }
}

/// Parses GeoJSON (`FeatureCollection`, `Feature`, or raw `Geometry`) into metric `Polyline`s,
/// automatically projecting WGS84 `(lon, lat)` to UTM if needed.
pub fn load_geojson_str(content: &str) -> Result<Vec<Polyline>, String> {
    let root: Value =
        serde_json::from_str(content).map_err(|e| format!("Invalid GeoJSON syntax: {e}"))?;
    let mut lines = Vec::new();

    match root.get("type").and_then(|v| v.as_str()) {
        Some("FeatureCollection") => {
            if let Some(features) = root.get("features").and_then(|v| v.as_array()) {
                for feat in features {
                    let (level, hw, name) = extract_props_meta(feat.get("properties"));
                    if let Some(geom) = feat.get("geometry") {
                        collect_geojson_geometry(geom, level, &hw, &name, &mut lines);
                    }
                }
            }
        }
        Some("Feature") => {
            let (level, hw, name) = extract_props_meta(root.get("properties"));
            if let Some(geom) = root.get("geometry") {
                collect_geojson_geometry(geom, level, &hw, &name, &mut lines);
            }
        }
        Some(_) => {
            collect_geojson_geometry(&root, 0.0, "", "", &mut lines);
        }
        None => return Err("GeoJSON root object missing 'type' field".to_string()),
    }

    let _ = project_wgs84_to_utm_if_needed(&mut lines);
    Ok(lines)
}

/// Parses AutoCAD DXF (`LINE`, `LWPOLYLINE`, `POLYLINE`/`VERTEX`) into `Polyline`s.
///
/// If the DXF contains a `SECTION` named `ENTITIES`, only entities inside the `ENTITIES`
/// section are parsed (ignoring symbol definitions at `(0, 0)` inside the `BLOCKS` section).
pub fn load_dxf_str(content: &str) -> Result<Vec<Polyline>, String> {
    let raw_lines: Vec<&str> = content.lines().collect();
    let mut pairs: Vec<(i32, &str)> = Vec::with_capacity(raw_lines.len() / 2);
    let mut idx = 0usize;
    while idx + 1 < raw_lines.len() {
        let code_str = raw_lines[idx].trim();
        let val_str = raw_lines[idx + 1].trim();
        if let Ok(code) = code_str.parse::<i32>() {
            pairs.push((code, val_str));
            idx += 2;
        } else {
            idx += 1;
        }
    }

    let n = pairs.len();
    let mut has_entities_section = false;
    for k in 0..n {
        if pairs[k].0 == 0 && pairs[k].1.eq_ignore_ascii_case("SECTION") {
            let mut m = k + 1;
            while m < n && pairs[m].0 != 0 {
                if pairs[m].0 == 2 && pairs[m].1.eq_ignore_ascii_case("ENTITIES") {
                    has_entities_section = true;
                    break;
                }
                m += 1;
            }
            if has_entities_section {
                break;
            }
        }
    }

    let mut out = Vec::new();
    let mut i = 0usize;
    let mut in_entities = !has_entities_section;

    while i < n {
        if pairs[i].0 == 0 {
            let entity = pairs[i].1.to_ascii_uppercase();
            if entity == "SECTION" {
                i += 1;
                let mut is_ent = false;
                while i < n && pairs[i].0 != 0 {
                    if pairs[i].0 == 2 && pairs[i].1.eq_ignore_ascii_case("ENTITIES") {
                        is_ent = true;
                    }
                    i += 1;
                }
                if has_entities_section {
                    in_entities = is_ent;
                }
                continue;
            } else if entity == "ENDSEC" {
                if has_entities_section {
                    in_entities = false;
                }
                i += 1;
                continue;
            } else if !in_entities {
                i += 1;
                continue;
            }
            if entity == "LINE" {
                i += 1;
                let mut x1 = None;
                let mut y1 = None;
                let mut x2 = None;
                let mut y2 = None;
                let mut layer = String::new();
                while i < n && pairs[i].0 != 0 {
                    match pairs[i].0 {
                        8 => layer = pairs[i].1.to_string(),
                        10 => x1 = pairs[i].1.parse::<f64>().ok(),
                        20 => y1 = pairs[i].1.parse::<f64>().ok(),
                        11 => x2 = pairs[i].1.parse::<f64>().ok(),
                        21 => y2 = pairs[i].1.parse::<f64>().ok(),
                        _ => {}
                    }
                    i += 1;
                }
                if let (Some(ax), Some(ay), Some(bx), Some(by)) = (x1, y1, x2, y2) {
                    let p0 = Point2D::new(ax, ay);
                    let p1 = Point2D::new(bx, by);
                    if p0.dist(p1) > 1e-9 {
                        out.push(Polyline::new(vec![p0, p1]).with_meta(0.0, layer, ""));
                    }
                }
                continue;
            } else if entity == "LWPOLYLINE" {
                i += 1;
                let mut pts = Vec::new();
                let mut cur_x: Option<f64> = None;
                let mut closed = false;
                let mut layer = String::new();
                while i < n && pairs[i].0 != 0 {
                    match pairs[i].0 {
                        8 => layer = pairs[i].1.to_string(),
                        70 => {
                            if let Ok(flags) = pairs[i].1.parse::<i32>() {
                                closed = (flags & 1) != 0;
                            }
                        }
                        10 => {
                            cur_x = pairs[i].1.parse::<f64>().ok();
                        }
                        20 => {
                            if let (Some(x), Ok(y)) = (cur_x.take(), pairs[i].1.parse::<f64>()) {
                                pts.push(Point2D::new(x, y));
                            }
                        }
                        _ => {}
                    }
                    i += 1;
                }
                if closed && pts.len() >= 3 {
                    let first = pts[0];
                    pts.push(first);
                }
                if pts.len() >= 2 {
                    out.push(Polyline::new(pts).with_meta(0.0, layer, ""));
                }
                continue;
            } else if entity == "POLYLINE" {
                i += 1;
                let mut closed = false;
                let mut layer = String::new();
                while i < n && pairs[i].0 != 0 {
                    match pairs[i].0 {
                        8 => layer = pairs[i].1.to_string(),
                        70 => {
                            if let Ok(flags) = pairs[i].1.parse::<i32>() {
                                closed = (flags & 1) != 0;
                            }
                        }
                        _ => {}
                    }
                    i += 1;
                }
                let mut pts = Vec::new();
                while i < n {
                    if pairs[i].0 == 0 {
                        let sub = pairs[i].1.to_ascii_uppercase();
                        if sub == "VERTEX" {
                            i += 1;
                            let mut vx = None;
                            let mut vy = None;
                            while i < n && pairs[i].0 != 0 {
                                match pairs[i].0 {
                                    10 => vx = pairs[i].1.parse::<f64>().ok(),
                                    20 => vy = pairs[i].1.parse::<f64>().ok(),
                                    _ => {}
                                }
                                i += 1;
                            }
                            if let (Some(x), Some(y)) = (vx, vy) {
                                pts.push(Point2D::new(x, y));
                            }
                            continue;
                        } else if sub == "SEQEND" {
                            i += 1;
                            while i < n && pairs[i].0 != 0 {
                                i += 1;
                            }
                            break;
                        } else {
                            break;
                        }
                    } else {
                        i += 1;
                    }
                }
                if closed && pts.len() >= 3 {
                    let first = pts[0];
                    pts.push(first);
                }
                if pts.len() >= 2 {
                    out.push(Polyline::new(pts).with_meta(0.0, layer, ""));
                }
                continue;
            }
        }
        i += 1;
    }

    Ok(out)
}

/// Parses MapInfo Interchange Format (`.mif`) `Line` and `Pline` geometries into `Polyline`s.
pub fn load_mif_str(content: &str) -> Result<Vec<Polyline>, String> {
    let mut out = Vec::new();
    let lines: Vec<&str> = content.lines().map(|l| l.trim()).collect();
    let mut i = 0usize;
    let n = lines.len();

    while i < n {
        let row = lines[i];
        if row.is_empty() {
            i += 1;
            continue;
        }
        let tokens: Vec<&str> = row.split_whitespace().collect();
        if tokens.is_empty() {
            i += 1;
            continue;
        }
        let kw = tokens[0].to_ascii_lowercase();
        if kw == "line" && tokens.len() >= 5 {
            if let (Ok(x1), Ok(y1), Ok(x2), Ok(y2)) = (
                tokens[1].parse::<f64>(),
                tokens[2].parse::<f64>(),
                tokens[3].parse::<f64>(),
                tokens[4].parse::<f64>(),
            ) {
                let p0 = Point2D::new(x1, y1);
                let p1 = Point2D::new(x2, y2);
                if p0.dist(p1) > 1e-9 {
                    out.push(Polyline::new(vec![p0, p1]));
                }
            }
            i += 1;
        } else if kw == "pline" {
            let mut count_opt: Option<usize> = None;
            if tokens.len() >= 2 {
                if let Ok(c) = tokens[1].parse::<usize>() {
                    count_opt = Some(c);
                } else if tokens[1].eq_ignore_ascii_case("multiple") && tokens.len() >= 3 {
                    count_opt = tokens[2].parse::<usize>().ok();
                }
            }
            i += 1;
            let mut pts = Vec::new();
            while i < n {
                let sub = lines[i];
                let stoks: Vec<&str> = sub.split_whitespace().collect();
                if stoks.len() == 2 {
                    if let (Ok(x), Ok(y)) = (stoks[0].parse::<f64>(), stoks[1].parse::<f64>()) {
                        pts.push(Point2D::new(x, y));
                        i += 1;
                        if let Some(expected) = count_opt {
                            if pts.len() >= expected && expected >= 2 {
                                break;
                            }
                        }
                        continue;
                    }
                }
                break;
            }
            if pts.len() >= 2 {
                out.push(Polyline::new(pts));
            }
        } else {
            i += 1;
        }
    }

    let _ = project_wgs84_to_utm_if_needed(&mut out);
    Ok(out)
}

fn extract_xml_attr<'a>(tag_line: &'a str, key: &str) -> Option<&'a str> {
    for quote in ['"', '\''] {
        let pattern = format!("{key}={quote}");
        if let Some(start_idx) = tag_line.find(&pattern) {
            let val_start = start_idx + pattern.len();
            let rest = &tag_line[val_start..];
            if let Some(end_idx) = rest.find(quote) {
                return Some(&rest[..end_idx]);
            }
        }
    }
    None
}

/// Parses OpenStreetMap XML (`.osm`) `<node>` and `<way>` highway elements and projects to metric UTM.
pub fn load_osm_xml_str(content: &str) -> Result<Vec<Polyline>, String> {
    let mut nodes: HashMap<i64, Point2D> = HashMap::new();
    let mut ways: Vec<Polyline> = Vec::new();

    let mut in_way = false;
    let mut way_nds: Vec<i64> = Vec::new();
    let mut way_highway = String::new();
    let mut way_name = String::new();
    let mut way_level = 0.0_f64;
    let mut way_area = false;

    for raw_line in content.lines() {
        let line = raw_line.trim();
        if line.starts_with("<node ") {
            if let (Some(id_s), Some(lat_s), Some(lon_s)) = (
                extract_xml_attr(line, "id"),
                extract_xml_attr(line, "lat"),
                extract_xml_attr(line, "lon"),
            ) {
                if let (Ok(id), Ok(lat), Ok(lon)) = (
                    id_s.parse::<i64>(),
                    lat_s.parse::<f64>(),
                    lon_s.parse::<f64>(),
                ) {
                    nodes.insert(id, Point2D::new(lon, lat));
                }
            }
        } else if line.starts_with("<way ") {
            in_way = true;
            way_nds.clear();
            way_highway.clear();
            way_name.clear();
            way_level = 0.0;
            way_area = false;
        } else if in_way && line.starts_with("<nd ") {
            if let Some(ref_s) = extract_xml_attr(line, "ref") {
                if let Ok(nid) = ref_s.parse::<i64>() {
                    way_nds.push(nid);
                }
            }
        } else if in_way && line.starts_with("<tag ") {
            if let (Some(k), Some(v)) = (extract_xml_attr(line, "k"), extract_xml_attr(line, "v")) {
                match k {
                    "highway" => way_highway = v.to_string(),
                    "name" => way_name = v.to_string(),
                    "layer" => {
                        if let Ok(lv) = v.trim().parse::<f64>() {
                            way_level = lv;
                        }
                    }
                    "bridge" if v != "no" && v != "false" && way_level == 0.0 => {
                        way_level = 1.0;
                    }
                    "tunnel" if v != "no" && v != "false" && way_level == 0.0 => {
                        way_level = -1.0;
                    }
                    "area" if v == "yes" => {
                        way_area = true;
                    }
                    _ => {}
                }
            }
        } else if in_way && line.starts_with("</way>") {
            in_way = false;
            if !way_highway.is_empty() && !way_area {
                let coords: Vec<Point2D> =
                    way_nds.iter().filter_map(|id| nodes.get(id).copied()).collect();
                if coords.len() >= 2 {
                    ways.push(Polyline::new(coords).with_meta(
                        way_level,
                        way_highway.clone(),
                        way_name.clone(),
                    ));
                }
            }
        }
    }

    let _ = project_wgs84_to_utm_if_needed(&mut ways);
    Ok(ways)
}

fn load_csv_str(content: &str) -> Result<Vec<Polyline>, String> {
    let mut out = Vec::new();
    for (line_idx, raw_line) in content.lines().enumerate() {
        let line = raw_line.trim();
        if line.is_empty() || (line_idx == 0 && line.chars().any(|c| c.is_alphabetic())) {
            // Check if first line contains WKT LINESTRING despite letters
            if !line.to_ascii_uppercase().contains("LINESTRING") {
                continue;
            }
        }
        // Case 1: WKT LINESTRING (x1 y1, x2 y2, ...)
        let upper = line.to_ascii_uppercase();
        if let Some(pos) = upper.find("LINESTRING") {
            let after = &line[pos + "LINESTRING".len()..];
            if let (Some(lpar), Some(rpar)) = (after.find('('), after.find(')')) {
                if rpar > lpar {
                    let body = &after[lpar + 1..rpar];
                    let mut pts = Vec::new();
                    for pair_str in body.split(',') {
                        let nums: Vec<&str> = pair_str.split_whitespace().collect();
                        if nums.len() >= 2 {
                            if let (Ok(x), Ok(y)) = (nums[0].parse::<f64>(), nums[1].parse::<f64>())
                            {
                                pts.push(Point2D::new(x, y));
                            }
                        }
                    }
                    if pts.len() >= 2 {
                        out.push(Polyline::new(pts));
                    }
                    continue;
                }
            }
        }

        // Case 2: Numeric columns x1,y1,x2,y2 or id,x1,y1,x2,y2
        let cols: Vec<&str> = line.split(',').map(|s| s.trim()).collect();
        if cols.len() >= 4 {
            let parsed: Vec<Option<f64>> = cols.iter().map(|c| c.parse::<f64>().ok()).collect();
            // Try cols[0..4] or cols[1..5]
            let (x1, y1, x2, y2) = if let (Some(a), Some(b), Some(c), Some(d)) =
                (parsed[0], parsed[1], parsed[2], parsed[3])
            {
                if cols.len() >= 5 && parsed[4].is_some() && a.fract() == 0.0 && a >= 0.0 {
                    (b, c, d, parsed[4].unwrap())
                } else {
                    (a, b, c, d)
                }
            } else {
                continue;
            };
            let p0 = Point2D::new(x1, y1);
            let p1 = Point2D::new(x2, y2);
            if p0.dist(p1) > 1e-9 {
                out.push(Polyline::new(vec![p0, p1]));
            }
        }
    }
    let _ = project_wgs84_to_utm_if_needed(&mut out);
    Ok(out)
}

/// Dispatches polyline loading by file extension (`.geojson`/`.json`, `.dxf`, `.mif`, `.osm`/`.xml`, `.csv`).
pub fn load_polylines_auto(path: &Path) -> Result<Vec<Polyline>, String> {
    let content = fs::read_to_string(path)
        .map_err(|e| format!("Failed to read '{}': {e}", path.display()))?;
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();

    match ext.as_str() {
        "geojson" | "json" => load_geojson_str(&content),
        "dxf" => load_dxf_str(&content),
        "mif" => load_mif_str(&content),
        "osm" | "xml" => load_osm_xml_str(&content),
        "csv" | "txt" => load_csv_str(&content),
        _ => {
            if content.trim_start().starts_with('{') {
                load_geojson_str(&content)
            } else if content.contains("<osm") {
                load_osm_xml_str(&content)
            } else if content.contains("SECTION") && content.contains("ENTITIES") {
                load_dxf_str(&content)
            } else {
                load_csv_str(&content)
            }
        }
    }
}

fn highway_allowed_for_mode(hw: &str, mode: &str) -> bool {
    let h = hw.trim().to_ascii_lowercase();
    if h.is_empty() {
        return false;
    }
    match mode.trim().to_ascii_lowercase().as_str() {
        "vehicular" | "drive" => matches!(
            h.as_str(),
            "motorway"
                | "trunk"
                | "primary"
                | "secondary"
                | "tertiary"
                | "unclassified"
                | "residential"
                | "living_street"
                | "service"
                | "motorway_link"
                | "trunk_link"
                | "primary_link"
                | "secondary_link"
                | "tertiary_link"
                | "road"
        ),
        "pedestrian" | "walk" => matches!(
            h.as_str(),
            "primary"
                | "secondary"
                | "tertiary"
                | "primary_link"
                | "secondary_link"
                | "tertiary_link"
                | "pedestrian"
                | "footway"
                | "path"
                | "steps"
                | "living_street"
                | "cycleway"
                | "residential"
                | "unclassified"
                | "track"
                | "service"
                | "corridor"
                | "road"
        ),
        _ => matches!(
            h.as_str(),
            "primary"
                | "secondary"
                | "tertiary"
                | "unclassified"
                | "residential"
                | "living_street"
                | "pedestrian"
                | "footway"
                | "path"
                | "steps"
                | "cycleway"
                | "service"
                | "trunk"
                | "motorway"
                | "motorway_link"
                | "trunk_link"
                | "primary_link"
                | "secondary_link"
                | "tertiary_link"
                | "track"
                | "road"
        ),
    }
}

/// Fetches OpenStreetMap highway ways within `(south, west, north, east)` (WGS84 degrees)
/// using `ureq` with a 10s timeout and multi-mirror failover, then projects to metric UTM.
pub fn fetch_osm_overpass_bbox(
    south: f64,
    west: f64,
    north: f64,
    east: f64,
    mode: &str,
) -> Result<Vec<Polyline>, String> {
    fetch_osm_overpass_boundary(south, west, north, east, None, mode)
}

/// Inverse Transverse Mercator projection: converts UTM `(easting_m, northing_m)` in `(zone, north)`
/// back to WGS84 `(lon_deg, lat_deg)`.
#[inline]
pub fn utm_to_wgs84_lon_lat(easting_m: f64, northing_m: f64, zone: u32, north: bool) -> (f64, f64) {
    const A: f64 = 6_378_137.0;
    const F: f64 = 1.0 / 298.257_223_563;
    const K0: f64 = 0.9996;
    let e2 = F * (2.0 - F);
    let ep2 = e2 / (1.0 - e2);
    let e1 = (1.0 - (1.0 - e2).sqrt()) / (1.0 + (1.0 - e2).sqrt());

    let x = easting_m - 500_000.0;
    let y = if north {
        northing_m
    } else {
        northing_m - 10_000_000.0
    };

    let m = y / K0;
    let e4 = e2 * e2;
    let e6 = e4 * e2;
    let mu = m / (A * (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0));

    let e1_2 = e1 * e1;
    let e1_3 = e1_2 * e1;
    let e1_4 = e1_2 * e1_2;
    let phi1 = mu
        + (3.0 * e1 / 2.0 - 27.0 * e1_3 / 32.0) * (2.0 * mu).sin()
        + (21.0 * e1_2 / 16.0 - 55.0 * e1_4 / 32.0) * (4.0 * mu).sin()
        + (151.0 * e1_3 / 96.0) * (6.0 * mu).sin()
        + (1097.0 * e1_4 / 512.0) * (8.0 * mu).sin();

    let sin_p = phi1.sin();
    let cos_p = phi1.cos().max(1e-12);
    let tan_p = phi1.tan();

    let n1 = A / (1.0 - e2 * sin_p * sin_p).sqrt();
    let t1 = tan_p * tan_p;
    let c1 = ep2 * cos_p * cos_p;
    let r1 = A * (1.0 - e2) / (1.0 - e2 * sin_p * sin_p).powf(1.5);
    let d = x / (n1 * K0);

    let d2 = d * d;
    let d3 = d2 * d;
    let d4 = d2 * d2;
    let d5 = d4 * d;
    let d6 = d4 * d2;

    let lat_rad = phi1
        - (n1 * tan_p / r1)
            * (d2 / 2.0
                - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1 * c1 - 9.0 * ep2) * d4 / 24.0
                + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1 * t1 - 252.0 * ep2 - 3.0 * c1 * c1)
                    * d6
                    / 720.0);

    let lon0 = (((zone.clamp(1, 60) as f64) - 1.0) * 6.0 - 180.0 + 3.0).to_radians();
    let lon_rad = lon0
        + (d - (1.0 + 2.0 * t1 + c1) * d3 / 6.0
            + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1 * c1 + 8.0 * ep2 + 24.0 * t1 * t1) * d5
                / 120.0)
            / cos_p;

    (lon_rad.to_degrees(), lat_rad.to_degrees())
}

/// Imported spatial boundary (from `.gpkg`, `.geojson`, `.dxf`, or `.mif`) converted to WGS84 `(lat, lon)`.
#[derive(Debug, Clone)]
pub struct ImportedBoundary {
    /// Boundary vertices in `(lat_deg, lon_deg)` order.
    pub polygon_lat_lon: Vec<(f64, f64)>,
    pub south: f64,
    pub west: f64,
    pub north: f64,
    pub east: f64,
    pub source_summary: String,
}

fn point_in_polygon_latlon(lat: f64, lon: f64, poly: &[(f64, f64)]) -> bool {
    if poly.len() < 3 {
        return true;
    }
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (yi, xi) = poly[i];
        let (yj, xj) = poly[j];
        let intersect = ((yi > lat) != (yj > lat))
            && (lon < (xj - xi) * (lat - yi) / ((yj - yi) + 1e-15) + xi);
        if intersect {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Result of geocoding a coordinate pair (`"lat, lon"`) or place name via Nominatim OSM.
#[derive(Debug, Clone)]
pub struct OsmGeocodedLocation {
    pub lat: f64,
    pub lon: f64,
    pub south: f64,
    pub west: f64,
    pub north: f64,
    pub east: f64,
    pub display_name: String,
}

fn urlencode_query_component(input: &str) -> String {
    let mut out = String::with_capacity(input.len() * 2);
    for b in input.trim().bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => {
                out.push(b as char);
            }
            b' ' => out.push('+'),
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

/// Parses a `"lat, lon"` coordinate string or queries OpenStreetMap Nominatim (`zero API key`)
/// to resolve a place/address string into a WGS84 center and urban study bounding box.
pub fn geocode_osm_location(query: &str) -> Result<OsmGeocodedLocation, String> {
    let trimmed = query.trim();
    if trimmed.is_empty() {
        return Err("Search query is empty. Enter 'lat, lon' or a place name.".to_string());
    }

    // 1. Check if the query is a direct numeric coordinate pair: "13.7446, 100.5331" or "13.7446 100.5331"
    let cleaned_nums: String = trimmed
        .chars()
        .map(|c| if c == ',' || c == ';' || c == '/' { ' ' } else { c })
        .collect();
    let parts: Vec<&str> = cleaned_nums.split_whitespace().collect();
    if parts.len() == 2 {
        if let (Ok(v0), Ok(v1)) = (parts[0].parse::<f64>(), parts[1].parse::<f64>()) {
            if v0.is_finite() && v1.is_finite() {
                let (lat, lon) = if v0.abs() <= 85.0 && v1.abs() <= 180.0 {
                    (v0, v1)
                } else if v1.abs() <= 85.0 && v0.abs() <= 180.0 {
                    (v1, v0)
                } else {
                    return Err(format!("Coordinates ({v0}, {v1}) out of valid WGS84 range."));
                };
                let d_lat = 0.0060;
                let d_lon = 0.0075;
                return Ok(OsmGeocodedLocation {
                    lat,
                    lon,
                    south: (lat - d_lat).clamp(-85.0, 85.0),
                    north: (lat + d_lat).clamp(-85.0, 85.0),
                    west: (lon - d_lon).clamp(-180.0, 180.0),
                    east: (lon + d_lon).clamp(-180.0, 180.0),
                    display_name: format!("Coordinates ({lat:.5}, {lon:.5})"),
                });
            }
        }
    }

    // 2. Query OpenStreetMap Nominatim API (Zero API Key)
    let encoded = urlencode_query_component(trimmed);
    let url = format!(
        "https://nominatim.openstreetmap.org/search?format=json&limit=1&accept-language=en&q={encoded}"
    );
    let agent = ureq::AgentBuilder::new()
        .timeout_connect(Duration::from_secs(6))
        .timeout(Duration::from_secs(10))
        .build();

    let resp = agent
        .get(&url)
        .set(
            "User-Agent",
            "SSE-SpaceSyntaxEngine/1.1 (Academic Urban Morphology Workstation)",
        )
        .set("Accept", "application/json")
        .call()
        .map_err(|e| format!("Nominatim geocoding HTTP error: {e}"))?;

    let arr: Value = resp
        .into_json()
        .map_err(|e| format!("Nominatim JSON parse error: {e}"))?;
    let first = arr
        .as_array()
        .and_then(|a| a.first())
        .ok_or_else(|| format!("No location found matching '{trimmed}'."))?;

    let lat = first
        .get("lat")
        .and_then(|v| v.as_str().and_then(|s| s.parse::<f64>().ok()).or_else(|| v.as_f64()))
        .ok_or_else(|| "Nominatim result missing 'lat'".to_string())?;
    let lon = first
        .get("lon")
        .and_then(|v| v.as_str().and_then(|s| s.parse::<f64>().ok()).or_else(|| v.as_f64()))
        .ok_or_else(|| "Nominatim result missing 'lon'".to_string())?;
    let display_name = first
        .get("display_name")
        .and_then(|v| v.as_str())
        .unwrap_or(trimmed)
        .to_string();

    // Compute a sensible urban neighborhood study bounding box (~1.2 km - 2.0 km) around (lat, lon)
    let mut half_lat = 0.0060_f64;
    let mut half_lon = 0.0075_f64;
    if let Some(bb) = first.get("boundingbox").and_then(|v| v.as_array()) {
        if bb.len() == 4 {
            let s_bb = bb[0].as_str().and_then(|s| s.parse::<f64>().ok());
            let n_bb = bb[1].as_str().and_then(|s| s.parse::<f64>().ok());
            let w_bb = bb[2].as_str().and_then(|s| s.parse::<f64>().ok());
            let e_bb = bb[3].as_str().and_then(|s| s.parse::<f64>().ok());
            if let (Some(s0), Some(n0), Some(w0), Some(e0)) = (s_bb, n_bb, w_bb, e_bb) {
                half_lat = ((n0 - s0).abs() * 0.5).clamp(0.0040, 0.0090);
                half_lon = ((e0 - w0).abs() * 0.5).clamp(0.0050, 0.0110);
            }
        }
    }

    Ok(OsmGeocodedLocation {
        lat,
        lon,
        south: (lat - half_lat).clamp(-85.0, 85.0),
        north: (lat + half_lat).clamp(-85.0, 85.0),
        west: (lon - half_lon).clamp(-180.0, 180.0),
        east: (lon + half_lon).clamp(-180.0, 180.0),
        display_name,
    })
}

/// Detailed stage output from fetching, filtering/clipping, and UTM-projecting an OSM network.
#[derive(Debug, Clone)]
pub struct OsmFetchStageReport {
    pub polylines: Vec<Polyline>,
    pub raw_elements_count: usize,
    pub filtered_ways_count: usize,
    pub utm_epsg: Option<u32>,
    pub source_endpoint: String,
}

/// Fetches OpenStreetMap highway ways inside a WGS84 bounding box or arbitrary polygon boundary
/// (`polygon_lat_lon` as `(lat_deg, lon_deg)` vertices), filters by `mode`, clips to boundary,
/// and projects to metric UTM.
pub fn fetch_osm_overpass_boundary(
    south: f64,
    west: f64,
    north: f64,
    east: f64,
    polygon_lat_lon: Option<&[(f64, f64)]>,
    mode: &str,
) -> Result<Vec<Polyline>, String> {
    let report = fetch_osm_overpass_boundary_staged(
        south,
        west,
        north,
        east,
        polygon_lat_lon,
        mode,
        None,
        |_, _, _| {},
    )?;
    Ok(report.polylines)
}

/// Stage-instrumented OSM network fetcher with Fastly-CDN OpenStreetMap API v0.6 primary endpoint
/// and automatic multi-mirror Overpass failover.
pub fn fetch_osm_overpass_boundary_staged<F>(
    south: f64,
    west: f64,
    north: f64,
    east: f64,
    polygon_lat_lon: Option<&[(f64, f64)]>,
    mode: &str,
    cancel_flag: Option<&std::sync::atomic::AtomicBool>,
    mut on_progress: F,
) -> Result<OsmFetchStageReport, String>
where
    F: FnMut(usize, f32, String),
{
    use std::sync::atomic::Ordering;
    let is_cancelled = || cancel_flag.map_or(false, |f| f.load(Ordering::Relaxed));

    let mut clean_poly: Vec<(f64, f64)> = Vec::new();
    if let Some(poly) = polygon_lat_lon {
        for &(lat, lon) in poly {
            if lat.is_finite() && lon.is_finite() {
                if clean_poly
                    .last()
                    .map(|&(pl, pn)| (pl - lat).hypot(pn - lon) > 1e-7)
                    .unwrap_or(true)
                {
                    clean_poly.push((lat, lon));
                }
            }
        }
        if clean_poly.len() >= 2 {
            let first = clean_poly[0];
            let last = *clean_poly.last().unwrap();
            if (first.0 - last.0).hypot(first.1 - last.1) <= 1e-7 {
                clean_poly.pop();
            }
        }
    }

    let (s_box, n_box, w_box, e_box) = if clean_poly.len() >= 3 {
        let mut min_lat = f64::INFINITY;
        let mut max_lat = f64::NEG_INFINITY;
        let mut min_lon = f64::INFINITY;
        let mut max_lon = f64::NEG_INFINITY;
        for &(la, lo) in &clean_poly {
            min_lat = min_lat.min(la);
            max_lat = max_lat.max(la);
            min_lon = min_lon.min(lo);
            max_lon = max_lon.max(lo);
        }
        (min_lat, max_lat, min_lon, max_lon)
    } else {
        (
            south.min(north),
            south.max(north),
            west.min(east),
            west.max(east),
        )
    };

    if s_box >= n_box || w_box >= e_box {
        return Err(format!(
            "Invalid boundary extent ({s_box:.4}, {w_box:.4}, {n_box:.4}, {e_box:.4}): south < north and west < east required."
        ));
    }

    let mut json_val: Option<Value> = None;
    let mut source_endpoint = String::new();
    let mut last_err = String::new();

    // 1A. Primary Fast-Path: Official OpenStreetMap API v0.6 JSON (Fastly CDN + cgimap) for urban extents <= 0.20 deg²
    let bbox_deg_area = (n_box - s_box) * (e_box - w_box);
    if bbox_deg_area <= 0.20 && !is_cancelled() {
        let osm_v06_url = format!(
            "https://api.openstreetmap.org/api/0.6/map.json?bbox={w_box:.6},{s_box:.6},{e_box:.6},{n_box:.6}"
        );
        on_progress(
            1,
            10.0,
            format!(
                "Querying OpenStreetMap CDN API v0.6 ({s_box:.4}, {w_box:.4} to {n_box:.4}, {e_box:.4})..."
            ),
        );
        let fast_agent = ureq::AgentBuilder::new()
            .timeout_connect(Duration::from_secs(6))
            .timeout(Duration::from_secs(18))
            .build();
        match fast_agent
            .get(&osm_v06_url)
            .set(
                "User-Agent",
                "SSE-SpaceSyntaxEngine/1.1 (Academic Urban Morphology Workstation)",
            )
            .set("Accept", "application/json")
            .call()
        {
            Ok(resp) => match resp.into_json::<Value>() {
                Ok(v) => {
                    json_val = Some(v);
                    source_endpoint = "api.openstreetmap.org/api/0.6/map.json".to_string();
                }
                Err(e) => {
                    last_err = format!("api.openstreetmap.org JSON parse error: {e}");
                }
            },
            Err(e) => {
                last_err = format!("api.openstreetmap.org HTTP error: {e}");
            }
        }
    }

    // 1B. Multi-Mirror Overpass API Failover
    if json_val.is_none() && !is_cancelled() {
        let query = if clean_poly.len() >= 3 {
            let step = (clean_poly.len() / 50).max(1);
            let mut poly_str = String::new();
            for (idx, &(lat, lon)) in clean_poly.iter().enumerate() {
                if idx % step == 0 || idx + 1 == clean_poly.len() {
                    if !poly_str.is_empty() {
                        poly_str.push(' ');
                    }
                    poly_str.push_str(&format!("{lat:.6} {lon:.6}"));
                }
            }
            format!(
                "[out:json][timeout:25];(way[\"highway\"][\"area\"!=\"yes\"][\"indoor\"!=\"yes\"](poly:\"{poly_str}\"););out body geom;"
            )
        } else {
            format!(
                "[out:json][timeout:25];(way[\"highway\"][\"area\"!=\"yes\"][\"indoor\"!=\"yes\"]({s_box:.6},{w_box:.6},{n_box:.6},{e_box:.6}););out body geom;"
            )
        };

        const MIRRORS: [&str; 4] = [
            "https://overpass-api.de/api/interpreter",
            "https://lz4.overpass-api.de/api/interpreter",
            "https://z.overpass-api.de/api/interpreter",
            "https://overpass.private.coffee/api/interpreter",
        ];

        let agent = ureq::AgentBuilder::new()
            .timeout_connect(Duration::from_secs(6))
            .timeout(Duration::from_secs(25))
            .build();

        for (m_idx, endpoint) in MIRRORS.iter().enumerate() {
            if is_cancelled() {
                return Err("Pipeline cancelled by user.".to_string());
            }
            on_progress(
                1,
                14.0 + (m_idx as f32) * 5.0,
                format!("Querying Overpass mirror ({}/{}) {}...", m_idx + 1, MIRRORS.len(), endpoint),
            );
            match agent
                .post(endpoint)
                .set(
                    "User-Agent",
                    "SSE-SpaceSyntaxEngine/1.1 (Academic Urban Morphology Workstation)",
                )
                .set("Accept", "*/*")
                .send_form(&[("data", query.as_str())])
            {
                Ok(resp) => match resp.into_json::<Value>() {
                    Ok(v) => {
                        json_val = Some(v);
                        source_endpoint = (*endpoint).to_string();
                        break;
                    }
                    Err(e) => {
                        last_err = format!("{endpoint} JSON parse error: {e}");
                    }
                },
                Err(e) => {
                    last_err = format!("{endpoint} HTTP error: {e}");
                }
            }
        }
    }

    if is_cancelled() {
        return Err("Pipeline cancelled by user.".to_string());
    }

    let root = json_val.ok_or_else(|| format!("All OSM/Overpass endpoints failed ({last_err})"))?;
    let elements = root
        .get("elements")
        .and_then(|v| v.as_array())
        .ok_or_else(|| "OSM JSON missing 'elements' array".to_string())?;

    let raw_elements_count = elements.len();
    on_progress(
        2,
        38.0,
        format!(
            "Downloaded {} raw OSM elements from {}. Filtering '{}' highway ways & clipping to boundary...",
            raw_elements_count, source_endpoint, mode
        ),
    );

    // Build node lookup table (id -> (lon, lat)) for OSM API v0.6 / skel node references
    let mut node_lut: HashMap<i64, (f64, f64)> = HashMap::new();
    for el in elements {
        if el.get("type").and_then(|v| v.as_str()) == Some("node") {
            if let (Some(id), Some(lon), Some(lat)) = (
                el.get("id").and_then(|v| v.as_i64()),
                el.get("lon").and_then(|v| v.as_f64()),
                el.get("lat").and_then(|v| v.as_f64()),
            ) {
                node_lut.insert(id, (lon, lat));
            }
        }
    }

    let inside_bbox = |lat: f64, lon: f64| -> bool {
        lat >= s_box && lat <= n_box && lon >= w_box && lon <= e_box
    };

    let mut lines = Vec::new();
    for el in elements {
        if el.get("type").and_then(|v| v.as_str()) != Some("way") {
            continue;
        }
        let tags = el.get("tags");
        if let Some(t_obj) = tags.and_then(|v| v.as_object()) {
            if t_obj.get("area").and_then(|v| v.as_str()) == Some("yes")
                || t_obj.get("indoor").and_then(|v| v.as_str()) == Some("yes")
            {
                continue;
            }
        }
        let (level, hw, name) = extract_props_meta(tags);
        if !highway_allowed_for_mode(&hw, mode) {
            continue;
        }

        let mut coords: Vec<Point2D> = Vec::new();
        if let Some(geom_arr) = el.get("geometry").and_then(|v| v.as_array()) {
            coords.reserve(geom_arr.len());
            for pt in geom_arr {
                if let (Some(lon), Some(lat)) = (
                    pt.get("lon").and_then(|v| v.as_f64()),
                    pt.get("lat").and_then(|v| v.as_f64()),
                ) {
                    coords.push(Point2D::new(lon, lat));
                }
            }
        } else if let Some(nodes_arr) = el.get("nodes").and_then(|v| v.as_array()) {
            coords.reserve(nodes_arr.len());
            for nid_val in nodes_arr {
                if let Some(nid) = nid_val.as_i64() {
                    if let Some(&(lon, lat)) = node_lut.get(&nid) {
                        coords.push(Point2D::new(lon, lat));
                    }
                }
            }
        }

        if coords.len() < 2 {
            continue;
        }

        // Clip sub-chains to polygon (if >= 3 vertices) or rectangle BBox
        let mut current_run: Vec<Point2D> = Vec::new();
        for w in coords.windows(2) {
            let p0 = w[0];
            let p1 = w[1];
            let mid_lat = 0.5 * (p0.y + p1.y);
            let mid_lon = 0.5 * (p0.x + p1.x);
            let keep = if clean_poly.len() >= 3 {
                point_in_polygon_latlon(p0.y, p0.x, &clean_poly)
                    || point_in_polygon_latlon(p1.y, p1.x, &clean_poly)
                    || point_in_polygon_latlon(mid_lat, mid_lon, &clean_poly)
            } else {
                inside_bbox(p0.y, p0.x)
                    || inside_bbox(p1.y, p1.x)
                    || inside_bbox(mid_lat, mid_lon)
            };
            if keep {
                if current_run.is_empty() {
                    current_run.push(p0);
                }
                current_run.push(p1);
            } else if current_run.len() >= 2 {
                lines.push(
                    Polyline::new(std::mem::take(&mut current_run)).with_meta(
                        level,
                        hw.clone(),
                        name.clone(),
                    ),
                );
            } else {
                current_run.clear();
            }
        }
        if current_run.len() >= 2 {
            lines.push(Polyline::new(current_run).with_meta(level, hw, name));
        }
    }

    if lines.is_empty() {
        return Err(format!(
            "No '{mode}' street ways found inside the selected boundary ({raw_elements_count} raw OSM elements inspected)."
        ));
    }

    let filtered_ways_count = lines.len();
    on_progress(
        3,
        55.0,
        format!(
            "Filtered & clipped {} street polylines. Projecting WGS84 (lat/lon) to Metric UTM...",
            filtered_ways_count
        ),
    );

    let utm_epsg = project_wgs84_to_utm_if_needed(&mut lines);
    Ok(OsmFetchStageReport {
        polylines: lines,
        raw_elements_count,
        filtered_ways_count,
        utm_epsg,
        source_endpoint,
    })
}

fn read_f64_endian(bytes: &[u8], offset: usize, little_endian: bool) -> Option<f64> {
    let slice = bytes.get(offset..offset + 8)?;
    let arr: [u8; 8] = slice.try_into().ok()?;
    let v = if little_endian {
        f64::from_le_bytes(arr)
    } else {
        f64::from_be_bytes(arr)
    };
    if v.is_finite() { Some(v) } else { None }
}

fn read_u32_endian(bytes: &[u8], offset: usize, little_endian: bool) -> Option<u32> {
    let slice = bytes.get(offset..offset + 4)?;
    let arr: [u8; 4] = slice.try_into().ok()?;
    Some(if little_endian {
        u32::from_le_bytes(arr)
    } else {
        u32::from_be_bytes(arr)
    })
}

fn read_i32_endian(bytes: &[u8], offset: usize, little_endian: bool) -> Option<i32> {
    let slice = bytes.get(offset..offset + 4)?;
    let arr: [u8; 4] = slice.try_into().ok()?;
    Some(if little_endian {
        i32::from_le_bytes(arr)
    } else {
        i32::from_be_bytes(arr)
    })
}

fn parse_wkb_geometry(
    bytes: &[u8],
    pos: &mut usize,
    rings_out: &mut Vec<Vec<(f64, f64)>>,
    depth: u32,
) -> Option<()> {
    if depth > 6 || *pos + 5 > bytes.len() {
        return None;
    }
    let byte_order = *bytes.get(*pos)?;
    if byte_order > 1 {
        return None;
    }
    let le = byte_order == 1;
    *pos += 1;

    let raw_type = read_u32_endian(bytes, *pos, le)?;
    *pos += 4;

    // Handle EWKB SRID flag (0x20000000) and Z/M flags (0x80000000, 0x40000000)
    let has_ewkb_srid = (raw_type & 0x2000_0000) != 0;
    if has_ewkb_srid {
        if *pos + 4 > bytes.len() {
            return None;
        }
        *pos += 4;
    }
    let ewkb_z = (raw_type & 0x8000_0000) != 0;
    let ewkb_m = (raw_type & 0x4000_0000) != 0;
    let base_code = raw_type & 0x0FFF_FFFF;

    let (geom_type, dims) = if base_code >= 3000 && base_code < 4000 {
        (base_code - 3000, 4usize)
    } else if base_code >= 2000 && base_code < 3000 {
        (base_code - 2000, 3usize)
    } else if base_code >= 1000 && base_code < 2000 {
        (base_code - 1000, 3usize)
    } else {
        let d = 2 + usize::from(ewkb_z) + usize::from(ewkb_m);
        (base_code, d)
    };
    let stride = dims * 8;

    match geom_type {
        1 => {
            // Point
            let x = read_f64_endian(bytes, *pos, le)?;
            let y = read_f64_endian(bytes, *pos + 8, le)?;
            *pos += stride;
            rings_out.push(vec![(x, y)]);
            Some(())
        }
        2 => {
            // LineString
            let n_pts = read_u32_endian(bytes, *pos, le)? as usize;
            *pos += 4;
            if n_pts == 0 || n_pts > 250_000 || *pos + n_pts * stride > bytes.len() {
                return None;
            }
            let mut ring = Vec::with_capacity(n_pts);
            for _ in 0..n_pts {
                let x = read_f64_endian(bytes, *pos, le)?;
                let y = read_f64_endian(bytes, *pos + 8, le)?;
                *pos += stride;
                ring.push((x, y));
            }
            rings_out.push(ring);
            Some(())
        }
        3 => {
            // Polygon
            let n_rings = read_u32_endian(bytes, *pos, le)? as usize;
            *pos += 4;
            if n_rings == 0 || n_rings > 10_000 {
                return None;
            }
            for r_idx in 0..n_rings {
                let n_pts = read_u32_endian(bytes, *pos, le)? as usize;
                *pos += 4;
                if n_pts < 3 || n_pts > 250_000 || *pos + n_pts * stride > bytes.len() {
                    return None;
                }
                let mut ring = Vec::with_capacity(n_pts);
                for _ in 0..n_pts {
                    let x = read_f64_endian(bytes, *pos, le)?;
                    let y = read_f64_endian(bytes, *pos + 8, le)?;
                    *pos += stride;
                    ring.push((x, y));
                }
                if r_idx == 0 {
                    rings_out.push(ring);
                }
            }
            Some(())
        }
        4..=7 => {
            // MultiPoint (4), MultiLineString (5), MultiPolygon (6), GeometryCollection (7)
            let n_sub = read_u32_endian(bytes, *pos, le)? as usize;
            *pos += 4;
            if n_sub == 0 || n_sub > 50_000 {
                return None;
            }
            for _ in 0..n_sub {
                parse_wkb_geometry(bytes, pos, rings_out, depth + 1)?;
            }
            Some(())
        }
        _ => None,
    }
}

/// Scans an OGC GeoPackage (`.gpkg`) SQLite file for `GeoPackageBinaryHeader` (`"GP"`) geometry blobs
/// and extracts all polygon/polyline coordinate rings along with the detected `srs_id`.
fn extract_gpkg_rings_and_srid(bytes: &[u8]) -> Result<(Vec<Vec<(f64, f64)>>, Option<i32>), String> {
    if bytes.len() < 16 {
        return Err("GeoPackage file is too small.".to_string());
    }
    let mut rings: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut envelope_rings: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut detected_srid: Option<i32> = None;

    let mut idx = 0usize;
    while idx + 16 < bytes.len() {
        // GeoPackage Binary Header starts with magic bytes 0x47 ('G'), 0x50 ('P'), version 0x00
        if bytes[idx] == 0x47 && bytes[idx + 1] == 0x50 && bytes[idx + 2] == 0x00 {
            let flags = bytes[idx + 3];
            let empty_flag = (flags & 0x10) != 0;
            let env_indicator = (flags >> 1) & 0x07;
            let header_le = (flags & 0x01) != 0;

            if !empty_flag && env_indicator <= 4 {
                if let Some(srid) = read_i32_endian(bytes, idx + 4, header_le) {
                    if srid == 4326
                        || srid == 3857
                        || (32601..=32660).contains(&srid)
                        || (32701..=32760).contains(&srid)
                        || srid == 0
                        || srid == -1
                    {
                        let env_doubles = match env_indicator {
                            0 => 0usize,
                            1 => 4usize,
                            2 | 3 => 6usize,
                            4 => 8usize,
                            _ => 0usize,
                        };
                        let env_bytes = env_doubles * 8;
                        let wkb_start = idx + 8 + env_bytes;
                        if env_doubles >= 4 && idx + 8 + 32 <= bytes.len() {
                            if let (Some(min_x), Some(max_x), Some(min_y), Some(max_y)) = (
                                read_f64_endian(bytes, idx + 8, header_le),
                                read_f64_endian(bytes, idx + 16, header_le),
                                read_f64_endian(bytes, idx + 24, header_le),
                                read_f64_endian(bytes, idx + 32, header_le),
                            ) {
                                if max_x > min_x && max_y > min_y && min_x.abs() < 2.0e7 && max_y.abs() < 2.0e7 {
                                    envelope_rings.push(vec![
                                        (min_x, min_y),
                                        (max_x, min_y),
                                        (max_x, max_y),
                                        (min_x, max_y),
                                    ]);
                                }
                            }
                        }
                        if wkb_start + 9 <= bytes.len() {
                            let mut pos = wkb_start;
                            let mut local_rings = Vec::new();
                            if parse_wkb_geometry(bytes, &mut pos, &mut local_rings, 0).is_some()
                                && !local_rings.is_empty()
                            {
                                if srid > 0 && detected_srid.is_none() {
                                    detected_srid = Some(srid);
                                }
                                rings.extend(local_rings);
                                idx = pos;
                                continue;
                            }
                        }
                    }
                }
            }
        }
        idx += 1;
    }

    if rings.is_empty() && !envelope_rings.is_empty() {
        rings = envelope_rings;
    }
    if rings.is_empty() {
        return Err("No valid GeoPackage (GPB/WKB) boundary geometries found in .gpkg file.".to_string());
    }
    Ok((rings, detected_srid))
}

fn convex_hull_xy(points: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut pts: Vec<(f64, f64)> = points
        .iter()
        .copied()
        .filter(|(x, y)| x.is_finite() && y.is_finite())
        .collect();
    pts.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    });
    pts.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9);
    if pts.len() <= 3 {
        return pts;
    }
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| -> f64 {
        (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
    };
    let mut lower = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn ring_signed_area_abs(ring: &[(f64, f64)]) -> f64 {
    if ring.len() < 3 {
        return 0.0;
    }
    let mut a = 0.0;
    for i in 0..ring.len() {
        let (x0, y0) = ring[i];
        let (x1, y1) = ring[(i + 1) % ring.len()];
        a += x0 * y1 - x1 * y0;
    }
    0.5 * a.abs()
}

/// Loads a spatial boundary polygon/rectangle from a GeoPackage (`.gpkg`), GeoJSON (`.geojson`/`.json`),
/// AutoCAD DXF (`.dxf`), or MapInfo (`.mif`) file and converts it to WGS84 `(lat, lon)`.
pub fn load_osm_boundary_file(
    path: &Path,
    ref_lat: f64,
    ref_lon: f64,
) -> Result<ImportedBoundary, String> {
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();

    let mut raw_rings: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut explicit_srid: Option<i32> = None;

    if ext == "gpkg" {
        let bytes =
            fs::read(path).map_err(|e| format!("Failed to read GeoPackage '{}': {e}", path.display()))?;
        let (rings, srid) = extract_gpkg_rings_and_srid(&bytes)?;
        raw_rings = rings;
        explicit_srid = srid;
    } else if ext == "geojson" || ext == "json" {
        let content = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read GeoJSON '{}': {e}", path.display()))?;
        let root: Value =
            serde_json::from_str(&content).map_err(|e| format!("Invalid GeoJSON: {e}"))?;
        let mut lines = Vec::new();
        match root.get("type").and_then(|v| v.as_str()) {
            Some("FeatureCollection") => {
                if let Some(features) = root.get("features").and_then(|v| v.as_array()) {
                    for feat in features {
                        if let Some(geom) = feat.get("geometry") {
                            collect_geojson_geometry(geom, 0.0, "", "", &mut lines);
                        }
                    }
                }
            }
            Some("Feature") => {
                if let Some(geom) = root.get("geometry") {
                    collect_geojson_geometry(geom, 0.0, "", "", &mut lines);
                }
            }
            Some(_) => {
                collect_geojson_geometry(&root, 0.0, "", "", &mut lines);
            }
            None => {}
        }
        if let Some(bbox) = root.get("bbox").and_then(|v| v.as_array()) {
            if bbox.len() >= 4 {
                if let (Some(x0), Some(y0), Some(x1), Some(y1)) = (
                    bbox[0].as_f64(),
                    bbox[1].as_f64(),
                    bbox[2].as_f64(),
                    bbox[3].as_f64(),
                ) {
                    raw_rings.push(vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)]);
                }
            }
        }
        for pl in lines {
            raw_rings.push(pl.coords.into_iter().map(|p| (p.x, p.y)).collect());
        }
    } else if ext == "dxf" {
        let content = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read DXF '{}': {e}", path.display()))?;
        let lines = load_dxf_str(&content)?;
        for pl in lines {
            raw_rings.push(pl.coords.into_iter().map(|p| (p.x, p.y)).collect());
        }
    } else if ext == "mif" {
        let content = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read MIF '{}': {e}", path.display()))?;
        let lines = load_mif_str(&content)?;
        for pl in lines {
            raw_rings.push(pl.coords.into_iter().map(|p| (p.x, p.y)).collect());
        }
    } else {
        let content = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read '{}': {e}", path.display()))?;
        if content.trim_start().starts_with('{') {
            return load_osm_boundary_file(&path.with_extension("geojson"), ref_lat, ref_lon);
        }
        let lines = load_dxf_str(&content)?;
        for pl in lines {
            raw_rings.push(pl.coords.into_iter().map(|p| (p.x, p.y)).collect());
        }
    }

    if raw_rings.is_empty() {
        return Err(format!("No boundary coordinates found in '{}'.", path.display()));
    }

    // Choose best outer polygon ring (largest enclosed area if >= 3 vertices, else convex hull of all vertices)
    let mut best_ring: Vec<(f64, f64)> = Vec::new();
    let mut best_area = 0.0_f64;
    for r in &raw_rings {
        let mut clean: Vec<(f64, f64)> = r.clone();
        if clean.len() >= 2 {
            let f = clean[0];
            let l = *clean.last().unwrap();
            if (f.0 - l.0).hypot(f.1 - l.1) < 1e-8 {
                clean.pop();
            }
        }
        if clean.len() >= 3 {
            let a = ring_signed_area_abs(&clean);
            if a > best_area {
                best_area = a;
                best_ring = clean;
            }
        }
    }

    if best_ring.len() < 3 {
        let all_pts: Vec<(f64, f64)> = raw_rings.into_iter().flatten().collect();
        best_ring = convex_hull_xy(&all_pts);
        if best_ring.len() == 2 {
            let (x0, y0) = best_ring[0];
            let (x1, y1) = best_ring[1];
            best_ring = vec![
                (x0.min(x1), y0.min(y1)),
                (x0.max(x1), y0.min(y1)),
                (x0.max(x1), y0.max(y1)),
                (x0.min(x1), y0.max(y1)),
            ];
        }
    }

    if best_ring.len() < 3 {
        return Err("Boundary file must contain at least 3 non-collinear vertices or a bounding box.".to_string());
    }

    // Convert (x, y) to WGS84 (lat, lon)
    let sample_x = best_ring[0].0;
    let sample_y = best_ring[0].1;

    let ref_zone = (((ref_lon + 180.0) / 6.0).floor() as i32 + 1).clamp(1, 60) as u32;
    let ref_north = ref_lat >= 0.0;

    let mut poly_lat_lon: Vec<(f64, f64)> = Vec::with_capacity(best_ring.len());
    let crs_label: String;

    if let Some(srid) = explicit_srid {
        if (32601..=32660).contains(&srid) {
            let z = (srid - 32600) as u32;
            crs_label = format!("EPSG:{srid} (UTM Zone {z}N)");
            for &(x, y) in &best_ring {
                let (lon, lat) = utm_to_wgs84_lon_lat(x, y, z, true);
                poly_lat_lon.push((lat, lon));
            }
        } else if (32701..=32760).contains(&srid) {
            let z = (srid - 32700) as u32;
            crs_label = format!("EPSG:{srid} (UTM Zone {z}S)");
            for &(x, y) in &best_ring {
                let (lon, lat) = utm_to_wgs84_lon_lat(x, y, z, false);
                poly_lat_lon.push((lat, lon));
            }
        } else if srid == 3857 {
            crs_label = "EPSG:3857 (Web Mercator)".to_string();
            for &(x, y) in &best_ring {
                let lon = (x / 20_037_508.342_789_244) * 180.0;
                let lat_rad = (std::f64::consts::PI * (y / 20_037_508.342_789_244))
                    .exp()
                    .atan()
                    * 2.0
                    - std::f64::consts::FRAC_PI_2;
                poly_lat_lon.push((lat_rad.to_degrees(), lon));
            }
        } else {
            crs_label = "EPSG:4326 (WGS84)".to_string();
            for &(x, y) in &best_ring {
                poly_lat_lon.push((y, x));
            }
        }
    } else if sample_x.abs() <= 180.0 && sample_y.abs() <= 90.0 {
        crs_label = "EPSG:4326 (WGS84)".to_string();
        for &(x, y) in &best_ring {
            poly_lat_lon.push((y, x));
        }
    } else if sample_x >= 100_000.0 && sample_x <= 900_000.0 && sample_y >= 0.0 && sample_y <= 10_000_000.0 {
        crs_label = format!(
            "Auto-UTM Zone {}{}",
            ref_zone,
            if ref_north { "N" } else { "S" }
        );
        for &(x, y) in &best_ring {
            let (lon, lat) = utm_to_wgs84_lon_lat(x, y, ref_zone, ref_north);
            poly_lat_lon.push((lat, lon));
        }
    } else {
        // Web Mercator fallback
        crs_label = "EPSG:3857 (Web Mercator)".to_string();
        for &(x, y) in &best_ring {
            let lon = (x / 20_037_508.342_789_244) * 180.0;
            let lat_rad = (std::f64::consts::PI * (y / 20_037_508.342_789_244))
                .exp()
                .atan()
                * 2.0
                - std::f64::consts::FRAC_PI_2;
            poly_lat_lon.push((lat_rad.to_degrees(), lon));
        }
    }

    let mut south = f64::INFINITY;
    let mut west = f64::INFINITY;
    let mut north = f64::NEG_INFINITY;
    let mut east = f64::NEG_INFINITY;
    for &(lat, lon) in &poly_lat_lon {
        south = south.min(lat);
        north = north.max(lat);
        west = west.min(lon);
        east = east.max(lon);
    }

    let fname = path
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("boundary");
    let source_summary = format!(
        "{} ({} vertices, {})",
        fname,
        poly_lat_lon.len(),
        crs_label
    );

    Ok(ImportedBoundary {
        polygon_lat_lon: poly_lat_lon,
        south,
        west,
        north,
        east,
        source_summary,
    })
}

/// Exports `SegmentRecord`s and their computed Space Syntax attributes as a GeoJSON `FeatureCollection`.
pub fn export_segments_geojson(
    path: &Path,
    segments: &[SegmentRecord],
    attributes: &BTreeMap<String, Vec<f64>>,
) -> Result<(), String> {
    let mut features = Vec::with_capacity(segments.len());

    for (idx, seg) in segments.iter().enumerate() {
        let mut props = serde_json::Map::new();
        props.insert("id".to_string(), json!(seg.id));
        props.insert("length_m".to_string(), json!(seg.length_m));
        props.insert("azimuth_deg".to_string(), json!(seg.azimuth_deg));
        props.insert("level".to_string(), json!(seg.level));
        if !seg.highway.is_empty() {
            props.insert("highway".to_string(), json!(seg.highway));
        }
        if !seg.name.is_empty() {
            props.insert("name".to_string(), json!(seg.name));
        }

        for (attr_name, col) in attributes {
            if let Some(&val) = col.get(idx) {
                if val.is_finite() {
                    props.insert(attr_name.clone(), json!(val));
                } else {
                    props.insert(attr_name.clone(), Value::Null);
                }
            }
        }

        features.push(json!({
            "type": "Feature",
            "properties": Value::Object(props),
            "geometry": {
                "type": "LineString",
                "coordinates": [
                    [seg.p0.x, seg.p0.y],
                    [seg.p1.x, seg.p1.y]
                ]
            }
        }));
    }

    let fc = json!({
        "type": "FeatureCollection",
        "features": features
    });

    let serialized = serde_json::to_string_pretty(&fc)
        .map_err(|e| format!("Failed to serialize GeoJSON: {e}"))?;
    fs::write(path, serialized)
        .map_err(|e| format!("Failed to write GeoJSON '{}': {e}", path.display()))
}

/// Exports `SegmentRecord`s and their computed Space Syntax attributes as a CSV table with WKT geometry.
pub fn export_segments_csv(
    path: &Path,
    segments: &[SegmentRecord],
    attributes: &BTreeMap<String, Vec<f64>>,
) -> Result<(), String> {
    let mut file = fs::File::create(path)
        .map_err(|e| format!("Failed to create CSV '{}': {e}", path.display()))?;

    let attr_keys: Vec<&String> = attributes.keys().collect();
    write!(
        file,
        "id,x1,y1,x2,y2,length_m,azimuth_deg,level,highway,name,wkt"
    )
    .map_err(|e| e.to_string())?;
    for k in &attr_keys {
        write!(file, ",\"{k}\"").map_err(|e| e.to_string())?;
    }
    writeln!(file).map_err(|e| e.to_string())?;

    for (idx, seg) in segments.iter().enumerate() {
        let safe_hw = seg.highway.replace('"', "'");
        let safe_name = seg.name.replace('"', "'");
        write!(
            file,
            "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.1},\"{}\",\"{}\",\"LINESTRING ({:.4} {:.4}, {:.4} {:.4})\"",
            seg.id,
            seg.p0.x,
            seg.p0.y,
            seg.p1.x,
            seg.p1.y,
            seg.length_m,
            seg.azimuth_deg,
            seg.level,
            safe_hw,
            safe_name,
            seg.p0.x,
            seg.p0.y,
            seg.p1.x,
            seg.p1.y
        )
        .map_err(|e| e.to_string())?;

        for k in &attr_keys {
            let val = attributes
                .get(*k)
                .and_then(|col| col.get(idx))
                .copied()
                .unwrap_or(f64::NAN);
            if val.is_finite() {
                write!(file, ",{val:.6}").map_err(|e| e.to_string())?;
            } else {
                write!(file, ",").map_err(|e| e.to_string())?;
            }
        }
        writeln!(file).map_err(|e| e.to_string())?;
    }

    Ok(())
}

/// Exports `SegmentRecord`s as an AutoCAD `AC1018` DXF file with `LINE` entities and true-color (`420`) coding.
pub fn export_segments_dxf(
    path: &Path,
    segments: &[SegmentRecord],
    attributes: &BTreeMap<String, Vec<f64>>,
    color_attr: Option<&str>,
) -> Result<(), String> {
    let mut file = fs::File::create(path)
        .map_err(|e| format!("Failed to create DXF '{}': {e}", path.display()))?;

    let col_opt = color_attr
        .and_then(|k| attributes.get(k))
        .or_else(|| attributes.values().next());
    let stats = col_opt.map(|c| SummaryStats::from_slice(c));

    writeln!(
        file,
        "0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1018\n0\nENDSEC\n0\nSECTION\n2\nENTITIES"
    )
    .map_err(|e| e.to_string())?;

    for (idx, seg) in segments.iter().enumerate() {
        let mut t = 0.5;
        if let (Some(col), Some(st)) = (col_opt, stats) {
            if let Some(&v) = col.get(idx) {
                if v.is_finite() && (st.max - st.min) > 1e-12 {
                    t = ((v - st.min) / (st.max - st.min)).clamp(0.0, 1.0);
                }
            }
        }
        let rgb = depthmapx_color_rgb(t, ColorPalette::DepthmapClassic);
        let true_color: i32 = ((rgb[0] as i32) << 16) | ((rgb[1] as i32) << 8) | (rgb[2] as i32);

        writeln!(
            file,
            "0\nLINE\n8\nSSE_SEGMENTS\n62\n7\n420\n{}\n10\n{:.4}\n20\n{:.4}\n30\n{:.2}\n11\n{:.4}\n21\n{:.4}\n31\n{:.2}",
            true_color, seg.p0.x, seg.p0.y, seg.level * 4.5, seg.p1.x, seg.p1.y, seg.level * 4.5
        )
        .map_err(|e| e.to_string())?;
    }

    writeln!(file, "0\nENDSEC\n0\nEOF").map_err(|e| e.to_string())?;
    Ok(())
}

/// Exports `SegmentRecord`s as a UCL `depthmapX`-compatible MapInfo `.mif` + `.mid` file pair.
pub fn export_segments_mif_mid(
    mif_path: &Path,
    segments: &[SegmentRecord],
    attributes: &BTreeMap<String, Vec<f64>>,
) -> Result<(), String> {
    let mid_path = mif_path.with_extension("mid");
    let mut mif = fs::File::create(mif_path)
        .map_err(|e| format!("Failed to create MIF '{}': {e}", mif_path.display()))?;
    let mut mid = fs::File::create(&mid_path)
        .map_err(|e| format!("Failed to create MID '{}': {e}", mid_path.display()))?;

    let attr_keys: Vec<&String> = attributes.keys().collect();
    writeln!(mif, "Version 300\nCharset \"WindowsLatin1\"\nDelimiter \",\"").map_err(|e| e.to_string())?;
    writeln!(mif, "Columns {}", 3 + attr_keys.len()).map_err(|e| e.to_string())?;
    writeln!(mif, "  Ref Integer\n  Length_m Float\n  Azimuth_deg Float").map_err(|e| e.to_string())?;
    for k in &attr_keys {
        let clean_name: String = k
            .chars()
            .map(|c| if c.is_alphanumeric() || c == '_' { c } else { '_' })
            .collect();
        writeln!(mif, "  {clean_name} Float").map_err(|e| e.to_string())?;
    }
    writeln!(mif, "Data\n").map_err(|e| e.to_string())?;

    for (idx, seg) in segments.iter().enumerate() {
        writeln!(
            mif,
            "Line {:.4} {:.4} {:.4} {:.4}\n    Pen (1,2,0)",
            seg.p0.x, seg.p0.y, seg.p1.x, seg.p1.y
        )
        .map_err(|e| e.to_string())?;

        write!(mid, "{},{:.4},{:.4}", seg.id, seg.length_m, seg.azimuth_deg)
            .map_err(|e| e.to_string())?;
        for k in &attr_keys {
            let val = attributes
                .get(*k)
                .and_then(|col| col.get(idx))
                .copied()
                .unwrap_or(-1.0);
            let out_v = if val.is_finite() { val } else { -1.0 };
            write!(mid, ",{out_v:.6}").map_err(|e| e.to_string())?;
        }
        writeln!(mid).map_err(|e| e.to_string())?;
    }

    Ok(())
}

/// Exports `VgaResults` PointMap cells and all computed VGA/Isovist/Agent attributes to CSV.
pub fn export_vga_csv(path: &Path, vga: &VgaResults) -> Result<(), String> {
    let mut file = fs::File::create(path)
        .map_err(|e| format!("Failed to create VGA CSV '{}': {e}", path.display()))?;

    let attr_keys: Vec<&String> = vga.attributes.keys().collect();
    write!(file, "Ref,x,y,grid_i,grid_j").map_err(|e| e.to_string())?;
    for k in &attr_keys {
        write!(file, ",\"{k}\"").map_err(|e| e.to_string())?;
    }
    writeln!(file).map_err(|e| e.to_string())?;

    for (idx, pt) in vga.points.iter().enumerate() {
        write!(
            file,
            "{},{:.4},{:.4},{},{}",
            pt.depthmap_ref, pt.x, pt.y, pt.grid_i, pt.grid_j
        )
        .map_err(|e| e.to_string())?;

        for k in &attr_keys {
            let val = vga
                .attributes
                .get(*k)
                .and_then(|col| col.get(idx))
                .copied()
                .unwrap_or(f64::NAN);
            if val.is_finite() {
                write!(file, ",{val:.6}").map_err(|e| e.to_string())?;
            } else {
                write!(file, ",").map_err(|e| e.to_string())?;
            }
        }
        writeln!(file).map_err(|e| e.to_string())?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wgs84_to_utm_projection() {
        // Bangkok (lon ~100.5018, lat ~13.7563) -> UTM Zone 47N (EPSG:32647)
        let mut lines = vec![Polyline::new(vec![
            Point2D::new(100.5018, 13.7563),
            Point2D::new(100.5118, 13.7663),
        ])];
        let epsg = project_wgs84_to_utm_if_needed(&mut lines);
        assert_eq!(epsg, Some(32647));
        let len = lines[0].length();
        assert!(len > 1400.0 && len < 1700.0, "utm len={}", len);
    }

    #[test]
    fn test_load_dxf_and_mif_roundtrip() {
        let dxf = "0\nSECTION\n2\nBLOCKS\n0\nLINE\n8\n0\n10\n-999.0\n20\n-999.0\n11\n-998.0\n21\n-998.0\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\nLINE\n8\n0\n10\n0.0\n20\n0.0\n11\n100.0\n21\n50.0\n0\nENDSEC\n0\nEOF\n";
        let lines = load_dxf_str(dxf).unwrap();
        assert_eq!(lines.len(), 1);
        assert!((lines[0].length() - 111.80339887).abs() < 1e-4);

        let mif = "Version 300\nData\nLine 0.0 0.0 30.0 40.0\n";
        let mif_lines = load_mif_str(mif).unwrap();
        assert_eq!(mif_lines.len(), 1);
        assert!((mif_lines[0].length() - 50.0).abs() < 1e-6);
    }

    #[test]
    fn test_utm_to_wgs84_and_boundary_importers_gpkg_dxf_geojson() {
        // 1. Roundtrip WGS84 <-> UTM Zone 47N (Bangkok)
        let lon0 = 100.5275;
        let lat0 = 13.7260;
        let utm_pt = wgs84_lon_lat_to_utm(lon0, lat0, 47, true);
        let (lon1, lat1) = utm_to_wgs84_lon_lat(utm_pt.x, utm_pt.y, 47, true);
        assert!((lon0 - lon1).abs() < 1e-6, "lon1={lon1}");
        assert!((lat0 - lat1).abs() < 1e-6, "lat1={lat1}");

        // 2. Test binary OGC GeoPackage (.gpkg) GPB header + WKB Polygon extraction
        let mut gpkg_bytes = Vec::new();
        gpkg_bytes.extend_from_slice(b"SQLite format 3\0");
        gpkg_bytes.extend_from_slice(&[0u8; 64]);
        // Append GeoPackage Binary Header: "GP", version=0, flags=0x03 (little-endian + XY envelope), srs_id=4326
        gpkg_bytes.extend_from_slice(b"GP");
        gpkg_bytes.push(0);
        gpkg_bytes.push(0x03);
        gpkg_bytes.extend_from_slice(&4326_i32.to_le_bytes());
        // Envelope [min_x, max_x, min_y, max_y]
        for v in [100.520_f64, 100.535_f64, 13.720_f64, 13.732_f64] {
            gpkg_bytes.extend_from_slice(&v.to_le_bytes());
        }
        // Append ISO WKB Polygon (little-endian=1, type=3, num_rings=1, num_pts=5)
        gpkg_bytes.push(1);
        gpkg_bytes.extend_from_slice(&3_u32.to_le_bytes());
        gpkg_bytes.extend_from_slice(&1_u32.to_le_bytes());
        gpkg_bytes.extend_from_slice(&5_u32.to_le_bytes());
        for (x, y) in [
            (100.520_f64, 13.720_f64),
            (100.535_f64, 13.720_f64),
            (100.535_f64, 13.732_f64),
            (100.520_f64, 13.732_f64),
            (100.520_f64, 13.720_f64),
        ] {
            gpkg_bytes.extend_from_slice(&x.to_le_bytes());
            gpkg_bytes.extend_from_slice(&y.to_le_bytes());
        }

        let (rings, srid) = extract_gpkg_rings_and_srid(&gpkg_bytes).unwrap();
        assert_eq!(srid, Some(4326));
        assert!(!rings.is_empty());
        assert!(rings[0].len() >= 4);
    }
}
