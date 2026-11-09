"""
MapInfo MIF/MID Reader and Writer
Reads and writes MapInfo Interchange format (.mif and .mid).
Directly parses DepthmapX Segment and Axial analysis export files.
"""

import os
import csv
from typing import List, Dict, Any, Optional
from shapely.geometry import LineString


def read_mif_mid(mif_path: str, mid_path: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Parses MapInfo MIF/MID pair into a list of feature dictionaries.
    Each feature dict contains:
        {"geometry": LineString, "properties": Dict[str, Any]}
    """
    if mid_path is None:
        mid_path = os.path.splitext(mif_path)[0] + ".mid"

    if not os.path.exists(mif_path):
        raise FileNotFoundError(f"MIF file not found: {mif_path}")
    if not os.path.exists(mid_path):
        raise FileNotFoundError(f"MID file not found: {mid_path}")

    # 1. Parse MIF Header and Geometries
    columns = []
    geometries: List[LineString] = []

    with open(mif_path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()

    in_data = False
    in_columns = False
    col_count = 0

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if not line:
            i += 1
            continue

        lower_line = line.lower()
        if lower_line.startswith("columns"):
            parts = line.split()
            col_count = int(parts[1]) if len(parts) > 1 else 0
            in_columns = True
            i += 1
            continue

        if in_columns:
            if col_count > 0:
                parts = line.split()
                if parts:
                    col_name = parts[0]
                    col_type = parts[1] if len(parts) > 1 else "char(254)"
                    columns.append((col_name, col_type))
                    col_count -= 1
                    if col_count == 0:
                        in_columns = False
            i += 1
            continue

        if lower_line == "data":
            in_data = True
            i += 1
            continue

        if in_data:
            if lower_line.startswith("line "):
                parts = line.split()
                if len(parts) >= 5:
                    x1, y1, x2, y2 = map(float, parts[1:5])
                    geometries.append(LineString([(x1, y1), (x2, y2)]))
            elif lower_line.startswith("pline"):
                parts = line.split()
                num_pts = int(parts[1]) if len(parts) > 1 else 2
                pts = []
                for _ in range(num_pts):
                    i += 1
                    pt_parts = lines[i].strip().split()
                    if len(pt_parts) >= 2:
                        pts.append((float(pt_parts[0]), float(pt_parts[1])))
                if len(pts) >= 2:
                    geometries.append(LineString(pts))
        i += 1

    # 2. Parse MID Attributes
    col_names = [c[0] for c in columns]
    rows = []
    with open(mid_path, "r", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            prop_dict = {}
            for col_idx, col_name in enumerate(col_names):
                val = row[col_idx] if col_idx < len(row) else None
                if val is not None:
                    try:
                        if "." in val:
                            val = float(val)
                        else:
                            val = int(val)
                    except ValueError:
                        pass
                prop_dict[col_name] = val
            rows.append(prop_dict)

    # 3. Combine
    features = []
    num_feats = min(len(geometries), len(rows))
    for idx in range(num_feats):
        features.append({
            "geometry": geometries[idx],
            "properties": rows[idx]
        })

    return features


def write_mif_mid(
    features: List[Dict[str, Any]],
    out_mif_path: str,
    coord_sys_clause: str = 'CoordSys Earth Projection 8, 104, "m", 99, 0, 0.9996, 500000, 0'
) -> None:
    """
    Writes geometries and properties to MapInfo MIF/MID files.
    """
    out_mid_path = os.path.splitext(out_mif_path)[0] + ".mid"
    os.makedirs(os.path.dirname(os.path.abspath(out_mif_path)), exist_ok=True)

    if not features:
        return

    first_props = features[0].get("properties", {})
    columns = []
    for k, v in first_props.items():
        if isinstance(v, float):
            columns.append((k, "Float"))
        elif isinstance(v, int):
            columns.append((k, "Integer"))
        else:
            columns.append((k, "Char(254)"))

    # Write MIF
    mif_lines = [
        "Version 300",
        "Charset \"WindowsLatin1\"",
        "Delimiter \",\"",
        f"{coord_sys_clause}",
        f"Columns {len(columns)}"
    ]
    for col_name, col_type in columns:
        mif_lines.append(f"  {col_name} {col_type}")
    mif_lines.append("Data\n")

    for feat in features:
        geom = feat["geometry"]
        coords = list(geom.coords)
        if len(coords) == 2:
            mif_lines.append(f"Line {coords[0][0]:.4f} {coords[0][1]:.4f} {coords[1][0]:.4f} {coords[1][1]:.4f}")
        else:
            mif_lines.append(f"Pline {len(coords)}")
            for pt in coords:
                mif_lines.append(f"{pt[0]:.4f} {pt[1]:.4f}")

    temp_mif = out_mif_path + ".tmp"
    with open(temp_mif, "w", encoding="ascii", errors="replace") as f:
        f.write("\n".join(mif_lines))

    # Write MID
    temp_mid = out_mid_path + ".tmp"
    with open(temp_mid, "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        for feat in features:
            props = feat.get("properties", {})
            row = [props.get(col_name, "") for col_name, _ in columns]
            writer.writerow(row)

    os.replace(temp_mif, out_mif_path)
    os.replace(temp_mid, out_mid_path)
