# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) IO: MapInfo MIF / MID Parser and Serializer
Provides full read and write compatibility with DepthmapX native .mif / .mid files.
Handles:
1. Parsing MIF header (Columns, CoordSys) and LINE coordinate geometries.
2. Parsing MID comma/tab-delimited attribute records.
3. Serializing analyzed spatial graphs to standard MIF/MID for legacy workflows.
"""

import os
from typing import List, Tuple
import pandas as pd
from shapely.geometry import LineString


def read_mif_lines(mif_path: str) -> Tuple[List[LineString], List[str], pd.DataFrame]:
    """
    Reads MapInfo .mif geometry and companion .mid attribute file.
    Returns:
        (lines, column_names, dataframe_of_attributes)
    """
    if not os.path.exists(mif_path):
        raise FileNotFoundError(f"MIF file not found: {mif_path}")

    mid_path = os.path.splitext(mif_path)[0] + ".mid"
    if not os.path.exists(mid_path):
        mid_path = os.path.splitext(mif_path)[0] + ".MID"

    columns = []
    lines: List[LineString] = []

    with open(mif_path, "r", encoding="utf-8", errors="replace") as f:
        in_columns = False
        in_data = False
        num_cols = 0

        for raw_line in f:
            line_str = raw_line.strip()
            if not line_str:
                continue

            upper = line_str.upper()

            if upper.startswith("COLUMNS"):
                parts = line_str.split()
                if len(parts) >= 2:
                    num_cols = int(parts[1])
                in_columns = True
                continue

            if in_columns:
                if upper == "DATA":
                    in_columns = False
                    in_data = True
                    continue
                parts = line_str.split()
                if len(parts) >= 2:
                    col_name = parts[0]
                    columns.append(col_name)
                    if len(columns) == num_cols:
                        in_columns = False
                continue

            if upper == "DATA":
                in_data = True
                continue

            if in_data:
                if upper.startswith("LINE"):
                    parts = line_str.split()
                    if len(parts) >= 5:
                        try:
                            x1, y1 = float(parts[1]), float(parts[2])
                            x2, y2 = float(parts[3]), float(parts[4])
                            if (x1, y1) != (x2, y2):
                                lines.append(LineString([(x1, y1), (x2, y2)]))
                        except ValueError:
                            pass

    # Read accompanying MID file
    if os.path.exists(mid_path):
        try:
            df = pd.read_csv(mid_path, header=None, names=columns if columns else None)
        except (OSError, pd.errors.ParserError, ValueError):
            df = pd.DataFrame()
    else:
        df = pd.DataFrame()

    return lines, columns, df


def write_mif_lines(
    lines: List[LineString],
    attributes_df: pd.DataFrame,
    output_mif_path: str,
    coordsys_str: str = 'CoordSys NonEarth Units "m"'
):
    """
    Writes spatial lines and attributes to MapInfo MIF and MID files.
    Matches DepthmapX's exact specification.
    """
    os.makedirs(os.path.dirname(os.path.abspath(output_mif_path)), exist_ok=True)
    mid_path = os.path.splitext(output_mif_path)[0] + ".mid"

    # Compute bounding box
    minx, miny, maxx, maxy = float("inf"), float("inf"), float("-inf"), float("-inf")
    for ln in lines:
        b = ln.bounds
        minx = min(minx, b[0])
        miny = min(miny, b[1])
        maxx = max(maxx, b[2])
        maxy = max(maxy, b[3])

    if minx == float("inf"):
        minx, miny, maxx, maxy = 0.0, 0.0, 1000.0, 1000.0

    cols = list(attributes_df.columns)

    # Write .mif file
    with open(output_mif_path, "w", encoding="utf-8") as f:
        f.write("Version 300\n")
        f.write('Charset "WindowsLatin1"\n')
        f.write('Delimiter ","\n')
        f.write("Index 1\n")
        f.write(f'{coordsys_str} Bounds ({minx:.4f}, {miny:.4f}) ({maxx:.4f}, {maxy:.4f})\n')
        f.write(f"Columns {len(cols)}\n")

        for c in cols:
            dtype = attributes_df[c].dtype
            if pd.api.types.is_integer_dtype(dtype):
                col_type = "Integer"
            elif pd.api.types.is_float_dtype(dtype):
                col_type = "Float"
            else:
                col_type = "Char(64)"
            f.write(f"  {c} {col_type}\n")

        f.write("Data\n\n")

        for ln in lines:
            coords = list(ln.coords)
            p1 = coords[0]
            p2 = coords[-1]
            f.write(f"LINE {p1[0]:.4f} {p1[1]:.4f} {p2[0]:.4f} {p2[1]:.4f}\n")
            f.write("    PEN (1,2,0)\n")

    # Write .mid file
    attributes_df.to_csv(mid_path, header=False, index=False)
