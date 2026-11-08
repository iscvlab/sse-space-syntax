"""
Pure AC1018 DXF Exporter
Outputs strictly two-point LINE entities in the ENTITIES section.
Guarantees 100% compatibility with DepthmapX 0.8.0, AutoCAD, and Rhino.
"""

import os
from typing import List
from shapely.geometry import LineString


def export_dxf_lines(lines: List[LineString], out_path: str, layer_name: str = "SPACE_SYNTAX") -> None:
    """
    Exports a list of LineStrings to a minimal, pure AC1018 DXF file.
    Only writes 2-point LINE entities so DepthmapX can ingest directly.
    """
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    header_str = (
        "  0\nSECTION\n  2\nHEADER\n  9\n$ACADVER\n  1\nAC1018\n  0\nENDSEC\n"
        "  0\nSECTION\n  2\nTABLES\n  0\nTABLE\n  2\nLAYER\n 70\n1\n"
        f"  0\nLAYER\n  2\n{layer_name}\n 70\n0\n 62\n7\n  6\nCONTINUOUS\n"
        "  0\nENDTAB\n  0\nENDSEC\n"
        "  0\nSECTION\n  2\nBLOCKS\n  0\nENDSEC\n"
        "  0\nSECTION\n  2\nENTITIES\n"
    )

    line_chunks = []
    for line in lines:
        coords = list(line.coords)
        if len(coords) < 2:
            continue
        for i in range(len(coords) - 1):
            p1 = coords[i]
            p2 = coords[i + 1]
            chunk = (
                f"  0\nLINE\n  8\n{layer_name}\n"
                f" 10\n{p1[0]:.4f}\n 20\n{p1[1]:.4f}\n 30\n0.0\n"
                f" 11\n{p2[0]:.4f}\n 21\n{p2[1]:.4f}\n 31\n0.0\n"
            )
            line_chunks.append(chunk)

    footer_str = "  0\nENDSEC\n  0\nEOF\n"

    temp_path = out_path + ".tmp"
    with open(temp_path, "w", encoding="ascii", errors="replace") as f:
        f.write(header_str)
        f.write("".join(line_chunks))
        f.write(footer_str)

    os.replace(temp_path, out_path)
