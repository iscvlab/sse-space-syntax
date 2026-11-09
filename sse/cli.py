"""
SSE — Space Syntax Engine (SSE) - CLI Runner
Author: Napat (Tan) Phasundhiae

Usage:
    sse clean --input <file> --output <file> [--snap 3.0] [--rungs]
    sse export-depthmapx --input <file> --out-dxf <file> [--out-mif <file>]
    sse solve --input <file> --output <file> [--radii 800,2000,5000] [--include-global]
    sse import-depthmapx --mif <file> --output <file>
"""

import sys
import time
import argparse

if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from .core.projection import reproject_geometry  # noqa: E402
from .core.snapping import snap_endpoints_rtree  # noqa: E402
from .core.dual_carriageway import synthesize_cross_rungs  # noqa: E402
from .core.topology import planarize_and_node, build_angular_dual_graph  # noqa: E402
from .core.solver import solve_space_syntax_metrics  # noqa: E402
from .io_handlers.dxf_exporter import export_dxf_lines  # noqa: E402
from .io_handlers.mif_io import read_mif_mid, write_mif_mid  # noqa: E402
from .io_handlers.geojson_io import read_geojson, write_geojson  # noqa: E402


def cmd_clean(args):
    """Watertight cleaning and planarization."""
    print(f"[*] Loading raw network from {args.input}...")
    t0 = time.time()
    raw_feats = read_geojson(args.input)
    print(f"[OK] Loaded {len(raw_feats):,} features in {time.time() - t0:.2f}s.")

    # Reproject to UTM Zone 47N if in WGS84
    lines_utm = []
    for f in raw_feats:
        geom = f["geometry"]
        coords = list(geom.coords)
        if coords[0][0] < 180.0:  # Longitude in degrees
            geom_utm = reproject_geometry(geom, 4326, 32647)
        else:
            geom_utm = geom
        lines_utm.append(geom_utm)

    print(f"[*] Snapping endpoints within {args.snap}m tolerance...")
    t_snap = time.time()
    healed_lines = snap_endpoints_rtree(lines_utm, tolerance_m=args.snap)
    print(f"[OK] Snapped in {time.time() - t_snap:.2f}s.")

    if args.rungs:
        print("[*] Synthesizing dual-carriageway transverse pedestrian rungs...")
        t_rungs = time.time()
        rungs = synthesize_cross_rungs(healed_lines)
        healed_lines.extend(rungs)
        print(f"[OK] Added {len(rungs):,} transverse connectors in {time.time() - t_rungs:.2f}s.")

    print("[*] Planarizing and noding all intersections...")
    t_node = time.time()
    noded_lines = planarize_and_node(healed_lines)
    print(f"[OK] Split into {len(noded_lines):,} watertight segments in {time.time() - t_node:.2f}s.")

    # Convert back to output CRS
    out_crs = args.crs
    final_lines = []
    for line in noded_lines:
        if out_crs == 4326:
            final_lines.append(reproject_geometry(line, 32647, 4326))
        else:
            final_lines.append(line)

    out_feats = [{"geometry": g, "properties": {"segment_id": i}} for i, g in enumerate(final_lines)]
    crs_urn = "urn:ogc:def:crs:OGC:1.3:CRS84" if out_crs == 4326 else "EPSG:32647"
    write_geojson(out_feats, args.output, crs_name=crs_urn)
    print(f"[SUCCESS] Exported watertight network to {args.output} ({len(final_lines):,} features).")


def cmd_solve(args):
    """Direct high-performance Space Syntax solver."""
    print(f"[*] Loading network from {args.input}...")
    t0 = time.time()
    raw_feats = read_geojson(args.input)
    print(f"[OK] Loaded {len(raw_feats):,} features in {time.time() - t0:.2f}s.")

    # Ensure UTM coordinates for metric calculations
    lines_utm = []
    for f in raw_feats:
        geom = f["geometry"]
        coords = list(geom.coords)
        if coords[0][0] < 180.0:
            lines_utm.append(reproject_geometry(geom, 4326, 32647))
        else:
            lines_utm.append(geom)

    # Planarize if not already planarized
    noded_lines = planarize_and_node(lines_utm)
    print(f"[*] Topologically verified: {len(noded_lines):,} noded segments.")

    # Build Angular Dual Graph
    G, seg_records, stats = build_angular_dual_graph(noded_lines)
    print(f"[OK] Dual graph: {stats['num_segments']:,} segments, {stats['num_graph_edges']:,} edges.")
    print(
        f"    Main component connectivity: {stats['main_component_pct']}% "
        f"({stats['main_component_size']:,} segments)."
    )

    # Parse Radii
    radii = [float(r.strip()) for r in args.radii.split(",") if r.strip()]
    print(f"[*] Solving metric Space Syntax for radii: {radii} (Include Global: {args.include_global})...")

    t_solve = time.time()
    results = solve_space_syntax_metrics(G, metric_radii=radii, include_global=args.include_global)
    print(f"[OK] Solved all metrics in {time.time() - t_solve:.2f}s.")

    # Assign attributes
    out_feats = []
    primary_r = int(radii[0]) if radii else 800

    for i, rec in enumerate(seg_records):
        geom_wgs = reproject_geometry(rec["geom"], 32647, 4326)

        props = {
            "segment_id": i,
            "length_m": round(rec["length_m"], 2),
            "space_syntax_nach": results.get(f"nach_r{primary_r}", {}).get(i, 0.55),
            "space_syntax_nain": results.get(f"nain_r{primary_r}", {}).get(i, 0.60),
        }

        for metric_name, val_map in results.items():
            if metric_name.startswith("nach_") or metric_name.startswith("nain_"):
                props[metric_name] = val_map.get(i, 0.0)
            elif metric_name.startswith("choice_") or metric_name.startswith("integration_"):
                props[metric_name] = round(val_map.get(i, 0.0), 1)

        out_feats.append({"geometry": geom_wgs, "properties": props})

    write_geojson(out_feats, args.output, crs_name="urn:ogc:def:crs:OGC:1.3:CRS84")
    print(f"[SUCCESS] Exported analyzed Space Syntax layer to {args.output} ({len(out_feats):,} features)!")


def cmd_export_depthmapx(args):
    """Export watertight AC1018 DXF and MapInfo MIF/MID for DepthmapX import."""
    print(f"[*] Loading network from {args.input}...")
    t0 = time.time()
    raw_feats = read_geojson(args.input)
    print(f"[OK] Loaded {len(raw_feats):,} features in {time.time() - t0:.2f}s.")

    lines_utm = []
    mif_feats = []
    for f in raw_feats:
        geom = f["geometry"]
        coords = list(geom.coords)
        geom_u = reproject_geometry(geom, 4326, 32647) if coords[0][0] < 180.0 else geom
        lines_utm.append(geom_u)
        mif_feats.append({"geometry": geom_u, "properties": f.get("properties", {})})

    # Export DXF
    print(f"[*] Exporting AutoCAD R2004 (AC1018) DXF to {args.out_dxf}...")
    export_dxf_lines(lines_utm, args.out_dxf)
    print(f"[SUCCESS] DXF written: {args.out_dxf}")

    if args.out_mif:
        print(f"[*] Exporting MapInfo MIF/MID to {args.out_mif}...")
        write_mif_mid(mif_feats, args.out_mif)
        print(f"[SUCCESS] MIF/MID written: {args.out_mif}")


def cmd_import_depthmapx(args):
    """Import DepthmapX exported MIF/MID back into GeoJSON."""
    print(f"[*] Reading DepthmapX export from {args.mif}...")
    mif_features = read_mif_mid(args.mif)
    print(f"[OK] Read {len(mif_features):,} features.")

    out_feats = []
    for feat in mif_features:
        geom_wgs = reproject_geometry(feat["geometry"], 32647, 4326)
        out_feats.append({"geometry": geom_wgs, "properties": feat.get("properties", {})})

    write_geojson(out_feats, args.output, crs_name="urn:ogc:def:crs:OGC:1.3:CRS84")
    print(f"[SUCCESS] Exported {len(out_feats):,} analyzed features to {args.output}.")


def main():
    parser = argparse.ArgumentParser(description="SSE — Space Syntax Engine CLI")
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # clean
    clean_p = subparsers.add_parser("clean", help="Watertight snapping, dual-carriageway rungs, and planarization")
    clean_p.add_argument("--input", required=True, help="Input raw GeoJSON path")
    clean_p.add_argument("--output", required=True, help="Output clean GeoJSON path")
    clean_p.add_argument("--snap", type=float, default=3.0, help="Snapping tolerance in meters")
    clean_p.add_argument("--rungs", action="store_true", help="Synthesize dual-carriageway rungs")
    clean_p.add_argument("--crs", type=int, default=4326, help="Output EPSG code (4326 or 32647)")

    # export-depthmapx
    dxf_p = subparsers.add_parser("export-depthmapx", help="Export watertight AC1018 DXF and MapInfo MIF/MID")
    dxf_p.add_argument("--input", required=True, help="Input clean GeoJSON path")
    dxf_p.add_argument("--out-dxf", required=True, help="Output DXF path")
    dxf_p.add_argument("--out-mif", help="Output MIF path")

    # solve
    solve_p = subparsers.add_parser("solve", help="High-performance metric Choice & Integration solver")
    solve_p.add_argument("--input", required=True, help="Input clean GeoJSON path")
    solve_p.add_argument("--output", required=True, help="Output analyzed GeoJSON path")
    solve_p.add_argument("--radii", default="800,2000", help="Comma-separated metric radii in meters")
    solve_p.add_argument("--include-global", action="store_true", help="Include global Rn through-movement")

    # import-depthmapx
    imp_p = subparsers.add_parser("import-depthmapx", help="Import DepthmapX MIF/MID results into GeoJSON")
    imp_p.add_argument("--mif", required=True, help="Input MIF file path")
    imp_p.add_argument("--output", required=True, help="Output GeoJSON path")

    args = parser.parse_args()
    if args.command == "clean":
        cmd_clean(args)
    elif args.command == "export-depthmapx":
        cmd_export_depthmapx(args)
    elif args.command == "solve":
        cmd_solve(args)
    elif args.command == "import-depthmapx":
        cmd_import_depthmapx(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
