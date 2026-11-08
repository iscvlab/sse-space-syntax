"""
OSSE Space Syntax Example: Synthetic Grid & Irregular Urban Network
Demonstrates end-to-end workflow:
1. Creating or importing network lines
2. Snapping endpoints and healing micro-gaps
3. Planarizing and noding segment intersections
4. Constructing the Angular Dual Graph
5. Solving metric Choice (NACH) and Integration (NAIN) across multiple radii (R400m, R800m, Rn)
6. Exporting to GeoJSON with standardized Space Syntax properties
"""

import os
import sys
from shapely.geometry import LineString

# Ensure osse package is importable when executed directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from osse.core.snapping import snap_endpoints_rtree  # noqa: E402
from osse.core.topology import planarize_and_node, build_angular_dual_graph  # noqa: E402
from osse.core.solver import solve_space_syntax_metrics  # noqa: E402
from osse.io_handlers.geojson_io import write_geojson  # noqa: E402


def main():
    print("=== Open Space Syntax Computational Engine (OSSE) Demo ===")

    # 1. Synthesize a sample urban street network (coordinates in meters)
    print("\n[Step 1] Synthesizing street centerline geometries...")
    streets = [
        # Main Arterials (East-West)
        LineString([(0, 0), (800, 0)]),
        LineString([(0, 400), (800, 400)]),
        LineString([(0, 800), (800, 800)]),

        # Main Arterials (North-South)
        LineString([(0, 0), (0, 800)]),
        LineString([(400, 0), (400, 800)]),
        LineString([(800, 0), (800, 800)]),

        # Diagonal Through-Corridor (High choice spine)
        LineString([(0, 0), (800, 800)]),

        # Secondary Local Streets
        LineString([(200, 0), (200, 400)]),
        LineString([(600, 400), (600, 800)]),
        LineString([(0, 200), (400, 200)]),
        LineString([(400, 600), (800, 600)]),

        # Cul-de-sac / Dead-end street (low integration & choice)
        LineString([(200, 200), (200, 320)]),
    ]
    print(f"  Synthesized {len(streets)} raw street centerlines.")

    # 2. Watertight Snapping
    print("\n[Step 2] Snapping endpoints (tolerance = 2.0m)...")
    healed_streets = snap_endpoints_rtree(streets, tolerance_m=2.0)
    print(f"  Snapped {len(healed_streets)} lines.")

    # 3. Planarize and node intersections
    print("\n[Step 3] Planarizing and noding intersections...")
    noded_segments = planarize_and_node(healed_streets)
    print(f"  Generated {len(noded_segments)} noded road segments.")

    # 4. Construct Angular Dual Graph
    print("\n[Step 4] Constructing Angular Dual Graph...")
    G, seg_records, stats = build_angular_dual_graph(noded_segments)
    print(f"  Nodes (Segments): {stats['num_segments']}")
    print(f"  Edges (Angular Transitions): {stats['num_graph_edges']}")
    print(f"  Connected Component: {stats['main_component_pct']}% ({stats['main_component_size']} segments)")

    # 5. Solve Space Syntax Metrics
    radii = [400.0, 800.0]
    print(f"\n[Step 5] Solving Space Syntax metrics (Radii = {radii}, Global Rn = True)...")
    results = solve_space_syntax_metrics(G, metric_radii=radii, include_global=True)

    print("\n  Summary of Solved Metrics:")
    for metric_name in results:
        vals = list(results[metric_name].values())
        min_v, max_v, avg_v = min(vals), max(vals), sum(vals) / len(vals)
        print(f"    - {metric_name:20s}: min={min_v:7.2f}, max={max_v:7.2f}, avg={avg_v:7.2f}")

    # 6. Export to GeoJSON
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(out_dir, "sample_network_analyzed.geojson")

    features = []
    for i, rec in enumerate(seg_records):
        props = {
            "segment_id": i,
            "length_m": round(rec["length_m"], 2),
            "nach_r400": results["nach_r400"].get(i, 0.55),
            "nach_r800": results["nach_r800"].get(i, 0.55),
            "nach_rn": results["nach_rn"].get(i, 0.55),
            "nain_r400": results["nain_r400"].get(i, 0.60),
            "nain_r800": results["nain_r800"].get(i, 0.60),
            "nain_rn": results["nain_rn"].get(i, 0.60),
        }
        features.append({"geometry": rec["geom"], "properties": props})

    write_geojson(features, out_file)
    print(f"\n[SUCCESS] Sample analyzed network saved to: {out_file}")


if __name__ == "__main__":
    main()
