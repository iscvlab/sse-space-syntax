# -*- coding: utf-8 -*-
"""
OpenDepthmap Local Workstation Web Server & Live CSR Space Syntax API
Serves the static web GUI workstation on http://localhost:5173 and exposes:
- GET  /api/health  -> Engine status probe
- POST /api/analyze -> Live Python CSR pipeline (cleaner -> topology -> CSRDualGraph -> ASA solver)
"""

import argparse
import http.server
import json
import math
import os
import socketserver
import sys
import time
import urllib.parse
from typing import Any, Dict, List

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from shapely.geometry import LineString  # noqa: E402

from core.cleaner import clean_network_adaptive_watertight  # noqa: E402
from core.graph import CSRDualGraph  # noqa: E402
from core.solvers_segment import (  # noqa: E402
    compute_network_summary_diagnostics,
    solve_angular_segment_analysis,
)
from core.topology import explode_to_2point_segments, planarize_and_node  # noqa: E402

PORT = 5173
ENGINE_NAME = "OpenDepthmap CSR v1.1.0"


def _parse_radii_payload(raw_radii: Any) -> List[float]:
    """Parses list or comma-separated radii into sorted metric radii with 'rn' (inf) & 800m guaranteed."""
    tokens: List[Any] = []
    if isinstance(raw_radii, str):
        tokens = [t.strip() for t in raw_radii.split(",") if t.strip()]
    elif isinstance(raw_radii, (list, tuple)):
        tokens = list(raw_radii)
    else:
        tokens = ["n", 800, 400]

    parsed: List[float] = []
    has_inf = False
    for tok in tokens:
        s_tok = str(tok).strip().lower()
        if s_tok in ("n", "rn", "inf", "infinity", "global", "all"):
            has_inf = True
        else:
            try:
                val = float(tok)
                if math.isinf(val) or val >= 1e8:
                    has_inf = True
                elif val > 0.0:
                    parsed.append(val)
            except (TypeError, ValueError):
                continue

    # Always guarantee 800m and global 'rn' so standard scorecard keys exist
    if not any(abs(r - 800.0) < 1e-3 for r in parsed):
        parsed.append(800.0)
    has_inf = True

    unique_finite = sorted(set(round(r, 3) for r in parsed))
    if has_inf:
        unique_finite.append(float("inf"))
    return unique_finite


def _parse_input_lines(raw_lines: Any) -> List[LineString]:
    """Builds validated Shapely LineString objects from coordinate arrays or segment dicts."""
    if not isinstance(raw_lines, list):
        return []

    lines: List[LineString] = []
    for item in raw_lines:
        coords = None
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            coords = item
        elif isinstance(item, dict):
            if "coords" in item and isinstance(item["coords"], (list, tuple)):
                coords = item["coords"]
            elif all(k in item for k in ("x1", "y1", "x2", "y2")):
                coords = [[item["x1"], item["y1"]], [item["x2"], item["y2"]]]

        if coords is None or len(coords) < 2:
            continue

        try:
            pts = [(float(pt[0]), float(pt[1])) for pt in coords if len(pt) >= 2]
            if len(pts) >= 2:
                line = LineString(pts)
                if not line.is_empty and line.length > 1e-4:
                    lines.append(line)
        except (TypeError, ValueError, IndexError):
            continue

    return lines


def run_csr_analysis_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Executes the full OpenDepthmap CSR pipeline on a parsed JSON request payload:
    1. Build Shapely LineStrings from payload['lines'].
    2. Run clean_network_adaptive_watertight when clean_mode in ('auto', 'manual').
    3. Explode to 2-point segments via explode_to_2point_segments.
    4. Construct CSRDualGraph via CSRDualGraph.from_segments(..., tulip_bins=tulip_bins).
    5. Solve Angular Segment Analysis via solve_angular_segment_analysis and compute R² diagnostics.
    """
    t0 = time.perf_counter()

    input_lines = _parse_input_lines(payload.get("lines", []))
    if not input_lines:
        raise ValueError("Request payload must contain at least one valid line in 'lines'.")

    clean_mode = str(payload.get("clean_mode", "auto")).strip().lower()
    extend_distance = float(payload.get("extend_distance", 15.0))
    include_normalized = bool(payload.get("include_normalized", True))
    metric_radii = _parse_radii_payload(payload.get("radii", ["n", 800, 400]))

    analysis_type = str(payload.get("analysis_type", "angular")).strip().lower()
    if analysis_type not in ("angular", "metric", "topological"):
        analysis_type = "angular"

    radius_type = str(payload.get("radius_type", "metric")).strip().lower()
    if radius_type not in ("metric", "angular", "topological"):
        radius_type = "metric"

    weight_mode = str(payload.get("weight_mode", "unweighted")).strip().lower()
    if weight_mode not in ("length", "unweighted", "custom"):
        weight_mode = "unweighted"

    try:
        tulip_bins = int(payload.get("tulip_bins", 0))
    except (TypeError, ValueError):
        tulip_bins = 0
    if tulip_bins not in (0, 1024):
        tulip_bins = 1024 if tulip_bins > 0 else 0

    if clean_mode in ("auto", "manual"):
        cleaned_lines, clean_report = clean_network_adaptive_watertight(
            input_lines,
            mode=clean_mode,
            extend_distance=extend_distance,
            run_assertions=False,
        )
        if not cleaned_lines:
            cleaned_lines = input_lines
    else:
        cleaned_lines = planarize_and_node(input_lines)
        if not cleaned_lines:
            cleaned_lines = input_lines
        clean_report = {
            "mode": clean_mode,
            "raw_lines_count": len(input_lines),
            "cleaned_lines_count": len(cleaned_lines),
        }

    segments_geom = explode_to_2point_segments(cleaned_lines)
    if not segments_geom:
        segments_geom = [
            LineString([list(line_obj.coords)[0], list(line_obj.coords)[-1]])
            for line_obj in cleaned_lines
            if len(line_obj.coords) >= 2 and line_obj.length > 1e-4
        ]

    graph, seg_records = CSRDualGraph.from_segments(segments_geom, tulip_bins=tulip_bins)
    asa_results = solve_angular_segment_analysis(
        graph,
        metric_radii=metric_radii,
        include_normalized=include_normalized,
        analysis_type=analysis_type,
        radius_type=radius_type,
        weight_mode=weight_mode,
    )
    diagnostics = compute_network_summary_diagnostics(graph, asa_results)

    num_seg = int(graph.num_nodes)
    segments_out: List[Dict[str, Any]] = []

    conn_arr = asa_results.get("connectivity", graph.connectivity)
    md_rn_arr = asa_results.get("mean_depth_rn")

    for i in range(num_seg):
        rec = seg_records[i]
        coords = list(rec["geom"].coords)
        p_start = [round(float(coords[0][0]), 3), round(float(coords[0][1]), 3)]
        p_end = [round(float(coords[-1][0]), 3), round(float(coords[-1][1]), 3)]

        md_val = float(md_rn_arr[i]) if md_rn_arr is not None else 0.0

        seg_item: Dict[str, Any] = {
            "id": i,
            "coords": [p_start, p_end],
            "length_m": round(float(rec["length_m"]), 3),
            "connectivity": int(conn_arr[i]) if conn_arr is not None else 0,
            "mean_depth": md_val,
            "mean_depth_rn": md_val,
            "nach_rn": float(asa_results["nach_rn"][i]),
            "nain_rn": float(asa_results["nain_rn"][i]),
            "choice_rn": float(asa_results["choice_rn"][i]),
            "integration_rn": float(asa_results["integration_rn"][i]),
            "nach_r800": float(asa_results["nach_r800"][i]),
            "nain_r800": float(asa_results["nain_r800"][i]),
        }

        for key, arr in asa_results.items():
            if key not in seg_item:
                if key == "connectivity" or key.startswith("node_count_"):
                    seg_item[key] = int(arr[i])
                else:
                    seg_item[key] = float(arr[i])

        segments_out.append(seg_item)

    elapsed_ms = round((time.perf_counter() - t0) * 1000.0, 2)
    graph_ram_kb = round(graph.memory_footprint_bytes() / 1024.0, 2)

    return {
        "status": "ok",
        "engine": ENGINE_NAME,
        "analysis_type": analysis_type,
        "radius_type": radius_type,
        "weight_mode": weight_mode,
        "tulip_bins": tulip_bins,
        "segment_count": num_seg,
        "node_count": num_seg,
        "edge_count": int(len(graph.target_nodes)),
        "graph_ram_kb": graph_ram_kb,
        "elapsed_ms": elapsed_ms,
        "clean_report": clean_report,
        "diagnostics": {
            "intelligibility_r2": diagnostics.get("intelligibility_r2", 0.0),
            "synergy_r2": diagnostics.get("synergy_r2", {}),
        },
        "segments": segments_out,
    }


class Handler(http.server.SimpleHTTPRequestHandler):
    """HTTP Request Handler serving both static workstation assets and CSR JSON endpoints."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=SCRIPT_DIR, **kwargs)

    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        super().end_headers()

    def _send_json(self, status_code: int, payload: Dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.end_headers()

    def do_GET(self) -> None:
        parsed_path = urllib.parse.urlparse(self.path).path
        if parsed_path == "/api/health":
            self._send_json(200, {"status": "ok", "engine": ENGINE_NAME})
            return
        super().do_GET()

    def do_POST(self) -> None:
        parsed_path = urllib.parse.urlparse(self.path).path
        if parsed_path != "/api/analyze":
            self._send_json(404, {"status": "error", "message": f"Not found: {parsed_path}"})
            return

        try:
            content_len = int(self.headers.get("Content-Length", "0"))
            raw_body = self.rfile.read(content_len) if content_len > 0 else b"{}"
            payload = json.loads(raw_body.decode("utf-8"))
            response_data = run_csr_analysis_payload(payload)
            self._send_json(200, response_data)
        except ValueError as exc:
            self._send_json(400, {"status": "error", "message": str(exc)})
        except Exception as exc:
            self._send_json(500, {"status": "error", "message": str(exc)})


def start_server(port: int = PORT) -> None:
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", port), Handler) as httpd:
        print("=" * 70)
        print("  OPENDEPTHMAP (OSSE PRO) — LOCAL WORKSTATION WEB SERVER")
        print(f"  URL: http://localhost:{port}")
        print(f"  API: http://localhost:{port}/api/health | POST /api/analyze")
        print("=" * 70)
        print("  Press Ctrl+C to stop the workstation server.\n")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nWorkstation server stopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OpenDepthmap Local Workstation Server")
    parser.add_argument("--port", type=int, default=PORT, help="Port to bind HTTP server (default: 5173)")
    args = parser.parse_args()
    start_server(port=args.port)
