# SSE — Space Syntax Engine (`v1.1.1`) · Standalone Windows Binaries & QGIS Plugin Archive

This directory contains the pre-compiled **Standalone Native Rust Windows Executables (`.exe`)** and the official **QGIS 3.22–4.x (`Qt6 Ready`) Plugin Archive (`.zip`)** for **SSE — Space Syntax Engine (`v1.1.1`)**.

| Binary / Archive | Size | Role & Capabilities |
| :--- | :---: | :--- |
| **[`sse-desktop.exe`](sse-desktop.exe)** | `9.41 MB` | **Standalone Interactive Space Syntax Workstation GUI (`egui` / `eframe`)** — Zero-dependency desktop application supporting all 5 urban street network algorithms (`5-Step OSM Cleaner`, `Tulip-1024 ASA`, `Origin Step Depth`, `Topological Axial`, `Natural Roads`) **plus** high-speed architectural visibility (`VGA`, `2D Benedikt Isovists`, `60 FPS Gibson Visual Agents`), interactive `Intelligibility / Synergy R²` scatter plots, and live MCP TCP control (`127.0.0.1:9884`). |
| **[`sse-cli.exe`](sse-cli.exe)** | `1.81 MB` | **Headless Batch Solver & 9-Suite Mathematical Parity Runner** — Command-line engine for batch urban/architectural processing and automated `UCL depthmapX v0.8.0` mathematical parity benchmarks (`R² = 1.00000000`). |
| **[`sse-mcp.exe`](sse-mcp.exe)** | `4.22 MB` | **9-Tool JSON-RPC `2.0` Model Context Protocol (MCP) Server** — Connects AI coding and spatial analysis agents directly to the Rust CSR solvers and the live `sse-desktop.exe` viewport over stdio + TCP (`127.0.0.1:9884`). |
| **[`sse_v1.1.1.zip`](sse_v1.1.1.zip)** | `248 KB` | **Official QGIS 3.22–4.x (`Qt5` / `Qt6 Ready`) Street Network Plugin** — Install via **QGIS → Plugins → Manage and Install Plugins → Install from ZIP** (or directly from the official [QGIS Plugin Repository (`plugins.qgis.org/plugins/sse/`)](https://plugins.qgis.org/plugins/sse/)). |

---

## Quick Launch Commands

```powershell
# 1. Launch the standalone desktop workstation GUI
.\standalone\sse-desktop.exe

# 2. Run the 9-suite UCL depthmapX v0.8.0 mathematical parity benchmark
.\standalone\sse-cli.exe benchmark

# 3. Start the 9-tool JSON-RPC 2.0 MCP server (stdio)
.\standalone\sse-mcp.exe
```
