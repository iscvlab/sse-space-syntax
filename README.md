# SSE — Space Syntax Engine

[![Release](https://img.shields.io/badge/Release-v1.1.1-121417?style=flat-square)](https://github.com/napattan/sse-space-syntax/releases/tag/v1.1.1)
[![Python 3.9+](https://img.shields.io/badge/Python-3.9%2B_%E2%80%A2_Numba_JIT-2563eb?style=flat-square)](https://www.python.org/downloads/)
[![Rust 1.75+](https://img.shields.io/badge/Rust-1.75%2B_%E2%80%A2_CSR_Core-ea580c?style=flat-square)](https://www.rust-lang.org/)
[![Standalone](https://img.shields.io/badge/Standalone-Windows_.exe_Suite-0d9488?style=flat-square)](standalone/)
[![QGIS 3.22-4.x Qt6 Ready](https://img.shields.io/badge/QGIS-3.22--4.x_Qt6_Ready-16a34a?style=flat-square)](https://plugins.qgis.org/plugins/sse/)
[![Lab](https://img.shields.io/badge/Lab-ISCV_%7C_Isocurve_Research_Lab_%28%40iscvlab%29-121417?style=flat-square&logo=github&logoColor=white)](https://github.com/iscvlab)
[![License: MIT](https://img.shields.io/badge/License-MIT-4f46e5?style=flat-square)](LICENSE)

**SSE — Space Syntax Engine** is an open-source computational graph and spatial visibility engine (**Vectorized Python / Numba JIT / Shapely 2.0 + Native Rust CSR Core**) for urban street network and architectural layout analysis. Engineered as a modern, scriptable replacement for legacy single-threaded C++ `UCL depthmapX v0.8.0`, SSE achieves **100% mathematical parity (`R² = 1.0000`)** at **35×–58× faster multi-core speed** across three modular layers:
1. **Modern Computational Scripts & Core Library ([`sse/`](sse/) & [`src/`](src/))**: Composable Python modules (`sse.core.*`), CLI commands (`sse clean`, `sse solve`), and native Rust Compressed Sparse Row (`CSR`) solvers for programmatic research pipelines.
2. **Standalone Desktop Workstation & Batch CLI ([`standalone/`](standalone/))**: Zero-dependency native Windows executables ([`sse-desktop.exe`](standalone/sse-desktop.exe), [`sse-cli.exe`](standalone/sse-cli.exe), [`sse-mcp.exe`](standalone/sse-mcp.exe)) covering both citywide street networks and architectural visibility (`VGA`, `2D Isovists`, `60 FPS Visual Agents`).
3. **QGIS 3.22–4.x (`Qt6 Ready`) Processing Plugin ([`sse/plugins/qgis_plugin/`](sse/plugins/qgis_plugin/))**: Integrated 5-algorithm street network toolbox ([`standalone/sse_v1.1.1.zip`](standalone/sse_v1.1.1.zip)) with automated symbology and interactive `Intelligibility / Synergy R²` scatter plots.

---

## ⚡ TL;DR — Capabilities & Performance at a Glance

- **Modern Vectorized & CSR Graph Solvers (`sse.core` & `src/lib.rs`)**: Replaces legacy pointer-heavy graph structures with cache-aligned Compressed Sparse Row (`CSRDualGraph`) arrays, `@njit(parallel=True)` Numba kernels, and `rayon` multi-core parallelism.
- **Exact `UCL depthmapX` Mathematical Parity (`R² = 1.0000`)**: Replicates official `depthmapX v0.8.0` outputs across Angular Segment Analysis (`Tulip-1024`), Topological Axial Analysis (`16` Hillier & Hanson metrics), Visibility Graph Analysis (`VGA`), 2D Benedikt Isovists, and Gibson Visual Agents.
- **City-Scale Multi-Core Speed (`35×–58× Faster`)**: Solves `30,493`-segment multi-radius urban networks in `15.98 s` (vs. `14.5 min` in `depthmapX`) and scales to `166,514`-segment metropolitan arterial graphs in `< 16 MB` RAM without UI freezes.
- **Automated 5-Step OpenStreetMap Network Cleaner (`sse.core.cleaner`)**: Heals raw digitizing gaps via empirical `P90` raycasting, preserves bridge/tunnel grade separation, and prunes dead-end micro-stubs prior to spatial graph construction.

### Horizontal Bar Comparison (`0 ms` Render)

| Evaluation Dimension | Engine Configuration | Visual Comparison Bar | Measured Result & Delta vs. Baseline |
| :--- | :--- | :--- | :--- |
| **1. Mathematical Parity vs. `UCL depthmapX v0.8.0` ↑** | **SSE (`Python/Numba Scripts` & `Rust CSR`)**<br>Undirected Graph Approximation | `████████████████████` **`100.0% Exact (R² = 1.0000)`**<br>`███████████████████░` `99.6% (R² = 0.9962)` | **`max\|Δ\| = 0.0000`** (`Barnsbury` `N = 178` & `On Nut` `N = 30,493`)<br>Eliminates junction-bounce artifacts on acute turns |
| **2. Multi-Radius Segment ASA Latency (`N = 30,493`) ↓**<br>*(Lower = Better)* | **SSE (`Rust CSR Core` / `Standalone .exe`)**<br>**SSE (`Python/Numba Scripts` & `QGIS`)**<br>UCL depthmapX v0.8.0 C++ Baseline | `█░░░░░░░░░░░░░░░░░░░` **`15.98 s`**<br>`█░░░░░░░░░░░░░░░░░░░` **`21.08 s (1.86 s Fast)`**<br>`████████████████████` `870.00 s (14.5 min)` | **`54.4× Faster`** (`R800, R1200, R2000, Rn`)<br>**`41.3× Faster`** (`467×` in bounded local-radius mode)<br>`870,000 ms` Single-Threaded GUI Loop |
| **3. Urban Axial & Floorplan VGA Latency ↓**<br>*(Lower = Better)* | **SSE (`Rust CSR Core` Axial / VGA)**<br>**SSE (`Python/Numba Scripts` Axial)**<br>UCL depthmapX v0.8.0 C++ Baseline | `█░░░░░░░░░░░░░░░░░░░` **`1.39 s Axial · 134 ms VGA`**<br>`█░░░░░░░░░░░░░░░░░░░` **`1.82 s Axial`**<br>`████████████████████` `48.50 s Axial · 4,800 ms VGA` | **`34.9×–35.8× Faster`** (`30,493` axial lines & `2,101` VGA cells)<br>**`26.6× Faster`** across `16` Hillier & Hanson axial metrics<br>`48,500 ms` Axial / `4,800 ms` VGA Baseline |
| **4. Metropolis Capacity (`Bangkok`, `N = 166,514`) ↑** | **SSE (`Rust CSR` & `Python/Numba`)**<br>UCL depthmapX v0.8.0 C++ Baseline | `████████████████████` **`100.0% Solved (16.69 s)`**<br>`░░░░░░░░░░░░░░░░░░░░` `0.0% (DNF / Freeze)` | **`< 16 MB` CSR RAM** (`16.69 s` Rust · `21.62 s` Python, `R² = 1.0000`)<br>`> 13.8B` `O(N²)` pairs (Application freeze) |

---

## 📊 Visual Parity & Benchmark Suites (`UCL depthmapX v0.8.0` vs. `SSE`)

| **1. `UCL Barnsbury` Angular Segment Analysis (`Tulip-1024` · `N = 178` · `R² = 1.000000` · `57.8×` Faster)** |
| :---: |
| ![Barnsbury ASA Parity](docs/benchmarks/comparison_01_asa_tulip1024_barnsbury_onnut.png) |

| **2. `Bangkok On Nut` Multi-Radius Segment Analysis (`Tulip-1024` · `N = 30,493` · `R² = 1.000000` · `54.4×` Faster)** |
| :---: |
| ![On Nut ASA Parity](docs/benchmarks/comparison_02_axial_onnut_30493.png) |

| **3. `Bangkok On Nut` Topological Axial Analysis (`16` Hillier Metrics · `N = 30,493` · `R² = 1.000000` · `34.9×` Faster)** |
| :---: |
| ![On Nut Axial Parity](docs/benchmarks/comparison_03_vga_isovists_onnut_gallery.png) |

| **4. `Bangkok Metropolis` Citywide Arterials (`Tulip-1024` CSR · `N = 166,514` · `R² = 1.000000` · `< 16 MB` RAM)** |
| :---: |
| ![Bangkok Citywide Arterials Parity](docs/benchmarks/comparison_04_benchmark_summary_and_agents.png) |

| **5. `Architectural Spatial Visibility` (`VGA + 2D Isovists + Gibson Visual Agents` · `N = 2,101` · `R² = 1.000000` · `35.8×` Faster)** |
| :---: |
| ![Architectural Spatial Visibility Parity](docs/benchmarks/comparison_05_spatial_vga_isovists_agents.png) |

### 3-Way Execution Speed & Mathematical Parity Summary

| Benchmark Suite & Empirical Dataset | Scale (`N`) | `UCL depthmapX v0.8.0` | `SSE Python / QGIS` | `SSE Rust CSR Core` | Speedup (`Python / Rust`) | Parity (`R²` & `max\|Δ\|`) |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **1. UCL Barnsbury Segment ASA (`Tulip-1024`)** | `178` segs (`R400 + Rn`) | `48.00 ms` | **`0.85 ms`** | **`0.83 ms`** | **`56.5×` / `57.8×`** | **`R² = 1.000000`** (`0.0000`) |
| **2. Bangkok On Nut Multi-Radius ASA (`Tulip-1024`)** | `30,493` segs (`R800..Rn`) | `870.00 s` | **`21.08 s`** (`1.86 s` Fast) | **`15.98 s`** | **`41.3×` / `54.4×`** | **`R² = 1.000000`** (`0.0000`) |
| **3. Bangkok On Nut Topological Axial (`16` Metrics)** | `30,493` lines (`Rn, R3, R5`) | `48.50 s` | **`1.82 s`** | **`1.39 s`** | **`26.6×` / `34.9×`** | **`R² = 1.000000`** (`0.0000`) |
| **4. Bangkok Metropolis Citywide Arterials (`CSR`)** | `166,514` segs (`R800..Rn`) | `DNF / Freeze` | **`21.62 s`** (`6.12 s` R2k) | **`16.69 s`** (`4.49 s` R2k) | **`1.29×` (`Rust/Py`)** | **`R² = 1.000000`** (`< 16 MB`) |
| **5. Architectural Spatial Visibility (`VGA + Isovists + Agents`)** | `2,101` cells (`1.04M` edges) | `4,800.0 ms` | **`574.0 ms`** *(Python Core)* | **`134.14 ms`** | **`35.8×` (`Rust CSR`)** | **`R² = 1.000000`** (`0.0001`) |

---

## 1. Architecture & Core Computational Modules

SSE is organized into three progressive execution tiers—starting from **Modern Computational Scripts & Core Solvers** down to desktop and GIS host integrations:

1. **Modern Computational Scripts & Core Engine ([`sse/core/`](sse/core/), [`sse/cli.py`](sse/cli.py) & [`src/lib.rs`](src/lib.rs))**:
   - Pure, scriptable Python & Numba JIT modules (`cleaner.py`, `graph.py`, `solvers_segment.py`, `solvers_axial.py`, `solvers_natural_roads.py`, `solvers_vga.py`, `solvers_agents.py`, `osm_downloader.py`) paired with the native Rust CSR library (`src/lib.rs`) and GeoJSON/GeoPackage/DXF/MIF I/O handlers (`sse/io_handlers/`).
2. **Standalone Native Rust Workstation & Batch CLI ([`standalone/`](standalone/))**:
   - Zero-dependency desktop workstation GUI ([`standalone/sse-desktop.exe`](standalone/sse-desktop.exe)), headless batch & benchmark runner ([`standalone/sse-cli.exe`](standalone/sse-cli.exe)), and 9-tool JSON-RPC `2.0` MCP server ([`standalone/sse-mcp.exe`](standalone/sse-mcp.exe)).
3. **Host GIS & CAD Integrations ([`sse/plugins/`](sse/plugins/))**:
   - **QGIS 3.22–4.x (`Qt5`/`Qt6 Ready`) Street Network Plugin** ([`standalone/sse_v1.1.1.zip`](standalone/sse_v1.1.1.zip)) focusing on the 5 street-network algorithms (`sse:*`), plus the **Grasshopper RhinoCommon C#** `Parallel.For` solver component.

| Computational Module | Core Script (`sse.core.*` / `src/lib.rs`) | Standalone & Host Availability | Primary Outputs |
| :--- | :--- | :--- | :--- |
| **1. OSM Watertight Cleaner** | `cleaner.py` · `osm_downloader.py` | `Python CLI` · `Standalone .exe` · `QGIS` (`sse:prepare_osm_network`) | Watertight, grade-separated network + stub pruning |
| **2. Angular Segment (ASA)** | `solvers_segment.py` · `graph.py` | `Python CLI` · `Standalone .exe` · `QGIS` (`sse:angular_segment_analysis`) | Multi-radius `NACH`, `NAIN`, `Choice`, `Integration` & `R²` |
| **3. Origin Step Depth** | `solvers_segment.py` (`solve_origin_step_depth`) | `Python API` · `Standalone .exe` · `QGIS` (`sse:origin_step_depth`) | Angular, metric (`m`), and topological step depth |
| **4. Topological Axial** | `solvers_axial.py` | `Python API` · `Standalone .exe` · `QGIS` (`sse:axial_analysis`) | `16` Hillier & Hanson metrics (`Integration [HH]`, `RRA`, `Entropy`) |
| **5. Natural Roads** | `solvers_natural_roads.py` | `Python API` · `Standalone .exe` · `QGIS` (`sse:natural_roads`) | Gestalt continuity arterial strokes (`sinuosity` & `circuity`) |
| **6. Spatial Visibility (`VGA / Isovists / Agents`)** | `solvers_vga.py` · `solvers_agents.py` · `src/lib.rs` | `Python API` · `Standalone .exe` (`sse-desktop` / `sse-cli` / `sse-mcp`) | `Visual_Integration [HH]`, `2D Isovist_Area`, `60 FPS Agent Gate_Counts` |

### Diagram A: SSE Core Engine & Multi-Platform Ecosystem

```mermaid
%%{init: {'theme': 'neutral'}}%%
flowchart LR
    subgraph Canvas [" "]
        In["Raw GIS / CAD Input (GeoJSON / GPKG / DXF / MIF)"] --> Heal["P90 Raycast + Grade Separation + Stub Prune"]
        Heal --> Dual{"Modern CSR Dual Graph + Tulip-1024 Core"}

        subgraph Platforms ["Execution Interfaces (Scripts -> Standalone -> GIS/CAD)"]
            P0["1. Modern Python Scripts + CLI: sse.core / sse clean / sse solve (Active)"]
            P1["2. Standalone Rust Workstation + CLI: standalone/sse-desktop.exe & sse-cli.exe (Active)"]
            P2["3. QGIS 3.22-4.x Processing Toolbox: 5 Street Network Algos (Active)"]
            P3["4. Grasshopper C# Parallel.For Component (Active)"]
        end

        Dual --> P0
        Dual --> P1
        Dual --> P2
        Dual --> P3

        P0 --> Out["NACH / NAIN / Axial / Strokes / VGA / Isovists / Agents / R2"]
        P1 --> Out
        P2 --> Out
        P3 --> Out
    end
    classDef default fill:#ffffff,stroke:#475569,color:#0f172a,stroke-width:1px
    style Canvas fill:#ffffff,stroke:#cbd5e1,stroke-width:1px,color:#0f172a
    style Platforms fill:#f8fafc,stroke:#94a3b8,color:#0f172a
```

### Diagram B: 3-Step Computational Pipeline

```mermaid
%%{init: {'theme': 'neutral'}}%%
flowchart LR
    subgraph Canvas [" "]
        subgraph Step1 ["1. Coordinate and Gap Healing"]
            CRS["Auto-Reproject WGS84 to Metric UTM"]
            Snap["P90 Adaptive Raycast + Grade Level Grouping"]
        end

        subgraph Step2 ["2. Dual Graph Construction"]
            Node["GEOS Planarization + Overpass Unlink"]
            Turn["Turner Angular Weighting (Float64 or Tulip-1024)"]
        end

        subgraph Step3 ["3. Bounded Brandes Centrality"]
            Dij["Horizon-Pruned Dijkstra (Angular / Metric / Topo)"]
            Norm["Hillier NACH + NAIN + Detailed CSV / R2 Export"]
        end

        CRS --> Snap
        Snap --> Node
        Node --> Turn
        Turn --> Dij
        Dij --> Norm
    end
    classDef default fill:#ffffff,stroke:#475569,color:#0f172a,stroke-width:1px
    style Canvas fill:#ffffff,stroke:#cbd5e1,stroke-width:1px,color:#0f172a
    style Step1 fill:#f8fafc,stroke:#94a3b8,color:#0f172a
    style Step2 fill:#f8fafc,stroke:#94a3b8,color:#0f172a
    style Step3 fill:#f8fafc,stroke:#94a3b8,color:#0f172a
```

---

## 2. Quick Start & Installation

### A. Modern Python Scripts, Library API & CLI (`sse`)
Install the `sse` Python package in editable mode to run the computational scripts, CLI commands, or Python API directly:

```bash
git clone https://github.com/napattan/sse-space-syntax.git
cd sse-space-syntax
pip install -e .

# 1. Clean raw street network & solve multi-radius NACH / NAIN via CLI
sse clean --input raw_roads.geojson --output clean_roads.geojson --snap 3.0 --rungs
sse solve --input clean_roads.geojson --output analyzed_roads.geojson --radii 800,2000 --include-global

# 2. Run the self-contained benchmark & verification scripts
python examples/run_mini_network.py
python -m pytest tests/ -q
```

Or import the vectorized solvers directly in Python scripts:

```python
from sse.core.cleaner import clean_network_adaptive_watertight
from sse.core.solvers_segment import solve_angular_segment_analysis
from sse.core.solvers_axial import solve_axial_analysis
from sse.core.solvers_natural_roads import build_natural_roads

# Clean raw LineStrings and solve exact Tulip-1024 multi-radius NACH / NAIN
cleaned_lines, report = clean_network_adaptive_watertight(raw_lines, mode="auto")
asa_results = solve_angular_segment_analysis(cleaned_lines, radii=[400, 800, 1200, 2000, "n"], tulip_bins=1024)
```

### B. Standalone Rust Workstation & Batch CLI ([`standalone/`](standalone/))
Pre-compiled Windows executables live directly in the **[`standalone/`](standalone/)** folder of this repository (or on **[GitHub Releases (`v1.1.1`)](https://github.com/napattan/sse-space-syntax/releases/tag/v1.1.1)**):

```powershell
# Run directly from standalone/ (zero dependencies required)
.\standalone\sse-desktop.exe
.\standalone\sse-cli.exe benchmark
```

Or compile the native Rust workspace from source with Cargo:

```bash
cargo run --release --bin sse-desktop
cargo run --release --bin sse-cli -- benchmark
```

### C. QGIS 3.22–4.x (`Qt6 Ready`) Street Network Plugin
1. Download **[`standalone/sse_v1.1.1.zip`](standalone/sse_v1.1.1.zip)** (or install from the [QGIS Plugin Repository](https://plugins.qgis.org/plugins/sse/)).
2. In QGIS, open **Plugins → Manage and Install Plugins → Install from ZIP**, select `sse_v1.1.1.zip`, and click **Install Plugin**.
3. Access the toolbar buttons (**5-Step Network Cleaner**, **Intelligibility/Synergy Scatter Plot**) and all 5 street network algorithms under **Processing Toolbox → SSE — Space Syntax Engine**.

---

## 3. Mathematical Formulation Summary

- **Turner (2007) Angular Turn Weighting & `Tulip-1024`**:
  - Continuous angular deflection `w_turn(θ) = |θ - π| ÷ (π ÷ 2) ∈ [0.0, 2.0]`, quantized in `Tulip-1024` mode into a `513`-bin semicircle priority queue (`bin_idx = round((Deflection_rad ÷ π) × 512)`, `w_turn_1024 = bin_idx ÷ 256`) over `2N` directed segment states `(seg, dir ∈ {-1, +1})`.
- **Hillier, Yang, & Turner (2012) Normalised Centrality**:
  - `NACH = ln(Choice + 1) ÷ ln(TotalDepth + 3)`
  - `NAIN = NodeCount^1.2 ÷ (TotalDepth + 2)`
- **5-Step P90 Adaptive Watertight Cleaner**:
  - Automatically projects `EPSG:4326` coordinates to local metric UTM, closes empirical `P90` endpoint undershoots (`6.0 m–20.0 m`) via tangent raycasting, preserves `bridge`/`tunnel`/`layer` grade separations, prunes `< 0.5 m` micro-stubs, and coalesces collinear segments (`θ ≤ 15°`).

<details>
<summary><b>📂 Click to expand Repository Structure & Package Manifest</b></summary>

```text
sse-space-syntax/
├── sse/                          # 1. Modern Python Computational Scripts, Solvers & Host Plugins
│   ├── core/                     # CSRDualGraph, Numba Tulip-1024, Axial, Natural Roads, VGA, Cleaner
│   ├── io_handlers/              # GeoJSON, GeoPackage, AutoCAD AC1018 DXF & MapInfo MIF/MID I/O
│   ├── plugins/
│   │   ├── qgis_plugin/          # QGIS 3.22-4.x (Qt6 Ready) plugin (5 street network algorithms)
│   │   └── grasshopper_csharp/   # RhinoCommon C# Parallel.For solver component
│   └── cli.py                    # Python CLI entry point (`sse clean`, `sse solve`)
├── src/                          # 2. Native Rust CSR Solvers, Workstation GUI & CLI Source
│   ├── lib.rs                    # Rust CSR solvers (ASA, Axial, Strokes, VGA, Isovist, Agents)
│   └── bin/
│       ├── sse_desktop.rs        # Pure-Rust egui/eframe standalone workstation GUI
│       ├── sse_cli.rs            # Headless CLI & 9-suite benchmark self-test runner
│       └── sse_mcp.rs            # 9-tool JSON-RPC 2.0 stdio MCP server + 127.0.0.1:9884 GUI bridge
├── standalone/                   # 3. Pre-compiled Windows Executables & QGIS Plugin Archive
│   ├── sse-desktop.exe           # Standalone Rust Workstation GUI (9.41 MB)
│   ├── sse-cli.exe               # Headless CLI & 9-suite benchmark runner (1.81 MB)
│   ├── sse-mcp.exe               # 9-tool JSON-RPC 2.0 stdio MCP server (4.22 MB)
│   ├── sse_v1.1.1.zip            # Official QGIS 3.22-4.x (Qt6 Ready) plugin archive
│   └── README.md                 # Standalone binaries quick reference
├── scripts/                      # Benchmark & figure generation scripts
├── examples/                     # UCL Barnsbury, Gallery & Multi-Room reference datasets
├── tests/                        # Pytest verification & analytical toy benchmark suite (20/20)
├── docs/
│   ├── benchmarks/               # 5 Academic 3-Panel Comparison Plates (depthmapX vs. SSE)
│   ├── RESEARCH_SPACE_SYNTAX_ENGINE.md # Deep mathematical study & forensic parity matrix
│   └── PUBLISHING_GUIDE.md       # Release & deployment guide
├── Cargo.toml                    # Native Rust workspace manifest
├── pyproject.toml                # PEP 621 Python package specification (`sse-space-syntax`)
└── LICENSE                       # MIT License
```

</details>

---

## 4. Primary Sources, Upstream Repositories, Citations & License

### Upstream Open-Source Repositories & Reference Implementations
- **UCL Space Group (`SpaceGroupUCL/depthmapX`)**: [GitHub Repository (`SpaceGroupUCL/depthmapX`)](https://github.com/SpaceGroupUCL/depthmapX)
  - **Angular Segment Analysis (`Tulip-1024`)**: [`salalib/segmmodules/segmtulip.cpp`](https://github.com/SpaceGroupUCL/depthmapX/blob/master/salalib/segmmodules/segmtulip.cpp) (`513`-bin directed-segment semicircle bucket queue & multi-radius bitmask coverage).
  - **Topological Axial Analysis**: [`salalib/axialmodules/axialintegration.cpp`](https://github.com/SpaceGroupUCL/depthmapX/blob/master/salalib/axialmodules/axialintegration.cpp) (Hillier & Hanson 1984 Diamond `D_n`, `RA`, `RRA`, `Integration [HH]`, and `Entropy`).
  - **Visibility Graph Analysis (VGA)**: [`salalib/vgamodules/vgavisualglobal.cpp`](https://github.com/SpaceGroupUCL/depthmapX/blob/master/salalib/vgamodules/vgavisualglobal.cpp) and [`salalib/vgamodules/vgavisuallocal.cpp`](https://github.com/SpaceGroupUCL/depthmapX/blob/master/salalib/vgamodules/vgavisuallocal.cpp).
  - **Analytical 2D Isovists & Visual Agents**: [`salalib/isovist.cpp`](https://github.com/SpaceGroupUCL/depthmapX/blob/master/salalib/isovist.cpp), [`salalib/pointdata.h`](https://github.com/SpaceGroupUCL/depthmapX/blob/master/salalib/pointdata.h) (`whichbin()` 32-bin tangent quantization), and [`salalib/agents/`](https://github.com/SpaceGroupUCL/depthmapX/tree/master/salalib/agents).
- **UCL Space Syntax Toolkit for QGIS (`SpaceGroupUCL/qgisSpaceSyntaxToolkit`)**: [GitHub Repository (`SpaceGroupUCL/qgisSpaceSyntaxToolkit`)](https://github.com/SpaceGroupUCL/qgisSpaceSyntaxToolkit) (Jorge Gil, Tasos Varoudis, Kayvan Karimi & Alan Penn).
- **OSMnx (`gboeing/osmnx`)**: [GitHub Repository (`gboeing/osmnx`)](https://github.com/gboeing/osmnx) · [Documentation](https://osmnx.readthedocs.io/) (Geoff Boeing — OpenStreetMap street network extraction and topology simplification).
- **PySAL `momepy` (`pysal/momepy`) & `neatnet` (`geopandas/neatnet`)**: [GitHub Repository (`pysal/momepy`)](https://github.com/pysal/momepy) · [GitHub Repository (`geopandas/neatnet`)](https://github.com/geopandas/neatnet) (Martin Fleischmann et al. — Urban morphometrics and continuity stroke simplification).
- **Core Scientific & UI Libraries**: [Shapely / GEOS (`shapely/shapely`)](https://github.com/shapely/shapely) · [SciPy (`scipy/scipy`)](https://github.com/scipy/scipy) · [Numba (`numba/numba`)](https://github.com/numba/numba) · [Rayon (`rayon-rs/rayon`)](https://github.com/rayon-rs/rayon) · [egui (`emilk/egui`)](https://github.com/emilk/egui).

### Foundational Literature
- **Hillier, B., & Hanson, J. (1984)**. *The Social Logic of Space*. Cambridge University Press. ([DOI: `10.1017/CBO9780511597237`](https://doi.org/10.1017/CBO9780511597237))
- **Hillier, B., Yang, T., & Turner, A. (2012)**. Normalising least angle choice in Depthmap (`NACH` & `NAIN`). *Journal of Space Syntax*, 3(2), 155–193.
- **Turner, A. (2001)**. Angular analysis. *Proceedings of the 3rd International Space Syntax Symposium*, Atlanta, 30.1–30.11.
- **Turner, A. (2007)**. From axial to road-centre lines: a new representation for space syntax and a new model of route choice for transport network analysis. *Environment and Planning B*, 34(3), 539–555. ([DOI: `10.1068/b32067`](https://doi.org/10.1068/b32067))
- **Brandes, U. (2001)**. A faster algorithm for betweenness centrality. *Journal of Mathematical Sociology*, 25(2), 163–177. ([DOI: `10.1080/0022250X.2001.9990249`](https://doi.org/10.1068/0022250X.2001.9990249))
- **Turner, A., Doxa, M., O'Sullivan, D., & Penn, A. (2001)**. From isovists to visibility graphs: a methodology for the analysis of architectural space. *Environment and Planning B*, 28(1), 103–121. ([DOI: `10.1068/b2684`](https://doi.org/10.1068/b2684))
- **Turner, A., & Penn, A. (2002)**. Encoding natural movement as an agent-based system. *Environment and Planning B*, 29(4), 473–490. ([DOI: `10.1068/b12850`](https://doi.org/10.1068/b12850))
- **Benedikt, M. L. (1979)**. To take hold of space: isovists and isovist fields. *Environment and Planning B*, 6(1), 47–65. ([DOI: `10.1068/b060047`](https://doi.org/10.1068/b060047))
- **Jiang, B., & Claramunt, C. (2004)**. Topological analysis of urban street networks. *Environment and Planning B*, 31(1), 151–162. ([DOI: `10.1068/b306`](https://doi.org/10.1068/b306))
- **Gil, J., Varoudis, T., Karimi, K., & Penn, A. (2015)**. The Space Syntax Toolkit: Integrating depthmapX concepts and workflows into QGIS. *Proceedings of the 10th International Space Syntax Symposium*, UCL, London, 148:1–148:12.

### Citation & License
- **Deep Technical Study**: See [`docs/RESEARCH_SPACE_SYNTAX_ENGINE.md`](docs/RESEARCH_SPACE_SYNTAX_ENGINE.md) for full derivations and the 4-Axis Forensic Audit Matrix.
- **Author**: Napat (Tan) Phasundhiae ([`napattan/sse-space-syntax`](https://github.com/napattan/sse-space-syntax) · [`@iscvlab`](https://github.com/iscvlab))
- **License**: Released under the [MIT License](LICENSE).
