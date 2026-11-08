# SSE — Space Syntax Engine (`sse`) for QGIS

**SSE — Space Syntax Engine** (`sse`) is a high-performance, crash-free replacement for UCL `depthmapX v0.8.0` and the legacy Space Syntax Toolkit, engineered natively for **QGIS 3.22+ and QGIS 4.x (Qt5 & Qt6 Ready)**.

Powered by contiguous **Compressed Sparse Row (CSR)** Dual Graphs (`NumPy` / `SciPy`) and **GEOS `STRtree` O(N log N)** spatial indexing, **SSE (Space Syntax Engine)** eliminates `std::bad_alloc` memory crashes, unindexed O(N²) geometry hangs, and coordinate precision corruption on metropolitan-scale street networks.

---

## Core Processing Toolbox Algorithms

**SSE — Space Syntax Engine** registers **6 native algorithms** in the QGIS Processing Toolbox under **SSE — Space Syntax Engine** (`sse`):

| Algorithm | Group | Description & Core Metrics |
|:---|:---|:---|
| **1. Angular Segment Analysis (ASA Pro)** | Segment Analysis | Multi-radius shortest angular path solver (`n`, `10000`, `5000`, `2000`, `1200`, `800`, `400` m) with both Exact IEEE-754 Continuous Radians and Exact `depthmapX` Directed-Segment `Tulip-1024` Semicircle Bucket Queue (`R² = 1.00000000`). Computes `Choice`, `Integration`, `NACH` (`ln(Choice + 1) / ln(TD + 3)`), `NAIN` (`NC^1.2 / (TD + 2)`), `Total Depth`, `Node Count`, and optional universal `[0–1]` normalized scores (`nach_norm_*`, `nain_norm_*`). |
| **2. Topological Axial Analysis** | Axial Analysis | Full 16-metric Hillier & Hanson (`depthmapX v0.8.0`) axial step-depth engine across global (`n`) and local (`R3`, `R5`) radii: `Connectivity`, `Control`, `Controllability`, `Integration [HH / P-value / Tekl]`, `Choice`, `Choice [Norm]`, `RA`, `RRA` (Diamond `D_n`), `Mean Depth`, `Entropy`, and `Intelligibility R²`. |
| **3. Visibility Graph Analysis (VGA)** | Visibility Analysis | Discretizes walkable polygons into an obstacle-aware grid and computes Line-of-Sight (LOS) visibility graphs, `Visual Integration [HH]`, `Visual Control`, `Clustering Coefficient`, `Through Vision`, 2D ray-cast `Isovist` area/perimeter/compactness/occlusivity, and optional Turner & Penn (2002) Gibson Ecological Visual Agent `Gate_Counts`. |
| **4. Origin Step Depth & Catchment Analysis** | Segment Analysis | Multi-origin Dijkstra step-depth solver from selected transit stations, park gates, or POIs. Outputs simultaneous `angular_step_depth`, `metric_step_depth_m`, `topo_step_depth`, `nearest_origin_id`, and `is_origin`. |
| **5. Natural Roads (Stroke Concatenation)** | Stroke Analysis | Jiang & Claramunt (2004) Gestalt good-continuity solver that concatenates fragmented street segments across junctions into continuous movement strokes based on angular deflection threshold (`θ ≤ 20°`). |
| **6. Prepare OSM Network for Space Syntax** | Network Cleansing | 5-Step Adaptive Watertight OSM Cleaner: empirical P90 gap analysis (`[6.0m, 20.0m]`) or manual raycasting, deflection angle guard (`15°–165°`), planarization, micro-stub pruning (`< 0.5m`), turn-conserving degree-2 collinear merge, optional dual-carriageway rungs, and 2-point segment explosion. |

---

## Interactive Dual-Mode Network Cleaner & Scatter Plot Dialogs

Accessible via **Vector → SSE — Space Syntax Engine → Enhanced 4-Step Network Cleaner (Dual-Mode)...** or **Space Syntax Scatter Plot (Intelligibility / Synergy R²)...** and the toolbar icon:

- **Auto Mode (Recommended)**: Dynamically profiles dangling endpoint distances using empirical 90th-percentile (`P90`) gap statistics clamped to `[6.0m, 20.0m]`.
- **Manual Mode**: User-defined extension distance (`1.0–50.0 m`, default `15.0 m`).
- **Topological Guards**: Toggle micro-stub pruning (`< 0.5 m`), turn conservation (`Δθ > 15°`), dual-carriageway cross-rungs, and live post-flight topological audit reporting.
- **Interactive Scatter Plot Explorer**: Bivariate OLS regression (`R²` & Pearson `r`) with rubber-band selection linking directly to the QGIS map canvas.

---

## Installation & Dependencies

Requires **QGIS 3.22+** or **QGIS 4.x** with standard scientific Python packages (`numpy`, `scipy`, `shapely >= 2.0`).

## License

Released under the **MIT License**. Copyright (c) 2026 Napat (Tan) Phasundhiae.
