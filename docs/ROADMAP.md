# OSSE Strategic Roadmap & Milestone Plan

> **Open Space Syntax Computational Engine (OSSE)**  
> Version Track: 2026-2027

---

## 🎯 Vision Statement
Transition Space Syntax from an isolated, crash-prone legacy desktop paradigm into a unified, high-performance computational geometry engine natively embedded across modern GIS (QGIS/ArcGIS), parametric design (Rhino/Grasshopper), web applications, and large-scale metropolitan simulations.

---

## 🗺️ Release Milestones

### Phase 1: Core Engine & Interoperability (v0.1.0 — Current Release) ✅
- [x] **C-GEOS R-Tree Watertight Snapping**: Sub-second geometric healing of micro-gaps.
- [x] **Transverse Carriageway Rung Synthesis**: Restores pedestrian connectivity across dual-lane avenues.
- [x] **Angular Dual-Graph Engine**: Full directional angular deflection calculation (0°–90° turn penalties).
- [x] **Brandes Metric Solver**: Priority-queue bounded betweenness and closeness centrality for arbitrary radii (R400m, R800m, R2000m, Rn).
- [x] **DepthmapX Bi-Directional Bridge**: AC1018 DXF generator and MapInfo MIF/MID parser/serializer.
- [x] **Standalone Package Architecture**: Complete PEP 621 packaging with CLI runner and test suite.

### Phase 2: QGIS Plugin & Processing Provider (v0.2.0 — Target: Q4 2026) 🔜
- [ ] **Native QGIS Processing Algorithms**:
  - `osse:clean_network` (watertight snapping + planarization)
  - `osse:angular_choice` (NACH across multi-radii)
  - `osse:angular_integration` (NAIN across multi-radii)
- [ ] **Real-Time Canvas Interactive Isochrone**:
  - Map tool to click any segment and display instant topological catchment and line of sight.
- [ ] **One-Click Cartographic Symbology**:
  - Bundled Swiss-Architectural 5-tier graduated QML styles (`0.25mm` thin strokes).
- [ ] **Official QGIS Plugin Portal Release**: Verified package on `plugins.qgis.org`.

### Phase 3: High-Performance RhinoCommon C# Solver (v0.3.0 — Target: Q1 2027)
- [ ] **RhinoCommon Multi-Threaded C# Component**:
  - Primitive memory arrays (`double[]`, `Point3d[]`) with `Parallel.For` loop acceleration.
  - Zero Python GIL overhead for 10× speedup over Python GH components.
- [ ] **Dynamic Slider Metric Tuning**:
  - Live metric radius slider (400m to 10,000m) with real-time viewport color rendering via Grasshopper preview.
- [ ] **Food4Rhino & Yak Package Publication**: Direct install via Rhino Package Manager (`Yak`).

### Phase 4: WebAssembly & GPU-Accelerated Web Engine (v0.4.0 — Target: Mid 2027)
- [ ] **WebAssembly (Pyodide / Rust) Browser Engine**:
  - Run topological space syntax directly inside browser client without server round-trip.
- [ ] **GPU Acceleration via CuGraph / WebGPU**:
  - Massive parallel shortest-path solving for citywide meshes with > 500,000 segments.
- [ ] **Interactive Web Dashboard Integration**:
  - Integration with MapLibre GL JS / Leaflet for interactive urban mobility exploration.

---

## 📈 Long-Term Research & Ecosystem Integration

* **Multi-Criteria Spatial Suitability Coupling**: Direct feed into multi-criteria spatial suitability engines (MCDA) for urban planning and infrastructure assessment.
* **Agent-Based Pedestrian Simulation**: Space Syntax NACH weights driving pedestrian agent route choice and gate-count calibration in 2D/3D urban solvers.
* **Microclimate & Environmental Cross-Correlation**: Correlating pedestrian choice corridors with urban heat island (LST), acoustic dispersion, and civic telemetry networks.
