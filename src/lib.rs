//! # SSE — Space Syntax Engine (`sse_core`)
//!
//! Native Rust computational core for UCL `depthmapX v0.8.0` parity:
//! - **Cleaner & Topology (`cleaner`)**: 5-Step P90 Adaptive Watertight OSM/CAD Network Cleaner,
//!   planarization (`O(N log N)` segment intersection noding), degree-2 collinear join with turn conservation,
//!   and dead-end stub pruning.
//! - **CSR Dual Graph (`graph`)**: Cache-aligned Compressed Sparse Row (CSR) Dual Graph with
//!   continuous `f64` or `Tulip-1024` angular turn deflection and multi-level bridge/tunnel grade unlinks.
//! - **Angular Segment Analysis (`solvers_asa`)**: Multi-radius `rayon`-parallel single-pass Brandes
//!   Choice, Closeness Integration, Total/Mean Depth, Node Count, `NACH`, `NAIN`, Control, Controllability,
//!   Intelligibility `R²`, Synergy `R²`, and Multi-Source Origin Step Depth.
//! - **Topological Axial Analysis (`solvers_axial`)**: Full 16-attribute Hillier & Hanson (1984) /
//!   `depthmapX v0.8.0` axial line analysis (`RRA` via Hillier Diamond `D_n`, `Integration [HH]`, `[P-value]`,
//!   `[Tekl]`, `Entropy`, `Relativised Entropy`, `Choice`, `Choice [Norm]`, `Harmonic Mean Depth`).
//! - **Natural Roads (`solvers_strokes`)**: Jiang & Claramunt (2004) Gestalt stroke concatenation with
//!   closed-loop `sinuosity` & `circuity_avg` zero-chord guard (`Option<f64>`).
//! - **Visibility Graph Analysis & Isovists (`solvers_vga`)**: PointMap grid generation, `32-bin` octant
//!   `whichbin` Line-of-Sight (`LOS`) graph, Visual Integration `[HH]`, Clustering Coefficient, Through Vision,
//!   and analytical 2D Benedikt corner-ray Isovists.
//! - **Gibson Ecological Visual Agents (`solvers_agents`)**: Turner & Penn (2002) active visual agent
//!   simulation (`15-bin` `~170°` FOV, `steps_before_turn`, batch & 60 FPS interactive stepping).
//! - **Spatial I/O & OSM Overpass (`io`)**: GeoJSON, CSV, DXF `AC1018`, MapInfo `MIF/MID`, OSM `.osm` XML,
//!   WGS84 → UTM metric projection, and multi-mirror Overpass HTTP fetcher.
//! - **Academic Benchmarks (`benchmarks`)**: Built-in reference street networks and architectural floorplans.

pub mod benchmarks;
pub mod cleaner;
pub mod graph;
pub mod io;
pub mod solvers_agents;
pub mod solvers_asa;
pub mod solvers_axial;
pub mod solvers_strokes;
pub mod solvers_vga;
pub mod types;

pub use types::*;
