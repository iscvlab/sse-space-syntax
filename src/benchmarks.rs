//! Built-in Academic Reference Street Networks & Architectural Floorplans for SSE.
//!
//! Embeds official UCL `depthmapX` (`SpaceGroupUCL/depthmapX/testdata`) street networks and
//! architectural floorplans alongside the `gboeing/osmnx#1410` closed-loop benchmark:
//! 1. `UrbanSuperblockGrid`: Official UCL Barnsbury, Islington street network (`barnsbury_segment_lines.mif`, 178 segments in OSGB36 meters).
//! 2. `AxialStarDiamond`: Official UCL Extended Barnsbury North London axial/street network (`barnsbury_extended1.dxf`, 1,100 lines in OSGB36 meters).
//! 3. `ArchitecturalGalleryVga`: Official UCL National Gallery Sainsbury Wing floorplan (`gallery.dxf`, flood-fill seed `(3.0, 5.0)`).
//! 4. `UclRoomsFloorplan`: Official UCL Multi-Room architectural floorplan (`rooms.dxf`, flood-fill seed `(7.5, 7.5)`).
//! 5. `RingRoadCulDeSacLoop`: Closed-loop roundabouts and cul-de-sac rings exercising the
//!    `gboeing/osmnx#1410` zero-chord `sinuosity` / `circuity_avg` guard.

use crate::types::{Point2D, Polyline};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BenchmarkPreset {
    UrbanSuperblockGrid,
    AxialStarDiamond,
    ArchitecturalGalleryVga,
    UclRoomsFloorplan,
    RingRoadCulDeSacLoop,
}

impl BenchmarkPreset {
    pub const ALL: [BenchmarkPreset; 5] = [
        Self::UrbanSuperblockGrid,
        Self::AxialStarDiamond,
        Self::ArchitecturalGalleryVga,
        Self::UclRoomsFloorplan,
        Self::RingRoadCulDeSacLoop,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            Self::UrbanSuperblockGrid => "UCL Barnsbury Street Network (London, MIF)",
            Self::AxialStarDiamond => "UCL Barnsbury Extended Street & Axial Network (London, DXF)",
            Self::ArchitecturalGalleryVga => "UCL National Gallery Floorplan (gallery.dxf)",
            Self::UclRoomsFloorplan => "UCL Multi-Room Architectural Plan (rooms.dxf)",
            Self::RingRoadCulDeSacLoop => "Ring-Road & Cul-de-Sac Loops (osmnx#1410 Sinuosity Guard)",
        }
    }

    pub fn is_spatial_vga(&self) -> bool {
        matches!(
            self,
            Self::ArchitecturalGalleryVga | Self::UclRoomsFloorplan
        )
    }
}

#[derive(Debug, Clone)]
pub struct BenchmarkDataset {
    pub name: &'static str,
    pub subtitle: &'static str,
    pub lines: Vec<Polyline>,
    pub boundary_rings: Vec<Vec<Point2D>>,
    pub obstacles: Vec<Polyline>,
    pub default_vga_cell_m: f64,
}

fn build_urban_superblock_grid() -> BenchmarkDataset {
    let lines = crate::io::load_mif_str(include_str!(
        "../examples/ucl_depthmapx/barnsbury_segment_lines.mif"
    ))
    .unwrap_or_default();

    BenchmarkDataset {
        name: "UCL Barnsbury Street Network (London, MIF)",
        subtitle: "Official SpaceGroupUCL/depthmapX benchmark: Street segment network in Barnsbury, Islington (OSGB36 meters)",
        lines,
        boundary_rings: Vec::new(),
        obstacles: Vec::new(),
        default_vga_cell_m: 2.0,
    }
}

fn build_axial_star_diamond() -> BenchmarkDataset {
    let lines = crate::io::load_dxf_str(include_str!(
        "../examples/ucl_depthmapx/barnsbury_extended1.dxf"
    ))
    .unwrap_or_default();

    BenchmarkDataset {
        name: "UCL Barnsbury Extended Street & Axial Network (London, DXF)",
        subtitle: "Official SpaceGroupUCL/depthmapX benchmark: 1,100 street & axial lines across North London (OSGB36 meters)",
        lines,
        boundary_rings: Vec::new(),
        obstacles: Vec::new(),
        default_vga_cell_m: 2.5,
    }
}

fn scale_polylines(lines: Vec<Polyline>, factor: f64) -> Vec<Polyline> {
    lines
        .into_iter()
        .map(|mut pl| {
            for p in &mut pl.coords {
                p.x *= factor;
                p.y *= factor;
            }
            pl
        })
        .collect()
}

fn build_architectural_gallery_vga() -> BenchmarkDataset {
    let raw =
        crate::io::load_dxf_str(include_str!("../examples/ucl_depthmapx/gallery.dxf"))
            .unwrap_or_default();
    // Scale 1:5 paper units into metric floorplan coordinates so 0.30 m grid = 1,866 cells (exact UCL depthmapX gallery VGA resolution)
    let obstacles = scale_polylines(raw, 5.0);
    let lines = obstacles.clone();

    BenchmarkDataset {
        name: "UCL National Gallery Floorplan (gallery.dxf)",
        subtitle: "Official SpaceGroupUCL/depthmapX benchmark: National Gallery Sainsbury Wing floorplan (VGA + Isovists + Gibson Agents)",
        lines,
        boundary_rings: vec![vec![Point2D::new(15.0, 25.0)]],
        obstacles,
        default_vga_cell_m: 0.30,
    }
}

fn build_ucl_rooms_floorplan() -> BenchmarkDataset {
    let raw =
        crate::io::load_dxf_str(include_str!("../examples/ucl_depthmapx/rooms.dxf"))
            .unwrap_or_default();
    let obstacles = scale_polylines(raw, 1.2);
    let lines = obstacles.clone();

    BenchmarkDataset {
        name: "UCL Multi-Room Architectural Plan (rooms.dxf)",
        subtitle: "Official SpaceGroupUCL/depthmapX benchmark: Enfilade multi-room building plan (VGA + Isovists + Gibson Agents)",
        lines,
        boundary_rings: vec![vec![Point2D::new(9.0, 9.0)]],
        obstacles,
        default_vga_cell_m: 0.30,
    }
}

fn build_ring_road_cul_de_sac_loop() -> BenchmarkDataset {
    let mut lines = Vec::new();

    // Closed polygonal ring road (start == end -> chord_length_m == 0.0!)
    let mut outer_ring = Vec::with_capacity(17);
    let cx = 300.0;
    let cy = 300.0;
    let r_outer = 200.0;
    for k in 0..=16 {
        let ang = (k as f64) * (2.0 * PI / 16.0);
        outer_ring.push(Point2D::new(cx + r_outer * ang.cos(), cy + r_outer * ang.sin()));
    }
    // Force exact closure so chord_length() == 0.0
    if let Some(&first) = outer_ring.first() {
        if let Some(last) = outer_ring.last_mut() {
            *last = first;
        }
    }
    lines.push(Polyline::new(outer_ring).with_meta(0.0, "primary", "Closed Orbital Ring Road"));

    // Inner roundabout closed loop (start == end -> chord_length_m == 0.0!)
    let mut inner_ring = Vec::with_capacity(13);
    let r_inner = 60.0;
    for k in 0..=12 {
        let ang = (k as f64) * (2.0 * PI / 12.0);
        inner_ring.push(Point2D::new(cx + r_inner * ang.cos(), cy + r_inner * ang.sin()));
    }
    if let Some(&first) = inner_ring.first() {
        if let Some(last) = inner_ring.last_mut() {
            *last = first;
        }
    }
    lines.push(Polyline::new(inner_ring).with_meta(0.0, "secondary", "Central Roundabout Loop"));

    // 4 Cardinal arterial spokes connecting the inner roundabout to the outer orbital ring
    for k in 0..4 {
        let ang = (k as f64) * (PI / 2.0);
        let p0 = Point2D::new(cx + r_inner * ang.cos(), cy + r_inner * ang.sin());
        let p1 = Point2D::new(cx + (r_outer + 80.0) * ang.cos(), cy + (r_outer + 80.0) * ang.sin());
        lines.push(
            Polyline::new(vec![p0, p1]).with_meta(0.0, "primary", format!("Arterial Connector {}", k + 1)),
        );
    }

    // 2 Cul-de-sac bulb loops attached to the outer ring
    for &(bx, by, name) in &[
        (cx + r_outer + 80.0, cy, "East Cul-de-Sac Bulb"),
        (cx - r_outer - 80.0, cy, "West Cul-de-Sac Bulb"),
    ] {
        let mut bulb = Vec::with_capacity(9);
        for k in 0..=8 {
            let ang = (k as f64) * (2.0 * PI / 8.0);
            bulb.push(Point2D::new(bx + 25.0 * ang.cos(), by + 25.0 * ang.sin()));
        }
        bulb[8] = bulb[0];
        lines.push(Polyline::new(bulb).with_meta(0.0, "residential", name));
    }

    BenchmarkDataset {
        name: "Ring-Road & Cul-de-Sac Loops",
        subtitle: "Closed orbital loops + arterial spokes verifying osmnx#1410 zero-chord sinuosity guard",
        lines,
        boundary_rings: Vec::new(),
        obstacles: Vec::new(),
        default_vga_cell_m: 3.0,
    }
}

/// Loads one of the 5 built-in academic benchmark datasets.
pub fn load_benchmark(preset: BenchmarkPreset) -> BenchmarkDataset {
    match preset {
        BenchmarkPreset::UrbanSuperblockGrid => build_urban_superblock_grid(),
        BenchmarkPreset::AxialStarDiamond => build_axial_star_diamond(),
        BenchmarkPreset::ArchitecturalGalleryVga => build_architectural_gallery_vga(),
        BenchmarkPreset::UclRoomsFloorplan => build_ucl_rooms_floorplan(),
        BenchmarkPreset::RingRoadCulDeSacLoop => build_ring_road_cul_de_sac_loop(),
    }
}
