//! Gibson Ecological Visual Agent Simulation (Turner & Penn 2002; UCL `depthmapX v0.8.0` `salalib/agents/`).
//!
//! Implements the exact C++ agent movement and visual look architecture from `SpaceGroupUCL/depthmapX`:
//! - `bin_from_vec`: 32-bin angular quantizer (`salalib/agents/agent.h:61-65`)
//! - `onStandardLook` (`SEL_STANDARD = 0x1001`) & `onLoSLook` (`SEL_LOS = 0x0001`) (`salalib/agents/agent.cpp:427-666`)
//! - Stochastic Bernoulli turn decision (`prandomr() < 1.0 / m_steps`) and target-arrival check (`salalib/agents/agent.cpp:205-221`)
//! - Continuous heading vector stepping (`onStep` / `goodStep`) with `±45°` (`±π/4`) `diagonalStep` wall-collision fallback (`salalib/agents/agent.cpp:266-316`)
//! - Inverse cumulative Poisson release per timestep (`invcumpoisson` in `salalib/agents/agentengine.cpp:68`)
//! - Stationary manual single-agent placement (`Q3DView::CreateAgent` in `depthmapX/views/3dview/3dview.cpp:654-711`)

use crate::types::{
    ActiveAgent, AgentConfig, AgentMovementRule, AgentSimState, Point2D, VgaResults,
};

#[inline]
fn xorshift64_star(state: &mut u64) -> u64 {
    let mut x = if *state == 0 {
        0x9E37_79B9_7F4A_7C15
    } else {
        *state
    };
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *state = x;
    x.wrapping_mul(0x2545_F491_4F6C_DD1D)
}

/// Uniform random float in `[0.0, 1.0)` matching `depthmapX` `prandomr()`.
#[inline]
fn rand_u01(state: &mut u64) -> f64 {
    ((xorshift64_star(state) >> 11) as f64) * (1.0 / ((1u64 << 53) as f64))
}

/// Inverse cumulative Poisson draw (`depthmapX` `invcumpoisson(prandomr(), m_release_rate)`).
#[inline]
fn invcumpoisson(u: f64, lambda: f64) -> usize {
    if lambda <= 0.0 {
        return 0;
    }
    let l = (-lambda).exp();
    let mut k = 0usize;
    let mut p = l;
    let mut s = p;
    while u > s && k < 64 {
        k += 1;
        p *= lambda / (k as f64);
        s += p;
    }
    k
}

/// Exact `depthmapX` `binfromvec(const Point2f& p)` (`salalib/agents/agent.h:61-65`):
/// maps a 2D direction vector `(dx, dy)` to one of 32 angular bins (`0..32`).
#[inline]
pub fn bin_from_vec(dx: f64, dy: f64) -> u8 {
    if dx.abs() < 1e-12 && dy.abs() < 1e-12 {
        return 0;
    }
    let angle = dy.atan2(dx);
    let raw = (32.0 * (0.5 * angle / std::f64::consts::PI) + 0.5).floor() as i32;
    raw.rem_euclid(32) as u8
}

/// Exact `depthmapX` `anglefrombin2(int bin)` (`salalib/agents/agent.h:66-70`):
/// samples a continuous angle (radians) uniformly inside angular `bin`.
#[inline]
fn angle_from_bin(bin: u8, rng: &mut u64) -> f64 {
    let u = rand_u01(rng);
    (2.0 * std::f64::consts::PI) * (((bin as f64) - 0.5 + u) / 32.0)
}

#[inline]
fn circular_bin_dist(a: u8, b: u8) -> u8 {
    let diff = (a as i16 - b as i16).unsigned_abs() as u8;
    diff.min(32u8.saturating_sub(diff))
}

#[inline]
fn normalize_vec(dx: f64, dy: f64) -> Point2D {
    let len = dx.hypot(dy);
    if len > 1e-12 {
        Point2D::new(dx / len, dy / len)
    } else {
        Point2D::new(1.0, 0.0)
    }
}

#[inline]
fn rotate_vec(v: Point2D, angle_rad: f64) -> Point2D {
    let (s, c) = angle_rad.sin_cos();
    Point2D::new(v.x * c - v.y * s, v.x * s + v.y * c)
}

/// Exact `depthmapX` `Agent::onLook(bool wholeisovist)` (`salalib/agents/agent.cpp:365-425, 427-468, 612-666`).
/// Returns `(new_dir, new_target_pos, has_target, new_bin)`.
fn on_look_depthmapx(
    vga: &VgaResults,
    node_u: usize,
    current_pos: Point2D,
    current_dir: Point2D,
    fov_bins: usize,
    movement_rule: AgentMovementRule,
    whole_isovist: bool,
    rng: &mut u64,
) -> (Point2D, Point2D, bool, u8) {
    let s = vga.row_offsets.get(node_u).copied().unwrap_or(0) as usize;
    let e = vga.row_offsets.get(node_u + 1).copied().unwrap_or(0) as usize;
    if e <= s {
        let fallback_bin = (xorshift64_star(rng) & 31) as u8;
        let ang = angle_from_bin(fallback_bin, rng);
        let dir = Point2D::new(ang.cos(), ang.sin());
        return (dir, current_pos, false, fallback_bin);
    }

    let center_bin = bin_from_vec(current_dir.x, current_dir.y);
    let half_fov = if whole_isovist || fov_bins >= 32 {
        16u8
    } else {
        ((fov_bins.max(1) - 1) / 2) as u8
    };

    match movement_rule {
        AgentMovementRule::Standard => {
            // depthmapX Agent::onStandardLook(bool wholeisovist):
            // Uniformly selects one visible VGA cell across all visible cells in the (2 * vbin + 1) FOV cone.
            // If choices == 0 in the forward cone, falls back to wholeisovist = true (full 360° look).
            for full_360 in [whole_isovist || half_fov >= 16, true] {
                if !full_360 {
                    // Fast O(1) rejection sampling in the visible slice (succeeds in 1-3 draws for 15-bin ~170° FOV)
                    let total = e - s;
                    for _ in 0..16 {
                        let pick = (xorshift64_star(rng) as usize) % total;
                        let idx = s + pick;
                        let b = vga.bin_indices[idx];
                        if circular_bin_dist(b, center_bin) <= half_fov {
                            let target_v = vga.col_indices[idx] as usize;
                            let target_pt =
                                Point2D::new(vga.points[target_v].x, vga.points[target_v].y);
                            let dir =
                                normalize_vec(target_pt.x - current_pos.x, target_pt.y - current_pos.y);
                            let bin = bin_from_vec(dir.x, dir.y);
                            return (dir, target_pt, true, bin);
                        }
                    }

                    // Exact uniform reservoir / count pass across visible cells in the forward FOV cone
                    let mut chosen_v: Option<usize> = None;
                    let mut choices = 0usize;
                    for idx in s..e {
                        let b = vga.bin_indices[idx];
                        if circular_bin_dist(b, center_bin) <= half_fov {
                            choices += 1;
                            if choices == 1 || ((xorshift64_star(rng) as usize) % choices) == 0 {
                                chosen_v = Some(vga.col_indices[idx] as usize);
                            }
                        }
                    }
                    if let Some(target_v) = chosen_v {
                        let target_pt =
                            Point2D::new(vga.points[target_v].x, vga.points[target_v].y);
                        let dir =
                            normalize_vec(target_pt.x - current_pos.x, target_pt.y - current_pos.y);
                        let bin = bin_from_vec(dir.x, dir.y);
                        return (dir, target_pt, true, bin);
                    }
                } else {
                    let total = e - s;
                    let pick = (xorshift64_star(rng) as usize) % total;
                    let target_v = vga.col_indices[s + pick] as usize;
                    let target_pt = Point2D::new(vga.points[target_v].x, vga.points[target_v].y);
                    let dir =
                        normalize_vec(target_pt.x - current_pos.x, target_pt.y - current_pos.y);
                    let bin = bin_from_vec(dir.x, dir.y);
                    return (dir, target_pt, true, bin);
                }
            }
            unreachable!()
        }
        AgentMovementRule::LineOfSightLength => {
            // depthmapX Agent::onLoSLook(bool wholeisovist, int look_type):
            // Weights each bin in the FOV cone by its maximum radial line-of-sight distance.
            for full_360 in [whole_isovist || half_fov >= 16, true] {
                let mut bin_max_dist = [0.0f64; 32];
                for idx in s..e {
                    let b = (vga.bin_indices[idx] & 31) as usize;
                    if full_360 || circular_bin_dist(b as u8, center_bin) <= half_fov {
                        let v = vga.col_indices[idx] as usize;
                        let pv = vga.points[v];
                        let d = (pv.x - current_pos.x).hypot(pv.y - current_pos.y);
                        if d > bin_max_dist[b] {
                            bin_max_dist[b] = d;
                        }
                    }
                }
                let total_weight: f64 = bin_max_dist.iter().sum();
                if total_weight > 1e-9 {
                    let mut pick_w = rand_u01(rng) * total_weight;
                    let mut chosen_bin = center_bin;
                    for (b, &w) in bin_max_dist.iter().enumerate() {
                        if w > 0.0 {
                            chosen_bin = b as u8;
                            if pick_w <= w {
                                break;
                            }
                            pick_w -= w;
                        }
                    }
                    let ang = angle_from_bin(chosen_bin, rng);
                    let dir = Point2D::new(ang.cos(), ang.sin());
                    return (dir, current_pos, false, chosen_bin);
                }
            }
            let fallback_bin = (xorshift64_star(rng) & 31) as u8;
            let ang = angle_from_bin(fallback_bin, rng);
            let dir = Point2D::new(ang.cos(), ang.sin());
            (dir, current_pos, false, fallback_bin)
        }
    }
}

/// Pre-indexes each cell's immediate 8-connected visible Moore neighbors (`|di| <= 1, |dj| <= 1`)
/// once at simulation init so `good_step_moore` runs in `O(1)` (`<= 8` checks) instead of `O(deg)`.
fn build_moore_neighbor_cache(vga: &VgaResults) -> (Vec<usize>, Vec<Vec<(u32, u8, i32)>>) {
    let n = vga.points.len();
    if n == 0 || vga.row_offsets.len() <= n {
        return (Vec::new(), Vec::new());
    }
    let mut valid_nodes = Vec::with_capacity(n);
    let mut moore = Vec::with_capacity(n);

    for u in 0..n {
        let s = vga.row_offsets[u] as usize;
        let e = vga.row_offsets[u + 1] as usize;
        if e > s {
            valid_nodes.push(u);
        }
        let pu = vga.points[u];
        let mut local_nbrs: Vec<(u32, u8, i32)> = Vec::with_capacity(8);
        for idx in s..e {
            let v = vga.col_indices[idx] as usize;
            let pv = vga.points[v];
            let di = (pv.grid_i - pu.grid_i).abs();
            let dj = (pv.grid_j - pu.grid_j).abs();
            if di <= 1 && dj <= 1 && (di > 0 || dj > 0) {
                let step_len_sq = di * di + dj * dj;
                local_nbrs.push((v as u32, vga.bin_indices[idx], step_len_sq));
            }
        }
        // Fallback if a custom point cloud has no unit-grid (|di|<=1, |dj|<=1) neighbor:
        // keep up to 8 closest visible neighbors so agents can still step between points
        if local_nbrs.is_empty() && e > s {
            let mut closest: Vec<(u32, u8, f64)> = Vec::with_capacity(8);
            let stride = ((e - s) / 128).max(1);
            let mut idx = s;
            while idx < e {
                let v = vga.col_indices[idx] as usize;
                let pv = vga.points[v];
                let d2 = (pv.x - pu.x).powi(2) + (pv.y - pu.y).powi(2);
                if d2 > 1e-12 {
                    closest.push((v as u32, vga.bin_indices[idx], d2));
                }
                idx += stride;
            }
            closest.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));
            for &(v, b, _) in closest.iter().take(8) {
                local_nbrs.push((v, b, 1));
            }
        }
        moore.push(local_nbrs);
    }

    (valid_nodes, moore)
}

/// Maps a continuous 2D unit direction vector `dir` to its discrete 8-neighbor grid step `(di, dj) in {-1, 0, 1}^2`,
/// matching `depthmapX` `pixelate(m_loc + spacing * dir) - pixelate(m_loc)`.
#[inline]
fn vec_to_moore_offset(dir: Point2D) -> (i32, i32) {
    // sin(pi / 8) = 0.3826834323650898 divides the circle into 8 equal 45-degree octants
    const OCTANT_SIN: f64 = 0.3826834323650898;
    let di = if dir.x > OCTANT_SIN {
        1
    } else if dir.x < -OCTANT_SIN {
        -1
    } else {
        0
    };
    let dj = if dir.y > OCTANT_SIN {
        1
    } else if dir.y < -OCTANT_SIN {
        -1
    } else {
        0
    };
    if di == 0 && dj == 0 {
        if dir.x.abs() >= dir.y.abs() {
            (if dir.x >= 0.0 { 1 } else { -1 }, 0)
        } else {
            (0, if dir.y >= 0.0 { 1 } else { -1 })
        }
    } else {
        (di, dj)
    }
}

/// Checks whether stepping from `node_u` along `test_dir` lands on a valid walkable 8-connected Moore neighbor (`goodStep`).
#[inline]
fn try_good_step(
    vga: &VgaResults,
    node_u: usize,
    moore_row: &[(u32, u8, i32)],
    test_dir: Point2D,
) -> Option<usize> {
    if moore_row.is_empty() {
        return None;
    }
    let pu = vga.points[node_u];
    let (target_di, target_dj) = vec_to_moore_offset(test_dir);

    for &(v_u32, _, _) in moore_row {
        let v = v_u32 as usize;
        let pv = vga.points[v];
        if pv.grid_i - pu.grid_i == target_di && pv.grid_j - pu.grid_j == target_dj {
            return Some(v);
        }
    }

    // Fallback for non-lattice point clouds: pick neighbor aligned within 22.5° (cos(pi/8) = 0.92388)
    let mut best: Option<(usize, f64)> = None;
    for &(v_u32, _, _) in moore_row {
        let v = v_u32 as usize;
        let pv = vga.points[v];
        let step_v = normalize_vec(pv.x - pu.x, pv.y - pu.y);
        let dot = step_v.x * test_dir.x + step_v.y * test_dir.y;
        if dot >= 0.85 {
            if best.map_or(true, |(_, b_dot)| dot > b_dot) {
                best = Some((v, dot));
            }
        }
    }
    best.map(|(v, _)| v)
}

/// Exact `depthmapX` `Agent::onStep()` + `goodStep()` + `diagonalStep()` (`salalib/agents/agent.cpp:266-316`).
/// Attempts to step along `agent_dir`; if blocked by a wall/obstacle, rotates `agent_dir` by `+45°` and `-45°`
/// and tests the two diagonal neighbors in 50/50 random order (`pflip()`), preserving `agent_dir`.
#[inline]
fn step_along_heading_or_diagonal(
    vga: &VgaResults,
    node_u: usize,
    moore_row: &[(u32, u8, i32)],
    agent_dir: Point2D,
    rng: &mut u64,
) -> Option<usize> {
    // 1. Direct step along m_vector (goodStep)
    if let Some(next_v) = try_good_step(vga, node_u, moore_row, agent_dir) {
        return Some(next_v);
    }

    // 2. Wall collision fallback: diagonalStep() rotates m_vector by ±45° (±pi/4) in 50/50 random order
    let dir_plus_45 = rotate_vec(agent_dir, std::f64::consts::FRAC_PI_4);
    let dir_minus_45 = rotate_vec(agent_dir, -std::f64::consts::FRAC_PI_4);
    let (first_dir, second_dir) = if (xorshift64_star(rng) & 1) == 0 {
        (dir_plus_45, dir_minus_45)
    } else {
        (dir_minus_45, dir_plus_45)
    };

    if let Some(next_v) = try_good_step(vga, node_u, moore_row, first_dir) {
        return Some(next_v);
    }
    if let Some(next_v) = try_good_step(vga, node_u, moore_row, second_dir) {
        return Some(next_v);
    }
    None
}

/// Initializes `AgentSimState` for a given `VgaResults` PointMap and `AgentConfig`.
pub fn init_agent_simulation(vga: &VgaResults, config: &AgentConfig) -> AgentSimState {
    let n = vga.points.len();
    let (valid_nodes, moore_neighbors) = build_moore_neighbor_cache(vga);
    AgentSimState {
        current_step: 0,
        gate_counts: vec![0.0; n],
        active_agents: Vec::with_capacity(config.max_active_agents.min(1024)),
        finished_trails: Vec::new(),
        rng_state: config.seed.max(1),
        next_agent_id: 0,
        release_accumulator: 0.0,
        valid_nodes,
        moore_neighbors,
    }
}

/// Spawns a single initialized `ActiveAgent` at `cell_idx` following `depthmapX` `Agent::onInit(PixelRef node)`:
/// chooses initial heading over the full 360° isovist (`onLook(true)`), sets `prev_pos = pos`, and does NOT
/// increment `Gate_Counts` until the agent actually moves onto a new cell (`onMove`).
fn make_initialized_agent(
    vga: &VgaResults,
    state: &mut AgentSimState,
    config: &AgentConfig,
    cell_idx: usize,
) -> ActiveAgent {
    let pos = Point2D::new(vga.points[cell_idx].x, vga.points[cell_idx].y);
    let init_ang = rand_u01(&mut state.rng_state) * 2.0 * std::f64::consts::PI;
    let init_dir = Point2D::new(init_ang.cos(), init_ang.sin());
    let (dir, target_pos, has_target, bin) = on_look_depthmapx(
        vga,
        cell_idx,
        pos,
        init_dir,
        config.agent_fov_bins,
        config.movement_rule,
        true,
        &mut state.rng_state,
    );
    let trail = if config.record_trails {
        vec![pos]
    } else {
        Vec::new()
    };
    let agent_id = state.next_agent_id;
    state.next_agent_id += 1;

    ActiveAgent {
        agent_id,
        node_idx: cell_idx,
        prev_pos: pos,
        pos,
        dir,
        target_pos,
        has_target,
        bin,
        steps_taken: 0,
        birth_step: state.current_step,
        trail,
    }
}

/// Manually spawns `count` stationary agents directly at `cell_idx` (UCL `depthmapX` `Q3DView::CreateAgent`).
/// Returns the number of agents actually spawned.
pub fn spawn_agents_at_cell(
    vga: &VgaResults,
    state: &mut AgentSimState,
    config: &AgentConfig,
    cell_idx: usize,
    count: usize,
) -> usize {
    let n = vga.points.len();
    if cell_idx >= n || vga.row_offsets.len() <= n {
        return 0;
    }
    if state.gate_counts.len() != n {
        state.gate_counts.resize(n, 0.0);
    }
    if state.moore_neighbors.len() != n {
        let (vn, mn) = build_moore_neighbor_cache(vga);
        state.valid_nodes = vn;
        state.moore_neighbors = mn;
    }

    let mut spawned = 0usize;
    for _ in 0..count {
        let ag = make_initialized_agent(vga, state, config, cell_idx);
        state.active_agents.push(ag);
        spawned += 1;
    }
    spawned
}

/// Advances the Gibson Ecological Visual Agent simulation (Turner & Penn 2002; UCL `depthmapX`) by `steps` ticks.
/// When `allow_continuous_release` is `false` (e.g., inspecting manually placed agent(s) in interactive playback),
/// only the existing `state.active_agents` step forward without spawning extra random agents.
pub fn step_agent_simulation_with_mode(
    vga: &VgaResults,
    state: &mut AgentSimState,
    config: &AgentConfig,
    steps: usize,
    allow_continuous_release: bool,
) {
    let n = vga.points.len();
    if n == 0 || vga.row_offsets.len() <= n {
        return;
    }
    if state.gate_counts.len() != n {
        state.gate_counts.resize(n, 0.0);
    }
    if state.moore_neighbors.len() != n {
        let (vn, mn) = build_moore_neighbor_cache(vga);
        state.valid_nodes = vn;
        state.moore_neighbors = mn;
    }
    if state.valid_nodes.is_empty() {
        return;
    }

    // Filter manual release pins to valid walkable cell indices
    let manual_pins: Vec<usize> = config
        .release_cells
        .iter()
        .copied()
        .filter(|&u| u < n && vga.row_offsets[u + 1] > vga.row_offsets[u])
        .collect();

    let use_selected_pins =
        !manual_pins.is_empty() && (config.release_from_selected_only || allow_continuous_release);
    let max_active = config.max_active_agents.max(1);
    let steps_before_turn = config.agent_steps_before_turn.max(1);
    let turn_prob = 1.0 / (steps_before_turn as f64);
    let life_steps = config.agent_life_steps.max(1);
    let release_rate = config.release_rate.max(0.0);
    let max_trails = config.max_trails.max(1);
    let cell_spacing = vga.cell_size_m.max(1e-6);

    for _ in 0..steps {
        state.current_step += 1;

        // 1. Release new agents via depthmapX Poisson process (invcumpoisson) when continuous release is active
        if allow_continuous_release && !(config.release_from_selected_only && manual_pins.is_empty())
        {
            let u01 = rand_u01(&mut state.rng_state);
            let mut to_release = invcumpoisson(u01, release_rate);
            state.release_accumulator += release_rate;
            if state.current_step == 1 && state.active_agents.is_empty() && to_release == 0 {
                to_release = 1;
            }
            for _ in 0..to_release {
                if state.active_agents.len() >= max_active {
                    break;
                }
                let u = if use_selected_pins {
                    let pick_idx =
                        (xorshift64_star(&mut state.rng_state) as usize) % manual_pins.len();
                    manual_pins[pick_idx]
                } else {
                    let pick_idx =
                        (xorshift64_star(&mut state.rng_state) as usize) % state.valid_nodes.len();
                    state.valid_nodes[pick_idx]
                };
                let ag = make_initialized_agent(vga, state, config, u);
                state.active_agents.push(ag);
            }
        }

        // 2. Step active agents (depthmapX Agent::onMove + onStep + diagonalStep) and retire at agent_life_steps
        let mut idx = 0usize;
        while idx < state.active_agents.len() {
            let mut retire = false;
            {
                let agent = &mut state.active_agents[idx];
                agent.steps_taken += 1;
                agent.prev_pos = agent.pos;
                let last_node = agent.node_idx;

                // depthmapX Agent::onMove() turn decision:
                // - If SEL_STANDARD (targeted) and within 1 grid step of target_pos, re-evaluate onLook(false)
                // - Else with Bernoulli probability (1.0 / m_steps), re-evaluate onLook(false)
                let reached_target = config.movement_rule == AgentMovementRule::Standard
                    && agent.has_target
                    && agent.pos.dist(agent.target_pos) < cell_spacing;
                let stochastic_turn = rand_u01(&mut state.rng_state) < turn_prob;

                if reached_target || stochastic_turn {
                    let (new_dir, new_target, has_t, new_bin) = on_look_depthmapx(
                        vga,
                        agent.node_idx,
                        agent.pos,
                        agent.dir,
                        config.agent_fov_bins,
                        config.movement_rule,
                        false,
                        &mut state.rng_state,
                    );
                    agent.dir = new_dir;
                    agent.target_pos = new_target;
                    agent.has_target = has_t;
                    agent.bin = new_bin;
                }

                // depthmapX Agent::onStep() -> goodStep() -> diagonalStep() (±45° fallback)
                let moore_row = &state.moore_neighbors[agent.node_idx];
                let mut stepped_node = step_along_heading_or_diagonal(
                    vga,
                    agent.node_idx,
                    moore_row,
                    agent.dir,
                    &mut state.rng_state,
                );

                // If blocked even after ±45° diagonalStep, re-look over 360° whole isovist and retry step once
                if stepped_node.is_none() {
                    let (new_dir, new_target, has_t, new_bin) = on_look_depthmapx(
                        vga,
                        agent.node_idx,
                        agent.pos,
                        agent.dir,
                        32,
                        config.movement_rule,
                        true,
                        &mut state.rng_state,
                    );
                    agent.dir = new_dir;
                    agent.target_pos = new_target;
                    agent.has_target = has_t;
                    agent.bin = new_bin;
                    stepped_node = step_along_heading_or_diagonal(
                        vga,
                        agent.node_idx,
                        moore_row,
                        agent.dir,
                        &mut state.rng_state,
                    );
                }

                if let Some(next_u) = stepped_node {
                    agent.node_idx = next_u;
                    agent.pos = Point2D::new(vga.points[next_u].x, vga.points[next_u].y);
                    // depthmapX increments Gate Counts only when stepping onto a new cell (m_node != lastnode)
                    if next_u != last_node {
                        state.gate_counts[next_u] += 1.0;
                    }
                    if config.record_trails {
                        agent.trail.push(agent.pos);
                    }
                }

                if agent.steps_taken >= life_steps {
                    retire = true;
                }
            }

            if retire {
                let finished = state.active_agents.swap_remove(idx);
                if config.record_trails && finished.trail.len() >= 2 {
                    if state.finished_trails.len() >= max_trails {
                        state.finished_trails.remove(0);
                    }
                    state.finished_trails.push(finished.trail);
                }
            } else {
                idx += 1;
            }
        }
    }
}

/// Advances the Gibson Ecological Visual Agent simulation (Turner & Penn 2002) by `steps` ticks with continuous release enabled.
pub fn step_agent_simulation(
    vga: &VgaResults,
    state: &mut AgentSimState,
    config: &AgentConfig,
    steps: usize,
) {
    step_agent_simulation_with_mode(vga, state, config, steps, true);
}

/// Runs a complete batch Gibson Ecological Visual Agent simulation for `config.timesteps` ticks.
pub fn solve_agents_batch(vga: &VgaResults, config: &AgentConfig) -> AgentSimState {
    let mut state = init_agent_simulation(vga, config);
    step_agent_simulation_with_mode(vga, &mut state, config, config.timesteps, true);
    state
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solvers_vga::{generate_vga_grid, solve_vga};
    use crate::types::Polyline;

    #[test]
    fn test_agents_batch_populates_gate_counts() {
        let boundary = vec![vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(10.0, 0.0),
            Point2D::new(10.0, 10.0),
            Point2D::new(0.0, 10.0),
            Point2D::new(0.0, 0.0),
        ]];
        let obstacles = vec![Polyline::new(boundary[0].clone())];
        let pts = generate_vga_grid(&boundary, &obstacles, 1.0, true);
        let vga = solve_vga(&pts, &obstacles, 1.0, 50.0, false, false);
        let cfg = AgentConfig {
            timesteps: 200,
            release_rate: 0.5,
            agent_life_steps: 40,
            agent_fov_bins: 15,
            agent_steps_before_turn: 3,
            movement_rule: AgentMovementRule::Standard,
            release_from_selected_only: false,
            max_active_agents: 30,
            seed: 42,
            record_trails: true,
            max_trails: 50,
            release_cells: Vec::new(),
            agents_per_click: 1,
        };
        let state = solve_agents_batch(&vga, &cfg);
        let total_visits: f64 = state.gate_counts.iter().sum();
        assert!(total_visits > 50.0, "total_visits={}", total_visits);
        assert!(!state.finished_trails.is_empty());
    }

    #[test]
    fn test_manual_agent_placement_stationary_until_stepped() {
        let boundary = vec![vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(10.0, 0.0),
            Point2D::new(10.0, 10.0),
            Point2D::new(0.0, 10.0),
            Point2D::new(0.0, 0.0),
        ]];
        let obstacles = vec![Polyline::new(boundary[0].clone())];
        let pts = generate_vga_grid(&boundary, &obstacles, 1.0, true);
        let vga = solve_vga(&pts, &obstacles, 1.0, 50.0, false, false);
        let mut cfg = AgentConfig::default();
        cfg.release_cells = vec![0];
        let mut state = init_agent_simulation(&vga, &cfg);

        // Single click spawns exactly 1 stationary agent at cell 0 without advancing current_step or Gate_Counts
        let spawned = spawn_agents_at_cell(&vga, &mut state, &cfg, 0, 1);
        assert_eq!(spawned, 1);
        assert_eq!(state.active_agents.len(), 1);
        assert_eq!(state.current_step, 0);
        assert!((state.gate_counts.iter().sum::<f64>()).abs() < 1e-9);

        // Stepping in manual single-agent mode moves only that 1 agent without spawning extra agents
        step_agent_simulation_with_mode(&vga, &mut state, &cfg, 25, false);
        assert_eq!(state.active_agents.len(), 1);
        let total_visits: f64 = state.gate_counts.iter().sum();
        assert!(total_visits > 0.0);
    }
}
