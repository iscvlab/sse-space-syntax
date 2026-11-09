# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core: Agent-Based Spatial Simulation Engine (Full depthmapX v0.8.0 Parity)
Implements Turner & Penn (2002) Ecological Visual Agents (`salalib/agents/agent.cpp` & `agentengine.cpp`):
1. Exact 32-bin non-uniform octant directional quantization (`whichbin` & `binfromvec`).
2. 15-bin (~170°) forward binocular field of view (`onStandardLook`).
3. Probabilistic turn decision every `steps_before_turn` steps (`1 / m_steps`).
4. 8-neighbor visibility-constrained stepping (`goodStep`) with ±45° diagonal deflection (`diagonalStep`).
5. Poisson agent release per timestep, `Gate_Counts` cell occupancy accumulation, and `Agent Trails` polylines.
"""

import math
from typing import List, Tuple, Dict, Any, Optional
import numpy as np
import shapely
from shapely.geometry import Point, LineString
from shapely.strtree import STRtree


def _whichbin_scalar(dx: float, dy: float) -> int:
    """Exact depthmapX salalib/pointdata.h whichbin(grad) 32-bin mapping."""
    b = 1 if abs(dy) > abs(dx) else 0
    if b == 0:
        if abs(dx) < 1e-15:
            return 0
        ratio = abs(dy) / abs(dx)
        if dx > 0.0:
            b = 0 if dy >= 0.0 else -32
        else:
            b = -16 if dy >= 0.0 else 16
    else:
        ratio = abs(dx) / abs(dy)
        if dy > 0.0:
            b = -8 if dx >= 0.0 else 8
        else:
            b = 24 if dx >= 0.0 else -24

    if ratio < 1e-12:
        pass
    elif ratio < 0.2679491924311227:  # < tan(15 deg)
        b += 1
    elif ratio < 0.5773502691896257:  # < tan(30 deg)
        b += 2
    elif ratio < 1.0 - 1e-12:         # < tan(45 deg)
        b += 3
    else:
        b += 4

    if b < 0:
        b = -b
    return b % 32


def solve_agent_analysis(
    grid_points: List[Point],
    obstacle_lines: List[LineString],
    timesteps: int = 10000,
    release_rate: float = 0.1,
    fov_bins: int = 15,
    steps_before_turn: int = 3,
    agent_lifetime: int = 1000,
    record_trails_count: int = 50,
    cell_size_m: float = 1.0,
    origin_xy: Optional[Tuple[float, float]] = None,
    vis_pairs: Optional[Tuple[np.ndarray, np.ndarray]] = None,
    seed: int = 42
) -> Dict[str, Any]:
    """
    Executes exact depthmapX v0.8.0 AgentEngine simulation on a VGA grid.
    Returns dictionary containing:
      - 'Gate_Counts': np.ndarray of shape (N,) with total step visits per cell
      - 'agent_trails': List[LineString] of recorded individual agent trajectories
    """
    N = len(grid_points)
    if N == 0:
        return {"Gate_Counts": np.zeros(0, dtype=np.float64), "agent_trails": []}

    coords = np.array([[float(pt.x), float(pt.y)] for pt in grid_points], dtype=np.float64)
    if origin_xy is not None:
        x0, y0 = float(origin_xy[0]), float(origin_xy[1])
    else:
        x0 = math.floor(float(np.min(coords[:, 0])) / cell_size_m) * cell_size_m
        y0 = math.floor(float(np.min(coords[:, 1])) / cell_size_m) * cell_size_m

    ix = np.round((coords[:, 0] - x0) / cell_size_m).astype(np.int32)
    iy = np.round((coords[:, 1] - y0) / cell_size_m).astype(np.int32)

    # 1. Build or reuse Line-of-Sight visibility pairs
    if vis_pairs is not None:
        u_arr, v_arr = vis_pairs
    else:
        ii, jj = np.triu_indices(N, k=1)
        if obstacle_lines:
            walls_union = shapely.union_all(obstacle_lines)
            pair_segs = shapely.linestrings(np.stack([coords[ii], coords[jj]], axis=1))
            seg_tree = STRtree(pair_segs)
            blocked_idx = seg_tree.query(walls_union, predicate="intersects")
            vis_mask = np.ones(len(ii), dtype=bool)
            vis_mask[blocked_idx] = False
            u_arr = ii[vis_mask]
            v_arr = jj[vis_mask]
        else:
            u_arr = ii
            v_arr = jj

    # 2. Build 2D grid index lookup and 32-bin directional visibility table
    min_ix = min(0, int(np.min(ix)))
    min_iy = min(0, int(np.min(iy)))
    off_x = -min_ix
    off_y = -min_iy
    W = int(np.max(ix)) + off_x + 3
    H = int(np.max(iy)) + off_y + 3
    grid_lookup = np.full((W, H), -1, dtype=np.int32)
    for idx_pt in range(N):
        grid_lookup[int(ix[idx_pt]) + off_x, int(iy[idx_pt]) + off_y] = idx_pt

    bins: List[List[List[int]]] = [[[] for _ in range(32)] for _ in range(N)]
    good_step = np.zeros((N, 3, 3), dtype=np.bool_)

    ix_py = [int(v) for v in ix]
    iy_py = [int(v) for v in iy]

    for u, v in zip(u_arr, v_arr):
        u_i, v_i = int(u), int(v)
        for a, b_node in ((u_i, v_i), (v_i, u_i)):
            dx = ix_py[b_node] - ix_py[a]
            dy = iy_py[b_node] - iy_py[a]
            b_idx = _whichbin_scalar(float(dx), float(dy))
            bins[a][b_idx].append(b_node)
            if abs(dx) <= 1 and abs(dy) <= 1:
                good_step[a, dx + 1, dy + 1] = True

    # 3. Run depthmapX AgentEngine loop
    rng = np.random.default_rng(seed)
    vbin = -1 if fov_bins >= 32 else max(0, (int(fov_bins) - 1) // 2)
    steps_turn = max(1, int(steps_before_turn))
    lifetime = max(1, int(agent_lifetime))
    gate_counts = np.zeros(N, dtype=np.float64)

    COS45 = math.sqrt(0.5)
    SIN45 = math.sqrt(0.5)

    def on_standard_look(node: int, vx: float, vy: float, whole: bool) -> Tuple[float, float, bool]:
        if whole or vbin == -1:
            dbin = 0
            span = 32
        else:
            ang = math.atan2(vy, vx)
            if ang < 0.0:
                ang += 2.0 * math.pi
            ahead = int(32.0 * (0.5 * ang / math.pi) + 0.5) % 32
            dbin = (32 + ahead - vbin) % 32
            span = min(32, vbin * 2 + 1)

        b_node_bins = bins[node]
        choices = 0
        for i in range(span):
            choices += len(b_node_bins[(dbin + i) % 32])
        if choices == 0:
            if not whole:
                return on_standard_look(node, vx, vy, True)
            return 0.0, 0.0, True

        chosen = int(rng.integers(0, choices))
        b_cur = dbin
        while chosen >= len(b_node_bins[b_cur % 32]):
            chosen -= len(b_node_bins[b_cur % 32])
            b_cur += 1
        tar = b_node_bins[b_cur % 32][chosen]
        return float(coords[tar, 0]), float(coords[tar, 1]), False

    trails_coords: List[List[Tuple[float, float]]] = [[] for _ in range(max(0, int(record_trails_count)))]
    next_trail_id = 0 if record_trails_count > 0 else -1

    # Active agent record: [node_idx, loc_x, loc_y, vec_x, vec_y, frame, stuck, trail_id]
    agents: List[List[Any]] = []

    for _ in range(int(timesteps)):
        q = int(rng.poisson(release_rate))
        for _ in range(q):
            nd = int(rng.integers(0, N))
            lx, ly = float(coords[nd, 0]), float(coords[nd, 1])
            tx, ty, stuck = on_standard_look(nd, 1.0, 0.0, True)
            dx, dy = tx - lx, ty - ly
            L = math.hypot(dx, dy)
            vx, vy = (dx / L, dy / L) if L > 1e-12 else (1.0, 0.0)
            t_id = next_trail_id
            if next_trail_id != -1:
                next_trail_id += 1
                if next_trail_id >= record_trails_count:
                    next_trail_id = -1
            agents.append([nd, lx, ly, vx, vy, 0, stuck, t_id])

        # Move backwards matching AgentSet::move()
        for idx_a in range(len(agents) - 1, -1, -1):
            ag = agents[idx_a]
            ag[5] += 1
            if rng.random() < (1.0 / steps_turn):
                tx, ty, stuck = on_standard_look(ag[0], ag[3], ag[4], False)
                if not stuck:
                    dx, dy = tx - ag[1], ty - ag[2]
                    L = math.hypot(dx, dy)
                    if L > 1e-12:
                        ag[3], ag[4] = dx / L, dy / L
                else:
                    ag[6] = True

            if not ag[6]:
                last_nd = ag[0]
                nx_loc = ag[1] + cell_size_m * ag[3]
                ny_loc = ag[2] + cell_size_m * ag[4]
                nix = int(math.floor((nx_loc - x0) / cell_size_m + 0.5))
                niy = int(math.floor((ny_loc - y0) / cell_size_m + 0.5))
                gx, gy = nix + off_x, niy + off_y
                next_nd = int(grid_lookup[gx, gy]) if (0 <= gx < W and 0 <= gy < H) else -1

                stopped = False
                if next_nd == last_nd:
                    ag[1], ag[2] = nx_loc, ny_loc
                else:
                    ddx = nix - ix_py[last_nd]
                    ddy = niy - iy_py[last_nd]
                    if next_nd != -1 and abs(ddx) <= 1 and abs(ddy) <= 1 and good_step[last_nd, ddx + 1, ddy + 1]:
                        ag[0], ag[1], ag[2] = next_nd, nx_loc, ny_loc
                        gate_counts[next_nd] += 1.0
                    else:
                        # diagonalStep (±45 deg rotation fallback)
                        vx, vy = ag[3], ag[4]
                        v1x, v1y = vx * COS45 - vy * SIN45, vx * SIN45 + vy * COS45
                        v2x, v2y = vx * COS45 + vy * SIN45, -vx * SIN45 + vy * COS45
                        pair = ((v1x, v1y), (v2x, v2y)) if int(rng.integers(0, 2)) == 0 else ((v2x, v2y), (v1x, v1y))
                        stepped_diag = False
                        for dvx, dvy in pair:
                            dlx = ag[1] + cell_size_m * dvx
                            dly = ag[2] + cell_size_m * dvy
                            dix = int(math.floor((dlx - x0) / cell_size_m + 0.5))
                            diy = int(math.floor((dly - y0) / cell_size_m + 0.5))
                            dgx, dgy = dix + off_x, diy + off_y
                            dnd = int(grid_lookup[dgx, dgy]) if (0 <= dgx < W and 0 <= dgy < H) else -1
                            ddx2 = dix - ix_py[last_nd]
                            ddy2 = diy - iy_py[last_nd]
                            if (
                                dnd != -1 and abs(ddx2) <= 1 and abs(ddy2) <= 1
                                and good_step[last_nd, ddx2 + 1, ddy2 + 1]
                            ):
                                ag[0], ag[1], ag[2] = dnd, dlx, dly
                                gate_counts[dnd] += 1.0
                                stepped_diag = True
                                break
                        if not stepped_diag:
                            stopped = True

                if not stopped and ag[7] != -1:
                    trails_coords[ag[7]].append((ag[1], ag[2]))

            if ag[5] >= lifetime:
                agents.pop(idx_a)

    trail_lines: List[LineString] = [
        LineString(pts) for pts in trails_coords if len(pts) >= 2
    ]

    return {
        "Gate_Counts": gate_counts,
        "agent_trails": trail_lines,
    }


class PedestrianAgent:
    def __init__(self, agent_id: int, start_pos: Tuple[float, float], heading_rad: float):
        self.agent_id = agent_id
        self.pos = start_pos
        self.heading = heading_rad
        self.steps_until_decision = 3
        self.history: List[Tuple[float, float]] = [start_pos]


class AgentSimulationEngine:
    """Backwards-compatible wrapper around solve_agent_analysis."""
    def __init__(
        self,
        grid_points: List[Point],
        obstacle_lines: List[LineString],
        fov_deg: float = 170.0,
        step_size_m: float = 1.0,
        momentum_steps: int = 3
    ):
        self.grid_points = grid_points
        self.obstacle_lines = obstacle_lines
        self.fov_bins = max(1, min(32, int(round(32.0 * (fov_deg / 360.0)))))
        self.step_size_m = step_size_m
        self.momentum_steps = momentum_steps

    def run_simulation(
        self,
        num_agents: int = 100,
        total_ticks: int = 200,
        gates: Optional[List[LineString]] = None
    ) -> Tuple[np.ndarray, Dict[int, int]]:
        res = solve_agent_analysis(
            self.grid_points,
            self.obstacle_lines,
            timesteps=max(100, total_ticks),
            release_rate=max(0.05, float(num_agents) / float(max(1, total_ticks))),
            fov_bins=self.fov_bins,
            steps_before_turn=self.momentum_steps,
            agent_lifetime=total_ticks,
            record_trails_count=min(50, num_agents),
            cell_size_m=self.step_size_m
        )
        gate_crossings: Dict[int, int] = {}
        if gates:
            for g_idx, gate in enumerate(gates):
                cnt = 0
                for tr in res["agent_trails"]:
                    if tr.intersects(gate):
                        cnt += 1
                gate_crossings[g_idx] = cnt
        return res["Gate_Counts"].astype(np.int32), gate_crossings
