import heapq
import math
from collections import deque
import ext_plant

# ---------------------------------------------------------------------
# Global Config & Constants
# ---------------------------------------------------------------------
id = ["00000000"]

MOVES = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}
MOVE_VECTORS = list(MOVES.items())
INF = 10 ** 18
DIST_INF = 10 ** 9


# ---------------------------------------------------------------------
# Core Logic: Pathfinding Engine
# ---------------------------------------------------------------------
class Node:
    """Wrapper for A* states."""
    __slots__ = ('state', 'cost', 'prev', 'op')

    def __init__(self, state, cost=0, prev=None, op=None):
        self.state = state
        self.cost = cost
        self.prev = prev
        self.op = op


def solve_scenario(model):
    """Executes the search algorithm on the provided model."""
    root = model.get_start_state()

    if model.check_solved(root):
        return []

    # Priority Queue: (f_score, g_score, tie_breaker, node_obj)
    open_set = []
    tie_breaker = 0

    initial_h = model.estimate_cost(root)
    start_node = Node(root, 0, None, None)

    heapq.heappush(open_set, (initial_h, 0, tie_breaker, start_node))

    min_cost_tracker = {root: 0}

    while open_set:
        _, current_g, _, current_node = heapq.heappop(open_set)
        current_state = current_node.state

        # Lazy deletion check
        if current_g != min_cost_tracker.get(current_state, INF):
            continue

        if model.check_solved(current_state):
            # Backtrack path
            path_sequence = []
            trace = current_node
            while trace.prev is not None:
                path_sequence.append(trace.op)
                trace = trace.prev
            return path_sequence[::-1]

        # Expand neighbors
        for action_name, next_state in model.get_transitions(current_state):
            next_g = current_g + 1
            if next_g < min_cost_tracker.get(next_state, INF):
                min_cost_tracker[next_state] = next_g
                tie_breaker += 1
                f_score = next_g + model.estimate_cost(next_state)
                new_node = Node(next_state, next_g, current_node, action_name)
                heapq.heappush(open_set, (f_score, next_g, tie_breaker, new_node))

    return []


# ---------------------------------------------------------------------
# Core Logic: Simulation Model
# ---------------------------------------------------------------------
class GridSimulation:
    def __init__(self, dim, barriers, agents_map, crops_map, water_map, temp_obs=None):
        self.H, self.W = dim
        self.static_obs = set(barriers)
        self.dynamic_obs = set(temp_obs) if temp_obs else set()

        # Canonize positions
        self.crop_locs = tuple(sorted(crops_map.keys()))
        self.src_locs = tuple(sorted(water_map.keys()))

        # Lookups
        self.c_idx = {loc: i for i, loc in enumerate(self.crop_locs)}
        self.s_idx = {loc: i for i, loc in enumerate(self.src_locs)}

        # Static Data
        self.agent_caps = {aid: c for aid, (_, _, _, c) in agents_map.items()}

        # Initial State Construction
        raw_agents = []
        for aid, (r, c, load, _) in agents_map.items():
            raw_agents.append((aid, r, c, load))
        raw_agents.sort()  # Sort by ID

        self.start_agents = tuple(raw_agents)
        self.start_crops = tuple(crops_map[p] for p in self.crop_locs)
        self.start_srcs = tuple(water_map[t] for t in self.src_locs)
        self.start_sum = sum(self.start_crops)

        self._path_cache = {}

    def get_start_state(self):
        return (self.start_agents, self.start_crops, self.start_srcs, self.start_sum)

    def check_solved(self, state):
        return state[3] == 0

    # --- Geometry & Distance ---
    def _compute_bfs(self, origin):
        if origin in self._path_cache:
            return

        frontier = deque([origin])
        depths = {origin: 0}

        while frontier:
            r, c = frontier.popleft()
            d = depths[(r, c)]

            for _, (dr, dc) in MOVE_VECTORS:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.H and 0 <= nc < self.W:
                    pt = (nr, nc)
                    if pt in self.static_obs or pt in self.dynamic_obs:
                        continue
                    if pt not in depths:
                        depths[pt] = d + 1
                        frontier.append(pt)
        self._path_cache[origin] = depths

    def get_travel_cost(self, p1, p2):
        if p1 == p2: return 0
        self._compute_bfs(p1)
        return self._path_cache[p1].get(p2, DIST_INF)

    # --- Heuristics ---
    def estimate_cost(self, state):
        agents, crops, sources, total_rem = state
        if total_rem == 0:
            return 0

        pending_crops = [self.crop_locs[i] for i, val in enumerate(crops) if val > 0]
        if not pending_crops:
            return 0

        active_sources = [self.src_locs[i] for i, val in enumerate(sources) if val > 0]
        if not active_sources:
            return total_rem + 10000

        current_load = sum(l for (_, _, _, l) in agents)
        shortage = max(0, total_rem - current_load)

        max_path_cost = 0
        for crop_pos in pending_crops:
            min_robot_cost = DIST_INF
            for (aid, r, c, load) in agents:
                agent_pos = (r, c)
                if load > 0:
                    val = self.get_travel_cost(agent_pos, crop_pos)
                else:
                    # Cost: Move to source -> Refill(1) -> Move to crop
                    dist_to_src = min(self.get_travel_cost(agent_pos, t) for t in active_sources)
                    dist_src_to_crop = min(self.get_travel_cost(t, crop_pos) for t in active_sources)
                    val = dist_to_src + 1 + dist_src_to_crop

                if val < min_robot_cost:
                    min_robot_cost = val

            if min_robot_cost > max_path_cost:
                max_path_cost = min_robot_cost

        # Formula: Total + Shortage + MaxCost + TieBreaker
        return total_rem + shortage + max_path_cost + (max_path_cost / 100.0)

    # --- Transitions ---
    def get_transitions(self, state):
        agents, crops, sources, total_rem = state
        transitions = []

        occupied_locs = {(r, c) for (_, r, c, _) in agents}

        for i, (aid, r, c, load) in enumerate(agents):
            others_locs = occupied_locs - {(r, c)}

            # 1. Movement
            for move_name, (dr, dc) in MOVE_VECTORS:
                nr, nc = r + dr, c + dc
                next_pos = (nr, nc)

                # Bounds check
                if not (0 <= nr < self.H and 0 <= nc < self.W): continue
                # Collision check
                if next_pos in self.static_obs or next_pos in self.dynamic_obs: continue
                if next_pos in others_locs: continue

                updated_agents = list(agents)
                updated_agents[i] = (aid, nr, nc, load)
                transitions.append((f"{move_name}({aid})", (tuple(updated_agents), crops, sources, total_rem)))

            # 2. Loading
            if (r, c) in self.s_idx:
                s_ptr = self.s_idx[(r, c)]
                if sources[s_ptr] > 0 and load < self.agent_caps[aid]:
                    mod_sources = list(sources)
                    mod_sources[s_ptr] -= 1
                    mod_agents = list(agents)
                    mod_agents[i] = (aid, r, c, load + 1)
                    transitions.append((f"LOAD({aid})", (tuple(mod_agents), crops, tuple(mod_sources), total_rem)))

            # 3. Pouring
            if (r, c) in self.c_idx:
                c_ptr = self.c_idx[(r, c)]
                if crops[c_ptr] > 0 and load > 0:
                    mod_crops = list(crops)
                    mod_crops[c_ptr] -= 1
                    mod_agents = list(agents)
                    mod_agents[i] = (aid, r, c, load - 1)
                    transitions.append((f"POUR({aid})", (tuple(mod_agents), tuple(mod_crops), sources, total_rem - 1)))

        return transitions


# ---------------------------------------------------------------------
# Core Logic: Bot Controller
# ---------------------------------------------------------------------
class Controller:
    def __init__(self, game_interface: ext_plant.Game):
        self.engine = game_interface
        self.raw_data = self.engine.get_problem()

        self.grid_size = self.raw_data["Size"]
        self.fixed_walls = set(self.raw_data.get("Walls", set()))
        self.max_loads = self.engine.get_capacities()

        # Probability & Reliability
        probs = dict(self.raw_data["robot_chosen_action_prob"])
        self.reliable_ids = {rid for rid, p in probs.items() if p >= 0.75}

        # Leader Selection
        def _score_bot(rid):
            # Prioritize: 1. Prob bucket (10%), 2. Capacity, 3. Raw Prob
            return (int(probs[rid] / 0.1), self.max_loads.get(rid, 0), probs[rid])

        self.primary_id = max(probs.keys(), key=_score_bot)

        if not self.reliable_ids:
            self.reliable_ids = {self.primary_id}

        # Internal State
        self.mode = "FULL_MAP"
        self.focus_crop = None
        self.temp_barriers = set()
        self.dead_crops = set()

        self.action_queue = []
        self.queue_idx = 0

        self.pos_tracker = {}
        self.just_reset = True
        self.crop_baseline = set()

        # Initialize logic
        self._generate_plan()

    # --- Helpers ---
    def _get_agent_state(self, agents_tuple, target_id):
        for rid, pos, ld in agents_tuple:
            if rid == target_id:
                return pos, ld
        return None, 0

    def _check_occupancy(self, agents_tuple, loc, ignore_id):
        for rid, pos, _ in agents_tuple:
            if pos == loc and rid != ignore_id:
                return rid
        return None

    def _calc_next_pos(self, current, move_key):
        dr, dc = MOVES[move_key]
        return (current[0] + dr, current[1] + dc)

    def _bfs_water_dist(self, start, water_tuples):
        # BFS ignoring dynamic blocks
        q = deque([start])
        dists = {start: 0}
        active_taps = {tp for tp, amt in water_tuples if amt > 0}

        while q:
            curr = q.popleft()
            d = dists[curr]
            if curr in active_taps:
                return d

            r, c = curr
            for _, (dr, dc) in MOVES.items():
                nr, nc = r + dr, c + dc
                nxt = (nr, nc)
                if 0 <= nr < self.grid_size[0] and 0 <= nc < self.grid_size[1]:
                    if nxt not in self.fixed_walls and nxt not in dists:
                        dists[nxt] = d + 1
                        q.append(nxt)
        return DIST_INF

    def _is_move_valid(self, op, agent_id, world_state):
        agents, crops, taps, _ = world_state
        curr_pos, curr_load = self._get_agent_state(agents, agent_id)

        if curr_pos is None: return False

        if op in MOVES:
            # Check Geometry & Collision
            target = self._calc_next_pos(curr_pos, op)
            if not (0 <= target[0] < self.grid_size[0] and 0 <= target[1] < self.grid_size[1]):
                return False
            if target in self.fixed_walls or target in self.temp_barriers:
                return False
            if self._check_occupancy(agents, target, ignore_id=agent_id) is not None:
                return False
            return True

        elif op == "LOAD":
            available = 0
            for t_pos, amt in taps:
                if t_pos == curr_pos:
                    available = amt
                    break

            if available <= 0 or curr_load >= self.max_loads[agent_id]:
                return False

            # Smart Loading Limit
            if self.focus_crop is not None:
                req = 0
                for c_pos, n in crops:
                    if c_pos == self.focus_crop:
                        req = n
                        break

                # Buffer for failure
                fail_rate = 1.0 - dict(self.raw_data["robot_chosen_action_prob"]).get(agent_id, 1.0)
                smart_cap = min(int(req * 1.25) + 1, self.max_loads[agent_id])
                if curr_load >= smart_cap:
                    return False
            return True

        elif op == "POUR":
            if curr_load <= 0: return False
            for c_pos, need in crops:
                if c_pos == curr_pos and need > 0:
                    return True
            return False

        return False

    # --- Strategy Setup ---
    def _setup_problem_instance(self, world_state):
        agents, crops, taps, _ = world_state

        # Analyze Rewards
        rewards_map = self.raw_data["plants_reward"]
        avg_rewards = {p: (sum(vals) / len(vals)) for p, vals in rewards_map.items() if vals}
        r_vals = list(avg_rewards.values()) or [0.0]

        is_greedy_map = (max(r_vals) > 2.0 * min(r_vals))
        map_area = self.grid_size[0] * self.grid_size[1]

        # Mode Selection
        self.mode = "GREEDY_PLANT" if (map_area >= 20 or is_greedy_map) else "FULL_MAP"

        # Filter Entities
        sub_crops = {}
        self.focus_crop = None

        if self.mode == "GREEDY_PLANT":
            # Select single best plant
            leader_xy, _ = self._get_agent_state(agents, self.primary_id)
            best_p, max_score = None, -INF

            for p_loc, amt in crops:
                if amt <= 0 or p_loc in self.dead_crops:
                    continue

                dist_est = self._bfs_water_dist(leader_xy, taps) + abs(p_loc[0] - leader_xy[0]) + abs(
                    p_loc[1] - leader_xy[1])
                score = avg_rewards.get(p_loc, 0.0) * 10.0 - min(dist_est, 10000)

                if score > max_score:
                    max_score, best_p = score, p_loc

            if best_p:
                sub_crops[best_p] = next(n for p, n in crops if p == best_p)
                self.focus_crop = best_p
        else:
            # Take all active
            for p_loc, amt in crops:
                if amt > 0 and p_loc not in self.dead_crops:
                    sub_crops[p_loc] = amt

        # Filter Agents
        active_ids = {self.primary_id} if self.mode == "GREEDY_PLANT" else set(self.reliable_ids)

        sim_agents = {}
        for rid, pos, load in agents:
            if rid in active_ids:
                sim_agents[rid] = (pos[0], pos[1], load, self.max_loads[rid])

        # Prepare Taps (Planner needs current levels)
        sim_taps = {t: amt for t, amt in taps}

        return sim_agents, sub_crops, sim_taps

    def _generate_plan(self):
        curr_state = self.engine.get_current_state()
        p_agents, p_crops, p_taps = self._setup_problem_instance(curr_state)

        sim = GridSimulation(
            dim=self.grid_size,
            barriers=self.fixed_walls,
            agents_map=p_agents,
            crops_map=p_crops,
            water_map=p_taps,
            temp_obs=self.temp_barriers
        )
        self.action_queue = solve_scenario(sim)
        self.queue_idx = 0

    def _trigger_reset(self):
        self.action_queue = []
        self.queue_idx = 0
        self.temp_barriers.clear()
        self.dead_crops.clear()
        self.pos_tracker.clear()
        self.just_reset = True
        self.focus_crop = None
        return "RESET"

    def _fallback_move(self, state):
        agents, _, _, _ = state
        xy, _ = self._get_agent_state(agents, self.primary_id)

        if xy is None: return self._trigger_reset()

        for m_key in MOVES:
            if self._is_move_valid(m_key, self.primary_id, state):
                self.pos_tracker[self.primary_id] = self._calc_next_pos(xy, m_key)
                return f"{m_key}({self.primary_id})"

        return self._trigger_reset()

    # --- Main Loop ---
    def choose_next_action(self, full_state):
        agents, crops, taps, _ = full_state
        active_now = {p for p, n in crops if n > 0}

        # 1. Post-Reset Cleanup
        if self.just_reset:
            self.crop_baseline = active_now
            self.just_reset = False
            self.pos_tracker.clear()

        # 2. Slip Check
        has_slipped = False
        for rid in self.reliable_ids:
            if rid in self.pos_tracker:
                real_pos, _ = self._get_agent_state(agents, rid)
                if real_pos is not None and real_pos != self.pos_tracker[rid]:
                    has_slipped = True
                    break
        if has_slipped:
            self._generate_plan()
            self.pos_tracker.clear()

        # 3. Greedy Strategy Checks
        if self.mode == "GREEDY_PLANT":
            # If a plant finished, reset to re-evaluate
            if len(active_now) < len(self.crop_baseline):
                return self._trigger_reset()

            # Teleport optimization
            l_pos, l_load = self._get_agent_state(agents, self.primary_id)
            if l_pos is not None and l_load == 0:
                start_coords = tuple(self.raw_data["Robots"][self.primary_id][0:2])
                dist_start = self._bfs_water_dist(start_coords, taps)
                dist_curr = self._bfs_water_dist(l_pos, taps)
                if dist_start < dist_curr:
                    return self._trigger_reset()

        # 4. Execute Plan
        while self.queue_idx < len(self.action_queue):
            cmd = self.action_queue[self.queue_idx]
            # Parse cmd: "UP(0)"
            action_type = cmd.split("(")[0]
            agent_id = int(cmd.split("(")[1].rstrip(")"))

            curr_pos, curr_load = self._get_agent_state(agents, agent_id)
            if curr_pos is None:
                self.queue_idx += 1
                continue

            # Opportunistic Fill
            if action_type in MOVES:
                is_on_water = any(t == curr_pos and amt > 0 for t, amt in taps)
                if is_on_water:
                    cap = self.max_loads[agent_id]
                    desired = cap
                    if self.focus_crop is not None:
                        # Find specific need
                        spec_need = 0
                        for cp, cn in crops:
                            if cp == self.focus_crop: spec_need = cn; break
                        desired = min(cap, max(0, spec_need))

                    if curr_load < desired and self._is_move_valid("LOAD", agent_id, full_state):
                        self.pos_tracker[agent_id] = curr_pos
                        return f"LOAD({agent_id})"

                # Blocker Logic
                target_cell = self._calc_next_pos(curr_pos, action_type)
                blocker_id = self._check_occupancy(agents, target_cell, ignore_id=agent_id)
                if blocker_id is not None:
                    self.temp_barriers.add(target_cell)
                    self._generate_plan()
                    return self._fallback_move(full_state)

            if not self._is_move_valid(action_type, agent_id, full_state):
                self.queue_idx += 1
                continue

            # Update tracking
            if action_type in MOVES:
                self.pos_tracker[agent_id] = self._calc_next_pos(curr_pos, action_type)
            else:
                self.pos_tracker[agent_id] = curr_pos

            self.queue_idx += 1
            return cmd

        # 5. Plan Exhausted
        self._generate_plan()
        if not self.action_queue:
            return self._fallback_move(full_state)

        # Attempt start of new plan
        next_cmd = self.action_queue[0]
        act_t = next_cmd.split("(")[0]
        aid = int(next_cmd.split("(")[1].rstrip(")"))

        if self._is_move_valid(act_t, aid, full_state):
            c_pos, _ = self._get_agent_state(agents, aid)
            if c_pos is not None:
                if act_t in MOVES:
                    self.pos_tracker[aid] = self._calc_next_pos(c_pos, act_t)
                else:
                    self.pos_tracker[aid] = c_pos
            self.queue_idx = 1
            return next_cmd

        return self._fallback_move(full_state)