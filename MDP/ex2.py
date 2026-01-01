import ext_plant
import collections
import numpy as np

# TODO: Update your ID
id = ["000000000"]


class Controller:
    def __init__(self, game: ext_plant.Game):
        self.game = game
        self.problem = game.get_problem()

        # 1. Extract Probabilities
        self.robot_probs = self.problem.get('robot_chosen_action_prob', {})

        # 2. Pre-calculate Plant Mean Rewards
        raw_rewards = self.problem.get('plants_reward', {})
        self.plant_values = {}
        for pos, rewards in raw_rewards.items():
            if rewards:
                self.plant_values[pos] = sum(rewards) / len(rewards)
            else:
                self.plant_values[pos] = 0

    def is_coordinate_contain_robot(self, coordinate, robots, current_robot):
        robot_id, (r, c), _ = current_robot
        for id, (i, j), _ in robots:
            if coordinate == (i, j) and id != robot_id:
                return True
        return False

    def is_coordinate_contain_wall(self, coordinate):
        return coordinate in self.game.walls

    def is_on_grid(self, coordinate):
        (r, c) = coordinate
        return 0 <= r < self.game.rows and 0 <= c < self.game.cols

    def can_pour(self, moving_robot, plants):
        _, (r, c), load = moving_robot
        for (i, j), need in plants:
            if (r, c) == (i, j) and need > 0 and load > 0:
                return True
        return False

    def can_load(self, moving_robot, taps):
        robot_id, (r, c), load = moving_robot
        capacities = self.game.get_capacities()
        capacity = capacities[robot_id]
        for (i, j), water in taps:
            if (r, c) == (i, j) and water > 0 and load < capacity:
                return True
        return False

    def is_action_legal(self, state, action, moving_robot):
        (robots, plants, taps, total_water_needed) = state
        (r, c) = moving_robot[1]

        if action == "UP":
            return (self.is_on_grid((r - 1, c)) and
                    not self.is_coordinate_contain_robot((r - 1, c), robots, moving_robot) and
                    not self.is_coordinate_contain_wall((r - 1, c)))
        if action == "DOWN":
            return (self.is_on_grid((r + 1, c)) and
                    not self.is_coordinate_contain_robot((r + 1, c), robots, moving_robot) and
                    not self.is_coordinate_contain_wall((r + 1, c)))
        if action == "LEFT":
            return (self.is_on_grid((r, c - 1)) and
                    not self.is_coordinate_contain_robot((r, c - 1), robots, moving_robot) and
                    not self.is_coordinate_contain_wall((r, c - 1)))
        if action == "RIGHT":
            return (self.is_on_grid((r, c + 1)) and
                    not self.is_coordinate_contain_robot((r, c + 1), robots, moving_robot) and
                    not self.is_coordinate_contain_wall((r, c + 1)))
        if action == "POUR":
            return total_water_needed > 0 and self.can_pour(moving_robot, plants)
        if action == "LOAD":
            return self.can_load(moving_robot, taps)
        if action == "RESET":
            return True
        return False

    def bfs_distance(self, start_pos, target_pos, max_depth):
        """
        Performs a Breadth-First Search to find the true shortest path
        avoiding walls. Returns infinity if unreachable or blocked.
        """
        if start_pos == target_pos:
            return 0

        queue = collections.deque([(start_pos, 0)])
        visited = {start_pos}

        while queue:
            (r, c), dist = queue.popleft()

            # Optimization: Stop if we exceeded remaining time
            if dist > max_depth:
                return float('inf')

            if (r, c) == target_pos:
                return dist

            # Explore neighbors
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc

                # Check Bounds
                if 0 <= nr < self.game.rows and 0 <= nc < self.game.cols:
                    # Check Walls
                    if (nr, nc) not in self.game.walls:
                        if (nr, nc) not in visited:
                            visited.add((nr, nc))
                            queue.append(((nr, nc), dist + 1))

        return float('inf')

    def choose_next_action(self, state):
        (robots, plants, taps, total_water_need) = state
        plants_map = {p[0]: p[1] for p in plants}
        taps_set = {t[0] for t in taps}

        current_step = self.game.get_current_steps()
        max_steps = self.game.get_max_steps()
        remaining_steps = max_steps - current_step

        # --- STEP 1: COMPUTE CLAIMS (using BFS) ---
        # Determining who is truly closest to each target (respecting walls)
        target_claims = {}  # target -> (best_rid, min_dist)
        all_targets = list(plants_map.keys()) + list(taps_set)

        # This part is computationally heavier but necessary for coordination
        for target in all_targets:
            best_r = -1
            min_d = float('inf')

            for (rid, pos, _) in robots:
                # Use BFS for claim distance too
                d = self.bfs_distance(pos, target, remaining_steps)
                if d < min_d:
                    min_d = d
                    best_r = rid
            target_claims[target] = (best_r, min_d)

        # --- STEP 2: EVALUATE MOVES ---
        best_action = "RESET"
        best_score = -float('inf')

        for robot in robots:
            rid, (r, c), load = robot
            prob = self.robot_probs.get(rid, 1.0)

            # Generate legal actions
            legal_actions = []
            for act in ["UP", "DOWN", "LEFT", "RIGHT", "POUR", "LOAD"]:
                if self.is_action_legal(state, act, robot):
                    legal_actions.append(act)

            for act_name in legal_actions:
                score = 0

                # --- STRATEGY: IMMEDIATE ACTIONS ---
                if act_name == "POUR":
                    expected_reward = self.plant_values.get((r, c), 0)
                    score = 2000 + (expected_reward * prob)

                elif act_name == "LOAD":
                    if load == 0:
                        score = 1000 * prob
                    else:
                        score = -500

                        # --- STRATEGY: NAVIGATION (Using BFS) ---
                else:
                    # Where do we end up after this move?
                    dr, dc = 0, 0
                    if act_name == "UP":
                        dr = -1
                    elif act_name == "DOWN":
                        dr = 1
                    elif act_name == "LEFT":
                        dc = -1
                    elif act_name == "RIGHT":
                        dc = 1
                    nr, nc = r + dr, c + dc

                    if load > 0:
                        # HAVE WATER -> FIND PLANT
                        if not plants_map:
                            score = 0
                        else:
                            max_val = -float('inf')
                            found = False
                            for p_pos in plants_map.keys():
                                # 1. BFS Distance (Exact steps around walls)
                                # We start from (nr, nc), so max search depth is remaining_steps - 1
                                dist = self.bfs_distance((nr, nc), p_pos, remaining_steps)

                                # 2. Horizon Check
                                if dist >= remaining_steps:
                                    continue

                                found = True

                                # 3. Effective Cost (Steps / Probability)
                                # Penalize clumsy robots
                                eff_cost = dist / max(prob, 0.1)

                                p_reward = self.plant_values.get(p_pos, 0)
                                val = (p_reward * 10) - eff_cost

                                # 4. Competition Check
                                claim_id, claim_dist = target_claims.get(p_pos, (-1, 0))
                                if claim_id != -1 and claim_id != rid:
                                    # Compare: My BFS distance vs Winner's BFS distance
                                    # If I am significantly further (more than 1 step), back off
                                    if dist > claim_dist + 1:
                                        val -= 50

                                if val > max_val:
                                    max_val = val
                            score = max_val if found else -999

                    else:
                        # EMPTY -> FIND TAP
                        if not taps_set:
                            score = -9999
                        else:
                            max_val = -float('inf')
                            found = False
                            for t_pos in taps_set:
                                dist = self.bfs_distance((nr, nc), t_pos, remaining_steps)

                                if dist >= remaining_steps:
                                    continue

                                found = True
                                eff_cost = dist / max(prob, 0.1)
                                val = -eff_cost

                                claim_id, claim_dist = target_claims.get(t_pos, (-1, 0))
                                if claim_id != -1 and claim_id != rid:
                                    if dist > claim_dist + 1:
                                        val -= 20

                                if val > max_val:
                                    max_val = val
                            score = max_val if found else -999

                if score > best_score:
                    best_score = score
                    best_action = f"{act_name}({rid})"

        return best_action