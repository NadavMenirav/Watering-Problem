import ext_plant
import numpy as np
import bisect
from collections import deque

id = ["000000000"]

# --- 1. THE PRIORITY QUEUE ---
# A list that keeps itself sorted so the best option is always first.
class PriorityQueue:
    def __init__(self, f=lambda x: x):
        self.A = []
        self.f = f

    def append(self, item):
        # Insert the item in the correct order based on its cost (f)
        bisect.insort(self.A, (self.f(item), item))

    def pop(self):
        # Remove and return the item with the lowest cost
        return self.A.pop(0)[1]

    def __len__(self):
        return len(self.A)


# --- 2. THE NODE ---
# A wrapper that holds the state and remembers the path.
class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def __lt__(self, other):
        # Needed for sorting in the queue
        return self.path_cost < other.path_cost

    def expand(self, problem):
        # Create new nodes for every possible move
        return [
            Node(next_state, self, act, problem.path_cost(self.path_cost, self.state, act, next_state))
            for (act, next_state) in problem.successor(self.state)
        ]


# --- 3. THE SEARCH FUNCTION ---
def astar_search(problem, h=None):
    # Use the problem's heuristic if none is provided
    h = h or problem.h

    # Calculate Total Cost (f) = Past Cost (path_cost) + Future Guess (h)
    def f(n):
        return n.path_cost + h(n)

    # The list of nodes we need to check, sorted by cost
    frontier = PriorityQueue(f=f)
    frontier.append(Node(problem.initial))

    # Keep track of states we have already visited to avoid loops
    explored = set()

    while frontier:
        # Get the best node
        node = frontier.pop()

        # Check if we won
        if problem.goal_test(node.state):
            return node

        # Add to explored set (Must be a tuple to be hashable!)
        if node.state not in explored:
            explored.add(node.state)

            # Add all neighbors to the queue
            for child in node.expand(problem):
                if child.state not in explored:
                    frontier.append(child)

    return None

class Problem:

    # Defines the starting point of the problem and the goal
    def __init__(self, initial, goal = None):
        self.initial = initial
        self.goal = goal

    # This function is a placeholder. WateringProblem will overwrite it with the actual logic
    def successor(self, state):
        raise NotImplementedError


    # This function checks if we had reached our goal. It is naive and our WateringProblem class overrides it anyway
    def goal_test(self, state):
        return state == self.goal

    # By default, every move costs 1.
    def path_cost(self, c, state1, action, state2):
        return c + 1

    # The heuristic function.
    # By default, return 0.
    def h(self, node):
        return 0


# This class is a helper class to help us with running the A* algorithm
class WateringProblem(Problem):
    def __init__(self, state, original_game):

        # Initializing the parrent
        super().__init__(state)

        # Saving the starting state for the A*
        # State is (robots_t, plants_t, taps_t, total_water_need)
        self.initial = state

        # Saving the rules of the game
        self.walls = original_game.walls
        self.capacities = original_game.get_capacities()
        self.rows = original_game.rows
        self.cols = original_game.cols

        # The BFS distances
        self.distances = {}

        # A matrix of the legal moves from every square.
        # Every entry represents the legal moves that can be made from the corresponding square.
        # It is a four booleans tuple:
        # 1st entry for moving up
        # 2nd entry for moving down
        # 3rd entry for moving left
        # 4th entry for moving right
        self.legal_moves = [
            [[False, False, False, False] for _ in range(self.cols)]
            for _ in range(self.rows)
        ]

        # Filling the matrix
        for x in range(self.rows):
            for y in range(self.cols):
                # Check UP
                if x - 1 >= 0 and (x - 1, y) not in self.walls:
                    self.legal_moves[x][y][0] = True
                # Check DOWN
                if x + 1 < self.rows and (x + 1, y) not in self.walls:
                    self.legal_moves[x][y][1] = True
                # Check LEFT
                if y - 1 >= 0 and (x, y - 1) not in self.walls:
                    self.legal_moves[x][y][2] = True
                # Check RIGHT
                if y + 1 < self.cols and (x, y + 1) not in self.walls:
                    self.legal_moves[x][y][3] = True

    # This function is used in the A* to check if we had reached our goal.
    # The state we get have holds a total_water_need parameter which is 0 when we reach the goal
    def goal_test(self, state):
        return state[3] == 0 # The fourth parameter is the total_water_need

    def BFS(self, coordinate):
        x = coordinate[0]
        y = coordinate[1]

        q = deque()
        q.append((coordinate, -1))
        closed = set()

        self.distances[(coordinate, coordinate)] = 0

        while len(q) > 0:
            current = q.popleft()
            current_coordinate = current[0]
            x = current_coordinate[0]
            y = current_coordinate[1]
            parent_distance = current[1]

            if current_coordinate in closed:
                continue

            # Cache the distance both ways
            self.distances[(coordinate, current_coordinate)] = parent_distance + 1
            self.distances[(current_coordinate, coordinate)] = parent_distance + 1

            closed.add(current_coordinate)

            # Using the legal moves matrix
            if self.legal_moves[x][y][0] and (x - 1, y) not in closed:
                q.append(((x - 1, y), parent_distance + 1))
            if self.legal_moves[x][y][1] and (x + 1, y) not in closed:
                q.append(((x + 1, y), parent_distance + 1))
            if self.legal_moves[x][y][2] and (x, y - 1) not in closed:
                q.append(((x, y - 1), parent_distance + 1))
            if self.legal_moves[x][y][3] and (x, y + 1) not in closed:
                q.append(((x, y + 1), parent_distance + 1))


    # The wrapper function. You calculate the distance calling this function
    def bfs_distance(self, coordinate1, coordinate2):
        distance = self.distances.get((coordinate1, coordinate2))
        if distance is not None:
            return distance

        self.BFS(coordinate1) # Calculating the BFS from this point.

        # Infinity value just in case path is impossible
        return self.distances.get((coordinate1, coordinate2), float('inf'))


    # This function receives a point on the grid and returns a boolean value based on whether there is a robot in that
    # coordinate
    def is_coordinate_contain_robot(self, coordinate, robots, current_robot):

        robot_id, (r, c), _ = current_robot
        for id, (i, j), _ in robots:
            if coordinate == (i, j) and id != robot_id:
                return True
        return False


    # This function receives a point on the grid and returns a boolean value based on whether there is a wall in that
    # coordinate
    def is_coordinate_contain_wall(self, coordinate):
        return coordinate in self.walls


    # This function receives a point on the grid and returns a boolean value based on whether the point is a legal
    # point on the gird
    def is_on_grid(self, coordinate):
        (r, c) = coordinate
        return 0 <= r < self.rows and 0 <= c < self.cols


    # This function receives a point on the grid, the robot on that point, and all the plants, and returns whether the
    # robot can pour water.
    # There are three things that needs to be checked:
    # 1. There is a plant on that point
    # 2. The plant needs a positive number of WU
    # 3. The robot has WU on him
    def can_pour(self, moving_robot, plants):

        _, (r, c), load = moving_robot
        for (i, j), need in plants:
            if (r, c) == (i, j) and need > 0 and load > 0:
                return True
        return False


    # This function receives a point on the grid, the robot on that point, and all the taps, and returns whether the
    # robot can load water.
    # There are three things that needs to be checked:
    # 1. There is a tap on that point
    # 2. The tap has WU on it
    # 3. The robot's current load is smaller than its capacity
    def can_load(self, moving_robot, taps):

        robot_id, (r, c), load = moving_robot
        capacities = self.capacities
        capacity = capacities[robot_id]

        for (i, j), water in taps:
            if (r, c) == (i, j) and water > 0 and load < capacity:
                return True
        return False


    # This function receives an action and returns a boolean value based on whether the action is legal
    # For moving actions (UP, DOWN, LEFT, RIGHT) you need to check 3 things about the coordinate you move to:
    # 1. It is a legal point on the map
    # 2. It does not contain another robot
    # 3. It does not contain a wall
    def is_action_legal(self, state, action, moving_robot):

        (robots, plants, taps, total_water_needed) = state
        (r, c) = moving_robot[1] # The coordinate of the robot

        if action == "UP":
            return (
                self.is_on_grid((r - 1, c))
                and not self.is_coordinate_contain_robot((r - 1, c), robots, moving_robot)
                and not self.is_coordinate_contain_wall((r - 1, c))
            )

        if action == "DOWN":
            return (
                self.is_on_grid((r + 1, c))
                and not self.is_coordinate_contain_robot((r + 1, c), robots, moving_robot)
                and not self.is_coordinate_contain_wall((r + 1, c))
            )

        if action == "LEFT":
            return (
                self.is_on_grid((r, c - 1))
                and not self.is_coordinate_contain_robot((r, c - 1), robots, moving_robot)
                and not self.is_coordinate_contain_wall((r, c - 1))
            )

        if action == "RIGHT":
            return (
                self.is_on_grid((r, c + 1))
                and not self.is_coordinate_contain_robot((r, c + 1), robots, moving_robot)
                and not self.is_coordinate_contain_wall((r, c + 1))
            )

        if action == "POUR":
            return total_water_needed > 0 and self.can_pour(moving_robot, plants)

        if action == "LOAD":
            return self.can_load(moving_robot, taps) # Note: if total_water_needed == 0 no reason to load

        # Reset is always allowed
        if action == "RESET":
            return True

        return False



    # This function returns a list of all the possible next states
    def successor(self, state):

        # A list of the next possible states
        possible_successors = []

        # This represents our current state
        (robots, plants, taps, total_water_needed) = state

        # The directions the robot can do. We iterate over them to check which ones are valid
        # Each direction is a tuple.
        # 1st element is a string represents the movement
        # 2nd and 3rd element are the differences in x and y coordinates
        directions = [("UP", -1, 0), ("DOWN", 1, 0), ("LEFT", 0, -1), ("RIGHT", 0, 1)]

        # Iterating over all the robots, checking what moves can they do
        # We use enumerate so we know which robot is the one moving
        for i, robot in enumerate(robots):

            # Extracting the parameters of the robot
            robot_id, (r, c), load = robot

            # Iterating over all directions to check which ones are valid
            for action_name, dr, dc in directions:

                # If this direction is valid for this robot we add this action to the possible_successors
                if self.is_action_legal(state, action_name, robot):

                    # Convert the tuple to a list
                    new_robots = list(robots)

                    # Make the change
                    new_pos = (r + dr, c + dc)
                    new_robots[i] = (robot_id, new_pos, load)

                    # Convert back to tuple
                    new_state = (tuple(new_robots), plants, taps, total_water_needed)

                    # Adding to the possible successors the new state along with the action name
                    possible_successors.append((f"{action_name}({robot_id})", new_state))


            # Checking if LOAD is valid
            if self.is_action_legal(state, "LOAD", robot):

                # Convert the tuple to a list
                new_robots = list(robots)
                new_taps = list(taps)

                # Make the change
                new_load = load + 1
                new_robots[i] = (robot_id, (r, c), new_load)

                for idx, (t_pos, t_water) in enumerate(new_taps):
                    if t_pos == (r, c):
                        if t_water - 1 == 0:
                            del new_taps[idx]  # Remove empty tap
                        else:
                            new_taps[idx] = (t_pos, t_water - 1)
                        break


                # Convert back to tuple
                new_state = (tuple(new_robots), plants, tuple(new_taps), total_water_needed)

                # Adding to the possible successors the new state along with the action name
                possible_successors.append((f"LOAD({robot_id})", new_state))


            # Checking if POUR is valid
            if self.is_action_legal(state, "POUR", robot):

                # Convert the tuple to a list
                new_robots = list(robots)
                new_plants = list(plants)

                # Make the change
                new_load = load - 1
                new_robots[i] = (robot_id, (r, c), new_load)

                for idx, (p_pos, p_need) in enumerate(new_plants):
                    if p_pos == (r, c):
                        if p_need - 1 == 0:
                            del new_plants[idx]  # Remove satisfied plant
                        else:
                            new_plants[idx] = (p_pos, p_need - 1)
                        break


                # Convert back to tuple
                new_state = (tuple(new_robots), tuple(new_plants), taps, total_water_needed - 1)

                # Adding to the possible successors the new state along with the action name
                possible_successors.append((f"POUR({robot_id})", new_state))


        return possible_successors

    # This function returns the cost of an action. Used for the A* search
    # c = cost so far (to get to state1)
    # state1 = where we came from
    # action = what we did
    # state2 = where we ended up
    def path_cost(self, c, state1, action, state2):
         return c + 1

    # The heuristic function. For now, it is very simple, returns the total water needed.
    def h(self, node):

        # 1. Unpack the state tuple
        robots_t, plants_t, taps_t, total_needed = node.state

        # If we have delivered all water, the cost is 0. We are done.
        if total_needed == 0:
            return 0

        # 2. Get Targets
        # Since your successor function deletes finished plants/taps,
        # anything left in these tuples is "active". We just need their positions.
        active_plants = [pos for pos, need in plants_t]
        active_taps = [pos for pos, amount in taps_t]

        # Safety check: If no plants left, cost is 0
        if not active_plants:
            return 0

        # Calculate how much water the robots are holding right now
        total_load = 0
        for r_id, r_pos, load in robots_t:
            total_load += load

        # The Water Needed = (Total Needed) - (What we already have in our hands)
        # We can't have a negative need of course, so we take max(0, ...)
        water_needed = max(0, total_needed - total_load)

class Controller:
    """This class is a controller for the ext_plant game."""

    def __init__(self, game: ext_plant.Game):
        """Initialize controller for given game model."""
        self.original_game = game

        # Where we store the A* path
        self.current_plan = []


    def choose_next_action(self, state):
        """ Choose the next action given a state."""

        # If we have a plan from the A* list, we follow it
        if self.current_plan:
            move = self.current_plan.pop(0)
            return move

        # Need to run a new A* search
        problem = WateringProblem(state, self.original_game)

        # Running the search
        goal_node = astar_search(problem)

        # If we found a path, reconstruct it
        if goal_node:
            path = []
            node = goal_node


            while node.parent:
                path.append(node.action)
                node = node.parent

            # Reverse the path to get it from the start to the goal
            self.current_plan = path[::-1]

            # Returning the first move
            return self.current_plan.pop(0)

        # If the search failed we reset
        return "RESET"

