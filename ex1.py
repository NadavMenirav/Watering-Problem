import ex1_check
import search
import utils

SIZE = "Size"
WALLS = "Walls"
TAPS = "Taps"
PLANTS = "Plants"
ROBOTS = "Robots"

id = ["No numbers - I'm special!"]

class State:
    size = (0, 0) # Size of the board. no reason to save it in every copy.
    walls: dict[tuple, bool] # The walls. no reason to save it in every copy.
    taps: dict[tuple, int]
    plants: dict[tuple, int]
    robots: dict[tuple, tuple]
    hash = None
    last_move = None
    plants_need = None
    robots_load = None


    def __init__(self, initial = None, size = None, walls = None, taps = None, plants = None, robots = None, last_move = None, plants_need = None, robots_load = None, taps_have = None):
        # If we construct using initial
        if initial is not None:
            State.size = initial[SIZE]
            State.walls = dict(((i, j), True) for (i, j) in initial[WALLS])
            self.taps = initial[TAPS]
            self.plants = initial[PLANTS]
            self.robots = dict(
                ((i, j), (id, load, capacity))
                for id, (i, j, load, capacity) in initial[ROBOTS].items()
            )
            self.hash = None
            self.last_move = None
            self.plants_need = sum(self.plants.values())
            self.robots_load = sum(load for (id, load, capacity) in self.robots.values())
            self.taps_have = sum(self.taps.values())

        # If we construct using size, walls, taps, plants, robots
        else:
            State.size = size
            State.walls = walls
            self.taps = taps
            self.plants = plants
            self.robots = robots
            self.hash = None
            self.last_move = last_move
            self.plants_need = plants_need
            self.robots_load = robots_load
            self.taps_have = taps_have

    def __hash__(self):
        if self.hash is not None:
            return self.hash

        self.hash = hash((
            tuple(sorted(self.taps.items())),
            tuple(sorted(self.plants.items())),
            tuple(sorted(self.robots.items())),
        ))

        return self.hash

    def __eq__(self, other):
        return (
                self.taps == other.taps and
                self.plants == other.plants and
                self.robots == other.robots)


class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""
    distances: dict[tuple[tuple[int, int], tuple[int, int]], int]

    def __init__(self, initial):
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        search.Problem.__init__(self, initial)
        self.initial = State(initial = initial)
        self.cache = {}
        self.distances = {}

        # size[0] is the height - number of rows
        # size[1] is the width - number of height
        height = self.initial.size[0]
        width = self.initial.size[1]

        # A matrix of the legal moves from every square.
        # Every entry represents the legal moves that can be made from the corresponding square.
        # It is a four booleans tuple:
        # 1st entry for moving up
        # 2nd entry for moving down
        # 3rd entry for moving left
        # 4th entry for moving right
        self.legal_moves = [
            [[False, False, False, False] for _ in range(width)]
            for _ in range(height)
        ]

        walls = self.initial.walls

        # Now we want to initialize the matrix with the legal moves available
        for x in range(height):
            for y in range(width):
                if x - 1 >= 0 and walls.get((x - 1, y)) is None:
                    self.legal_moves[x][y][0] = True
                if x + 1 < height and walls.get((x + 1, y)) is None:
                    self.legal_moves[x][y][1] = True
                if y - 1 >= 0 and walls.get((x, y - 1)) is None:
                    self.legal_moves[x][y][2] = True
                if y + 1 < width and walls.get((x, y + 1)) is None:
                    self.legal_moves[x][y][3] = True


    def BFS(self, coordinate: tuple[int, int]):
        x = coordinate[0]
        y = coordinate[1]

        q = (utils.FIFOQueue())
        q.append((coordinate, -1))
        closed = set()

        self.distances[(coordinate, coordinate)] = 0

        while len(q) > 0:
            current = q.pop()
            current_coordinate = current[0]
            x = current_coordinate[0]
            y = current_coordinate[1]
            parent_distance = current[1]

            if current_coordinate in closed:
                continue

            self.distances[(coordinate, current_coordinate)] = parent_distance + 1
            self.distances[(current_coordinate, coordinate)] = parent_distance + 1

            closed.add(current_coordinate)

            if self.legal_moves[x][y][0] and (x - 1, y) not in closed:
                q.append(((x - 1, y), parent_distance + 1))
            if self.legal_moves[x][y][1] and (x + 1, y) not in closed:
                q.append(((x + 1, y), parent_distance + 1))
            if self.legal_moves[x][y][2] and (x, y - 1) not in closed:
                q.append(((x, y - 1), parent_distance + 1))
            if self.legal_moves[x][y][3] and (x, y + 1) not in closed:
                q.append(((x, y + 1), parent_distance + 1))

    def bfs_distance(self, coordinate1: tuple[int, int], coordinate2: tuple[int, int]):
        distance = self.distances.get((coordinate1, coordinate2))
        if distance is not None:
            return distance

        self.BFS(coordinate1)
        return self.distances.get((coordinate1, coordinate2))


    def successor(self, state: State):

        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        number_of_robots = len(state.robots)
        possible_successors = []
        for (x, y), (id, load, capacity) in state.robots.items():

            # We want to check whether the robot has any WU on it
            if load > 0:

                # We now want to check whether the robot is on a plant it can water
                # We don't care whether there is a plant that needs 0 WU or if there isn't a plant at all
                water_needed_in_plant_under_robot = state.plants.get((x, y), 0)

                if water_needed_in_plant_under_robot > 0:
                    # Changing the robot's load
                    new_robot_key_tuple = (x, y)
                    new_robot_value_tuple = (id, load - 1, capacity)

                    # Changing the plant
                    new_plant_key_tuple = (x, y)
                    new_plant_value = water_needed_in_plant_under_robot - 1

                    # Creating the new state
                    move = f"POUR{{{id}}}"
                    new_state = State(size = state.size,
                                      walls = state.walls,
                                      taps = state.taps,
                                      plants = dict(state.plants),
                                      robots = dict(state.robots),
                                      last_move = move,
                                      plants_need = state.plants_need - 1,
                                      robots_load = state.robots_load - 1,
                                      taps_have = state.taps_have)

                    # Deleting the previous state of robot and inserting the new one
                    del new_state.robots[(x, y)]
                    new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                    # Deleting the previous state of plant and inserting the new one
                    del new_state.plants[(x, y)]
                    new_state.plants[new_plant_key_tuple] = new_plant_value

                    # Inserting the new state to the possible next states
                    if self.cache.get(new_state) is None:
                        possible_successors.append((move, new_state))

                    # If there is only one robot, there is no reason not to pour all he has onto the robot
                    if (number_of_robots == 1 and state.plants_need > 1) or water_needed_in_plant_under_robot == state.plants_need: continue


            # We now want to check whether the robot can load more WU from a tap
            if capacity - load > 0 and load <= sum(state.plants.values()):

                # We now want to check whether the robot is on a tap it can load water from
                # We don't care whether there is a tap that can give 0 WU or if there isn't a tap at all
                water_available_in_tap_under_robot = state.taps.get((x, y), 0)
                if water_available_in_tap_under_robot > 0:
                    # Changing the robot's load
                    new_robot_key_tuple = (x, y)
                    new_robot_value_tuple = (id, load + 1, capacity)

                    # Changing the tap
                    new_tap_key_tuple = (x, y)
                    new_tap_value = water_available_in_tap_under_robot - 1

                    # Creating the new state
                    move = f"LOAD{{{id}}}"
                    new_state = State(size = state.size,
                                      walls = state.walls,
                                      taps = dict(state.taps),
                                      plants = state.plants,
                                      robots = dict(state.robots),
                                      last_move = move,
                                      plants_need = state.plants_need,
                                      robots_load = state.robots_load + 1,
                                      taps_have = state.taps_have - 1)

                    # Deleting the previous state of robot and inserting the new one
                    del new_state.robots[(x, y)]
                    new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                    # Deleting the previous state of tap and inserting the new one
                    del new_state.taps[(x, y)]
                    new_state.taps[new_tap_key_tuple] = new_tap_value

                    # Inserting the new state to the possible next states
                    if self.cache.get(new_state) is None:
                        possible_successors.append((move, new_state))

                    # If there is one robot he should fill his tank until full
                    # Or until he has enough WU to water all plants
                    if (number_of_robots == 1 and capacity + 1 < state.plants_need) or (water_available_in_tap_under_robot == state.taps_have): continue


            # If the robot can move UP
            if self.legal_moves[x][y][0] and state.robots.get((x - 1, y)) is None and state.last_move != f"DOWN{{{id}}}":

                # Changing the robot's position
                new_robot_key_tuple = (x - 1,  y)
                new_robot_value_tuple = (id, load, capacity)

                # Creating the new state
                move = f"UP{{{id}}}"
                new_state = State(size = state.size,
                                  walls = state.walls,
                                  taps = state.taps,
                                  plants = state.plants,
                                  robots = dict(state.robots),
                                  last_move = move,
                                  plants_need = state.plants_need,
                                  robots_load = state.robots_load,
                                  taps_have = state.taps_have)
                del new_state.robots[(x, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                if self.cache.get(new_state) is None:
                    possible_successors.append((move, new_state))


            # If the robot can move DOWN
            if self.legal_moves[x][y][1] and state.robots.get((x + 1, y)) is None and state.last_move != f"UP{{{id}}}":

                # Changing the robot's position
                new_robot_key_tuple = (x + 1,  y)
                new_robot_value_tuple = (id, load, capacity)

                # Creating the new state
                move = f"DOWN{{{id}}}"
                new_state = State(size = state.size,
                                  walls = state.walls,
                                  taps = state.taps,
                                  plants = state.plants,
                                  robots = dict(state.robots),
                                  last_move = move,
                                  plants_need = state.plants_need,
                                  robots_load = state.robots_load,
                                  taps_have = state.taps_have)
                del new_state.robots[(x, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                if self.cache.get(new_state) is None:
                    possible_successors.append((move, new_state))


            # If the robot can move LEFT
            if self.legal_moves[x][y][2] and state.robots.get((x, y - 1)) is None and state.last_move != f"RIGHT{{{id}}}":

                # Changing the robot's position
                new_robot_key_tuple = (x,  y - 1)
                new_robot_value_tuple = (id, load, capacity)

                # Creating the new state
                move = f"LEFT{{{id}}}"
                new_state = State(size = state.size,
                                  walls = state.walls,
                                  taps = state.taps,
                                  plants = state.plants,
                                  robots = dict(state.robots),
                                  last_move = move,
                                  plants_need = state.plants_need,
                                  robots_load = state.robots_load,
                                  taps_have = state.taps_have)
                del new_state.robots[(x, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                if self.cache.get(new_state) is None:
                    possible_successors.append((move, new_state))

            # If the robot can move RIGHT
            if self.legal_moves[x][y][3] and state.robots.get((x, y + 1)) is None and state.last_move != f"LEFT{{{id}}}":

                # Changing the robot's position
                new_robot_key_tuple = (x,  y + 1)
                new_robot_value_tuple = (id, load, capacity)

                # Creating the new state
                move = f"RIGHT{{{id}}}"
                new_state = State(size = state.size,
                                  walls = state.walls,
                                  taps = state.taps,
                                  plants = state.plants,
                                  robots = dict(state.robots),
                                  last_move = move,
                                  plants_need = state.plants_need,
                                  robots_load = state.robots_load,
                                  taps_have = state.taps_have)

                # Deleting the robot from its previous position and adding the new position
                del new_state.robots[(x, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                if self.cache.get(new_state) is None:
                    possible_successors.append((move, new_state))

        return possible_successors


    def goal_test(self, state: State) -> bool:
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        # If there is a plant which still needs water, we have not reached the goal yet.
        for value in state.plants.values():
            if value != 0: return False

        # All plants are watered
        return True


    def h_astar(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        cache_val = self.cache.get(node.state)

        if cache_val is not None:
            return cache_val

        # This is an admissible heuristic, we need at least the remaining WU for the plants,
        # plus the remaining WU the robots need to load
        # In addition, we calculate the shortest path a robot need to do in order to reach a state it can water a plant
        wu_needed =  2 * node.state.plants_need - node.state.robots_load

        # This represents the shortest path any robot need to do in order to help
        shortest_path_to_water = float('inf')

        # This represents the shortest path the current robot need to do to help
        current_shortest = float('inf')

        # This represents the shortest path from the current robot to a tap
        current_shortest_path_to_tap = float('inf')

        # This represents the shortest path from the current robot to a plant
        current_shortest_path_to_plant = float('inf')

        # This represents the shortest path a robot can do from its point to a tap and then to a plant
        current_shortest_path_to_tap_then_plant = float('inf')

        for (x_robot, y_robot), (id, load, capacity) in node.state.robots.items():

            # For every robot, if the robot has WU on him, he can either go to a tap or go to a plant

            # Now we calculate the plant closest to him, if all plants are watered we return -1.
            # We use Manhattan distances
            if node.state.plants_need == 0:
                current_shortest_path_to_plant = -1
            else:
                current_shortest_path_to_plant = min(
                    (
                        self.bfs_distance((x_robot, y_robot), (x_plant, y_plant))
                        for ((x_plant, y_plant), remaining_wu) in node.state.plants.items()
                        if remaining_wu > 0
                    ),
                    default = -1
                )

            # If all plants are watered we return the heuristic 0
            if current_shortest_path_to_plant == -1:
                return 0

            # Now we calculate its closest tap
            if node.state.taps_have == 0:
                current_shortest_path_to_tap = -1
            else:
                current_shortest_path_to_tap = min(
                    (
                        self.bfs_distance((x_robot, y_robot), (x_tap, y_tap))
                        for ((x_tap, y_tap), remaining_wu) in node.state.taps.items()
                        if remaining_wu > 0
                    ),
                    default = -1
                )

            # If all taps are empty (and not all plants are fully watered) we have to go to a plant
            if current_shortest_path_to_tap == -1:

                # If there are no WU left in the taps, a robot which has WU on him need to go to the closest plant
                # And robots that have no WU on them, their shortest path is infinity
                if load > 0:
                    current_shortest = current_shortest_path_to_plant

                else:
                    current_shortest = float('inf')

            # If there are WU on the taps, the shortest path (but not always the best path) will be
            # 1. If a robot has WU on him, he will go to the nearest plant
            # 2. If a robot has no WU on him, in order to help he has to go to a near tap and then to a plant
            else:

                if load > 0:
                    current_shortest = current_shortest_path_to_plant

                else:

                    # Now we iterate over all pairs of tap and plant and pick the pair the robot should go

                    current_shortest_path_to_tap_then_plant = min(
                        self.bfs_distance((x_robot, y_robot), (x_tap, y_tap))
                        + self.bfs_distance((x_plant, y_plant), (x_tap, y_tap))
                        for ((x_tap, y_tap), remaining_wu_tap) in node.state.taps.items()
                        for ((x_plant, y_plant), remaining_wu_plant) in node.state.plants.items()
                        if remaining_wu_plant > 0 and remaining_wu_tap > 0
                    )

                    current_shortest = current_shortest_path_to_tap_then_plant

            if current_shortest < shortest_path_to_water: shortest_path_to_water = current_shortest

        heuristic = shortest_path_to_water + wu_needed

        self.cache[node.state] = heuristic
        return heuristic

    def h_gbfs(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""

        # This is an admissible heuristic, we need at least the remaining WU for the plants,
        # plus the remaining WU the robots need to load
        # In addition, we calculate the shortest path a robot need to do in order to reach a state it can water a plant
        return (2 * sum(node.state.plants.values())
                      - sum(load for (id, load, capacity) in node.state.robots.values()))





def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()
