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
    robots_last_moves: dict[int, str]
    hash = None
    last_move = None
    plants_need = None
    robots_load = None
    objective: tuple[int, int]
    initial_cords: tuple[int, int]
    active_only: bool


    # We know that the first robot that loads is the only one loading and pouring until he pours all his WU
    # Other robots can only move if they block him
    current_active_robot = None


    def __init__(self, initial = None, size = None, walls = None, taps = None, plants = None, robots = None,
                 last_move = None, plants_need = None, robots_load = None, taps_have = None, robot_last_moves = None,
                 current_active_robot = None, objective = None, active_only = False, initial_cords = None):
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
            self.robots_last_moves = dict((id, "None") for id in initial[ROBOTS].keys())
            self.hash = None
            self.last_move = None
            self.plants_need = sum(self.plants.values())
            self.robots_load = sum(load for (id, load, capacity) in self.robots.values())
            self.taps_have = sum(self.taps.values())
            self.current_active_robot = None
            self.objective = None
            self.active_only = False
            self.initial_cords = None

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
            self.robots_last_moves = robot_last_moves
            self.taps_have = taps_have
            self.current_active_robot = current_active_robot
            self.objective = objective
            self.active_only = active_only
            self.initial_cords = initial_cords

    def __hash__(self):
        if self.hash is not None:
            return self.hash

        self.hash = hash((
            tuple(sorted(self.taps.items())),
            tuple(sorted(self.plants.items())),
            tuple(sorted(self.robots.items())),
            self.objective,
            self.current_active_robot,
            self.active_only,
        ))

        return self.hash

    def __eq__(self, other):
        return (
                self.taps == other.taps and
                self.plants == other.plants and
                self.robots == other.robots and
                self.objective == other.objective and
                self.active_only == other.active_only and
                self.current_active_robot == other.current_active_robot)



class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""
    distances: dict[tuple[tuple[int, int], tuple[int, int]], int]
    bfs_paths: dict[tuple[tuple[int, int], tuple[int, int]], set[tuple[int, int]]]

    def __init__(self, initial):
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        search.Problem.__init__(self, initial)
        self.initial = State(initial = initial)
        self.cache = {}
        self.distances = {}
        self.bfs_paths = {}

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


        # BFS paths



        # Initializing the bfs for the plants and taps
        for ((i, j), needed) in self.initial.plants.items():
            self.BFS((i, j))
        for ((i, j), have) in self.initial.taps.items():
            self.BFS((i, j))

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
        self.bfs_paths[(coordinate, coordinate)] = {coordinate}

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
                self.bfs_paths[(coordinate, (x - 1, y))] = self.bfs_paths[(coordinate, current_coordinate)].union(
                    {(x - 1, y)}
                )

            if self.legal_moves[x][y][1] and (x + 1, y) not in closed:
                q.append(((x + 1, y), parent_distance + 1))
                self.bfs_paths[(coordinate, (x + 1, y))] = self.bfs_paths[(coordinate, current_coordinate)].union(
                    {(x + 1, y)}
                )

            if self.legal_moves[x][y][2] and (x, y - 1) not in closed:
                q.append(((x, y - 1), parent_distance + 1))
                self.bfs_paths[(coordinate, (x, y - 1))] = self.bfs_paths[(coordinate, current_coordinate)].union(
                    {(x, y - 1)}
                )

            if self.legal_moves[x][y][3] and (x, y + 1) not in closed:
                q.append(((x, y + 1), parent_distance + 1))
                self.bfs_paths[(coordinate, (x, y + 1))] = self.bfs_paths[(coordinate, current_coordinate)].union(
                    {(x, y + 1)}
                )


    def bfs_distance(self, coordinate1: tuple[int, int], coordinate2: tuple[int, int]):
        distance = self.distances.get((coordinate1, coordinate2))
        if distance is not None:
            return distance

        self.BFS(coordinate1)
        return self.distances.get((coordinate1, coordinate2))


    def successor(self, state: State):

        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        number_of_robots = len(state.robots)
        number_of_taps = len(state.taps)
        current_active = state.current_active_robot


        possible_successors = []
        for (x, y), (id, load, capacity) in state.robots.items():

            # We really want to only the active robot to move! If we are not on an active only state, maybe we should
            # be at one, let's check! It will save us a lot of headache in this iteration
            if not state.active_only and id == current_active:

                path_to_objective = self.bfs_paths.get((state.objective, (x, y))) or set()

                is_active_only = True
                for (x_else, y_else) in state.robots.keys():
                    if (x_else, y_else) != (x, y) and (x_else, y_else) in path_to_objective:
                        is_active_only = False

                state.active_only = is_active_only


            # If it's king-only mode, and I'm not the king: TOO BAD!
            if state.active_only and id != current_active:
                continue



            # What a miracle! We have reached the objective!
            # The objective was either a plant or a tap, so we first need to do the thing we got here for of course
            # After that, we need a new objective...
            # But don't worry! We would still be able to load/pour from our current objective before going to the new
            # one.
            if id == current_active and (x, y) == state.objective:

                # Now we check if the current coordinate is a plant. we don't care if there isn't a plant
                # or if there is a plant which doesn't need water anymore
                water_needed_in_plant_under_robot = state.plants.get((x, y), 0)

                # POUR! After that, create new states with all the different objectives!
                if water_needed_in_plant_under_robot > 0 and load > 0:

                    # Changing the robot's load
                    new_robot_key_tuple = (x, y)
                    new_robot_value_tuple = (id, load - 1, capacity)

                    # Changing the plant
                    new_plant_key_tuple = (x, y)
                    new_plant_value = water_needed_in_plant_under_robot - 1

                    # Creating the new state

                    new_active_robot = None if load - 1 == 0 else current_active
                    move = f"POUR{{{id}}}"

                    # I have lost my crown! but a new robot will be crowned as the new active state
                    # But this is not my job, it's the job of the next iteration!
                    # I just need to let it know that it should do it
                    if new_active_robot is None:
                        new_state = State(size = state.size,
                                          walls = state.walls,
                                          taps = state.taps,
                                          plants = dict(state.plants),
                                          robots = dict(state.robots),
                                          last_move = move,
                                          plants_need = state.plants_need - 1,
                                          robots_load = state.robots_load - 1,
                                          taps_have = state.taps_have,
                                          robot_last_moves = state.robots_last_moves,
                                          current_active_robot = None,
                                          objective = None,
                                          active_only = False,
                                          initial_cords = None,)

                        # robot last moves
                        del new_state.robots_last_moves[id]
                        new_state.robots_last_moves[id] = "POUR"

                        # Deleting the previous state of robot and inserting the new one
                        del new_state.robots[(x, y)]
                        new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                        # Deleting the previous state of plant and inserting the new one
                        del new_state.plants[(x, y)]
                        if new_plant_value > 0:
                            new_state.plants[new_plant_key_tuple] = new_plant_value

                        # Inserting the new state to the possible next states
                        if self.cache.get(new_state) is None:
                            possible_successors.append((move, new_state))

                        continue

                    # Phew, I'm still the active robot. But now I need to create a lot of new states for my next
                    # objectives:
                    if new_active_robot is not None:

                        # First let's start with the plants
                        # We need to be careful not to put the current objective as one of the next objectives
                        for (i, j) in state.plants.keys():

                            # If this objective is the same previous one.
                            if (x, y) == (i ,j): continue

                            new_state = State(size=state.size,
                                              walls=state.walls,
                                              taps=state.taps,
                                              plants=dict(state.plants),
                                              robots=dict(state.robots),
                                              last_move=move,
                                              plants_need=state.plants_need - 1,
                                              robots_load=state.robots_load - 1,
                                              taps_have=state.taps_have,
                                              robot_last_moves=state.robots_last_moves,
                                              current_active_robot=id, # I'm still the active robot
                                              objective=(i, j),
                                              active_only=False, # I don't know if it's only me or not yet
                                              initial_cords=(x, y), )

                            # robot last moves
                            del new_state.robots_last_moves[id]
                            new_state.robots_last_moves[id] = "POUR"

                            # Deleting the previous state of robot and inserting the new one
                            del new_state.robots[(x, y)]
                            new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                            # Deleting the previous state of plant and inserting the new one
                            del new_state.plants[(x, y)]
                            if new_plant_value > 0:
                                new_state.plants[new_plant_key_tuple] = new_plant_value

                            # Inserting the new state to the possible next states
                            if self.cache.get(new_state) is None:
                                possible_successors.append((move, new_state))


                        # Okay we are done with the plants as the next objectives!
                        # Now let's do the same thing but for the taps

                        for (i, j) in state.taps.keys():

                            # If this objective is the same previous one.
                            if (x, y) == (i ,j): continue

                            new_state = State(size=state.size,
                                              walls=state.walls,
                                              taps=state.taps,
                                              plants=dict(state.plants),
                                              robots=dict(state.robots),
                                              last_move=move,
                                              plants_need=state.plants_need - 1,
                                              robots_load=state.robots_load - 1,
                                              taps_have=state.taps_have,
                                              robot_last_moves=state.robots_last_moves,
                                              current_active_robot=id, # I'm still the active robot
                                              objective=(i, j),
                                              active_only=False, # I don't know if it's only me or not yet
                                              initial_cords=(x, y), )

                            # robot last moves
                            del new_state.robots_last_moves[id]
                            new_state.robots_last_moves[id] = "POUR"

                            # Deleting the previous state of robot and inserting the new one
                            del new_state.robots[(x, y)]
                            new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                            # Deleting the previous state of plant and inserting the new one
                            del new_state.plants[(x, y)]
                            if new_plant_value > 0:
                                new_state.plants[new_plant_key_tuple] = new_plant_value

                            # Inserting the new state to the possible next states
                            if self.cache.get(new_state) is None:
                                possible_successors.append((move, new_state))



                    # That was a lot of new states filled with a lot of new objectives.
                    # Now I know that I need to continue, I added all the cases in which my objective was a plant
                    # Now for the easier part: What if my objective was not a plant and was a tap!
                    # I now need to repeat the process, kind of exhausting, but we are near the end
                    continue

                # Now we know that the objective was a tap, let's check how many WU it has
                water_available_in_tap_under_robot = state.taps.get((x, y), 0)

                # LOAD!!! After that, create the new states
                # We know that we are still the king! we had WU and now we are loading more!
                # So there is no reason to crown anyone else
                if water_available_in_tap_under_robot > 0 and capacity - load > 0 and load < state.plants_need:

                    # Changing the robot's load
                    new_robot_key_tuple = (x, y)
                    new_robot_value_tuple = (id, load + 1, capacity)

                    # Changing the tap
                    new_tap_key_tuple = (x, y)
                    new_tap_value = water_available_in_tap_under_robot - 1

                    # Creating the new state
                    move = f"LOAD{{{id}}}"

                    # We need to iterate over all the remaining taps and plants to create objectives to them!
                    # A lot of work but at least we are still the active robot!

                    # First let's start with the plants.
                    # There won't be any duplicates since now we are on a tap.
                    for (i, j) in state.plants.keys():

                        new_state = State(size=state.size,
                                          walls=state.walls,
                                          taps=state.taps,
                                          plants=dict(state.plants),
                                          robots=dict(state.robots),
                                          last_move=move,
                                          plants_need=state.plants_need,
                                          robots_load=state.robots_load + 1,
                                          taps_have=state.taps_have - 1,
                                          robot_last_moves=state.robots_last_moves,
                                          current_active_robot=id,  # I'm still the active robot:)
                                          objective=(i, j),
                                          active_only=False,  # I don't know if it's only me or not yet
                                          initial_cords=(x, y), )

                        # robot last moves
                        del new_state.robots_last_moves[id]
                        new_state.robots_last_moves[id] = "LOAD"

                        # Deleting the previous state of robot and inserting the new one
                        del new_state.robots[(x, y)]
                        new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                        # Deleting the previous state of plant and inserting the new one
                        del new_state.taps[(x, y)]
                        if new_tap_value > 0:
                            new_state.taps[new_tap_key_tuple] = new_tap_value

                        # Inserting the new state to the possible next states
                        if self.cache.get(new_state) is None:
                            possible_successors.append((move, new_state))



                    # Okay we are done with the plants let's move on to the taps
                    # Now we need to be careful not to put the current objective also as a new objective!
                    for (i, j) in state.taps.keys():

                        # Careful! Duplicates!
                        if (x, y) == (i ,j): continue

                        new_state = State(size=state.size,
                                          walls=state.walls,
                                          taps=state.taps,
                                          plants=dict(state.plants),
                                          robots=dict(state.robots),
                                          last_move=move,
                                          plants_need=state.plants_need,
                                          robots_load=state.robots_load + 1,
                                          taps_have=state.taps_have - 1,
                                          robot_last_moves=state.robots_last_moves,
                                          current_active_robot=id,  # I'm still the active robot:)
                                          objective=(i, j),
                                          active_only=False,  # I don't know if it's only me or not yet
                                          initial_cords=(x, y), )

                        # robot last moves
                        del new_state.robots_last_moves[id]
                        new_state.robots_last_moves[id] = "LOAD"

                        # Deleting the previous state of robot and inserting the new one
                        del new_state.robots[(x, y)]
                        new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                        # Deleting the previous state of plant and inserting the new one
                        del new_state.taps[(x, y)]
                        if new_tap_value > 0:
                            new_state.plants[new_tap_key_tuple] = new_tap_value

                        # Inserting the new state to the possible next states
                        if self.cache.get(new_state) is None:
                            possible_successors.append((move, new_state))

                    # Okay wow that was long! we have successfully added all the new states:
                    # We have dealt with cases where the objective we got to was plant and cases where it was a tap.
                    # We have added all the new states filled with the new objectives.
                    # The job of the next iteration is to check the 'active_only', and start the way to the new
                    # objective!
                    # For now, we are done with this iteration, let's continue
                    continue



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
                    new_active_robot = None if load - 1 == 0 else current_active

                    move = f"POUR{{{id}}}"

                    new_state = State(size = state.size,
                                      walls = state.walls,
                                      taps = state.taps,
                                      plants = dict(state.plants),
                                      robots = dict(state.robots),
                                      last_move = move,
                                      plants_need = state.plants_need - 1,
                                      robots_load = state.robots_load - 1,
                                      taps_have = state.taps_have,
                                      robot_last_moves = state.robots_last_moves,
                                      current_active_robot = None,
                                      objective = None,
                                      active_only = False)

                    # robot last moves
                    del new_state.robots_last_moves[id]
                    new_state.robots_last_moves[id] = "POUR"

                    # Deleting the previous state of robot and inserting the new one
                    del new_state.robots[(x, y)]
                    new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                    # Deleting the previous state of plant and inserting the new one
                    del new_state.plants[(x, y)]
                    if new_plant_value > 0:
                        new_state.plants[new_plant_key_tuple] = new_plant_value

                    # Inserting the new state to the possible next states
                    if self.cache.get(new_state) is None:
                        possible_successors.append((move, new_state))

                    # If there is only one robot, there is no reason not to pour all he has onto the robot
                    if (number_of_robots == 1
                        or water_needed_in_plant_under_robot == state.plants_need
                        or load >= state.plants_need):
                            continue


            # We now want to check whether the robot can load more WU from a tap
            # This is where we initialize the first drinker to be the active robot.
            # We allow drinking if and only if one of the following cases is satisfied:
            # 1. There is no active robot (everyone is free)
            # 2. There is an active robot (which is the current robot)
            #and we are on the first coordinates to its new objective
            if (current_active == id and (x, y) == state.initial_cords) or current_active is None:

                # Now we check if the current robot is even able to load water (and should)
                if capacity - load > 0 and load < state.plants_need:

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

                        # We are the active robot. we already has an objective.
                        if current_active == id:
                            new_state = State(size = state.size,
                                                walls = state.walls,
                                                  taps = dict(state.taps),
                                                  plants = state.plants,
                                                  robots = dict(state.robots),
                                                  last_move = move,
                                                  plants_need = state.plants_need,
                                                  robots_load = state.robots_load + 1,
                                                  taps_have = state.taps_have - 1,
                                                  robot_last_moves = state.robots_last_moves,
                                                  current_active_robot = id,
                                                  objective = state.objective,
                                                  active_only = state.active_only,
                                                  initial_cords = (x, y))

                            # robot last moves
                            del new_state.robots_last_moves[id]
                            new_state.robots_last_moves[id] = "LOAD"

                            # Deleting the previous state of robot and inserting the new one
                            del new_state.robots[(x, y)]
                            new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                            # Deleting the previous state of tap and inserting the new one
                            del new_state.taps[(x, y)]
                            if new_tap_value > 0:
                                new_state.taps[new_tap_key_tuple] = new_tap_value

                            # Inserting the new state to the possible next states
                            if self.cache.get(new_state) is None:
                                possible_successors.append((move, new_state))


                        # We are now crowned as the new active robot!
                        else:
                            # We now create a lot of "mini states": we are now the active robot, the king of the jungle
                            # And for every plant and tap on the board we create an objective.
                            for ((i, j), needed) in state.plants.items():
                                path_to_objective = self.bfs_paths.get(((i, j), (x, y)))

                                # If I cannot reach this plant:(
                                if not path_to_objective: continue

                                # Now checking if there is a robot on the path to the objective
                                is_active_only = True
                                for (x_else, y_else) in state.robots.keys():
                                    if (x_else, y_else) != (x, y) and (x_else, y_else) in path_to_objective:
                                        is_active_only = False

                                new_state = State(size = state.size,
                                                  walls = state.walls,
                                                  taps = dict(state.taps),
                                                  plants = state.plants,
                                                  robots = dict(state.robots),
                                                  last_move = move,
                                                  plants_need = state.plants_need,
                                                  robots_load = state.robots_load + 1,
                                                  taps_have = state.taps_have - 1,
                                                  robot_last_moves = state.robots_last_moves,
                                                  current_active_robot = id,
                                                  objective = (i, j),
                                                  active_only = is_active_only)

                                # robot last moves
                                del new_state.robots_last_moves[id]
                                new_state.robots_last_moves[id] = "LOAD"

                                # Deleting the previous state of robot and inserting the new one
                                del new_state.robots[(x, y)]
                                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                                # Deleting the previous state of tap and inserting the new one
                                del new_state.taps[(x, y)]
                                if new_tap_value > 0:
                                    new_state.taps[new_tap_key_tuple] = new_tap_value

                                # Inserting the new state to the possible next states
                                if self.cache.get(new_state) is None:
                                    possible_successors.append((move, new_state))

                            # Now we do the same thing but for all the taps! Each tap is a new objective
                            for ((i, j), have) in state.taps.items():
                                path_to_objective = self.bfs_paths.get(((i, j), (x, y)))

                                # If I cannot reach this tap:(
                                if not path_to_objective: continue

                                # Now checking if there is a robot on the path to the objective
                                is_active_only = True
                                for (x_else, y_else) in state.robots.keys():
                                    if (x_else, y_else) != (x, y) and (x_else, y_else) in path_to_objective:
                                        is_active_only = False


                                new_state = State(size=state.size,
                                                  walls=state.walls,
                                                  taps=dict(state.taps),
                                                  plants=state.plants,
                                                  robots=dict(state.robots),
                                                  last_move=move,
                                                  plants_need=state.plants_need,
                                                  robots_load=state.robots_load + 1,
                                                  taps_have=state.taps_have - 1,
                                                  robot_last_moves=state.robots_last_moves,
                                                  current_active_robot=id,
                                                  objective=(i, j),
                                                  active_only=is_active_only)

                                # robot last moves
                                del new_state.robots_last_moves[id]
                                new_state.robots_last_moves[id] = "LOAD"

                                # Deleting the previous state of robot and inserting the new one
                                del new_state.robots[(x, y)]
                                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                                # Deleting the previous state of tap and inserting the new one
                                del new_state.taps[(x, y)]
                                if new_tap_value > 0:
                                    new_state.taps[new_tap_key_tuple] = new_tap_value

                                # Inserting the new state to the possible next states
                                if self.cache.get(new_state) is None:
                                    possible_successors.append((move, new_state))


                        # If there is one robot he should fill his tank until full
                        # Or until he has enough WU to water all plants
                        if number_of_robots == 1 or (
                                number_of_taps == 1
                                and state.robots_last_moves.get(id) == "LOAD"
                                and load < min(state.plants.values())
                        ):
                            continue


            # If the robot can move UP
            path_to_objective = self.bfs_paths.get((state.objective, (x, y))) or set()
            if (self.legal_moves[x][y][0]
                    and state.robots.get((x - 1, y)) is None
                    and state.last_move != f"DOWN{{{id}}}"
                    and (not state.active_only
                         or (state.active_only and id == current_active and (x - 1, y) in path_to_objective))):

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
                                  taps_have = state.taps_have,
                                  robot_last_moves = state.robots_last_moves,
                                  current_active_robot = current_active,
                                  objective = state.objective,
                                  active_only = state.active_only)

                # robot last moves
                del new_state.robots_last_moves[id]
                new_state.robots_last_moves[id] = "UP"

                del new_state.robots[(x, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                if self.cache.get(new_state) is None:
                    possible_successors.append((move, new_state))


            # If the robot can move DOWN
            if (self.legal_moves[x][y][1]
                    and state.robots.get((x + 1, y)) is None
                    and state.last_move != f"UP{{{id}}}"
                    and (not state.active_only
                         or (state.active_only and id == current_active and (x + 1, y) in path_to_objective))):

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
                                  taps_have = state.taps_have,
                                  robot_last_moves = state.robots_last_moves,
                                  current_active_robot = current_active,
                                  objective=state.objective,
                                  active_only=state.active_only)

                # robot last moves
                del new_state.robots_last_moves[id]
                new_state.robots_last_moves[id] = "DOWN"

                del new_state.robots[(x, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                if self.cache.get(new_state) is None:
                    possible_successors.append((move, new_state))


            # If the robot can move LEFT
            if (self.legal_moves[x][y][2]
                    and state.robots.get((x, y - 1)) is None
                    and state.last_move != f"RIGHT{{{id}}}"
                    and (not state.active_only
                         or (state.active_only and id == current_active and (x, y - 1) in path_to_objective))):

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
                                  taps_have = state.taps_have,
                                  robot_last_moves = state.robots_last_moves,
                                  current_active_robot = current_active,
                                  objective=state.objective,
                                  active_only=state.active_only)

                # robot last moves
                del new_state.robots_last_moves[id]
                new_state.robots_last_moves[id] = "LEFT"

                del new_state.robots[(x, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                if self.cache.get(new_state) is None:
                    possible_successors.append((move, new_state))

            # If the robot can move RIGHT
            if (self.legal_moves[x][y][3]
                    and state.robots.get((x, y + 1)) is None
                    and state.last_move != f"LEFT{{{id}}}"
                    and (not state.active_only
                         or (state.active_only and id == current_active and (x, y + 1) in path_to_objective))):

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
                                  taps_have = state.taps_have,
                                  robot_last_moves = state.robots_last_moves,
                                  current_active_robot = current_active,
                                  objective=state.objective,
                                  active_only=state.active_only)

                # robot last moves
                del new_state.robots_last_moves[id]
                new_state.robots_last_moves[id] = "RIGHT"

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
        return len(state.plants) == 0

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


        still_need = node.state.plants_need - node.state.robots_load
        if node.state.plants_need == 0:
            self.cache[node.state] = 0
            return 0

        if still_need > node.state.taps_have :
            self.cache[node.state] = float('inf')
            return float('inf')

        for (x_robot, y_robot), (id, load, capacity) in node.state.robots.items():
            if node.state.taps_have == 0:
                current_shortest = min(self.bfs_distance((x_robot, y_robot), (x_plant, y_plant)) for (x_plant, y_plant) in node.state.plants.keys())
            else:
                if load == 0:
                    current_shortest = min(
                        self.bfs_distance((x_tap, y_tap), (x_robot, y_robot))
                        + self.bfs_distance((x_plant, y_plant), (x_tap, y_tap))
                        for ((x_tap, y_tap), remaining_wu_tap) in node.state.taps.items()
                        for ((x_plant, y_plant), remaining_wu_plant) in node.state.plants.items()
                    )
                else:
                    if still_need > 0:
                        current_shortest = min(
                        self.bfs_distance((x_tap, y_tap), (x_robot, y_robot))
                        + self.bfs_distance((x_plant, y_plant), (x_tap, y_tap))
                        for ((x_tap, y_tap), remaining_wu_tap) in node.state.taps.items()
                        for ((x_plant, y_plant), remaining_wu_plant) in node.state.plants.items()
                        )
                    else:
                        current_shortest = min(
                            self.bfs_distance((x_plant, y_plant), (x_robot, y_robot))
                            for ((x_plant, y_plant), remaining_wu) in node.state.plants.items()
                        )
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
