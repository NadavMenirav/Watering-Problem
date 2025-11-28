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


    def __init__(self, initial = None, size = None, walls = None, taps = None, plants = None, robots = None):
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

        # If we construct using size, walls, taps, plants, robots
        else:
            State.size = size
            State.walls = walls
            self.taps = dict(taps)
            self.plants = dict(plants)
            self.robots = dict(robots)


class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""

    def __init__(self, initial):
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        search.Problem.__init__(self, initial)
        self.state = State(initial)


    def successor(self, state: State):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        possible_successors = []
        for (x, y), (id, load, capacity) in state.robots.items():

            # If the robot can move left
            if x - 1 >= 0 and State.walls.get((x - 1, y)) is None and state.robots.get((x - 1, y)) is None:

                # Changing the robot's position
                new_robot_key_tuple = (x - 1,  y)
                new_robot_value_tuple = (id, load, capacity)

                # Creating the new state
                new_state = State(size = state.size,
                                  walls = state.walls,
                                  taps = state.taps,
                                  plants = state.plants,
                                  robots = state.robots)
                del new_state.robots[(x - 1, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                possible_successors.append((f"LEFT{{{id}}}", new_state))


            # If the robot can move right
            if x + 1 < State.size[0] and State.walls.get((x + 1, y)) is None and state.robots.get((x + 1, y)) is None:

                # Changing the robot's position
                new_robot_key_tuple = (x + 1,  y)
                new_robot_value_tuple = (id, load, capacity)

                # Creating the new state
                new_state = State(size = state.size,
                                  walls = state.walls,
                                  taps = state.taps,
                                  plants = state.plants,
                                  robots = state.robots)
                del new_state.robots[(x + 1, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                possible_successors.append((f"RIGHT{{{id}}}", new_state))


            # If the robot can move down
            if y - 1 >= 0 and State.walls.get((x, y - 1)) is None and state.robots.get((x, y - 1)) is None:

                # Changing the robot's position
                new_robot_key_tuple = (x,  y - 1)
                new_robot_value_tuple = (id, load, capacity)

                # Creating the new state
                new_state = State(size = state.size,
                                  walls = state.walls,
                                  taps = state.taps,
                                  plants = state.plants,
                                  robots = state.robots)
                del new_state.robots[(x, y - 1)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                possible_successors.append((f"DOWN{{{id}}}", new_state))

            # If the robot can move up
            if y + 1 < State.size[1] and State.walls.get((x, y + 1)) is None and state.robots.get((x, y + 1)) is None:

                # Changing the robot's position
                new_robot_key_tuple = (x,  y + 1)
                new_robot_value_tuple = (id, load, capacity)

                # Creating the new state
                new_state = State(size = state.size,
                                  walls = state.walls,
                                  taps = state.taps,
                                  plants = state.plants,
                                  robots = state.robots)

                # Deleting the robot from its previous position and adding the new position
                del new_state.robots[(x, y + 1)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Adding the new state to the result of all possible states we can go to
                possible_successors.append((f"UP{{{id}}}", new_state))



            # We want to check whether the robot has any WU on it
            if load > 0:

                # We now want to check whether the robot is on a plant it can water
                # We don't care whether there is a plant that needs 0 WU or if there isn't a plant at all
                water_needed_in_plant_under_robot = state.plants.get((x, y), 0)

                if water_needed_in_plant_under_robot == 0:
                    break

                # For testing:
                if water_needed_in_plant_under_robot < 0:
                    print("ERROR WATER NEEDED IN PLANT IS NEGATIVE")
                    break

                # Changing the robot's load
                new_robot_key_tuple = (x, y)
                new_robot_value_tuple = (id, load - 1, capacity)

                # Changing the plant
                new_plant_key_tuple = (x, y)
                new_plant_value = water_needed_in_plant_under_robot - 1

                # Creating the new state
                new_state = State(size = state.size,
                                  walls = state.walls,
                                  taps = state.taps,
                                  plants = state.plants,
                                  robots = state.robots)

                # Deleting the previous state of robot and inserting the new one
                del new_state.robots[(x, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Deleting the previous state of plant and inserting the new one
                del new_state.plants[(x, y)]
                new_state.plants[new_plant_key_tuple] = new_plant_value

                # Inserting the new state to the possible next states
                possible_successors.append((f"POUR{{{id}}}", new_state))


            # We now want to check whether the robot can load more WU from a tap
            if capacity - load > 0:

                # We now want to check whether the robot is on a tap it can load water from
                # We don't care whether there is a tap that can give 0 WU or if there isn't a tap at all
                water_available_in_tap_under_robot = state.robots.get((x, y), 0)

                if water_available_in_tap_under_robot == 0:
                    break

                # For testing:
                if water_available_in_tap_under_robot < 0:
                    print("ERROR WATER AVAILABLE IN TAP IS NEGATIVE")
                    break

                # Changing the robot's load
                new_robot_key_tuple = (x, y)
                new_robot_value_tuple = (id, load + 1, capacity)

                # Changing the tap
                new_tap_key_tuple = (x, y)
                new_tap_value = water_available_in_tap_under_robot - 1

                # Creating the new state
                new_state = State(size = state.size,
                                  walls = state.walls,
                                  taps = state.taps,
                                  plants = state.plants,
                                  robots = state.robots)

                # Deleting the previous state of robot and inserting the new one
                del new_state.robots[(x, y)]
                new_state.robots[new_robot_key_tuple] = new_robot_value_tuple

                # Deleting the previous state of tap and inserting the new one
                del new_state.taps[(x, y)]
                new_state.plants[new_tap_key_tuple] = new_tap_value

                # Inserting the new state to the possible next states
                possible_successors.append((f"LOAD{{{id}}}", new_state))


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

        # This is an admissible heuristic, we need at least the remaining WU for the plants,
        # plus the remaining WU the robots need to load
        return 2 * sum(node.state.plants.values()) - sum(node.state.robots.values()[1])

    def h_gbfs(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""

        # This is an admissible heuristic, we need at least the remaining WU for the plants,
        # plus the remaining WU the robots need to load
        return 2 * sum(node.state.plants.values()) - sum(node.state.robots.values()[1])


def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()