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
    walls = frozenset() # The walls. no reason to save it in every copy.
    taps: dict[tuple, int]
    plants: dict[tuple, int]
    robots: dict[int, tuple]


    def __init__(self, initial):
        State.size = initial.size
        State.walls = initial.walls
        self.taps = initial.taps
        self.plants = initial.plants
        self.robots = initial.robots

    def __init__(self, size, walls, taps, plants, robots):
        State.size = size
        State.walls = walls
        self.taps = taps
        self.plants = plants
        self.robots = robots


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
        new_robots = state.robots
        for key, robot in state.robots.items():
            x = robot[0]
            y = robot[1]

            # If the robot can move left
            if x - 1 >= 0 and (x - 1, y) not in State.walls:

                # Changing the robot's position
                new_robot_tuple = (x + 1, y, robot[2], robot[3])

                # Creating the new state
                new_state = State(state.size, state.walls, state.taps, state.plants, new_robot_tuple)

                # Adding the new state to the result of all possible states we can go to
                possible_successors.append(new_state)


            # If the robot can move right
            if x + 1 < State.size[0] and (x + 1, y) not in State.walls:

                # Changing the robot's position
                new_robot_tuple = (x + 1, y, robot[2], robot[3])

                # Creating the new state
                new_state = State(state.size, state.walls, state.taps, state.plants, new_robot_tuple)

                # Adding the new state to the result of all possible states we can go to
                possible_successors.append(new_state)


            # If the robot can move down
            if y - 1 >= 0 and (x, y - 1) not in State.walls:

                # Changing the robot's position
                new_robot_tuple = (x, y - 1, robot[2], robot[3])

                # Creating the new state
                new_state = State(state.size, state.walls, state.taps, state.plants, new_robot_tuple)

                # Adding the new state to the result of all possible states we can go to
                possible_successors.append(new_state)

            # If the robot can move up
            if y + 1 < State.size[1] and (x, y + 1) not in State.walls:

                # Changing the robot's position
                new_robot_tuple = (x, y + 1, robot[2], robot[3])

                # Creating the new state
                new_state = State(state.size, state.walls, state.taps, state.plants, new_robot_tuple)

                # Adding the new state to the result of all possible states we can go to
                possible_successors.append(new_state)




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
        return 0;

    def h_gbfs(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        return 0;


def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()