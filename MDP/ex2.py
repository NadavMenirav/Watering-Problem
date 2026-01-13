import ext_plant
import numpy as np

id = ["000000000"]

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


# This class is a helper class to help us with running the A* algorithm
class WateringProblem:
    def __init__(self, state, original_game):

        # Saving the starting state for the A*
        # State is (robots_t, plants_t, taps_t, total_water_need)
        self.initial = state

        # Saving the rules of the game
        self.walls = original_game.walls
        self.capacities = original_game.get_capacities()
        self.rows = original_game.rows
        self.cols = original_game.cols

    # This function is used in the A* to check if we had reached our goal.
    # The state we get have holds a total_water_need parameter which is 0 when we reach the goal
    def goal_test(self, state):
        return state[3] == 0 # The fourth parameter is the total_water_need


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


class Controller:
    """This class is a controller for the ext_plant game."""

    def __init__(self, game: ext_plant.Game):
        """Initialize controller for given game model."""
        self.original_game = game


    def choose_next_action(self, state):
        """ Choose the next action given a state."""
        (robots, plants, taps, total_water_needed) = state
        actions = {"UP", "DOWN", "LEFT", "RIGHT", "POUR", "LOAD"}

        possible_actions = []
        for robot in robots:
            robot_id, (r, c), load = robot

            # For every action, we want to check if its legal and if so, insert it to possible_actions
            for action in actions:
                if self.is_action_legal(state, action, robot):
                    possible_actions.append(f"{action}({robot_id})")

        possible_actions.append("RESET")
        return np.random.choice(np.array(possible_actions))
