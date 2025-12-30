import ext_plant

id = ["000000000"]


class Controller:
    """This class is a controller for the ext_plant game."""

    def __init__(self, game: ext_plant.Game):
        """Initialize controller for given game model."""
        self.original_game = game


    # This function receives a point on the grid and returns a boolean value based on whether there is a robot in that
    # coordinate
    def is_coordinate_contain_robot(self, coordinate, robots):

        for _, (r, c), _ in robots:
            if coordinate == (r, c):
                return True
        return False


    # This function receives a point on the grid and returns a boolean value based on whether there is a wall in that
    # coordinate
    def is_coordinate_contain_wall(self, coordinate):

        for wall in self.original_game.walls:
            if coordinate in wall:
                return True
        return False


    # This function receives a point on the grid and returns a boolean value based on whether the point is a legal
    # point on the gird
    def is_on_grid(self, coordinate):
        (r, c) = coordinate
        return 0 <= r < self.original_game.rows and 0 <= c < self.original_game.cols


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
                and not self.is_coordinate_contain_robot((r - 1, c), robots)
                and not self.is_coordinate_contain_wall((r - 1, c))
            )

        if action == "DOWN":
            return (
                self.is_on_grid((r + 1, c))
                and not self.is_coordinate_contain_robot((r + 1, c), robots)
                and not self.is_coordinate_contain_wall((r + 1, c))
            )

        if action == "LEFT":
            return (
                self.is_on_grid((r, c - 1))
                and not self.is_coordinate_contain_robot((r, c - 1), robots)
                and not self.is_coordinate_contain_wall((r, c - 1))
            )

        if action == "RIGHT":
            return (
                self.is_on_grid((r, c + 1))
                and not self.is_coordinate_contain_robot((r, c + 1), robots)
                and not self.is_coordinate_contain_wall((r, c + 1))
            )

        if action == "POUR":
            return total_water_needed > 0 and self.can_pour(moving_robot, plants)








    def choose_next_action(self, state):
        """ Choose the next action given a state."""
        # (robots, plants, taps, total_water_needed) = state
        #
        # possible_actions = []
        # for robot in robots:

