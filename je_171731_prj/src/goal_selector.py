from goal import Goal, GOAL_RADIUS_SQRT
from grid import Grid
from path_finder import PathFinder

from robot_state import RobotState


def _path_distance(path):
    # type: (list) -> float
    distance_sqrt = 0.0
    if len(path) <= 1:
        return 0.0
    previous_position = path[0]
    for position in path[1::]:
        distance_sqrt += (position[0] - previous_position[0]) ** 2 + (position[1] - previous_position[1]) ** 2

    return distance_sqrt


class GoalSelector:
    def __init__(self):  # type: () -> None
        self._path_finder = PathFinder()

    def select_goal(self, goals, grid, robot_state):
        # type: (list, Grid, RobotState) -> Goal
        """ Selects the next best goal to collect.

        Args:
            goals: The list of goals, which come into question to collect.
            grid: The grid of the world.
            robot_state: The robot state.
        Returns: The selected goal.
        """
        min_distance_reward = None
        nearest_goal = None
        for goal in list(goals):

            # optimization: check if direct distance is short enough to possibly beat current nearest goal
            if min_distance_reward is not None:
                max_direct_distance = min_distance_reward * goal.reward
                if goal.distance_sqrt(robot_state.exact_position) > max_direct_distance:
                    continue

            goal_grid_position = grid.nearby_free_grid_position((goal.x, goal.y), GOAL_RADIUS_SQRT)
            if goal_grid_position is None:
                continue

            path = self._path_finder.find_path(grid.obstacles, robot_state.proximal_position, goal_grid_position)
            if len(path) <= 1:
                continue

            distance_sqrt = _path_distance(path)
            distance_reward = distance_sqrt * (1.0 / goal.reward)
            if min_distance_reward is None or min_distance_reward > distance_reward:
                min_distance_reward = distance_reward
                nearest_goal = goal

        return nearest_goal
