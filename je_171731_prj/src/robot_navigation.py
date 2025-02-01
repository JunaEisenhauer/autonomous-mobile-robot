import rospy

from goal import GoalPool, GOAL_RADIUS_SQRT
from goal_selector import GoalSelector
from grid import Grid
from marker_drawer import MarkerDrawer
from path_finder import PathFinder
from robot_control import RobotControl
from robot_state import RobotState


class RobotNavigation:
    def __init__(self):  # type: () -> None
        self._robot_state = RobotState()
        self._robot_control = RobotControl()
        self._goal_pool = GoalPool()
        self._grid = Grid()
        self._goal_selector = GoalSelector()
        self._path_finder = PathFinder()
        self._marker_drawer = MarkerDrawer()
        self._current_goal = None

    def update(self):  # type: () -> None
        """ Updates the navigation of the robot. """
        if not self._robot_state.received_all_data():
            return

        self._grid.update(self._robot_state)
        self._marker_drawer.draw_obstacles(self._grid.obstacles, self._robot_state)

        self._goal_pool.check_goals(self._robot_state)

        new_goal = self._goal_selector.select_goal(self._goal_pool.get_uncollected_goals(), self._grid,
                                                   self._robot_state)
        if new_goal is not self._current_goal and new_goal is not None:
            rospy.loginfo("Target: (%s %s), reward=%s" % (new_goal.x, new_goal.y, new_goal.reward))
        self._current_goal = new_goal

        self._marker_drawer.draw_goals(self._current_goal, self._goal_pool.goals, self._robot_state)

        # stop robot if no goal is selected
        if self._current_goal is None:
            self._robot_control.stop()
            return

        goal_grid_position = self._grid.nearby_free_grid_position((self._current_goal.x, self._current_goal.y),
                                                                  GOAL_RADIUS_SQRT)
        # set goal as unreachable if no grid position found
        if goal_grid_position is None:
            self._current_goal.unreachable = True
            self._robot_control.stop()
            return

        # find the path to the goal
        path = self._path_finder.find_path(self._grid.obstacles, self._robot_state.proximal_position,
                                           goal_grid_position)

        # check if path was found
        if len(path) <= 1:
            self._current_goal.unreachable = True
            self._robot_control.stop()
            return

        self._marker_drawer.draw_path(path, self._robot_state)

        # get furthest away target position which is still in sight
        target_position = self._grid.first_in_sight(path[:0:-1], self._robot_state.proximal_position)
        if target_position is None:
            target_position = path[1]

        # check if target position is in obstacle
        if self._grid.obstacles.__contains__(target_position):
            self._robot_control.stop()
            return

        self._robot_control.navigate(target_position, self._robot_state)
        self._marker_drawer.draw_direction(target_position, self._robot_state)
