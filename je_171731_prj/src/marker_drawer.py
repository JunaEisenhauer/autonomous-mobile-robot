from math import cos, sin

import rospy
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from goal import GOAL_RADIUS
from goal import Goal
from grid import GRID_SIZE
from robot_state import RobotState

VISUALIZATION_MARKER_TOPIC = "/visualization_marker"

GOAL_NAMESPACE = "goals"
GOAL_SCALE = Vector3(GOAL_RADIUS, GOAL_RADIUS, GOAL_RADIUS)
TARGET_GOAL_COLOR = ColorRGBA(1, 0.75, 0, 1)
COLLECTED_GOAL_COLOR = ColorRGBA(0, 1, 0, 1)
UNCOLLECTED_GOAL_COLOR = ColorRGBA(0, 0, 1, 1)

OBSTACLE_NAMESPACE = "obstacles"
OBSTACLE_SCALE = Vector3(GRID_SIZE, GRID_SIZE, GRID_SIZE)
OBSTACLE_COLOR = ColorRGBA(1, 0.8, 0.2, 1)

PATH_NAMESPACE = "path"
PATH_SCALE = Vector3(0.05, 0.15, 0.1)
PATH_COLOR = ColorRGBA(1, 0, 0, 1)

DIRECTION_NAMESPACE = "direction"
DIRECTION_SCALE = Vector3(0.07, 0.2, 0.12)
DIRECTION_COLOR = ColorRGBA(0.2, 0.6, 0.6, 1)


def _global2local_point(position, global_position, rotation):
    # type: ((float, float), (float, float), float) -> Point
    relative_x = position[0] - global_position[0]
    relative_y = position[1] - global_position[1]
    transformed_x = cos(-rotation) * relative_x - sin(-rotation) * relative_y
    transformed_y = cos(-rotation) * relative_y + sin(-rotation) * relative_x
    return Point(transformed_x, transformed_y, 0)


class MarkerDrawer:
    def __init__(self):  # type: () -> None
        self._previous_path_size = 0
        self._visualization_publisher = rospy.Publisher(VISUALIZATION_MARKER_TOPIC, Marker, queue_size=4)

    def draw_goals(self, target_goal, goals, robot_state):
        # type: (Goal, list, RobotState) -> None
        """ Creates markers for all goals.

        Args:
            target_goal: The current targeted goal.
            goals: The list of goals.
            robot_state: The robot state.
        """
        points = []
        colors = []

        for goal in list(goals):
            point = _global2local_point((goal.x, goal.y), robot_state.exact_position, robot_state.exact_rotation)
            points.append(point)

            if goal == target_goal:
                color = TARGET_GOAL_COLOR
            elif goal.collected:
                color = COLLECTED_GOAL_COLOR
            else:
                color = UNCOLLECTED_GOAL_COLOR

            colors.append(color)

        self._draw_sphere_list(0, GOAL_NAMESPACE, GOAL_SCALE, points, colors=colors)

    def draw_obstacles(self, obstacles, robot_state):
        # type: (list, RobotState) -> None
        """ Creates markers for all obstacles.

        Args:
            obstacles: The list of obstacles.
            robot_state: The robot state.
        """
        points = []
        for position in obstacles:
            point = _global2local_point(position, robot_state.exact_position, robot_state.exact_rotation)
            points.append(point)

        self._draw_sphere_list(0, OBSTACLE_NAMESPACE, OBSTACLE_SCALE, points, color=OBSTACLE_COLOR)

    def draw_path(self, path, robot_state):
        # type: (list, RobotState) -> None
        """ Draws the path with arrows. Previous drawn paths are removed.

        Args:
            path: The path to draw.
            robot_state: The robot state.
        """
        for i in range(self._previous_path_size + 1):
            self._clear_marker(i, PATH_NAMESPACE)

        previous_position = path[0]
        for index, position in enumerate(path[1::]):
            start = _global2local_point(previous_position, robot_state.exact_position, robot_state.exact_rotation)
            end = _global2local_point(position, robot_state.exact_position, robot_state.exact_rotation)
            self._draw_arrow(index, PATH_NAMESPACE, PATH_SCALE, start, end, PATH_COLOR)
            previous_position = position

        self._previous_path_size = len(path)

    def draw_direction(self, direction, robot_state):
        # type: ((float, float), RobotState) -> None
        """ Draws an arrow from the robot to the given direction.

        Args:
            direction: The direction to draw the arrow to.
            robot_state: The robot state.
        """
        start = Point(0, 0, 0)
        end = _global2local_point(direction, robot_state.exact_position, robot_state.exact_rotation)
        self._draw_arrow(2, DIRECTION_NAMESPACE, DIRECTION_SCALE, start, end, DIRECTION_COLOR)

    def _draw_sphere_list(self, uid, namespace, scale, points, color=None, colors=None):
        # type: (int, str, Vector3, list, ColorRGBA, list) -> None
        marker = Marker()
        # Header
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time()

        # Body
        marker.ns = namespace
        marker.id = uid
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale = scale
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.points = points
        if color is not None:
            marker.color = color
        if colors is not None:
            marker.colors = colors

        self._visualization_publisher.publish(marker)

    def _draw_arrow(self, uid, namespace, scale, start, end, color):
        # type: (int, str, Vector3, Point, Point, ColorRGBA) -> None
        marker = Marker()
        # Header
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time()

        # Body
        marker.ns = namespace
        marker.id = uid
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale = scale
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.points = [start, end]
        marker.color = color

        self._visualization_publisher.publish(marker)

    def _clear_marker(self, uid, namespace):
        # type: (int, str) -> None
        marker = Marker()
        # Header
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time()

        # Body
        marker.ns = namespace
        marker.id = uid
        marker.action = Marker.DELETE

        self._visualization_publisher.publish(marker)
