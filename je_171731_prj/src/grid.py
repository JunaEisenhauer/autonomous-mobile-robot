import math
from Queue import Queue
from math import cos, sin

from robot_state import RobotState

GRID_SIZE = 0.25


def _polar2cartesian(laser_range, angle, position):
    # type: (float, float, (float, float)) -> (float, float)
    x = laser_range * cos(angle) + position[0]
    y = laser_range * sin(angle) + position[1]
    return x, y


def position2grid(position):
    # type: ((float, float)) -> (float, float)
    """ Translates any given position to the position of the grid.

    Args:
        position: The position to translate.
    Returns: The translated grid position.
    """
    x = _coord2grid(position[0])
    y = _coord2grid(position[1])
    return x, y


def _coord2grid(coord):
    # type: (float) -> float
    translated_coord = math.floor(coord / GRID_SIZE)
    grid_coord = translated_coord * GRID_SIZE + GRID_SIZE * 0.5
    return grid_coord


class Grid:
    def __init__(self):  # type: () -> None
        self.obstacles = set()

    def update(self, robot_state):
        # type: (RobotState) -> None
        """ Update the grid obstacles.

        Args:
            robot_state: The robot state.
        """
        position = robot_state.proximal_position
        rotation = robot_state.proximal_rotation
        laser_scan = robot_state.laser_scan

        obstacles_to_keep = set()
        obstacles_to_remove = set()

        for i, laser_range in enumerate(laser_scan.ranges):
            angle = laser_scan.angle_min + (i * laser_scan.angle_increment) + rotation

            if math.isinf(laser_range):
                max_range_hit = _polar2cartesian(laser_scan.range_max - GRID_SIZE * 1.5, angle, position)
                obstacles_to_remove.update(self._obstacles_on_line(position, max_range_hit))
                continue

            obstacle_hit_position = _polar2cartesian(laser_range, angle, position)
            obstacle_grid_position = position2grid(obstacle_hit_position)
            self.obstacles.add(obstacle_grid_position)

            obstacles_to_keep.add(obstacle_grid_position)

            range_hit = _polar2cartesian(laser_range - GRID_SIZE * 1.5, angle, position)
            max_range_hit = _polar2cartesian(laser_scan.range_max - GRID_SIZE * 1.5, angle, position)
            obstacles_to_remove.update(self._obstacles_on_line(position, range_hit))
            obstacles_to_keep.update(self._obstacles_on_line(range_hit, max_range_hit))

        obstacles_to_remove.difference_update(obstacles_to_keep)
        self.obstacles.difference_update(obstacles_to_remove)

    def nearby_free_grid_position(self, position, radius_sqrt):
        # type: ((float, float), float) -> (float, float)
        """ Finds a free grid position near the given position without an obstacle within the given radius.

        Args:
            position: The origin position to find the near free grid position.
            radius_sqrt: The radius around the origin position to find the grid position.
        Returns: The nearest free grid position without an obstacle within the radius.
                 If there is no free grid position, None is returned.
        """
        grid_position = position2grid(position)
        if not self.obstacles.__contains__(grid_position):
            return grid_position

        neighbor_queue = Queue()
        visited = set()
        neighbor_queue.put_nowait(grid_position)
        visited.add(grid_position)
        while not neighbor_queue.empty():
            current = neighbor_queue.get_nowait()
            if not self.obstacles.__contains__(current):
                return current

            neighbors = self._expand_neighbors(current, position, radius_sqrt)
            for neighbor in neighbors:
                if not visited.__contains__(neighbor):
                    neighbor_queue.put_nowait(neighbor)

        return None

    def _expand_neighbors(self, grid_position, origin, radius_sqrt):
        # type: ((float, float), (float, float), float) -> list
        neighbors = []
        for new_position in [(0, -GRID_SIZE), (0, GRID_SIZE), (-GRID_SIZE, 0), (GRID_SIZE, 0), (-GRID_SIZE, -GRID_SIZE),
                             (-GRID_SIZE, GRID_SIZE), (GRID_SIZE, -GRID_SIZE), (GRID_SIZE, GRID_SIZE)]:
            neighbor_position = (grid_position[0] + new_position[0], grid_position[1] + new_position[1])
            if self.obstacles.__contains__(neighbor_position):
                continue

            distance2origin = ((neighbor_position[0] - origin[0]) ** 2) + ((neighbor_position[1] - origin[1]) ** 2)
            if distance2origin > radius_sqrt:
                continue

            neighbors.append(neighbor_position)

        return neighbors

    def first_in_sight(self, positions, start):
        # type: ((float, float), (float, float)) -> (float, float)
        """ Calculates the first position which is in sight to the start position.

        Args:
            positions: The positions to loop through.
            start: The start position.
        Returns: The first position which is in sight.
        """
        for position in positions:
            if self._is_in_sight(start, position):
                return position

        return None

    def _is_in_sight(self, start, end):
        # type: ((float, float), (float, float)) -> bool
        obstacles_in_sight = self._obstacles_on_line(start, end)
        return len(obstacles_in_sight) == 0

    def _obstacles_on_line(self, start, end):
        # type: ((float, float), (float, float)) -> set
        obstacles = set()

        difference_x = end[0] - start[0]
        difference_y = end[1] - start[1]
        root_distance = abs(difference_x) + abs(difference_y)
        if root_distance == 0:
            return obstacles

        dx = (difference_x / root_distance) * GRID_SIZE
        dy = (difference_y / root_distance) * GRID_SIZE

        i = 0
        max_i = math.ceil(root_distance / GRID_SIZE)
        while i <= max_i:
            x = start[0] + dx * i
            y = start[1] + dy * i
            grid_position = position2grid((x, y))
            if self.obstacles.__contains__(grid_position):
                obstacles.add(grid_position)

            i += 0.1

        return obstacles
