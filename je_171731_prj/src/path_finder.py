from heapq import heappush, heappop

from grid import GRID_SIZE
from grid import position2grid

MAX_ITERATIONS = 3000
MAX_COST = 300


class PathFinder:
    def find_path(self, obstacles, start, end):
        # type: (list, (float, float), (float, float)) -> list
        """ Finds a path from the start position to the end position avoiding the obstacles.

        Args:
            obstacles: The obstacles of the grid.
            start: The start position.
            end: The end position.
        Returns: The path list to the end position.
        """
        return self._a_star(obstacles, position2grid(start), position2grid(end))

    def _a_star(self, obstacles, start, end):
        # type: (list, (float, float), (float, float)) -> list
        # open list is a min heap
        open_list = []
        heappush(open_list, (0, start))

        parents = {}

        g_score = {start: 0}
        start_heuristic = self._heuristic(start, end)
        f_score = {start: start_heuristic}

        iterations = 0
        while len(open_list) > 0 and iterations < MAX_ITERATIONS:
            iterations += 1
            current = heappop(open_list)[1]

            # check if end reached
            if current == end:
                return self._reconstruct_path(current, parents)

            # generate children
            children, g_score_increments = self._expand_children(current, obstacles)

            for index, child in enumerate(children):
                child_g_score = g_score[current] + g_score_increments[index]
                if child_g_score < g_score.get(child, MAX_COST):
                    parents[child] = current
                    g_score[child] = child_g_score
                    heuristic = self._heuristic(child, end)
                    f_score[child] = g_score[child] + heuristic
                    if not open_list.__contains__(child):
                        heappush(open_list, (f_score[child], child))

        return []

    def _reconstruct_path(self, current, parents):
        # type: ((float, float), dict) -> list
        path = [current]

        while current in parents:
            current = parents[current]
            path.append(current)

        return path[::-1]

    def _heuristic(self, point, end):
        # type: ((float, float), (float, float)) -> float
        return ((point[0] - end[0]) ** 2) + ((point[1] - end[1]) ** 2)

    def _expand_children(self, current, obstacles):
        # type: ((float, float), list) -> (list, list)
        children = []
        g_score_increments = []

        # expand the direct neighbors
        for new_position in [(0, -GRID_SIZE), (0, GRID_SIZE), (-GRID_SIZE, 0), (GRID_SIZE, 0)]:
            child_point = (current[0] + new_position[0], current[1] + new_position[1])
            if obstacles.__contains__(child_point):
                continue

            children.append(child_point)
            g_score_increments.append(1)

        # expand the diagonal neighbors
        for new_position in [(-GRID_SIZE, -GRID_SIZE), (-GRID_SIZE, GRID_SIZE),
                             (GRID_SIZE, -GRID_SIZE), (GRID_SIZE, GRID_SIZE)]:
            child_point = (current[0] + new_position[0], current[1] + new_position[1])
            right_edge_point = (current[0], current[1] + new_position[1])
            left_edge_point = (current[0] + new_position[0], current[1])
            if obstacles.__contains__(child_point) \
                    or obstacles.__contains__(right_edge_point) \
                    or obstacles.__contains__(left_edge_point):
                continue

            children.append(child_point)
            g_score_increments.append(1.5)

        return children, g_score_increments
