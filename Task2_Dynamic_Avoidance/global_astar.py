"""Grid-based global A* planner for Task 2."""
from __future__ import annotations

import heapq
from itertools import count
from typing import Iterable

from Task2_Dynamic_Avoidance.config_task2 import GridMap


GridCell = tuple[int, int]


def manhattan(a: GridCell, b: GridCell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class GlobalAStarPlanner:
    """Plan a single-robot global path on a static 2D grid map."""

    def __init__(self, grid_map: GridMap, allow_diagonal: bool = False):
        self.grid_map = grid_map
        self.allow_diagonal = allow_diagonal

    def neighbors(self, cell: GridCell) -> Iterable[GridCell]:
        moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if self.allow_diagonal:
            moves += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dx, dy in moves:
            neighbor = (cell[0] + dx, cell[1] + dy)
            if self.grid_map.passable(neighbor):
                yield neighbor

    @staticmethod
    def reconstruct(came_from: dict[GridCell, GridCell], current: GridCell) -> list[GridCell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def plan(self, start: GridCell, goal: GridCell) -> list[GridCell]:
        """Return the A* global path from start to goal, or raise ValueError."""
        start = tuple(map(int, start))
        goal = tuple(map(int, goal))
        if not self.grid_map.passable(start):
            raise ValueError(f"Start cell {start} is not passable")
        if not self.grid_map.passable(goal):
            raise ValueError(f"Goal cell {goal} is not passable")

        open_heap = []
        push_index = count()
        heapq.heappush(open_heap, (manhattan(start, goal), 0, next(push_index), start))
        came_from: dict[GridCell, GridCell] = {}
        g_score = {start: 0.0}
        closed: set[GridCell] = set()

        while open_heap:
            _, _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                return self.reconstruct(came_from, current)
            closed.add(current)

            for neighbor in self.neighbors(current):
                step_cost = ((neighbor[0] - current[0]) ** 2 + (neighbor[1] - current[1]) ** 2) ** 0.5
                tentative_g = g_score[current] + step_cost
                if tentative_g >= g_score.get(neighbor, float("inf")):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + manhattan(neighbor, goal)
                heapq.heappush(open_heap, (f_score, manhattan(neighbor, goal), next(push_index), neighbor))

        raise ValueError(f"No A* path found from {start} to {goal}")
