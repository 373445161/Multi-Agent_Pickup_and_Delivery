"""Configuration and map-loading helpers for Task 2 dynamic avoidance."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MAP_PATH = PROJECT_ROOT / "Environments" / "input_warehouse_small.yaml"
DEFAULT_RESULTS_DIR = PROJECT_ROOT / "Results_Task2"


@dataclass
class Task2Config:
    """Central simulation configuration for a Task 2 run."""

    map_path: Path = DEFAULT_MAP_PATH
    results_dir: Path = DEFAULT_RESULTS_DIR
    start: tuple[float, float] | None = None
    goal: tuple[float, float] | None = None
    random_seed: int = 7
    max_steps: int = 260
    dt: float = 0.25
    robot_speed: float = 1.2
    robot_radius: float = 0.22
    goal_tolerance: float = 0.28
    waypoint_tolerance: float = 0.35
    lookahead_waypoints: int = 3
    sensor_radius: float = 3.0
    sensor_noise_std: float = 0.08
    obstacle_radius: float = 0.28
    dynamic_obstacle_count: int = 4
    dynamic_obstacle_speed: float = 0.75
    attraction_gain: float = 1.5
    repulsion_gain: float = 1.4
    repulsion_radius: float = 1.7
    static_repulsion_gain: float = 0.75
    static_repulsion_radius: float = 0.85
    max_force: float = 2.2
    stagnation_window: int = 12
    stagnation_distance: float = 0.08
    escape_force: float = 0.85
    save_animation: bool = True
    animation_interval_ms: int = 70

    def ensure_results_dir(self) -> Path:
        self.results_dir.mkdir(parents=True, exist_ok=True)
        return self.results_dir


@dataclass(frozen=True)
class GridMap:
    """Discrete grid map loaded from the existing YAML environment."""

    dimensions: tuple[int, int]
    obstacles: set[tuple[int, int]] = field(default_factory=set)
    non_task_endpoints: set[tuple[int, int]] = field(default_factory=set)
    raw: dict = field(default_factory=dict)

    @property
    def width(self) -> int:
        return self.dimensions[0]

    @property
    def height(self) -> int:
        return self.dimensions[1]

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, cell: tuple[int, int]) -> bool:
        return self.in_bounds(cell) and cell not in self.obstacles

    def static_clearance(self, point: Iterable[float]) -> float:
        """Return distance from a continuous point to the nearest static obstacle center."""
        px, py = point
        if not self.obstacles:
            return float("inf")
        return min(((px - ox) ** 2 + (py - oy) ** 2) ** 0.5 for ox, oy in self.obstacles)

    def collides_static(self, point: Iterable[float], radius: float = 0.0) -> bool:
        """Check continuous point collision with grid bounds or obstacle centers."""
        px, py = point
        if px < 0 or py < 0 or px >= self.width or py >= self.height:
            return True
        obstacle_safety_radius = radius + 0.25
        return self.static_clearance((px, py)) <= obstacle_safety_radius


def _to_tuple_set(items: Iterable[Iterable[int]] | None) -> set[tuple[int, int]]:
    return {tuple(map(int, item)) for item in (items or [])}


def load_grid_map(map_path: str | Path = DEFAULT_MAP_PATH) -> GridMap:
    """Load the warehouse YAML map used by the original MAPD project."""
    with open(map_path, "r") as map_file:
        raw = yaml.load(map_file, Loader=yaml.FullLoader)
    map_data = raw["map"]
    return GridMap(
        dimensions=tuple(map(int, map_data["dimensions"])),
        obstacles=_to_tuple_set(map_data.get("obstacles")),
        non_task_endpoints=_to_tuple_set(map_data.get("non_task_endpoints")),
        raw=raw,
    )


def default_start_goal(grid_map: GridMap) -> tuple[tuple[int, int], tuple[int, int]]:
    """Choose a reproducible start/goal pair from the warehouse task definition."""
    tasks = grid_map.raw.get("tasks", [])
    if tasks:
        first_task = tasks[0]
        return tuple(first_task["start"]), tuple(first_task["goal"])
    free_cells = [
        (x, y)
        for x in range(grid_map.width)
        for y in range(grid_map.height)
        if grid_map.passable((x, y))
    ]
    return free_cells[0], free_cells[-1]
