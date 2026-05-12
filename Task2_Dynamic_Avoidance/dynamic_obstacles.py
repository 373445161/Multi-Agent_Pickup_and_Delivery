"""Dynamic obstacle models and noisy observation utilities for Task 2."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from Task2_Dynamic_Avoidance.config_task2 import GridMap


@dataclass
class ObservedObstacle:
    """Noisy sensor observation of a dynamic obstacle."""

    obstacle_id: str
    position: np.ndarray
    true_position: np.ndarray
    radius: float
    distance: float


class DynamicObstacle:
    """Base class for dynamic obstacles."""

    def __init__(self, obstacle_id: str, position: Iterable[float], radius: float = 0.28):
        self.obstacle_id = obstacle_id
        self.position = np.array(position, dtype=float)
        self.radius = radius
        self.active = True

    def step(self, dt: float, grid_map: GridMap, rng: np.random.Generator) -> np.ndarray:
        return self.position

    def is_active(self, time_step: int) -> bool:
        return self.active


class PatrolObstacle(DynamicObstacle):
    """Obstacle that moves back and forth between two waypoints."""

    def __init__(self, obstacle_id: str, start: Iterable[float], end: Iterable[float], speed: float, radius: float = 0.28):
        super().__init__(obstacle_id, start, radius)
        self.start = np.array(start, dtype=float)
        self.end = np.array(end, dtype=float)
        self.speed = speed
        self.direction = 1

    def step(self, dt: float, grid_map: GridMap, rng: np.random.Generator) -> np.ndarray:
        target = self.end if self.direction == 1 else self.start
        to_target = target - self.position
        distance = float(np.linalg.norm(to_target))
        if distance < 1e-9:
            self.direction *= -1
            return self.position
        move = min(distance, self.speed * dt)
        next_position = self.position + to_target / distance * move
        if grid_map.collides_static(next_position, self.radius * 0.5):
            self.direction *= -1
        else:
            self.position = next_position
        if np.linalg.norm(target - self.position) < 1e-6:
            self.direction *= -1
        return self.position


class RandomWalkObstacle(DynamicObstacle):
    """Obstacle that randomly changes direction while avoiding static walls."""

    def __init__(self, obstacle_id: str, position: Iterable[float], speed: float, radius: float = 0.28):
        super().__init__(obstacle_id, position, radius)
        self.speed = speed
        self.heading = 0.0

    def step(self, dt: float, grid_map: GridMap, rng: np.random.Generator) -> np.ndarray:
        if rng.random() < 0.22:
            self.heading = float(rng.uniform(0.0, 2.0 * np.pi))
        direction = np.array([np.cos(self.heading), np.sin(self.heading)])
        candidate = self.position + direction * self.speed * dt
        if grid_map.collides_static(candidate, self.radius * 0.5):
            self.heading = float(rng.uniform(0.0, 2.0 * np.pi))
        else:
            self.position = candidate
        return self.position


class TemporaryObstacle(DynamicObstacle):
    """Obstacle that appears for a fixed time window."""

    def __init__(
        self,
        obstacle_id: str,
        position: Iterable[float],
        appear_step: int,
        disappear_step: int,
        radius: float = 0.28,
    ):
        super().__init__(obstacle_id, position, radius)
        self.appear_step = appear_step
        self.disappear_step = disappear_step

    def is_active(self, time_step: int) -> bool:
        return self.appear_step <= time_step <= self.disappear_step


def observe_obstacles(
    robot_position: Iterable[float],
    obstacles: Iterable[DynamicObstacle],
    sensor_radius: float,
    noise_std: float,
    rng: np.random.Generator,
    time_step: int,
) -> list[ObservedObstacle]:
    """Observe active obstacles within sensor range and add Gaussian position noise."""
    robot_position = np.array(robot_position, dtype=float)
    observations: list[ObservedObstacle] = []
    for obstacle in obstacles:
        if not obstacle.is_active(time_step):
            continue
        true_position = obstacle.position.copy()
        distance = float(np.linalg.norm(true_position - robot_position))
        if distance > sensor_radius:
            continue
        noisy_position = true_position + rng.normal(0.0, noise_std, size=2)
        observations.append(
            ObservedObstacle(
                obstacle_id=obstacle.obstacle_id,
                position=noisy_position,
                true_position=true_position,
                radius=obstacle.radius,
                distance=distance,
            )
        )
    return observations


def create_default_obstacles(
    count: int,
    speed: float,
    radius: float,
    grid_map: GridMap,
    rng: np.random.Generator,
) -> list[DynamicObstacle]:
    """Create a mixed set of patrol, random-walk, and temporary obstacles."""
    templates: list[DynamicObstacle] = [
        PatrolObstacle("patrol_0", (10.0, 1.0), (10.0, 11.0), speed, radius),
        PatrolObstacle("patrol_1", (11.0, 3.0), (11.0, 9.0), speed * 0.8, radius),
        RandomWalkObstacle("random_0", (12.0, 5.0), speed * 0.7, radius),
        TemporaryObstacle("temporary_0", (12.0, 8.0), appear_step=28, disappear_step=105, radius=radius),
        RandomWalkObstacle("random_1", (10.0, 5.0), speed * 0.65, radius),
        TemporaryObstacle("temporary_1", (11.0, 10.0), appear_step=60, disappear_step=150, radius=radius),
    ]
    obstacles = templates[: max(0, min(count, len(templates)))]

    while len(obstacles) < count:
        free_cells = [
            (x, y)
            for x in range(grid_map.width)
            for y in range(grid_map.height)
            if grid_map.passable((x, y))
        ]
        cell = free_cells[int(rng.integers(0, len(free_cells)))]
        obstacles.append(RandomWalkObstacle(f"random_{len(obstacles)}", cell, speed, radius))
    return obstacles
