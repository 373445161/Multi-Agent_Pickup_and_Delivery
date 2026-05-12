"""Artificial Potential Field local planner for dynamic obstacle avoidance."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

from Task2_Dynamic_Avoidance.config_task2 import GridMap, Task2Config
from Task2_Dynamic_Avoidance.dynamic_obstacles import ObservedObstacle


@dataclass
class APFDebugInfo:
    """Planner diagnostics for metrics and debugging."""

    avoidance_event: bool = False
    escaped_local_minimum: bool = False
    target_index: int = 0
    attractive_force: np.ndarray = field(default_factory=lambda: np.zeros(2))
    repulsive_force: np.ndarray = field(default_factory=lambda: np.zeros(2))


class APFLocalPlanner:
    """Local APF controller using global-path lookahead and obstacle repulsion."""

    def __init__(self, grid_map: GridMap, global_path: list[tuple[int, int]], config: Task2Config):
        self.grid_map = grid_map
        self.global_path = [np.array(cell, dtype=float) for cell in global_path]
        self.config = config
        self.current_waypoint_index = 0
        self.recent_positions: list[np.ndarray] = []
        self.escape_sign = 1.0

    def _update_waypoint_index(self, robot_position: np.ndarray) -> None:
        future_indices = range(self.current_waypoint_index, len(self.global_path))
        nearest_index = min(future_indices, key=lambda idx: np.linalg.norm(self.global_path[idx] - robot_position))
        self.current_waypoint_index = max(self.current_waypoint_index, nearest_index)
        while self.current_waypoint_index < len(self.global_path) - 1:
            waypoint = self.global_path[self.current_waypoint_index]
            if np.linalg.norm(waypoint - robot_position) > self.config.waypoint_tolerance:
                break
            self.current_waypoint_index += 1

    def _lookahead_target(self, robot_position: np.ndarray) -> tuple[np.ndarray, int]:
        self._update_waypoint_index(robot_position)
        target_index = min(self.current_waypoint_index + self.config.lookahead_waypoints, len(self.global_path) - 1)
        return self.global_path[target_index], target_index

    def _attraction(self, robot_position: np.ndarray, target: np.ndarray) -> np.ndarray:
        vector = target - robot_position
        norm = np.linalg.norm(vector)
        if norm < 1e-9:
            return np.zeros(2)
        return self.config.attraction_gain * vector / norm

    def _dynamic_repulsion(self, robot_position: np.ndarray, observations: Iterable[ObservedObstacle]) -> tuple[np.ndarray, bool]:
        force = np.zeros(2)
        avoidance_event = False
        for observation in observations:
            away = robot_position - observation.position
            distance = max(float(np.linalg.norm(away)), 1e-6)
            influence = self.config.repulsion_radius + observation.radius
            if distance > influence:
                continue
            avoidance_event = True
            strength = self.config.repulsion_gain * (1.0 / distance - 1.0 / influence) / (distance**2)
            force += strength * away / distance
        return force, avoidance_event

    def _static_repulsion(self, robot_position: np.ndarray) -> np.ndarray:
        force = np.zeros(2)
        for ox, oy in self.grid_map.obstacles:
            obstacle_position = np.array([ox, oy], dtype=float)
            away = robot_position - obstacle_position
            distance = max(float(np.linalg.norm(away)), 1e-6)
            if distance > self.config.static_repulsion_radius:
                continue
            strength = self.config.static_repulsion_gain * (
                1.0 / distance - 1.0 / self.config.static_repulsion_radius
            ) / (distance**2)
            force += strength * away / distance
        return force

    def _is_stagnating(self, robot_position: np.ndarray, force: np.ndarray) -> bool:
        self.recent_positions.append(robot_position.copy())
        if len(self.recent_positions) > self.config.stagnation_window:
            self.recent_positions.pop(0)
        if len(self.recent_positions) < self.config.stagnation_window:
            return False
        displacement = np.linalg.norm(self.recent_positions[-1] - self.recent_positions[0])
        return displacement < self.config.stagnation_distance or np.linalg.norm(force) < 1e-5

    def _escape_force(self, robot_position: np.ndarray, target: np.ndarray) -> np.ndarray:
        to_target = target - robot_position
        norm = np.linalg.norm(to_target)
        if norm < 1e-9:
            return np.zeros(2)
        tangent = np.array([-to_target[1], to_target[0]]) / norm
        self.escape_sign *= -1.0
        return tangent * self.escape_sign * self.config.escape_force

    def compute_velocity(
        self,
        robot_position: Iterable[float],
        observations: Iterable[ObservedObstacle],
    ) -> tuple[np.ndarray, APFDebugInfo]:
        """Compute the next velocity command from APF forces."""
        robot_position = np.array(robot_position, dtype=float)
        target, target_index = self._lookahead_target(robot_position)
        attractive = self._attraction(robot_position, target)
        dynamic_repulsive, avoidance_event = self._dynamic_repulsion(robot_position, observations)
        static_repulsive = self._static_repulsion(robot_position)
        force = attractive + dynamic_repulsive + static_repulsive

        escaped = False
        if self._is_stagnating(robot_position, force):
            force += self._escape_force(robot_position, target)
            escaped = True

        force_norm = float(np.linalg.norm(force))
        if force_norm > self.config.max_force:
            force = force / force_norm * self.config.max_force
            force_norm = self.config.max_force

        if force_norm < 1e-9:
            velocity = np.zeros(2)
        else:
            velocity = force / force_norm * self.config.robot_speed

        debug = APFDebugInfo(
            avoidance_event=avoidance_event,
            escaped_local_minimum=escaped,
            target_index=target_index,
            attractive_force=attractive,
            repulsive_force=dynamic_repulsive + static_repulsive,
        )
        return velocity, debug
