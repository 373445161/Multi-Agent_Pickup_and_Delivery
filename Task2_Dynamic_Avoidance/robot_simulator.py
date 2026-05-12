"""Robot simulation loop for Task 2 dynamic obstacle avoidance."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from Task2_Dynamic_Avoidance.apf_local_planner import APFDebugInfo, APFLocalPlanner
from Task2_Dynamic_Avoidance.config_task2 import GridMap, Task2Config, default_start_goal, load_grid_map
from Task2_Dynamic_Avoidance.dynamic_obstacles import DynamicObstacle, create_default_obstacles, observe_obstacles
from Task2_Dynamic_Avoidance.global_astar import GlobalAStarPlanner
from Task2_Dynamic_Avoidance.metrics import SimulationMetrics, path_length


@dataclass
class SimulationFrame:
    step: int
    robot_position: np.ndarray
    dynamic_obstacles: list[tuple[str, np.ndarray, bool]]
    observed_obstacles: list[np.ndarray]
    debug: APFDebugInfo


@dataclass
class SimulationResult:
    config: Task2Config
    grid_map: GridMap
    global_path: list[tuple[int, int]]
    trajectory: list[np.ndarray]
    frames: list[SimulationFrame]
    metrics: SimulationMetrics
    start: tuple[float, float]
    goal: tuple[float, float]
    output_dir: Path
    obstacles: list[DynamicObstacle] = field(default_factory=list)

    def metrics_dict(self) -> dict:
        return self.metrics.to_dict()


class RobotSimulator:
    """Run global A* followed by APF local dynamic avoidance."""

    def __init__(
        self,
        config: Task2Config | None = None,
        grid_map: GridMap | None = None,
        obstacles: list[DynamicObstacle] | None = None,
    ):
        self.config = config or Task2Config()
        self.grid_map = grid_map or load_grid_map(self.config.map_path)
        self.rng = np.random.default_rng(self.config.random_seed)
        self.obstacles = obstacles

    def _resolve_start_goal(self) -> tuple[tuple[float, float], tuple[float, float]]:
        default_start, default_goal = default_start_goal(self.grid_map)
        start = self.config.start or default_start
        goal = self.config.goal or default_goal
        return tuple(map(float, start)), tuple(map(float, goal))

    def _move_with_static_safety(self, current: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        candidate = current + velocity * self.config.dt
        if not self.grid_map.collides_static(candidate, self.config.robot_radius):
            return candidate
        # Try axis-wise sliding before declaring a static collision.
        slide_x = np.array([candidate[0], current[1]])
        if not self.grid_map.collides_static(slide_x, self.config.robot_radius):
            return slide_x
        slide_y = np.array([current[0], candidate[1]])
        if not self.grid_map.collides_static(slide_y, self.config.robot_radius):
            return slide_y
        return current

    def run(self, run_name: str = "task2_demo") -> SimulationResult:
        output_dir = self.config.ensure_results_dir() / run_name
        output_dir.mkdir(parents=True, exist_ok=True)
        start, goal = self._resolve_start_goal()
        planner = GlobalAStarPlanner(self.grid_map)
        global_path = planner.plan(tuple(map(round, start)), tuple(map(round, goal)))
        obstacles = self.obstacles or create_default_obstacles(
            self.config.dynamic_obstacle_count,
            self.config.dynamic_obstacle_speed,
            self.config.obstacle_radius,
            self.grid_map,
            self.rng,
        )
        local_planner = APFLocalPlanner(self.grid_map, global_path, self.config)

        robot_position = np.array(start, dtype=float)
        goal_position = np.array(goal, dtype=float)
        trajectory = [robot_position.copy()]
        frames: list[SimulationFrame] = []
        dynamic_collision_count = 0
        static_collision_count = 0
        avoidance_events = 0
        minimum_clearance = float("inf")
        success = False

        for step in range(self.config.max_steps):
            for obstacle in obstacles:
                if obstacle.is_active(step):
                    obstacle.step(self.config.dt, self.grid_map, self.rng)

            observations = observe_obstacles(
                robot_position,
                obstacles,
                self.config.sensor_radius,
                self.config.sensor_noise_std,
                self.rng,
                step,
            )
            velocity, debug = local_planner.compute_velocity(robot_position, observations)
            if debug.avoidance_event:
                avoidance_events += 1

            next_position = self._move_with_static_safety(robot_position, velocity)
            if np.allclose(next_position, robot_position) and self.grid_map.collides_static(robot_position + velocity * self.config.dt, self.config.robot_radius):
                static_collision_count += 1
            robot_position = next_position
            trajectory.append(robot_position.copy())

            for obstacle in obstacles:
                active = obstacle.is_active(step)
                if not active:
                    continue
                clearance = float(np.linalg.norm(robot_position - obstacle.position) - self.config.robot_radius - obstacle.radius)
                minimum_clearance = min(minimum_clearance, clearance)
                if clearance <= 0.0:
                    dynamic_collision_count += 1

            static_clearance = self.grid_map.static_clearance(robot_position) - self.config.robot_radius
            minimum_clearance = min(minimum_clearance, static_clearance)
            if self.grid_map.collides_static(robot_position, self.config.robot_radius):
                static_collision_count += 1

            frames.append(
                SimulationFrame(
                    step=step,
                    robot_position=robot_position.copy(),
                    dynamic_obstacles=[
                        (obstacle.obstacle_id, obstacle.position.copy(), obstacle.is_active(step)) for obstacle in obstacles
                    ],
                    observed_obstacles=[observation.position.copy() for observation in observations],
                    debug=debug,
                )
            )

            if np.linalg.norm(robot_position - goal_position) <= self.config.goal_tolerance:
                success = True
                break

        metrics = SimulationMetrics(
            success=success,
            collision_count=dynamic_collision_count + static_collision_count,
            dynamic_collision_count=dynamic_collision_count,
            static_collision_count=static_collision_count,
            path_length=path_length(trajectory),
            steps_used=len(trajectory) - 1,
            avoidance_events=avoidance_events,
            minimum_clearance=minimum_clearance if np.isfinite(minimum_clearance) else 0.0,
        )
        result = SimulationResult(
            config=self.config,
            grid_map=self.grid_map,
            global_path=global_path,
            trajectory=trajectory,
            frames=frames,
            metrics=metrics,
            start=start,
            goal=goal,
            output_dir=output_dir,
            obstacles=obstacles,
        )
        self._write_metrics(result)
        return result

    @staticmethod
    def _write_metrics(result: SimulationResult) -> None:
        metrics_path = result.output_dir / "metrics.json"
        payload = {
            "success": result.metrics.success,
            "collision_count": result.metrics.collision_count,
            "dynamic_collision_count": result.metrics.dynamic_collision_count,
            "static_collision_count": result.metrics.static_collision_count,
            "path_length": result.metrics.path_length,
            "steps_used": result.metrics.steps_used,
            "avoidance_events": result.metrics.avoidance_events,
            "minimum_clearance": result.metrics.minimum_clearance,
            "global_path": result.global_path,
            "start": result.start,
            "goal": result.goal,
        }
        with open(metrics_path, "w") as metrics_file:
            json.dump(payload, metrics_file, indent=2)
