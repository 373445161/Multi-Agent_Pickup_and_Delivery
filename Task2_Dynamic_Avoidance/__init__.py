"""Task 2: dynamic obstacle real-time avoidance package.

This package contains a lightweight single-robot simulation pipeline:
global A* planning on a warehouse grid, dynamic obstacle simulation, noisy
sensor observations, APF local avoidance, metrics, visualization, and batch
experiments.
"""

__all__ = [
    "config_task2",
    "global_astar",
    "dynamic_obstacles",
    "apf_local_planner",
    "robot_simulator",
    "metrics",
    "visualization_task2",
]
