"""Metric calculation for Task 2 dynamic avoidance simulations."""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class SimulationMetrics:
    success: bool
    collision_count: int
    dynamic_collision_count: int
    static_collision_count: int
    path_length: float
    steps_used: int
    avoidance_events: int
    minimum_clearance: float

    def to_dict(self) -> dict:
        return asdict(self)


def path_length(points: list[np.ndarray]) -> float:
    if len(points) < 2:
        return 0.0
    return float(sum(np.linalg.norm(points[i] - points[i - 1]) for i in range(1, len(points))))


def success_rate(metrics: list[SimulationMetrics]) -> float:
    if not metrics:
        return 0.0
    return sum(1 for metric in metrics if metric.success) / len(metrics)
