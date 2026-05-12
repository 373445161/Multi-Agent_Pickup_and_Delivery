"""Metrics for Task 3 communication-delay experiments."""
from __future__ import annotations

from collections import defaultdict
from typing import Any


def _path_cost(path: list[dict[str, int]]) -> int:
    if not path:
        return 0
    return max(0, len(path) - 1)


def count_collisions(actual_paths: dict[str, list[dict[str, int]]]) -> int:
    if not actual_paths:
        return 0
    max_t = max(path[-1]["t"] for path in actual_paths.values() if path)
    collisions = 0
    for t in range(max_t + 1):
        occupied: dict[tuple[int, int], str] = {}
        for agent_name, path in actual_paths.items():
            state = next((p for p in path if p["t"] == t), path[-1])
            pos = (state["x"], state["y"])
            if pos in occupied:
                collisions += 1
            occupied[pos] = agent_name
    return collisions


def compute_waiting_steps(actual_paths: dict[str, list[dict[str, int]]]) -> dict[str, int]:
    waiting = {}
    for agent_name, path in actual_paths.items():
        steps = 0
        for prev, cur in zip(path, path[1:]):
            if prev["x"] == cur["x"] and prev["y"] == cur["y"]:
                steps += 1
        waiting[agent_name] = steps
    return waiting


def compute_task3_metrics(simulation: Any, token_passing: Any, n_tasks: int) -> dict[str, Any]:
    actual_paths = simulation.get_actual_paths()
    completed_tasks = token_passing.get_completed_tasks()
    completed_times = token_passing.get_completed_tasks_times()
    makespan = max((path[-1]["t"] for path in actual_paths.values() if path), default=0)
    total_cost = sum(_path_cost(path) for path in actual_paths.values())
    waiting_per_agent = compute_waiting_steps(actual_paths)
    avg_completion = 0.0
    if completed_times:
        start_times = token_passing.get_token().get("start_tasks_times", {})
        durations = [finish - start_times.get(task_name, 0) for task_name, finish in completed_times.items()]
        avg_completion = sum(durations) / len(durations)
    return {
        "completed_tasks": completed_tasks,
        "completion_rate": completed_tasks / n_tasks if n_tasks else 0.0,
        "makespan": makespan,
        "total_cost": total_cost,
        "average_task_completion_time": avg_completion,
        "avg_task_completion_time": avg_completion,
        "n_replans": token_passing.get_n_replans(),
        "waiting_steps_total": sum(waiting_per_agent.values()),
        "waiting_steps_per_agent": dict(waiting_per_agent),
        "throughput": completed_tasks / makespan if makespan else 0.0,
        "collision_count": count_collisions(actual_paths),
    }


def summarize_by_delay(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[int(row["comm_delay_steps"])].append(row)
    summary = []
    metrics = [
        "completed_tasks",
        "completion_rate",
        "makespan",
        "total_cost",
        "avg_task_completion_time",
        "n_replans",
        "waiting_steps_total",
        "throughput",
        "collision_count",
    ]
    for delay in sorted(grouped):
        values = grouped[delay]
        item = {"comm_delay_steps": delay, "runs": len(values)}
        for metric in metrics:
            item[f"{metric}_mean"] = sum(float(row[metric]) for row in values) / len(values)
        summary.append(item)
    return summary
