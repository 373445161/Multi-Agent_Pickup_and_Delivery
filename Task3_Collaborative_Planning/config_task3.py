"""Configuration helpers for Task 3 smart-campus fleet scheduling."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

import RoothPath


RESULTS_DIR = Path(RoothPath.get_root()) / "Results_Task3"
DEFAULT_INPUT = Path(RoothPath.get_root()) / "Environments" / "input_warehouse_small_random.yaml"
DEFAULT_OUTPUT = RESULTS_DIR / "task3_output.yaml"
DEFAULT_SCENARIO = RESULTS_DIR / "task3_smart_campus_input.yaml"
DEFAULT_A_STAR_MAX_ITER = 5000
DEFAULT_MAX_STEPS = 1000
DEFAULT_TASK_FREQ = 1
DEFAULT_N_DELAYS_PER_AGENT = 0
DEFAULT_DELAY_INTERVAL = 300
N_AGENTS = 4
N_TASKS = 20
SEEDS = list(range(10))
COMM_DELAYS = [0, 1, 2, 3, 5]


@dataclass(frozen=True)
class Task3Scenario:
    """Loaded smart-campus scenario data backed by the existing grid map."""

    param: dict[str, Any]
    agents: list[dict[str, Any]]
    tasks: list[dict[str, Any]]
    delays: dict[str, list[int]]
    scenario_path: Path
    output_path: Path


def ensure_results_dir(path: Path = RESULTS_DIR) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def load_base_map(input_path: str | Path = DEFAULT_INPUT) -> dict[str, Any]:
    with open(input_path, "r", encoding="utf-8") as param_file:
        return yaml.load(param_file, Loader=yaml.FullLoader)


def select_agents(param: dict[str, Any], n_agents: int) -> list[dict[str, Any]]:
    agents = [dict(agent) for agent in param["agents"][:n_agents]]
    if len(agents) < n_agents:
        raise ValueError(f"Requested {n_agents} agents, but map only defines {len(param['agents'])}.")
    return agents


def describe_smart_campus(param: dict[str, Any]) -> None:
    """Attach human-readable scenario metadata without changing planning fields."""
    param["scenario"] = {
        "name": "智慧园区无人车集群调度",
        "description": "原 warehouse 网格被包装为智慧园区路网；agents 为无人配送车，"
        "task start/pickup 为物流取货点，task goal 为配送点或巡检点。",
        "agent_label": "无人配送车",
        "pickup_label": "物流取货点",
        "goal_label": "配送点/巡检点",
    }


def write_yaml(path: str | Path, data: dict[str, Any]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as out:
        yaml.dump(data, out, allow_unicode=True, sort_keys=False)
    return path
