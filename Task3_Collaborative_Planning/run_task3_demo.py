"""Run Task 3 smart-campus collaborative planning demo."""
from __future__ import annotations

import argparse
import contextlib
import json
import random
import subprocess
import sys
from pathlib import Path
from typing import Any

import yaml

from Simulation.TP_with_recovery import TokenPassingRecovery
from Simulation.simulation_new_recovery import SimulationNewRecovery
from Simulation.tasks_and_delays_maker import gen_tasks_and_delays

from Task3_Collaborative_Planning.communication_delay import CommunicationDelayManager
from Task3_Collaborative_Planning.config_task3 import (
    DEFAULT_A_STAR_MAX_ITER,
    DEFAULT_INPUT,
    DEFAULT_MAX_STEPS,
    DEFAULT_N_DELAYS_PER_AGENT,
    DEFAULT_OUTPUT,
    DEFAULT_SCENARIO,
    DEFAULT_TASK_FREQ,
    RESULTS_DIR,
    describe_smart_campus,
    ensure_results_dir,
    load_base_map,
    select_agents,
    write_yaml,
)
from Task3_Collaborative_Planning.metrics_task3 import compute_task3_metrics
from Task3_Collaborative_Planning.plot_task3_results import plot_true_vs_reported_positions, save_delay_overlay_animation


def build_scenario(
    n_agents: int,
    n_tasks: int,
    seed: int,
    input_path: str | Path = DEFAULT_INPUT,
    scenario_path: str | Path = DEFAULT_SCENARIO,
    output_path: str | Path = DEFAULT_OUTPUT,
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]], dict[str, list[int]], Path, Path]:
    random.seed(seed)
    param = load_base_map(input_path)
    agents = select_agents(param, n_agents)
    starts = param["map"].get("start_locations") or [task["start"] for task in param.get("tasks", [])]
    goals = param["map"].get("goal_locations") or [task["goal"] for task in param.get("tasks", [])]
    tasks, delays = gen_tasks_and_delays(
        agents,
        starts,
        goals,
        n_tasks,
        param.get("task_freq", DEFAULT_TASK_FREQ),
        DEFAULT_N_DELAYS_PER_AGENT,
    )
    param["agents"] = agents
    param["tasks"] = tasks
    param["delays"] = delays
    param["n_tasks"] = n_tasks
    describe_smart_campus(param)
    scenario_path = write_yaml(scenario_path, param)
    return param, agents, tasks, delays, scenario_path, Path(output_path)


def run_simulation(
    comm_delay: int = 0,
    n_agents: int = 4,
    n_tasks: int = 20,
    seed: int = 42,
    slow_factor: int = 2,
    max_steps: int = DEFAULT_MAX_STEPS,
    output_dir: str | Path = RESULTS_DIR,
    make_animation: bool = True,
    quiet: bool = False,
) -> dict[str, Any]:
    output_dir = ensure_results_dir(Path(output_dir))
    scenario_path = output_dir / f"task3_smart_campus_delay_{comm_delay}_seed_{seed}.yaml"
    output_path = output_dir / f"task3_output_delay_{comm_delay}_seed_{seed}.yaml"
    param, agents, tasks, delays, scenario_path, output_path = build_scenario(
        n_agents=n_agents,
        n_tasks=n_tasks,
        seed=seed,
        scenario_path=scenario_path,
        output_path=output_path,
    )

    delay_manager = CommunicationDelayManager(comm_delay)
    simulation = SimulationNewRecovery(tasks, agents, delays=delays, communication_delay_manager=delay_manager)
    token_passing = TokenPassingRecovery(
        agents,
        param["map"]["dimensions"],
        param["map"]["obstacles"],
        param["map"]["non_task_endpoints"],
        simulation,
        a_star_max_iter=DEFAULT_A_STAR_MAX_ITER,
        k=1,
        replan_every_k_delays=False,
        pd=0.02,
        p_max=1,
        p_iter=1,
        new_recovery=True,
    )

    stream = open(Path(output_dir) / f"task3_delay_{comm_delay}_seed_{seed}.log", "w", encoding="utf-8") if quiet else None
    with contextlib.redirect_stdout(stream or sys.stdout):
        while token_passing.get_completed_tasks() != len(tasks) and simulation.get_time() < max_steps:
            simulation.time_forward(token_passing)
    if stream:
        stream.close()

    metrics = compute_task3_metrics(simulation, token_passing, len(tasks))
    metrics.update({"comm_delay_steps": comm_delay, "n_agents": n_agents, "n_tasks": n_tasks, "seed": seed})

    output = {
        "schedule": simulation.get_actual_paths(),
        "cost": metrics["total_cost"],
        "completed_tasks_times": token_passing.get_completed_tasks_times(),
        "n_replans": token_passing.get_n_replans(),
        "communication_delay_steps": comm_delay,
        "reported_positions": delay_manager.get_reported_positions(simulation.get_time()),
        "metrics": metrics,
    }
    write_yaml(output_path, output)
    with open(output_dir / f"task3_metrics_delay_{comm_delay}_seed_{seed}.json", "w", encoding="utf-8") as out:
        json.dump(metrics, out, ensure_ascii=False, indent=2)

    plot_true_vs_reported_positions(
        param,
        simulation.get_actual_paths(),
        delay_manager,
        output_dir / f"task3_true_vs_reported_delay_{comm_delay}_seed_{seed}.png",
        comm_delay,
    )

    if make_animation:
        video_path = output_dir / f"task3_animation_delay_{comm_delay}_seed_{seed}.gif"
        cmd = [
            sys.executable,
            "-m",
            "Utils.Visualization.visualize",
            "-map",
            str(scenario_path),
            "-schedule",
            str(output_path),
            "-slow_factor",
            str(slow_factor),
            "--video",
            str(video_path),
        ]
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL if quiet else None)
        except Exception as exc:  # original animation is optional in headless test environments
            metrics["animation_warning"] = str(exc)
        try:
            save_delay_overlay_animation(
                param,
                simulation.get_actual_paths(),
                delay_manager,
                output_dir / f"task3_delay_overlay_delay_{comm_delay}_seed_{seed}.gif",
                comm_delay,
                slow_factor=slow_factor,
            )
        except Exception as exc:
            metrics["delay_overlay_animation_warning"] = str(exc)
    return {"metrics": metrics, "scenario_path": str(scenario_path), "output_path": str(output_path)}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Task 3 智慧园区无人车集群调度 demo")
    parser.add_argument("--comm-delay", type=int, default=2, help="communication_delay_steps=tau")
    parser.add_argument("--n-agents", type=int, default=4, help="number of unmanned delivery vehicles")
    parser.add_argument("--n-tasks", type=int, default=20, help="number of pickup-to-delivery tasks")
    parser.add_argument("--seed", type=int, default=42, help="random seed")
    parser.add_argument("--slow-factor", type=int, default=2, help="visualization slow factor")
    parser.add_argument("--max-steps", type=int, default=DEFAULT_MAX_STEPS, help="simulation step cap")
    parser.add_argument("--no-animation", action="store_true", help="skip mp4 generation for fast/headless runs")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = run_simulation(
        comm_delay=args.comm_delay,
        n_agents=args.n_agents,
        n_tasks=args.n_tasks,
        seed=args.seed,
        slow_factor=args.slow_factor,
        max_steps=args.max_steps,
        make_animation=not args.no_animation,
    )
    print(json.dumps(result["metrics"], ensure_ascii=False, indent=2))
    print(f"Results saved under: {RESULTS_DIR}")


if __name__ == "__main__":
    main()
