"""Batch experiments for communication delay in Task 3."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

from Task3_Collaborative_Planning.config_task3 import COMM_DELAYS, N_AGENTS, N_TASKS, RESULTS_DIR, SEEDS, ensure_results_dir
from Task3_Collaborative_Planning.export_task3_report_tables import export_summary_tables
from Task3_Collaborative_Planning.plot_task3_results import plot_delay_experiment_results
from Task3_Collaborative_Planning.run_task3_demo import run_simulation


FIELDS = [
    "seed",
    "comm_delay_steps",
    "n_agents",
    "n_tasks",
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


def _parse_int_list(value: str) -> list[int]:
    return [int(v.strip()) for v in value.split(",") if v.strip()]


def run_batch(seeds: list[int], comm_delays: list[int], n_agents: int, n_tasks: int, max_steps: int) -> Path:
    output_dir = ensure_results_dir(RESULTS_DIR)
    csv_path = output_dir / "task3_comm_delay_results.csv"
    rows = []
    for seed in seeds:
        for delay in comm_delays:
            result = run_simulation(
                comm_delay=delay,
                n_agents=n_agents,
                n_tasks=n_tasks,
                seed=seed,
                max_steps=max_steps,
                make_animation=False,
                quiet=True,
            )
            metrics = result["metrics"]
            rows.append({field: metrics[field] for field in FIELDS})
            print(
                f"seed={seed} delay={delay}: completed={metrics['completed_tasks']}/{n_tasks}, "
                f"makespan={metrics['makespan']}, replans={metrics['n_replans']}, "
                f"waiting={metrics['waiting_steps_total']}"
            )
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    plot_delay_experiment_results(csv_path, output_dir)
    export_summary_tables(csv_path, output_dir)
    return csv_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Task 3 communication-delay batch experiments")
    parser.add_argument("--seeds", default=",".join(str(s) for s in SEEDS), help="comma-separated random seeds")
    parser.add_argument("--comm-delays", default=",".join(str(d) for d in COMM_DELAYS), help="comma-separated delays")
    parser.add_argument("--n-agents", type=int, default=N_AGENTS)
    parser.add_argument("--n-tasks", type=int, default=N_TASKS)
    parser.add_argument("--max-steps", type=int, default=1000)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_path = run_batch(_parse_int_list(args.seeds), _parse_int_list(args.comm_delays), args.n_agents, args.n_tasks, args.max_steps)
    print(f"Saved batch results to {csv_path}")


if __name__ == "__main__":
    main()
