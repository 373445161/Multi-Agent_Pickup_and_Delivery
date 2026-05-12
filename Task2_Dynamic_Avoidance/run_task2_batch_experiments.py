"""Run Task 2 batch experiments for dynamic obstacle avoidance.

Command:
    python -m Task2_Dynamic_Avoidance.run_task2_batch_experiments
"""
from __future__ import annotations

import argparse
import csv
import json
from itertools import product
from statistics import mean

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from Task2_Dynamic_Avoidance.config_task2 import DEFAULT_RESULTS_DIR, Task2Config
from Task2_Dynamic_Avoidance.robot_simulator import RobotSimulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Task2 batch experiments.")
    parser.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3], help="Random seeds per parameter group.")
    parser.add_argument("--obstacle-counts", nargs="+", type=int, default=[2, 4, 6], help="Dynamic obstacle counts.")
    parser.add_argument("--noise-stds", nargs="+", type=float, default=[0.0, 0.08, 0.16], help="Sensor noise std values.")
    parser.add_argument("--obstacle-speeds", nargs="+", type=float, default=[0.45, 0.75, 1.05], help="Dynamic obstacle speeds.")
    parser.add_argument("--max-steps", type=int, default=260, help="Maximum steps per run.")
    return parser.parse_args()


def write_csv(rows: list[dict], path) -> None:
    with open(path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def aggregate(rows: list[dict]) -> list[dict]:
    grouped: dict[tuple[int, float, float], list[dict]] = {}
    for row in rows:
        key = (row["obstacle_count"], row["noise_std"], row["obstacle_speed"])
        grouped.setdefault(key, []).append(row)

    summary = []
    for (obstacle_count, noise_std, obstacle_speed), group in sorted(grouped.items()):
        summary.append(
            {
                "obstacle_count": obstacle_count,
                "noise_std": noise_std,
                "obstacle_speed": obstacle_speed,
                "runs": len(group),
                "success_rate": sum(1 for row in group if row["success"]) / len(group),
                "avg_collision_count": mean(row["collision_count"] for row in group),
                "avg_dynamic_collision_count": mean(row["dynamic_collision_count"] for row in group),
                "avg_static_collision_count": mean(row["static_collision_count"] for row in group),
                "avg_path_length": mean(row["path_length"] for row in group),
                "avg_steps_used": mean(row["steps_used"] for row in group),
                "avg_avoidance_events": mean(row["avoidance_events"] for row in group),
                "avg_minimum_clearance": mean(row["minimum_clearance"] for row in group),
            }
        )
    return summary


def plot_success_rates(summary: list[dict], output_path) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))
    labels = [
        f"n={row['obstacle_count']}\nσ={row['noise_std']}\nv={row['obstacle_speed']}" for row in summary
    ]
    rates = [row["success_rate"] for row in summary]
    ax.bar(range(len(summary)), rates, color="#2ca02c")
    ax.set_ylim(0.0, 1.05)
    ax.set_ylabel("avoidance success rate")
    ax.set_xlabel("parameter group")
    ax.set_title("Task2 batch success rate by obstacle count, noise, and speed")
    ax.set_xticks(range(len(summary)))
    ax.set_xticklabels(labels, rotation=90, fontsize=7)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    results_dir = DEFAULT_RESULTS_DIR / "batch_experiments"
    results_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    for obstacle_count, noise_std, obstacle_speed, seed in product(
        args.obstacle_counts,
        args.noise_stds,
        args.obstacle_speeds,
        args.seeds,
    ):
        run_name = f"n{obstacle_count}_noise{noise_std:.2f}_speed{obstacle_speed:.2f}_seed{seed}"
        config = Task2Config(
            results_dir=results_dir,
            random_seed=seed,
            dynamic_obstacle_count=obstacle_count,
            sensor_noise_std=noise_std,
            dynamic_obstacle_speed=obstacle_speed,
            max_steps=args.max_steps,
            save_animation=False,
        )
        result = RobotSimulator(config).run(run_name)
        metric_row = result.metrics_dict()
        metric_row.update(
            {
                "run_name": run_name,
                "seed": seed,
                "obstacle_count": obstacle_count,
                "noise_std": noise_std,
                "obstacle_speed": obstacle_speed,
            }
        )
        rows.append(metric_row)
        print(
            f"{run_name}: success={result.metrics.success}, "
            f"collisions={result.metrics.collision_count}, steps={result.metrics.steps_used}"
        )

    summary = aggregate(rows)
    write_csv(rows, results_dir / "batch_results.csv")
    write_csv(summary, results_dir / "batch_summary.csv")
    with open(results_dir / "batch_summary.json", "w") as json_file:
        json.dump(summary, json_file, indent=2)
    plot_success_rates(summary, results_dir / "success_rate_summary.png")

    overall_success_rate = sum(1 for row in rows if row["success"]) / len(rows)
    print("Batch experiments finished.")
    print(f"Overall success rate: {overall_success_rate:.3f}")
    print(f"Results directory: {results_dir}")


if __name__ == "__main__":
    main()
