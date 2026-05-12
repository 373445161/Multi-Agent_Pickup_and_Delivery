"""Plotting helpers for Task 3 result analysis."""
from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

from Task3_Collaborative_Planning.config_task3 import RESULTS_DIR


PLOTS = [
    ("makespan", "comm_delay_vs_makespan.png", "Makespan"),
    ("n_replans", "comm_delay_vs_replans.png", "Replans"),
    ("total_cost", "comm_delay_vs_total_cost.png", "Total cost"),
    ("waiting_steps_total", "comm_delay_vs_waiting_steps.png", "Waiting steps"),
    ("throughput", "comm_delay_vs_throughput.png", "Throughput"),
]


def _load_rows(csv_path: str | Path) -> list[dict[str, Any]]:
    with open(csv_path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def plot_delay_experiment_results(csv_path: str | Path, output_dir: str | Path = RESULTS_DIR) -> list[Path]:
    rows = _load_rows(csv_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    grouped: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[int(row["comm_delay_steps"])].append(row)
    delays = sorted(grouped)
    paths = []
    for metric, filename, ylabel in PLOTS:
        means = [sum(float(row[metric]) for row in grouped[d]) / len(grouped[d]) for d in delays]
        plt.figure(figsize=(6, 4))
        plt.plot(delays, means, marker="o", linewidth=2)
        plt.xlabel("Communication delay steps")
        plt.ylabel(ylabel)
        plt.title(f"Communication delay vs {ylabel}")
        plt.grid(True, alpha=0.3)
        path = output_dir / filename
        plt.tight_layout()
        plt.savefig(path, dpi=180)
        plt.close()
        paths.append(path)
    return paths


def plot_true_vs_reported_positions(
    param: dict[str, Any],
    schedule: dict[str, list[dict[str, int]]],
    delay_manager: Any,
    output_path: str | Path,
    comm_delay: int,
) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    dimensions = param["map"]["dimensions"]
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(-0.5, dimensions[0] - 0.5)
    ax.set_ylim(-0.5, dimensions[1] - 0.5)
    ax.set_aspect("equal")
    ax.set_title(f"True vs delayed reported positions (delay={comm_delay})")
    for x in range(dimensions[0]):
        for y in range(dimensions[1]):
            ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, fill=False, edgecolor="lightgray", linewidth=0.3))
    for ox, oy in param["map"].get("obstacles", []):
        ax.add_patch(Rectangle((ox - 0.5, oy - 0.5), 1, 1, color="black"))
    final_time = max((path[-1]["t"] for path in schedule.values() if path), default=0)
    reported = delay_manager.get_reported_positions(final_time)
    for agent_name, path in schedule.items():
        xs = [p["x"] for p in path]
        ys = [p["y"] for p in path]
        ax.plot(xs, ys, linewidth=1, alpha=0.55)
        true_pos = (path[-1]["x"], path[-1]["y"])
        ax.add_patch(Circle(true_pos, 0.28, color="tab:blue", alpha=0.8))
        if agent_name in reported:
            ax.add_patch(Circle(reported[agent_name], 0.18, color="red", alpha=0.75))
            ax.plot([true_pos[0], reported[agent_name][0]], [true_pos[1], reported[agent_name][1]], "r--", linewidth=1)
        ax.text(true_pos[0], true_pos[1], agent_name.replace("agent", ""), ha="center", va="center", fontsize=8)
    ax.scatter([], [], color="tab:blue", label="true current position")
    ax.scatter([], [], color="red", label="delayed reported position")
    ax.legend(loc="upper left", fontsize=8)
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close(fig)
    return output_path


def main() -> None:
    plot_delay_experiment_results(RESULTS_DIR / "task3_comm_delay_results.csv", RESULTS_DIR)


if __name__ == "__main__":
    main()


def save_delay_overlay_animation(
    param: dict[str, Any],
    schedule: dict[str, list[dict[str, int]]],
    delay_manager: Any,
    output_path: str | Path,
    comm_delay: int,
    slow_factor: int = 2,
) -> Path:
    """Create a Task-3-specific GIF showing true and delayed reported positions."""
    from matplotlib import animation

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    dimensions = param["map"]["dimensions"]
    max_t = max((path[-1]["t"] for path in schedule.values() if path), default=0)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(-0.5, dimensions[0] - 0.5)
    ax.set_ylim(-0.5, dimensions[1] - 0.5)
    ax.set_aspect("equal")
    for x in range(dimensions[0]):
        for y in range(dimensions[1]):
            ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, fill=False, edgecolor="lightgray", linewidth=0.25))
    for ox, oy in param["map"].get("obstacles", []):
        ax.add_patch(Rectangle((ox - 0.5, oy - 0.5), 1, 1, color="black"))
    true_artists = {}
    reported_artists = {}
    labels = {}
    colors = plt.cm.tab10.colors
    for idx, agent_name in enumerate(schedule):
        (true_line,) = ax.plot([], [], "o", color=colors[idx % len(colors)], markersize=9, label=f"{agent_name} true")
        (reported_line,) = ax.plot([], [], "x", color="red", markersize=8, label="reported" if idx == 0 else None)
        true_artists[agent_name] = true_line
        reported_artists[agent_name] = reported_line
        labels[agent_name] = ax.text(0, 0, agent_name.replace("agent", ""), ha="center", va="center", fontsize=8)
    title = ax.text(0.5, 1.02, "", transform=ax.transAxes, ha="center")
    ax.legend(loc="upper left", fontsize=7)

    def state_at(agent_path: list[dict[str, int]], t: int) -> tuple[int, int]:
        if t < len(agent_path):
            item = agent_path[t]
            return item["x"], item["y"]
        item = agent_path[-1]
        return item["x"], item["y"]

    def update(frame: int):
        t = min(max_t, frame // max(1, slow_factor))
        reported = delay_manager.get_reported_positions(t)
        title.set_text(f"Smart campus fleet | communication_delay_steps={comm_delay} | t={t}")
        artists = [title]
        for agent_name, path in schedule.items():
            x, y = state_at(path, t)
            true_artists[agent_name].set_data([x], [y])
            labels[agent_name].set_position((x, y))
            rx, ry = reported.get(agent_name, (x, y))
            reported_artists[agent_name].set_data([rx], [ry])
            artists.extend([true_artists[agent_name], reported_artists[agent_name], labels[agent_name]])
        return artists

    anim = animation.FuncAnimation(fig, update, frames=(max_t + 1) * max(1, slow_factor), interval=120, blit=True)
    anim.save(output_path, writer="pillow", fps=6)
    plt.close(fig)
    return output_path
