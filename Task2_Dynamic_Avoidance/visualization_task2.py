"""Visualization helpers for Task 2 dynamic avoidance outputs."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Rectangle

from Task2_Dynamic_Avoidance.robot_simulator import SimulationResult


def _setup_axes(ax, result: SimulationResult) -> None:
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-0.5, result.grid_map.width - 0.5)
    ax.set_ylim(-0.5, result.grid_map.height - 0.5)
    ax.set_xticks(range(result.grid_map.width))
    ax.set_yticks(range(result.grid_map.height))
    ax.grid(True, color="#dddddd", linewidth=0.5)
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    for ox, oy in result.grid_map.obstacles:
        ax.add_patch(Rectangle((ox - 0.5, oy - 0.5), 1.0, 1.0, color="#333333", alpha=0.85))

    global_x = [point[0] for point in result.global_path]
    global_y = [point[1] for point in result.global_path]
    ax.plot(global_x, global_y, "--", color="#1f77b4", linewidth=1.8, label="A* global path")
    ax.scatter([result.start[0]], [result.start[1]], marker="o", s=90, color="green", label="start", zorder=5)
    ax.scatter([result.goal[0]], [result.goal[1]], marker="*", s=150, color="red", label="goal", zorder=5)


def save_final_trajectory(result: SimulationResult, filename: str = "final_trajectory.png") -> Path:
    """Save a static final trajectory figure."""
    output_path = result.output_dir / filename
    fig, ax = plt.subplots(figsize=(7, 7))
    _setup_axes(ax, result)
    trajectory_x = [point[0] for point in result.trajectory]
    trajectory_y = [point[1] for point in result.trajectory]
    ax.plot(trajectory_x, trajectory_y, color="#ff7f0e", linewidth=2.2, label="actual trajectory")
    if result.frames:
        last_frame = result.frames[-1]
        for obstacle_id, position, active in last_frame.dynamic_obstacles:
            if active:
                ax.add_patch(Circle(position, result.config.obstacle_radius, color="purple", alpha=0.55))
                ax.text(position[0] + 0.05, position[1] + 0.05, obstacle_id, fontsize=7)
    ax.set_title(
        "Task2 Dynamic Avoidance\n"
        f"success={result.metrics.success}, collisions={result.metrics.collision_count}, "
        f"min_clearance={result.metrics.minimum_clearance:.3f}"
    )
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path


def save_animation(result: SimulationResult, filename: str = "avoidance_animation.gif") -> Path:
    """Save an animation of the avoidance process as a GIF."""
    output_path = result.output_dir / filename
    fig, ax = plt.subplots(figsize=(7, 7))
    _setup_axes(ax, result)

    trajectory_line, = ax.plot([], [], color="#ff7f0e", linewidth=2.2, label="actual trajectory")
    robot_patch = Circle(result.start, result.config.robot_radius, color="#ff7f0e", zorder=8)
    ax.add_patch(robot_patch)
    dynamic_patches = [Circle((0, 0), result.config.obstacle_radius, color="purple", alpha=0.55, visible=False) for _ in result.obstacles]
    for patch in dynamic_patches:
        ax.add_patch(patch)
    observed_scatter = ax.scatter([], [], marker="x", color="magenta", s=35, label="noisy observations", zorder=9)
    ax.legend(loc="upper right", fontsize=8)

    def init():
        trajectory_line.set_data([], [])
        robot_patch.center = result.start
        observed_scatter.set_offsets(np.empty((0, 2)))
        return [trajectory_line, robot_patch, observed_scatter, *dynamic_patches]

    def update(frame_index: int):
        frame = result.frames[frame_index]
        traj = result.trajectory[: frame_index + 2]
        trajectory_line.set_data([p[0] for p in traj], [p[1] for p in traj])
        robot_patch.center = frame.robot_position
        for patch, (_, position, active) in zip(dynamic_patches, frame.dynamic_obstacles):
            patch.center = position
            patch.set_visible(active)
        if frame.observed_obstacles:
            observed_scatter.set_offsets(np.array(frame.observed_obstacles))
        else:
            observed_scatter.set_offsets(np.empty((0, 2)))
        ax.set_title(f"Task2 Dynamic Avoidance - step {frame.step}")
        return [trajectory_line, robot_patch, observed_scatter, *dynamic_patches]

    frame_step = max(1, len(result.frames) // 180)
    frame_indices = list(range(0, len(result.frames), frame_step)) or [0]
    animation = FuncAnimation(
        fig,
        update,
        frames=frame_indices,
        init_func=init,
        interval=result.config.animation_interval_ms,
        blit=True,
    )
    animation.save(output_path, writer=PillowWriter(fps=max(1, int(1000 / result.config.animation_interval_ms))))
    plt.close(fig)
    return output_path


def save_visual_outputs(result: SimulationResult) -> dict[str, Path]:
    """Save all requested visual outputs for a simulation."""
    outputs = {"trajectory": save_final_trajectory(result)}
    if result.config.save_animation and result.frames:
        outputs["animation"] = save_animation(result)
    return outputs
