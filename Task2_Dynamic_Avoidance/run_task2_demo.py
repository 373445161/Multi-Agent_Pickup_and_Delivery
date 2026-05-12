"""Run one Task 2 dynamic avoidance demonstration.

Command:
    python -m Task2_Dynamic_Avoidance.run_task2_demo
"""
from __future__ import annotations

import argparse
import json

from Task2_Dynamic_Avoidance.config_task2 import Task2Config
from Task2_Dynamic_Avoidance.robot_simulator import RobotSimulator
from Task2_Dynamic_Avoidance.visualization_task2 import save_visual_outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Task2 dynamic obstacle avoidance demo.")
    parser.add_argument("--no-animation", action="store_true", help="Skip GIF animation generation.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed.")
    parser.add_argument("--obstacles", type=int, default=4, help="Number of dynamic obstacles.")
    parser.add_argument("--noise", type=float, default=0.08, help="Gaussian observation noise standard deviation.")
    parser.add_argument("--speed", type=float, default=0.75, help="Base dynamic obstacle speed.")
    parser.add_argument("--max-steps", type=int, default=260, help="Maximum simulation steps.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = Task2Config(
        random_seed=args.seed,
        dynamic_obstacle_count=args.obstacles,
        sensor_noise_std=args.noise,
        dynamic_obstacle_speed=args.speed,
        max_steps=args.max_steps,
        save_animation=not args.no_animation,
    )
    simulator = RobotSimulator(config)
    result = simulator.run("demo")
    visual_outputs = save_visual_outputs(result)

    print("Task2 demo finished.")
    print(json.dumps(result.metrics_dict(), indent=2))
    print(f"Results directory: {result.output_dir}")
    for name, path in visual_outputs.items():
        print(f"{name}: {path}")


if __name__ == "__main__":
    main()
