"""Export report-ready Task 3 communication-delay summary tables."""
from __future__ import annotations

import csv
from pathlib import Path

from Task3_Collaborative_Planning.config_task3 import RESULTS_DIR
from Task3_Collaborative_Planning.metrics_task3 import summarize_by_delay


SUMMARY_COLUMNS = [
    "comm_delay_steps",
    "runs",
    "completed_tasks_mean",
    "completion_rate_mean",
    "makespan_mean",
    "total_cost_mean",
    "avg_task_completion_time_mean",
    "n_replans_mean",
    "waiting_steps_total_mean",
    "throughput_mean",
    "collision_count_mean",
]


def export_summary_tables(csv_path: str | Path, output_dir: str | Path = RESULTS_DIR) -> tuple[Path, Path]:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with open(csv_path, "r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    summary = summarize_by_delay(rows)
    csv_out = output_dir / "table_comm_delay_summary.csv"
    md_out = output_dir / "table_comm_delay_summary.md"
    with open(csv_out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        writer.writerows(summary)
    with open(md_out, "w", encoding="utf-8") as f:
        f.write("| " + " | ".join(SUMMARY_COLUMNS) + " |\n")
        f.write("| " + " | ".join(["---"] * len(SUMMARY_COLUMNS)) + " |\n")
        for row in summary:
            values = []
            for col in SUMMARY_COLUMNS:
                val = row[col]
                values.append(f"{val:.4f}" if isinstance(val, float) else str(val))
            f.write("| " + " | ".join(values) + " |\n")
    return csv_out, md_out


def main() -> None:
    export_summary_tables(RESULTS_DIR / "task3_comm_delay_results.csv", RESULTS_DIR)


if __name__ == "__main__":
    main()
