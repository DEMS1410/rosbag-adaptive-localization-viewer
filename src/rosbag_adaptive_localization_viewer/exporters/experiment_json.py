from __future__ import annotations

import json
from pathlib import Path

from rosbag_adaptive_localization_viewer.models import TrajectorySeries


def make_experiment_payload(
    experiment_name: str,
    trajectories: list[TrajectorySeries],
    metrics: dict | None = None,
    scene: dict | None = None,
) -> dict:
    return {
        "metadata": {
            "experiment_name": experiment_name,
        },
        "trajectories": [
            {
                "id": trajectory.id,
                "label": trajectory.label,
                "source_type": trajectory.source_type,
                "samples": [
                    {
                        "t_sec": sample.t_sec,
                        "x": sample.x,
                        "y": sample.y,
                        "z": sample.z,
                        "yaw_rad": sample.yaw_rad,
                        "error_m": sample.error_m,
                        "cov_xx": sample.cov_xx,
                        "cov_xy": sample.cov_xy,
                        "cov_yy": sample.cov_yy,
                        "yaw_var": sample.yaw_var,
                    }
                    for sample in trajectory.samples
                ],
            }
            for trajectory in trajectories
        ],
        "metrics": metrics or {},
        "scene": scene or {},
    }


def export_experiment_json(
    output_path: str | Path,
    experiment_name: str,
    trajectories: list[TrajectorySeries],
    metrics: dict | None = None,
    scene: dict | None = None,
) -> None:
    payload = make_experiment_payload(experiment_name, trajectories, metrics, scene)

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
