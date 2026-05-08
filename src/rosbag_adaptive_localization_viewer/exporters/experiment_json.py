from __future__ import annotations

import json
from pathlib import Path

from rosbag_adaptive_localization_viewer.models import TrajectorySeries


def export_experiment_json(
    output_path: str | Path,
    experiment_name: str,
    trajectories: list[TrajectorySeries],
    metrics: dict | None = None,
) -> None:
    payload = {
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
                    }
                    for sample in trajectory.samples
                ],
            }
            for trajectory in trajectories
        ],
        "metrics": metrics or {},
    }

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
