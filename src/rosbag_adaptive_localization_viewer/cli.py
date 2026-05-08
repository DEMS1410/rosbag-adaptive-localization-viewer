from __future__ import annotations

import argparse
import os
import sys

from rosbag_adaptive_localization_viewer.config import load_session_config
from rosbag_adaptive_localization_viewer.exporters.experiment_json import export_experiment_json
from rosbag_adaptive_localization_viewer.loaders.optitrack_csv import load_optitrack_rigid_body
from rosbag_adaptive_localization_viewer.loaders.rosbag2 import (
    extract_trajectory_from_topic,
    inspect_bag_topics,
)
from rosbag_adaptive_localization_viewer.models import TrajectorySample, TrajectorySeries
from rosbag_adaptive_localization_viewer.processing.trajectories import (
    annotate_position_error,
    compute_position_error_metrics,
    downsample_series,
    normalize_series_time,
    path_length,
)


def inspect_bag(db3_path: str) -> int:
    topics = inspect_bag_topics(db3_path)
    for topic in topics:
        print(f"{topic['name']} | {topic['type']} | messages={topic['message_count']}")
    return 0


def inspect_optitrack(csv_path: str, rigid_body: str = "ROBOT") -> int:
    frame = load_optitrack_rigid_body(csv_path, rigid_body)
    print(f"rows={len(frame)} rigid_body={rigid_body}")
    print(frame.head(5).to_string(index=False))
    return 0


def build_experiment(config_path: str, output_path: str) -> int:
    config = load_session_config(config_path)
    config_dir = os.path.dirname(os.path.abspath(config_path))
    estimate = config["estimate"]
    ground_truth = config.get("ground_truth", {})
    bag_path = os.path.abspath(os.path.join(config_dir, config["bag_path"]))

    trajectories: list[TrajectorySeries] = []

    primary_topic = estimate["primary_topic"]
    primary_samples = extract_trajectory_from_topic(bag_path, primary_topic)
    primary_series = normalize_series_time(
        TrajectorySeries(
            id=primary_topic.strip("/").replace("/", "_"),
            label=primary_topic,
            source_type="estimate",
            samples=primary_samples,
        )
    )
    trajectories.append(primary_series)

    for topic in estimate.get("comparison_topics", []):
        samples = extract_trajectory_from_topic(bag_path, topic)
        trajectories.append(
            normalize_series_time(
                TrajectorySeries(
                    id=topic.strip("/").replace("/", "_"),
                    label=topic,
                    source_type="comparison",
                    samples=samples,
                )
            )
        )

    gt_series: TrajectorySeries | None = None
    if ground_truth.get("enabled"):
        csv_path = os.path.abspath(os.path.join(config_dir, ground_truth["csv_path"]))
        gt_frame = load_optitrack_rigid_body(
            csv_path,
            ground_truth.get("rigid_body_name", "ROBOT"),
        )
        gt_samples = [
            TrajectorySample(
                t_sec=float(row.timestamp_sec),
                x=float(row.x_m),
                y=float(row.y_m),
                z=float(row.z_m),
                yaw_rad=None,
                source="ground_truth",
            )
            for row in gt_frame.itertuples(index=False)
        ]
        gt_series = normalize_series_time(
            TrajectorySeries(
                id="ground_truth",
                label=ground_truth.get("rigid_body_name", "Ground Truth"),
                source_type="ground_truth",
                samples=gt_samples,
            )
        )
        primary_series = annotate_position_error(primary_series, gt_series)
        trajectories[0] = primary_series
        trajectories.append(downsample_series(gt_series, max_samples=4000))

    for index, series in enumerate(trajectories[:-1] if gt_series is not None else trajectories):
        if series.source_type == "comparison":
            trajectories[index] = downsample_series(series, max_samples=2500)

    trajectories[0] = downsample_series(trajectories[0], max_samples=2500)

    metrics = {
        "duration_sec": max((series.samples[-1].t_sec for series in trajectories if series.samples), default=0.0),
        "path_length_m": path_length(primary_series.samples),
    }
    if gt_series is not None:
        metrics.update(compute_position_error_metrics(primary_series, gt_series))

    export_experiment_json(output_path, config["experiment_name"], trajectories, metrics)
    print(f"Exported experiment to {output_path}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="ralv")
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_bag_parser = subparsers.add_parser("inspect-bag")
    inspect_bag_parser.add_argument("db3_path")

    inspect_optitrack_parser = subparsers.add_parser("inspect-optitrack")
    inspect_optitrack_parser.add_argument("csv_path")
    inspect_optitrack_parser.add_argument("--rigid-body", default="ROBOT")

    build_parser = subparsers.add_parser("build-experiment")
    build_parser.add_argument("config_path")
    build_parser.add_argument("output_path")

    args = parser.parse_args(argv)
    if args.command == "inspect-bag":
        return inspect_bag(args.db3_path)
    if args.command == "inspect-optitrack":
        return inspect_optitrack(args.csv_path, args.rigid_body)
    if args.command == "build-experiment":
        return build_experiment(args.config_path, args.output_path)

    parser.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
