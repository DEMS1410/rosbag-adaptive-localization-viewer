from __future__ import annotations

from pathlib import Path

from rosbag_adaptive_localization_viewer.loaders.optitrack_csv import load_optitrack_rigid_body
from rosbag_adaptive_localization_viewer.loaders.rosbag2 import (
    extract_map_base_trajectory,
    extract_map_points,
    extract_scan_points,
    extract_trajectory_from_topic,
)
from rosbag_adaptive_localization_viewer.models import TrajectorySample, TrajectorySeries
from rosbag_adaptive_localization_viewer.processing.trajectories import (
    annotate_position_error,
    compute_position_error_metrics,
    downsample_series,
    normalize_series_time,
    path_length,
)


def build_experiment_payload(
    *,
    experiment_name: str,
    bag_path: str | Path,
    primary_topic: str,
    comparison_topics: list[str] | None = None,
    optitrack_csv_path: str | Path | None = None,
    rigid_body_name: str = "ROBOT",
    ros_distro: str = "humble",
) -> dict:
    bag_path = Path(bag_path)
    comparison_topics = comparison_topics or []

    trajectories: list[TrajectorySeries] = []
    primary_samples = extract_trajectory_from_topic(bag_path, primary_topic, ros_distro=ros_distro)
    primary_series = normalize_series_time(
        TrajectorySeries(
            id=primary_topic.strip("/").replace("/", "_"),
            label=primary_topic,
            source_type="estimate",
            samples=primary_samples,
        )
    )
    trajectories.append(primary_series)

    for topic in comparison_topics:
        samples = extract_trajectory_from_topic(bag_path, topic, ros_distro=ros_distro)
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
    if optitrack_csv_path is not None:
        gt_frame = load_optitrack_rigid_body(optitrack_csv_path, rigid_body_name)
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
                label=rigid_body_name,
                source_type="ground_truth",
                samples=gt_samples,
            )
        )
        primary_series = annotate_position_error(primary_series, gt_series)
        trajectories[0] = primary_series
        trajectories.append(downsample_series(gt_series, max_samples=4000))

    map_base_samples, map_base_method = extract_map_base_trajectory(bag_path, ros_distro=ros_distro)
    map_base_series: TrajectorySeries | None = None
    if map_base_samples:
        map_base_series = normalize_series_time(
            TrajectorySeries(
                id="map_base",
                label="map -> base",
                source_type="comparison",
                samples=map_base_samples,
            )
        )
        trajectories.append(downsample_series(map_base_series, max_samples=3000))

    upper_bound = trajectories[:-1] if gt_series is not None else trajectories
    for index, series in enumerate(upper_bound):
        if series.source_type == "comparison":
            trajectories[index] = downsample_series(series, max_samples=2500)

    trajectories[0] = downsample_series(trajectories[0], max_samples=2500)

    metrics: dict[str, float] = {
        "duration_sec": max((series.samples[-1].t_sec for series in trajectories if series.samples), default=0.0),
        "path_length_m": path_length(primary_series.samples),
    }
    if gt_series is not None:
        metrics.update(compute_position_error_metrics(primary_series, gt_series))

    reference_samples = map_base_samples if map_base_samples else primary_samples
    t0 = reference_samples[0].t_sec if reference_samples else 0.0
    map_points = extract_map_points(bag_path, ros_distro=ros_distro)
    scans = extract_scan_points(bag_path, reference_samples, ros_distro=ros_distro)
    scene = {
        "map": map_points,
        "scans": [
            {
                "t_sec": scan["t_sec"] - t0,
                "points": scan["points"],
            }
            for scan in scans
        ],
        "map_base_method": map_base_method,
    }

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
        "metrics": metrics,
        "scene": scene,
    }
