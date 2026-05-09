from __future__ import annotations

import math

from rosbag_adaptive_localization_viewer.models import TrajectorySample, TrajectorySeries


def normalize_series_time(series: TrajectorySeries) -> TrajectorySeries:
    if not series.samples:
        return series
    t0 = series.samples[0].t_sec
    normalized_samples = [
        TrajectorySample(
            t_sec=sample.t_sec - t0,
            x=sample.x,
            y=sample.y,
            z=sample.z,
            yaw_rad=sample.yaw_rad,
            error_m=sample.error_m,
            cov_xx=sample.cov_xx,
            cov_xy=sample.cov_xy,
            cov_yy=sample.cov_yy,
            yaw_var=sample.yaw_var,
            source=sample.source,
        )
        for sample in series.samples
    ]
    return TrajectorySeries(
        id=series.id,
        label=series.label,
        source_type=series.source_type,
        samples=normalized_samples,
    )


def path_length(samples: list[TrajectorySample]) -> float:
    if len(samples) < 2:
        return 0.0
    total = 0.0
    for prev, curr in zip(samples, samples[1:]):
        total += math.dist((prev.x, prev.y), (curr.x, curr.y))
    return total


def interpolate_xy(samples: list[TrajectorySample], t_sec: float) -> tuple[float, float] | None:
    if not samples:
        return None
    if t_sec <= samples[0].t_sec:
        return (samples[0].x, samples[0].y)
    if t_sec >= samples[-1].t_sec:
        return (samples[-1].x, samples[-1].y)

    for left, right in zip(samples, samples[1:]):
        if left.t_sec <= t_sec <= right.t_sec:
            dt = right.t_sec - left.t_sec
            if dt == 0:
                return (left.x, left.y)
            alpha = (t_sec - left.t_sec) / dt
            x = left.x + alpha * (right.x - left.x)
            y = left.y + alpha * (right.y - left.y)
            return (x, y)
    return None


def compute_position_error_metrics(
    estimate: TrajectorySeries,
    ground_truth: TrajectorySeries,
) -> dict[str, float]:
    errors: list[float] = []
    for sample in estimate.samples:
        gt_xy = interpolate_xy(ground_truth.samples, sample.t_sec)
        if gt_xy is None:
            continue
        errors.append(math.dist((sample.x, sample.y), gt_xy))

    if not errors:
        return {
            "rmse_position_m": 0.0,
            "max_position_error_m": 0.0,
        }

    rmse = math.sqrt(sum(err * err for err in errors) / len(errors))
    return {
        "rmse_position_m": rmse,
        "max_position_error_m": max(errors),
    }


def annotate_position_error(
    estimate: TrajectorySeries,
    ground_truth: TrajectorySeries,
) -> TrajectorySeries:
    annotated: list[TrajectorySample] = []
    for sample in estimate.samples:
        gt_xy = interpolate_xy(ground_truth.samples, sample.t_sec)
        error = None if gt_xy is None else math.dist((sample.x, sample.y), gt_xy)
        annotated.append(
            TrajectorySample(
                t_sec=sample.t_sec,
                x=sample.x,
                y=sample.y,
                z=sample.z,
                yaw_rad=sample.yaw_rad,
                error_m=error,
                cov_xx=sample.cov_xx,
                cov_xy=sample.cov_xy,
                cov_yy=sample.cov_yy,
                yaw_var=sample.yaw_var,
                source=sample.source,
            )
        )
    return TrajectorySeries(
        id=estimate.id,
        label=estimate.label,
        source_type=estimate.source_type,
        samples=annotated,
    )


def downsample_series(series: TrajectorySeries, max_samples: int) -> TrajectorySeries:
    if len(series.samples) <= max_samples:
        return series

    step = max(1, len(series.samples) // max_samples)
    reduced = series.samples[::step]
    if reduced[-1] is not series.samples[-1]:
        reduced = [*reduced, series.samples[-1]]

    return TrajectorySeries(
        id=series.id,
        label=series.label,
        source_type=series.source_type,
        samples=reduced,
    )
