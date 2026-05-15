from __future__ import annotations

import math

import numpy as np

from rosbag_adaptive_localization_viewer.models import TrajectorySample, TrajectorySeries


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _interpolate_yaw(samples: list[TrajectorySample], t_sec: float) -> float | None:
    """Linearly interpolate yaw at t_sec. Returns None if no yaw data is available."""
    yaw_samples = [s for s in samples if s.yaw_rad is not None]
    if not yaw_samples:
        return None
    if t_sec <= yaw_samples[0].t_sec:
        return yaw_samples[0].yaw_rad
    if t_sec >= yaw_samples[-1].t_sec:
        return yaw_samples[-1].yaw_rad
    for left, right in zip(yaw_samples, yaw_samples[1:]):
        if left.t_sec <= t_sec <= right.t_sec:
            dt = right.t_sec - left.t_sec
            if dt == 0:
                return left.yaw_rad
            alpha = (t_sec - left.t_sec) / dt
            d_yaw = _wrap_angle((right.yaw_rad or 0.0) - (left.yaw_rad or 0.0))
            return _wrap_angle((left.yaw_rad or 0.0) + alpha * d_yaw)
    return None


# ---------------------------------------------------------------------------
# Time normalization
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Path geometry
# ---------------------------------------------------------------------------

def path_length(samples: list[TrajectorySample]) -> float:
    if len(samples) < 2:
        return 0.0
    total = 0.0
    for prev, curr in zip(samples, samples[1:]):
        total += math.dist((prev.x, prev.y), (curr.x, curr.y))
    return total


# ---------------------------------------------------------------------------
# Interpolation
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def compute_position_error_metrics(
    estimate: TrajectorySeries,
    ground_truth: TrajectorySeries,
) -> dict[str, float]:
    """Compute RMSE, max error, ATE (with SE(2) alignment), RTE, and yaw RMSE."""
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
    metrics: dict[str, float] = {
        "rmse_position_m": rmse,
        "max_position_error_m": max(errors),
    }

    ate = compute_ate(estimate, ground_truth)
    if ate is not None:
        metrics["ate_m"] = ate

    rte = compute_rte(estimate, ground_truth)
    if rte is not None:
        metrics["rte_m"] = rte

    yaw_rmse = compute_yaw_rmse(estimate, ground_truth)
    if yaw_rmse is not None:
        metrics["yaw_rmse_rad"] = yaw_rmse

    return metrics


def compute_ate(
    estimate: TrajectorySeries,
    ground_truth: TrajectorySeries,
) -> float | None:
    """Absolute Trajectory Error with SE(2) Umeyama alignment.

    Aligns the estimate to the ground truth via SVD to remove global
    translation/rotation offsets before computing RMSE.  Returns None
    when fewer than three matched pairs exist.
    """
    P_list: list[list[float]] = []
    Q_list: list[list[float]] = []
    for sample in estimate.samples:
        gt_xy = interpolate_xy(ground_truth.samples, sample.t_sec)
        if gt_xy is None:
            continue
        P_list.append([sample.x, sample.y])
        Q_list.append(list(gt_xy))

    if len(P_list) < 3:
        return None

    P = np.array(P_list, dtype=float)
    Q = np.array(Q_list, dtype=float)

    mu_P = P.mean(axis=0)
    mu_Q = Q.mean(axis=0)
    P_c = P - mu_P
    Q_c = Q - mu_Q

    H = P_c.T @ Q_c
    U, _, Vt = np.linalg.svd(H)
    d = float(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    t = mu_Q - R @ mu_P

    P_aligned = (R @ P.T).T + t
    errors = np.linalg.norm(P_aligned - Q, axis=1)
    return float(np.sqrt(np.mean(errors ** 2)))


def compute_rte(
    estimate: TrajectorySeries,
    ground_truth: TrajectorySeries,
    segment_sec: float = 1.0,
) -> float | None:
    """Relative Trajectory Error over fixed time segments of *segment_sec* seconds.

    For each starting pose in the estimate, finds the first subsequent pose
    that is at least *segment_sec* later, then compares the relative motion
    of estimate vs ground-truth over that segment.  Returns None when no
    valid segments exist.
    """
    samples = estimate.samples
    errors: list[float] = []

    for i, s0 in enumerate(samples):
        target_t = s0.t_sec + segment_sec
        s1: TrajectorySample | None = None
        for j in range(i + 1, len(samples)):
            if samples[j].t_sec >= target_t:
                s1 = samples[j]
                break
        if s1 is None:
            continue

        gt0 = interpolate_xy(ground_truth.samples, s0.t_sec)
        gt1 = interpolate_xy(ground_truth.samples, s1.t_sec)
        if gt0 is None or gt1 is None:
            continue

        de_x = s1.x - s0.x
        de_y = s1.y - s0.y
        dg_x = gt1[0] - gt0[0]
        dg_y = gt1[1] - gt0[1]

        errors.append(math.sqrt((de_x - dg_x) ** 2 + (de_y - dg_y) ** 2))

    if not errors:
        return None
    return math.sqrt(sum(e * e for e in errors) / len(errors))


def compute_yaw_rmse(
    estimate: TrajectorySeries,
    ground_truth: TrajectorySeries,
) -> float | None:
    """Yaw RMSE in radians.

    Returns None if neither trajectory carries yaw data (e.g. OptiTrack GT
    without heading).
    """
    errors: list[float] = []
    for sample in estimate.samples:
        if sample.yaw_rad is None:
            continue
        gt_yaw = _interpolate_yaw(ground_truth.samples, sample.t_sec)
        if gt_yaw is None:
            continue
        errors.append(_wrap_angle(sample.yaw_rad - gt_yaw))

    if not errors:
        return None
    return math.sqrt(sum(e * e for e in errors) / len(errors))


# ---------------------------------------------------------------------------
# Error annotation
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Geometric downsampling — Ramer-Douglas-Peucker
# ---------------------------------------------------------------------------

def _rdp_simplify(samples: list[TrajectorySample], epsilon: float) -> list[TrajectorySample]:
    """Iterative Ramer-Douglas-Peucker simplification on 2-D (x, y) positions.

    Preserves the start and end points unconditionally.  Uses an explicit
    stack to avoid Python recursion-depth limits on long trajectories.
    """
    n = len(samples)
    if n <= 2:
        return list(samples)

    keep = [False] * n
    keep[0] = True
    keep[n - 1] = True

    stack: list[tuple[int, int]] = [(0, n - 1)]
    while stack:
        start, end = stack.pop()
        if end - start <= 1:
            continue

        sx, sy = samples[start].x, samples[start].y
        ex, ey = samples[end].x, samples[end].y
        dx = ex - sx
        dy = ey - sy
        line_len = math.sqrt(dx * dx + dy * dy)

        max_dist = 0.0
        max_idx = start + 1

        for i in range(start + 1, end):
            if line_len > 0.0:
                # Perpendicular distance from point to line
                d = abs(dy * samples[i].x - dx * samples[i].y + ex * sy - ey * sx) / line_len
            else:
                d = math.dist((samples[i].x, samples[i].y), (sx, sy))
            if d > max_dist:
                max_dist = d
                max_idx = i

        if max_dist > epsilon:
            keep[max_idx] = True
            stack.append((start, max_idx))
            stack.append((max_idx, end))

    return [s for s, k in zip(samples, keep) if k]


def downsample_series(
    series: TrajectorySeries,
    max_samples: int,
    rdp_epsilon: float = 0.01,
) -> TrajectorySeries:
    """Downsample a trajectory series to at most *max_samples* points.

    First applies Ramer-Douglas-Peucker geometric simplification with
    *rdp_epsilon* metres tolerance (default 1 cm).  If the result still
    exceeds *max_samples*, a uniform stride is applied as a hard cap.
    This preserves high-curvature regions (corners, loops) much better
    than pure uniform downsampling.
    """
    if len(series.samples) <= max_samples:
        return series

    reduced = _rdp_simplify(series.samples, rdp_epsilon)

    if len(reduced) > max_samples:
        step = max(1, len(reduced) // max_samples)
        reduced = reduced[::step]
        if reduced and reduced[-1] is not series.samples[-1]:
            reduced = [*reduced, series.samples[-1]]

    return TrajectorySeries(
        id=series.id,
        label=series.label,
        source_type=series.source_type,
        samples=reduced,
    )
