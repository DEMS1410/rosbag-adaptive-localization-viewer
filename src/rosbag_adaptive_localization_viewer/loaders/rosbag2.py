from __future__ import annotations

import math
import sqlite3
from pathlib import Path

from rosbag_adaptive_localization_viewer.models import TrajectorySample


def inspect_bag_topics(db3_path: str | Path) -> list[dict[str, object]]:
    bag_path = Path(db3_path)
    with sqlite3.connect(bag_path) as conn:
        cursor = conn.cursor()
        rows = cursor.execute(
            """
            SELECT t.id, t.name, t.type, COUNT(m.id) AS message_count
            FROM topics t
            LEFT JOIN messages m ON m.topic_id = t.id
            GROUP BY t.id, t.name, t.type
            ORDER BY t.id
            """
        ).fetchall()

    return [
        {
            "id": row[0],
            "name": row[1],
            "type": row[2],
            "message_count": row[3],
        }
        for row in rows
    ]


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _compose_2d(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    ax, ay, ayaw = a
    bx, by, byaw = b
    ca = math.cos(ayaw)
    sa = math.sin(ayaw)
    x = ax + ca * bx - sa * by
    y = ay + sa * bx + ca * by
    yaw = _wrap_angle(ayaw + byaw)
    return x, y, yaw


def _interpolate_pose(samples: list[TrajectorySample], t_sec: float) -> tuple[float, float, float] | None:
    if not samples:
        return None
    if t_sec <= samples[0].t_sec:
        first = samples[0]
        return first.x, first.y, first.yaw_rad or 0.0
    if t_sec >= samples[-1].t_sec:
        last = samples[-1]
        return last.x, last.y, last.yaw_rad or 0.0

    for left, right in zip(samples, samples[1:]):
        if left.t_sec <= t_sec <= right.t_sec:
            dt = right.t_sec - left.t_sec
            if dt <= 0:
                return left.x, left.y, left.yaw_rad or 0.0
            alpha = (t_sec - left.t_sec) / dt
            yaw_left = left.yaw_rad or 0.0
            yaw_right = right.yaw_rad or yaw_left
            return (
                left.x + alpha * (right.x - left.x),
                left.y + alpha * (right.y - left.y),
                yaw_left + alpha * (yaw_right - yaw_left),
            )
    return None


def _get_typestore(ros_distro: str = "humble"):
    """Return a rosbags typestore for the given ROS 2 distribution name.

    Falls back to ROS2_HUMBLE if the requested distribution is unknown or
    not supported by the installed version of rosbags.
    """
    from rosbags.typesys import Stores, get_typestore

    _STORE_MAP: dict[str, str] = {
        "foxy": "ROS2_FOXY",
        "galactic": "ROS2_GALACTIC",
        "humble": "ROS2_HUMBLE",
        "iron": "ROS2_IRON",
        "jazzy": "ROS2_JAZZY",
        "rolling": "ROS2_ROLLING",
    }
    store_attr = _STORE_MAP.get(ros_distro.lower(), "ROS2_HUMBLE")
    store = getattr(Stores, store_attr, Stores.ROS2_HUMBLE)
    return get_typestore(store)


def extract_trajectory_from_topic(
    db3_path: str | Path,
    topic: str,
    ros_distro: str = "humble",
) -> list[TrajectorySample]:
    from rosbags.highlevel import AnyReader

    bag_path = Path(db3_path)
    samples: list[TrajectorySample] = []
    typestore = _get_typestore(ros_distro)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            raise ValueError(f"Topic '{topic}' not found in bag.")

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            is_odometry = connection.msgtype == "nav_msgs/msg/Odometry"
            is_pose_stamped = connection.msgtype == "geometry_msgs/msg/PoseStamped"

            if is_odometry:
                pose = msg.pose.pose
                position = pose.position
                orientation = pose.orientation
                # Only Odometry carries PoseWithCovariance which has .covariance
                cov_xx = float(msg.pose.covariance[0])
                cov_xy = float(msg.pose.covariance[1])
                cov_yy = float(msg.pose.covariance[7])
                yaw_var = float(msg.pose.covariance[35])
            elif is_pose_stamped:
                pose = msg.pose
                position = pose.position
                orientation = pose.orientation
                # PoseStamped.pose is a plain Pose — no covariance field
                cov_xx = cov_xy = cov_yy = yaw_var = None
            else:
                raise ValueError(
                    f"Topic '{topic}' uses unsupported message type '{connection.msgtype}' "
                    "for trajectory extraction."
                )

            samples.append(
                TrajectorySample(
                    t_sec=timestamp / 1e9,
                    x=float(position.x),
                    y=float(position.y),
                    z=float(position.z),
                    yaw_rad=_yaw_from_quaternion(
                        float(orientation.x),
                        float(orientation.y),
                        float(orientation.z),
                        float(orientation.w),
                    ),
                    cov_xx=cov_xx,
                    cov_xy=cov_xy,
                    cov_yy=cov_yy,
                    yaw_var=yaw_var,
                    source=topic,
                )
            )

    return samples


def extract_tf_pair_trajectory(
    db3_path: str | Path,
    parent_frame: str,
    child_frame: str,
    topics: tuple[str, ...] = ("/tf", "/tf_static"),
    ros_distro: str = "humble",
) -> list[TrajectorySample]:
    from rosbags.highlevel import AnyReader

    bag_path = Path(db3_path)
    typestore = _get_typestore(ros_distro)
    samples: list[TrajectorySample] = []

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [connection for connection in reader.connections if connection.topic in topics]
        if not connections:
            return []

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            for transform_stamped in msg.transforms:
                if transform_stamped.header.frame_id != parent_frame:
                    continue
                if transform_stamped.child_frame_id != child_frame:
                    continue

                transform = transform_stamped.transform
                translation = transform.translation
                rotation = transform.rotation
                transform_time_ns = (
                    int(transform_stamped.header.stamp.sec) * 1_000_000_000
                    + int(transform_stamped.header.stamp.nanosec)
                )
                if transform_time_ns <= 0:
                    transform_time_ns = timestamp

                samples.append(
                    TrajectorySample(
                        t_sec=transform_time_ns / 1e9,
                        x=float(translation.x),
                        y=float(translation.y),
                        z=float(translation.z),
                        yaw_rad=_yaw_from_quaternion(
                            float(rotation.x),
                            float(rotation.y),
                            float(rotation.z),
                            float(rotation.w),
                        ),
                        source=f"{parent_frame}->{child_frame}",
                    )
                )

    samples.sort(key=lambda sample: sample.t_sec)
    return samples


def extract_map_base_trajectory(
    db3_path: str | Path,
    base_candidates: tuple[str, ...] = ("base_footprint", "base_link"),
    tolerance_sec: float = 0.05,
    ros_distro: str = "humble",
) -> tuple[list[TrajectorySample], str | None]:
    for base_frame in base_candidates:
        direct = extract_tf_pair_trajectory(db3_path, "map", base_frame, ros_distro=ros_distro)
        if direct:
            return direct, f"direct: map->{base_frame}"

    map_odom = extract_tf_pair_trajectory(db3_path, "map", "odom", ros_distro=ros_distro)
    if not map_odom:
        return [], None

    for base_frame in base_candidates:
        odom_base = extract_tf_pair_trajectory(db3_path, "odom", base_frame, ros_distro=ros_distro)
        if not odom_base:
            continue

        records: list[TrajectorySample] = []
        right_index = 0
        for left in map_odom:
            while right_index + 1 < len(odom_base) and odom_base[right_index + 1].t_sec <= left.t_sec:
                right_index += 1

            candidates = [odom_base[right_index]]
            if right_index + 1 < len(odom_base):
                candidates.append(odom_base[right_index + 1])

            nearest = min(candidates, key=lambda sample: abs(sample.t_sec - left.t_sec))
            if abs(nearest.t_sec - left.t_sec) > tolerance_sec:
                continue

            x, y, yaw = _compose_2d(
                (left.x, left.y, left.yaw_rad or 0.0),
                (nearest.x, nearest.y, nearest.yaw_rad or 0.0),
            )
            records.append(
                TrajectorySample(
                    t_sec=left.t_sec,
                    x=x,
                    y=y,
                    yaw_rad=yaw,
                    source=f"map->{base_frame}",
                )
            )

        if records:
            return records, f"composed: map->odom + odom->{base_frame}"

    return [], None


def extract_map_points(
    db3_path: str | Path,
    topic: str = "/map",
    ros_distro: str = "humble",
) -> dict | None:
    from rosbags.highlevel import AnyReader

    bag_path = Path(db3_path)
    typestore = _get_typestore(ros_distro)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            return None

        latest_map: dict | None = None
        for connection, _, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            resolution = float(msg.info.resolution)
            origin_x = float(msg.info.origin.position.x)
            origin_y = float(msg.info.origin.position.y)
            width = int(msg.info.width)
            height = int(msg.info.height)
            occupied: list[list[float]] = []
            grid: list[list[int]] = [[-1 for _ in range(width)] for _ in range(height)]
            for index, value in enumerate(msg.data):
                row = index // width
                col = index % width
                grid[row][col] = int(value)
                if value < 50:
                    continue
                x = origin_x + (col + 0.5) * resolution
                y = origin_y + (row + 0.5) * resolution
                occupied.append([x, y])
            latest_map = {
                "resolution": resolution,
                "width": width,
                "height": height,
                "origin": [origin_x, origin_y],
                "occupied_points": occupied,
                "grid": grid,
            }
        return latest_map
    return None


def extract_scan_points(
    db3_path: str | Path,
    reference_samples: list[TrajectorySample],
    topic: str = "/scan",
    max_scans: int = 140,
    range_stride: int = 4,
    ros_distro: str = "humble",
) -> list[dict]:
    from rosbags.highlevel import AnyReader

    bag_path = Path(db3_path)
    typestore = _get_typestore(ros_distro)
    scans: list[dict] = []

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            return []

        messages = list(reader.messages(connections=connections))
        if not messages:
            return []

        step = max(1, len(messages) // max_scans)
        selected = messages[::step]

        for connection, timestamp, rawdata in selected:
            msg = reader.deserialize(rawdata, connection.msgtype)
            t_sec = timestamp / 1e9
            pose = _interpolate_pose(reference_samples, t_sec)
            if pose is None:
                continue
            px, py, yaw = pose
            angle = float(msg.angle_min)
            points: list[list[float]] = []
            for index, value in enumerate(msg.ranges):
                if index % range_stride != 0:
                    angle += float(msg.angle_increment)
                    continue
                if not math.isfinite(value) or value <= 0.02 or value > float(msg.range_max):
                    angle += float(msg.angle_increment)
                    continue
                wx = px + float(value) * math.cos(yaw + angle)
                wy = py + float(value) * math.sin(yaw + angle)
                points.append([wx, wy])
                angle += float(msg.angle_increment)
            scans.append(
                {
                    "t_sec": t_sec,
                    "points": points,
                }
            )

    return scans
