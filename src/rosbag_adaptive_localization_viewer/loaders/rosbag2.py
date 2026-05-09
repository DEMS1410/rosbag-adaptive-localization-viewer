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


def extract_trajectory_from_topic(db3_path: str | Path, topic: str) -> list[TrajectorySample]:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    bag_path = Path(db3_path)
    samples: list[TrajectorySample] = []
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            raise ValueError(f"Topic '{topic}' not found in bag.")

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            if connection.msgtype == "nav_msgs/msg/Odometry":
                pose = msg.pose.pose
                position = pose.position
                orientation = pose.orientation
            elif connection.msgtype == "geometry_msgs/msg/PoseStamped":
                pose = msg.pose
                position = pose.position
                orientation = pose.orientation
            else:
                raise ValueError(
                    f"Topic '{topic}' uses unsupported message type '{connection.msgtype}' for trajectory extraction."
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
                    cov_xx=float(msg.pose.covariance[0]) if hasattr(msg, "pose") else None,
                    cov_xy=float(msg.pose.covariance[1]) if hasattr(msg, "pose") else None,
                    cov_yy=float(msg.pose.covariance[7]) if hasattr(msg, "pose") else None,
                    yaw_var=float(msg.pose.covariance[35]) if hasattr(msg, "pose") else None,
                    source=topic,
                )
            )

    return samples


def extract_map_points(db3_path: str | Path, topic: str = "/map") -> dict | None:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    bag_path = Path(db3_path)
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            return None

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
            return {
                "resolution": resolution,
                "width": width,
                "height": height,
                "origin": [origin_x, origin_y],
                "occupied_points": occupied,
                "grid": grid,
            }
    return None


def extract_scan_points(
    db3_path: str | Path,
    reference_samples: list[TrajectorySample],
    topic: str = "/scan",
    max_scans: int = 140,
    range_stride: int = 4,
) -> list[dict]:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    bag_path = Path(db3_path)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
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
