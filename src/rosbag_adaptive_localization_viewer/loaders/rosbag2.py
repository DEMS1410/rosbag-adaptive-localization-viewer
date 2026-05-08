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
                    source=topic,
                )
            )

    return samples
