from __future__ import annotations

import argparse
import json
import os
import sys

from rosbag_adaptive_localization_viewer.builder import build_experiment_payload
from rosbag_adaptive_localization_viewer.config import load_session_config
from rosbag_adaptive_localization_viewer.loaders.optitrack_csv import load_optitrack_rigid_body
from rosbag_adaptive_localization_viewer.loaders.rosbag2 import inspect_bag_topics
from rosbag_adaptive_localization_viewer.server import serve


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
    csv_path = None
    if ground_truth.get("enabled"):
        csv_path = os.path.abspath(os.path.join(config_dir, ground_truth["csv_path"]))

    payload = build_experiment_payload(
        experiment_name=config["experiment_name"],
        bag_path=bag_path,
        primary_topic=estimate["primary_topic"],
        comparison_topics=estimate.get("comparison_topics", []),
        optitrack_csv_path=csv_path,
        rigid_body_name=ground_truth.get("rigid_body_name", "ROBOT"),
    )
    with open(output_path, "w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, indent=2))
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

    serve_parser = subparsers.add_parser("serve")
    serve_parser.add_argument("--host", default="127.0.0.1")
    serve_parser.add_argument("--port", type=int, default=8765)

    args = parser.parse_args(argv)
    if args.command == "inspect-bag":
        return inspect_bag(args.db3_path)
    if args.command == "inspect-optitrack":
        return inspect_optitrack(args.csv_path, args.rigid_body)
    if args.command == "build-experiment":
        return build_experiment(args.config_path, args.output_path)
    if args.command == "serve":
        serve(args.host, args.port)
        return 0

    parser.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
