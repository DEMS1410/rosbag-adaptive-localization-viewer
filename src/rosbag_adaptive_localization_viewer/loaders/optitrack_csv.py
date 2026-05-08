from __future__ import annotations

import csv
from pathlib import Path

import pandas as pd


def _load_raw_rows(csv_path: Path) -> list[list[str]]:
    with csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.reader(handle))


def _find_rigid_body_columns(rows: list[list[str]], rigid_body_name: str) -> dict[str, int]:
    type_row = rows[2]
    name_row = rows[3]
    group_row = rows[5]
    axis_row = rows[6]

    mapping: dict[str, int] = {}
    for idx in range(len(name_row)):
        if idx >= len(type_row) or idx >= len(group_row) or idx >= len(axis_row):
            continue
        if type_row[idx] != "Rigid Body":
            continue
        if name_row[idx] != rigid_body_name:
            continue

        group = group_row[idx]
        axis = axis_row[idx]
        if group == "Rotation" and axis in {"X", "Y", "Z", "W"}:
            mapping[f"q{axis.lower()}"] = idx
        if group == "Position" and axis in {"X", "Y", "Z"}:
            mapping[f"{axis.lower()}_m"] = idx

    if 1 < len(axis_row) and axis_row[1] == "Time (Seconds)":
        mapping["timestamp_sec"] = 1

    required = {"timestamp_sec", "qx", "qy", "qz", "qw", "x_m", "y_m", "z_m"}
    missing = sorted(required - set(mapping))
    if missing:
        raise ValueError(
            f"Could not resolve OptiTrack columns for rigid body '{rigid_body_name}'. Missing: {missing}"
        )

    return mapping


def load_optitrack_rigid_body(csv_path: str | Path, rigid_body_name: str) -> pd.DataFrame:
    path = Path(csv_path)
    rows = _load_raw_rows(path)
    if len(rows) < 8:
        raise ValueError("OptiTrack CSV does not contain the expected multi-row header.")

    mapping = _find_rigid_body_columns(rows, rigid_body_name)
    data_rows = rows[7:]

    records: list[dict[str, float]] = []
    for row in data_rows:
        if not row:
            continue
        if len(row) <= mapping["timestamp_sec"]:
            continue
        if not row[mapping["timestamp_sec"]].strip():
            continue

        try:
            records.append(
                {
                    "timestamp_sec": float(row[mapping["timestamp_sec"]]),
                    "qx": float(row[mapping["qx"]]),
                    "qy": float(row[mapping["qy"]]),
                    "qz": float(row[mapping["qz"]]),
                    "qw": float(row[mapping["qw"]]),
                    "x_m": float(row[mapping["x_m"]]),
                    "y_m": float(row[mapping["y_m"]]),
                    "z_m": float(row[mapping["z_m"]]),
                }
            )
        except (IndexError, ValueError):
            continue

    if not records:
        raise ValueError(f"No OptiTrack trajectory samples were parsed for rigid body '{rigid_body_name}'.")

    return pd.DataFrame.from_records(records)
