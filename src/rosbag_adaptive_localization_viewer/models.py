from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


@dataclass(slots=True)
class TrajectorySample:
    t_sec: float
    x: float
    y: float
    z: float | None = None
    yaw_rad: float | None = None
    error_m: float | None = None
    source: str = "unknown"


@dataclass(slots=True)
class TrajectorySeries:
    id: str
    label: str
    source_type: Literal["estimate", "ground_truth", "comparison"]
    samples: list[TrajectorySample]
