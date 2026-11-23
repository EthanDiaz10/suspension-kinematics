"""Writing sweep results to CSV (for spreadsheets/plots) and JSON (for Fusion).

The JSON file is the hand-off to the CAD side: ``cad/fusion`` reads it to draw
the suspension at any pose, so the CAD never needs to re-solve anything.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from .kinematics import Pose
from .metrics import METRIC_COLUMNS


def write_metrics_csv(metrics: dict[str, np.ndarray], path: str | Path) -> None:
    """Bump-sweep metrics as CSV, one row per wheel travel."""
    write_metrics_table(metrics, METRIC_COLUMNS, path)


def write_metrics_table(metrics: dict[str, np.ndarray], columns, path: str | Path) -> None:
    """Any metrics dict as CSV, with the given column order (e.g. steering)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(columns)
        for row in zip(*(metrics[c] for c in columns)):
            w.writerow([f"{v:.6f}" for v in row])


def write_poses_json(poses: list[Pose], path: str | Path, name: str = "",
                     rocker=None) -> None:
    """Every hardpoint at every pose, in mm, SAE/ISO axes.

    ``rocker`` (a ``RockerGeometry``, optional): its fixed pivot direction and
    the pushrod mount are stored once at the top, so the Fusion script can
    draw the rocker pivot and orient the rod ends.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "name": name,
        "units": "mm",
        "coordinate_system": "SAE/ISO: +X forward, +Y left, +Z up",
        **({"rocker_axis": [round(float(c), 8) for c in rocker.axis],
            "pushrod_mount": rocker.mount} if rocker is not None else {}),
        "poses": [
            {
                "travel_mm": p.travel,
                "rack_mm": p.rack,
                **({} if np.isnan(p.rocker_angle) else
                   {"rocker_angle_deg": round(float(np.degrees(p.rocker_angle)), 6)}),
                "wheel_axis": [round(float(c), 8) for c in p.wheel_axis],
                "points": {k: [round(float(c), 5) for c in v] for k, v in p.points.items()},
            }
            for p in poses
        ],
    }
    path.write_text(json.dumps(data, indent=1) + "\n")
