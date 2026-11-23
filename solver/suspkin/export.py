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
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(METRIC_COLUMNS)
        for row in zip(*(metrics[c] for c in METRIC_COLUMNS)):
            w.writerow([f"{v:.6f}" for v in row])


def write_poses_json(poses: list[Pose], path: str | Path, name: str = "") -> None:
    """Every hardpoint at every pose, in mm, SAE/ISO axes."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "name": name,
        "units": "mm",
        "coordinate_system": "SAE/ISO: +X forward, +Y left, +Z up",
        "poses": [
            {
                "travel_mm": p.travel,
                "rack_mm": p.rack,
                "wheel_axis": [round(float(c), 8) for c in p.wheel_axis],
                "points": {k: [round(float(c), 5) for c in v] for k, v in p.points.items()},
            }
            for p in poses
        ],
    }
    path.write_text(json.dumps(data, indent=1) + "\n")
