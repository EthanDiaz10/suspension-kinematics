"""Loading and saving suspension hardpoints.

A *hardpoint* is a 3D location (x, y, z) in millimetres where a suspension
part attaches. Hardpoints come in two groups:

* **chassis points** are bolted to the frame and never move during a sweep;
* **upright points** ride on the upright (the knuckle that carries the wheel)
  and move together as one rigid body.

Coordinate system (SAE/ISO vehicle axes): +X forward, +Y left, +Z up, in mm.
We model the front-LEFT corner, so "outboard" means +Y.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# The names every hardpoint file must provide.
CHASSIS_POINTS = ("UAF", "UAA", "LAF", "LAA", "RTI", "SHC")
UPRIGHT_POINTS = ("UBJ", "LBJ", "RTO", "SHU", "WC", "CP")


def wheel_axis_from_angles(camber_deg: float, toe_deg: float) -> np.ndarray:
    """Unit vector along the wheel spin axis, pointing outboard (+Y).

    Sign conventions (left wheel):
      * negative camber = top of the tyre leans inboard
        -> the outboard end of the axle tips UP (axis z > 0)
      * positive toe = toe-in = front of the tyre points inboard
        -> the outboard end of the axle swings FORWARD (axis x > 0)
    """
    c = np.radians(camber_deg)
    t = np.radians(toe_deg)
    return np.array([np.sin(t) * np.cos(c), np.cos(t) * np.cos(c), -np.sin(c)])


@dataclass
class Hardpoints:
    """All the geometry needed to describe one suspension corner."""

    name: str
    chassis: dict[str, np.ndarray]
    upright: dict[str, np.ndarray]
    static_camber_deg: float = 0.0
    static_toe_deg: float = 0.0
    extra: dict = field(default_factory=dict)  # descriptions, notes, etc.

    @property
    def wheel_axis(self) -> np.ndarray:
        """Spin axis at the design (static) position."""
        return wheel_axis_from_angles(self.static_camber_deg, self.static_toe_deg)

    def copy(self) -> "Hardpoints":
        """Deep copy, handy when an optimiser wants to nudge points."""
        return Hardpoints(
            name=self.name,
            chassis={k: v.copy() for k, v in self.chassis.items()},
            upright={k: v.copy() for k, v in self.upright.items()},
            static_camber_deg=self.static_camber_deg,
            static_toe_deg=self.static_toe_deg,
            extra=dict(self.extra),
        )


def load_hardpoints(path: str | Path) -> Hardpoints:
    """Read a hardpoints JSON file (see ``data/baseline_fsae_front.json``)."""
    raw = json.loads(Path(path).read_text())

    def read_group(group: str, names: tuple[str, ...]) -> dict[str, np.ndarray]:
        pts = raw[group]
        missing = [n for n in names if n not in pts]
        if missing:
            raise ValueError(f"{path}: '{group}' is missing points {missing}")
        return {n: np.asarray(pts[n], dtype=float) for n in names}

    known = {"name", "chassis", "upright", "static_camber_deg", "static_toe_deg"}
    return Hardpoints(
        name=raw.get("name", Path(path).stem),
        chassis=read_group("chassis", CHASSIS_POINTS),
        upright=read_group("upright", UPRIGHT_POINTS),
        static_camber_deg=float(raw.get("static_camber_deg", 0.0)),
        static_toe_deg=float(raw.get("static_toe_deg", 0.0)),
        extra={k: v for k, v in raw.items() if k not in known},
    )


def save_hardpoints(hp: Hardpoints, path: str | Path) -> None:
    """Write hardpoints back to JSON (same format as ``load_hardpoints``)."""
    out = {
        "name": hp.name,
        **hp.extra,
        "static_camber_deg": hp.static_camber_deg,
        "static_toe_deg": hp.static_toe_deg,
        "chassis": {k: [round(float(c), 4) for c in v] for k, v in hp.chassis.items()},
        "upright": {k: [round(float(c), 4) for c in v] for k, v in hp.upright.items()},
    }
    Path(path).write_text(dumps_compact(out) + "\n")


def dumps_compact(data) -> str:
    """JSON with 2-space indents, but short lists kept on one line.

    So a point reads as ``"UBJ": [-8.0, 555.0, 335.0]`` instead of five lines.
    """
    text = json.dumps(data, indent=2)
    return re.sub(
        r"\[\s*([^\[\]{}]*?)\s*\]",
        lambda m: "[" + ", ".join(x.strip() for x in m.group(1).split(",")) + "]",
        text,
    )
