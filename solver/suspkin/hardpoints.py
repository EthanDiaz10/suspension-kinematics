"""Loading and saving suspension hardpoints.

A *hardpoint* is a 3D location (x, y, z) in millimetres where a suspension
part attaches. Hardpoints come in two groups:

* **chassis points** are bolted to the frame and never move during a sweep;
* **upright points** ride on the upright (the knuckle that carries the wheel)
  and move together as one rigid body.

An optional ``"rocker"`` block adds a pushrod and rocker (see ``rocker.py``).
With it, the damper runs from ``SHC`` on the chassis to ``SHR`` on the rocker,
and the upright no longer needs the direct-acting placeholder point ``SHU``.

Coordinate system (SAE/ISO vehicle axes): +X forward, +Y left, +Z up, in mm.
We model the front-LEFT corner, so "outboard" means +Y.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .rocker import ROCKER_POINTS, RockerGeometry

# The names every hardpoint file must provide.
CHASSIS_POINTS = ("UAF", "UAA", "LAF", "LAA", "RTI", "SHC")
UPRIGHT_POINTS = ("UBJ", "LBJ", "RTO", "WC", "CP")
# Only needed when there is no "rocker" block (spring/damper bolted straight
# to the upright, the Phase 1 placeholder).
DIRECT_ACTING_POINT = "SHU"


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
    rocker: RockerGeometry | None = None       # None = direct-acting damper

    def find_point(self, name: str) -> np.ndarray:
        """The (mutable) array for any named point, whichever group it's in.

        Handy for the optimiser, which nudges points by name.
        """
        for group in (self.chassis, self.upright,
                      self.rocker.points if self.rocker else {}):
            if name in group:
                return group[name]
        raise KeyError(f"no hardpoint called {name!r}")

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
            rocker=self.rocker.copy() if self.rocker else None,
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

    rocker = None
    upright_names = UPRIGHT_POINTS + (DIRECT_ACTING_POINT,)
    if "rocker" in raw:
        rocker = parse_rocker(raw["rocker"], source=str(path))
        upright_names = UPRIGHT_POINTS  # SHU not needed with a rocker

    upright = read_group("upright", upright_names)
    # Keep any other upright points the file defines (e.g. an old SHU).
    for k, v in raw["upright"].items():
        upright.setdefault(k, np.asarray(v, dtype=float))

    known = {"name", "chassis", "upright", "static_camber_deg", "static_toe_deg", "rocker"}
    return Hardpoints(
        name=raw.get("name", Path(path).stem),
        chassis=read_group("chassis", CHASSIS_POINTS),
        upright=upright,
        static_camber_deg=float(raw.get("static_camber_deg", 0.0)),
        static_toe_deg=float(raw.get("static_toe_deg", 0.0)),
        extra={k: v for k, v in raw.items() if k not in known},
        rocker=rocker,
    )


def parse_rocker(block: dict, source: str = "JSON") -> RockerGeometry:
    """Read the ``"rocker"`` block of a hardpoints file.

    Format::

        "rocker": {
          "pushrod_mount": "lower_arm",       # or "upper_arm" (pullrod), "upright"
          "axis": [1.0, 0.0, 0.0],            # rocker pivot axis direction
          "PRO": [...], "PRI": [...], "RKP": [...], "SHR": [...]
        }

    Keys starting with ``_`` are comments and are kept when saving.
    """
    missing = [n for n in ROCKER_POINTS + ("axis",) if n not in block]
    if missing:
        raise ValueError(f"{source}: 'rocker' is missing {missing}")
    return RockerGeometry(
        mount=block.get("pushrod_mount", "lower_arm"),
        axis=np.asarray(block["axis"], dtype=float),
        points={n: np.asarray(block[n], dtype=float) for n in ROCKER_POINTS},
        notes={k: v for k, v in block.items() if k.startswith("_")},
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
    if hp.rocker is not None:
        r = hp.rocker
        out["rocker"] = {
            **r.notes,
            "pushrod_mount": r.mount,
            "axis": [round(float(c), 6) for c in r.axis],
            **{k: [round(float(c), 4) for c in v] for k, v in r.points.items()},
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
