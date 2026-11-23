"""Packaging check: do the upright's joints fit inside the wheel rim?

The ball joints and tie-rod end usually sit *inside* the rim barrel, so moving
them is limited by the rim. We check each point in the wheel's own frame:

* ``axial``  = distance along the spin axis from the wheel centre plane
               (negative = inboard, toward the car);
* ``radial`` = distance from the spin axis.

Rules (all set in the hardpoints file under ``"packaging"``):

1. Every checked point must be inboard of the hub mounting face:
   ``axial <= hub_face_axial_mm``.
2. If a point is within the rim's width (``axial >= -rim_half_width_mm``), it
   must clear the barrel: ``radial <= rim_inner_radius_mm - radial_clearance_mm``.
   Points further inboard than the rim edge have no radial limit.

Because the upright and the wheel move together as one rigid body, a point's
position relative to the rim never changes with wheel travel or steering, so
checking the design position is enough.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .hardpoints import Hardpoints

# Used when a hardpoints file has no "packaging" block: a 10 inch rim.
DEFAULT_PACKAGING = {
    "rim_inner_radius_mm": 117.0,  # inside of the 10" barrel (approximate)
    "rim_half_width_mm": 89.0,     # 7" wide rim, centred on the wheel centre plane
    "radial_clearance_mm": 8.0,    # room for the joint housing and rim flex
    "hub_face_axial_mm": -20.0,    # joints must sit at least 20 mm inboard
    "checked_points": ["UBJ", "LBJ", "RTO", "SHU"],
}


@dataclass
class PointClearance:
    name: str
    axial_mm: float
    radial_mm: float
    radial_margin_mm: float  # > 0 means clear of the barrel (inf if outside the rim width)
    axial_margin_mm: float   # > 0 means inboard of the hub face

    @property
    def ok(self) -> bool:
        return self.radial_margin_mm >= 0 and self.axial_margin_mm >= 0


def packaging_config(hp: Hardpoints) -> dict:
    cfg = dict(DEFAULT_PACKAGING)
    cfg.update({k: v for k, v in hp.extra.get("packaging", {}).items() if not k.startswith("_")})
    return cfg


def check_packaging(hp: Hardpoints) -> list[PointClearance]:
    """Clearance of each checked point to the rim envelope (design position)."""
    cfg = packaging_config(hp)
    wc, axis = hp.upright["WC"], hp.wheel_axis
    r_max = cfg["rim_inner_radius_mm"] - cfg["radial_clearance_mm"]
    results = []
    for name in cfg["checked_points"]:
        if name not in hp.upright:
            continue  # e.g. SHU on a pushrod/rocker design: nothing to check
        v = hp.upright[name] - wc
        axial = float(v @ axis)
        radial = float(np.linalg.norm(v - axial * axis))
        inside_rim = axial >= -cfg["rim_half_width_mm"]
        results.append(PointClearance(
            name=name,
            axial_mm=axial,
            radial_mm=radial,
            radial_margin_mm=(r_max - radial) if inside_rim else float("inf"),
            axial_margin_mm=cfg["hub_face_axial_mm"] - axial,
        ))
    return results


def packaging_violation_mm(hp: Hardpoints) -> float:
    """Total amount (mm) by which points break the envelope. 0 = all clear."""
    total = 0.0
    for c in check_packaging(hp):
        total += max(0.0, -c.radial_margin_mm) + max(0.0, -c.axial_margin_mm)
    return total


def packaging_margins(hp: Hardpoints, cap_mm: float = 1000.0) -> np.ndarray:
    """Every clearance margin as one array (all must be >= 0 to fit).

    Used as an inequality constraint by the optimiser. "Unlimited" margins
    (points outside the rim width) are capped so the numbers stay finite.
    """
    margins = []
    for c in check_packaging(hp):
        margins += [min(c.radial_margin_mm, cap_mm), c.axial_margin_mm]
    return np.array(margins)
