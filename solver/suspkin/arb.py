"""Anti-roll bar (ARB): a torsion bar that only resists roll.

Layout (a "U-bar", the usual FSAE layout)::

    bar arm  ARA ●────────● ARP ═══════ torsion bar across the car ═══════ (mirror)
                 │            (the bar spins in bearings about the Y axis)
     drop link   │
                 │
                 ● ARL  on the rocker (or a wishbone / the upright)

Points (all in the hardpoints file's ``"arb"`` block, design position):

====  ==================================================================
ARL   drop link lower end, on whatever ``link_mount`` says (default the rocker)
ARA   drop link upper end = the end of the bar's arm
ARP   where the arm joins the bar (a point on the bar's axis)
====  ==================================================================

The bar runs straight across the car, so its axis is the Y axis. One bar
serves both sides, so it twists by the DIFFERENCE between the left and right
arm angles:

* In **heave** (both wheels up together) both arms turn the same way by the
  same amount: no twist, the bar does nothing.
* In **roll** one arm goes up and the other down: the bar twists and pushes
  back. That's the whole point of it: extra roll stiffness without making the
  ride (heave) any stiffer.

How it is solved
----------------
Exactly like the rocker (``rocker.py``): ARL moves with its mount, the arm can
only spin about the bar axis, and the drop link has a fixed length. So ARA
sits where a circle (the arm end) meets a sphere (the drop link), which is the
same closed-form ``A cos + B sin = C`` solve. We reuse that code by describing
the bar arm as a "rocker" whose pivot axis is +Y.

The mirror trick and the twist
------------------------------
We only solve the left corner. Mirroring (y -> -y) a rotation about the Y axis
gives a rotation by the SAME angle about the same axis, so the right arm's
angle at travel ``+s`` is just the left arm's angle at ``+s``. The bar twist in
roll is therefore ``angle(-s) - angle(+s)`` from two left-corner solves.

The bar's rate is given as a torsional rate (N m per degree of twist), as
bar makers and most calculators quote it. Its strain energy is
``U = rate * twist^2 / 2``, which ``roll.py`` adds to the springs' energy.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .rocker import MOUNTS as ARM_MOUNT_NAMES
from .rocker import RockerGeometry, rotate_about_axis, solve_rocker

ARB_POINTS = ("ARL", "ARA", "ARP")
BAR_AXIS = np.array([0.0, 1.0, 0.0])  # the bar runs straight across the car
# The drop link can hang off the rocker, a wishbone or the upright.
LINK_MOUNTS = ("rocker",) + ARM_MOUNT_NAMES


@dataclass
class ARBGeometry:
    """Design-position geometry of one side of the anti-roll bar."""

    link_mount: str                  # one of LINK_MOUNTS
    rate_nm_per_deg: float           # torsional rate of the whole bar
    points: dict[str, np.ndarray]    # ARL, ARA, ARP (design position, mm)
    notes: dict = field(default_factory=dict)  # "_note" text etc. kept for saving

    def __post_init__(self):
        if self.link_mount not in LINK_MOUNTS:
            raise ValueError(f"arb link_mount must be one of {LINK_MOUNTS}, not {self.link_mount!r}")
        missing = [p for p in ARB_POINTS if p not in self.points]
        if missing:
            raise ValueError(f"arb block is missing points {missing}")
        if self.rate_nm_per_deg < 0:
            raise ValueError("arb rate_nm_per_deg can't be negative")

    def copy(self) -> "ARBGeometry":
        return ARBGeometry(self.link_mount, self.rate_nm_per_deg,
                           {k: v.copy() for k, v in self.points.items()}, dict(self.notes))

    @property
    def drop_link_length(self) -> float:
        return float(np.linalg.norm(self.points["ARA"] - self.points["ARL"]))

    @property
    def arm_length(self) -> float:
        """Lever arm: distance from the bar axis to the arm end, in the arm's plane."""
        v = self.points["ARA"] - self.points["ARP"]
        return float(np.linalg.norm(v - (v @ BAR_AXIS) * BAR_AXIS))

    def as_rocker(self) -> RockerGeometry:
        """The bar arm described as a rocker, so ``rocker.solve_rocker`` can solve it.

        Rocker names -> bar names: PRO (moving end) = ARL, PRI (end on the
        rotating part) = ARA, RKP (pivot) = ARP. SHR isn't needed, so it's just
        a copy of ARA.
        """
        p = self.points
        return RockerGeometry(
            mount="upright",  # unused here: we move ARL ourselves
            axis=BAR_AXIS,
            points={"PRO": p["ARL"], "PRI": p["ARA"], "RKP": p["ARP"], "SHR": p["ARA"]},
        )


def parse_arb(block: dict, source: str = "JSON") -> ARBGeometry:
    """Read the ``"arb"`` block of a hardpoints file.

    Format::

        "arb": {
          "link_mount": "rocker",       # or "lower_arm", "upper_arm", "upright"
          "rate_nm_per_deg": 30.0,      # torsional rate of the bar
          "ARL": [...], "ARA": [...], "ARP": [...]
        }
    """
    missing = [n for n in ARB_POINTS + ("rate_nm_per_deg",) if n not in block]
    if missing:
        raise ValueError(f"{source}: 'arb' is missing {missing}")
    return ARBGeometry(
        link_mount=block.get("link_mount", "rocker"),
        rate_nm_per_deg=float(block["rate_nm_per_deg"]),
        points={n: np.asarray(block[n], dtype=float) for n in ARB_POINTS},
        notes={k: v for k, v in block.items() if k.startswith("_")},
    )


def arb_to_dict(arb: ARBGeometry) -> dict:
    """The ``"arb"`` block for saving (inverse of ``parse_arb``)."""
    return {
        **arb.notes,
        "link_mount": arb.link_mount,
        "rate_nm_per_deg": arb.rate_nm_per_deg,
        **{k: [round(float(c), 4) for c in v] for k, v in arb.points.items()},
    }


def solve_arb(arb: ARBGeometry, arl: np.ndarray, guess: float = 0.0) -> tuple[float, dict[str, np.ndarray]]:
    """Bar arm angle (rad, about +Y, 0 = design) and the moved ARL, ARA, ARP."""
    geom = arb.as_rocker()
    angle = solve_rocker(geom, arl, guess)
    # Turn the arm end about the bar axis by that angle.
    ara = rotate_about_axis(arb.points["ARA"], arb.points["ARP"], BAR_AXIS, angle)
    return angle, {"ARL": np.asarray(arl, dtype=float), "ARA": ara, "ARP": arb.points["ARP"].copy()}


def bar_twist_deg(left_angle_rad: float, right_src_angle_rad: float) -> float:
    """Bar twist (deg) from the left arm angle and the right arm's source angle.

    ``right_src_angle_rad`` is the LEFT corner's arm angle at the right wheel's
    travel; see "The mirror trick" above for why no sign flip is needed.
    """
    return float(np.degrees(left_angle_rad - right_src_angle_rad))
