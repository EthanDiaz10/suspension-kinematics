"""Pushrod and rocker stage: from wheel travel to damper length.

Most FSAE-style cars don't bolt the spring/damper straight to the upright.
Instead a **pushrod** runs from a bracket on the lower wishbone (or the
upright) up to a **rocker** (also called a bellcrank), which pivots on the
chassis and pushes the damper::

        SHC ===damper=== SHR
                           \\   rocker (rotates about a fixed axis through RKP)
                            RKP
                           /
                        PRI
                       /
                pushrod
                     /
                  PRO  (on the lower wishbone, near the ball joint)

Points (all in the hardpoints file's ``"rocker"`` block, design position):

====  ==================================================================
PRO   pushrod outer end, on the lower arm (or upper arm / upright)
PRI   pushrod inner end, on the rocker
RKP   rocker pivot (a point on the rocker's rotation axis, on the chassis)
SHR   damper end on the rocker
====  ==================================================================

plus ``SHC`` (damper chassis end) from the ``"chassis"`` group and the
rocker ``axis`` direction.

How it is solved
----------------
This runs **after** the upright solve, because nothing in the rocker pushes
back on the wishbone geometry (kinematically it's a passive chain):

1. Move PRO with whatever it's mounted on. For a wishbone, the arm is a rigid
   body spinning about its two chassis pivots, so we find the arm's rotation
   angle from where its ball joint went and rotate PRO by the same angle.
2. Find the rocker angle that keeps the pushrod at its design length. The
   rocker can only spin about its axis, so PRI moves on a circle; the pushrod
   says PRI must sit on a sphere around PRO. A circle meets a sphere at (up to)
   two points, which we can find exactly with a little trigonometry (below).
   We keep the one nearest the previous solution.
3. Rotate SHR by the same angle. The damper length is just |SHR - SHC|.

Motion ratio (= damper travel / wheel travel) then comes out of the sweep in
``metrics.py``, exactly as it did for the direct-acting placeholder.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

ROCKER_POINTS = ("PRO", "PRI", "RKP", "SHR")

# What the pushrod's outer end (PRO) can be bolted to, and which hardpoints
# define that part's pivot axis and the moving joint on it.
#   "lower_arm": classic pushrod on the lower wishbone
#   "upper_arm": a pullrod hangs from the upper wishbone
#   "upright":   pushrod straight onto the upright (moves with the wheel)
ARM_MOUNTS = {
    "lower_arm": ("LAF", "LAA", "LBJ"),
    "upper_arm": ("UAF", "UAA", "UBJ"),
}
MOUNTS = tuple(ARM_MOUNTS) + ("upright",)


@dataclass
class RockerGeometry:
    """Design-position geometry of the pushrod and rocker."""

    mount: str                      # one of MOUNTS
    axis: np.ndarray                # unit vector along the rocker's pivot axis
    points: dict[str, np.ndarray]   # PRO, PRI, RKP, SHR (design position, mm)
    notes: dict = field(default_factory=dict)  # "_about" text etc. kept for saving

    def __post_init__(self):
        if self.mount not in MOUNTS:
            raise ValueError(f"pushrod_mount must be one of {MOUNTS}, not {self.mount!r}")
        self.axis = np.asarray(self.axis, dtype=float)
        n = np.linalg.norm(self.axis)
        if n < 1e-9:
            raise ValueError("rocker axis must not be zero length")
        self.axis = self.axis / n
        missing = [p for p in ROCKER_POINTS if p not in self.points]
        if missing:
            raise ValueError(f"rocker block is missing points {missing}")

    def copy(self) -> "RockerGeometry":
        return RockerGeometry(self.mount, self.axis.copy(),
                              {k: v.copy() for k, v in self.points.items()}, dict(self.notes))

    @property
    def pushrod_length(self) -> float:
        return float(np.linalg.norm(self.points["PRI"] - self.points["PRO"]))


# ---------------------------------------------------------------------------
# Rotations about a fixed axis
# ---------------------------------------------------------------------------
def rotate_about_axis(p: np.ndarray, origin: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """Rotate point ``p`` by ``angle`` radians about the line (origin, axis).

    Right-hand rule: thumb along ``axis``, fingers curl in the +angle direction.
    """
    return Rotation.from_rotvec(axis * angle).apply(p - origin) + origin


def angle_about_axis(origin: np.ndarray, axis: np.ndarray,
                     p_from: np.ndarray, p_to: np.ndarray) -> float:
    """Signed angle (rad) that takes ``p_from`` to ``p_to`` about the axis.

    Both points are first projected onto the plane perpendicular to the axis,
    so only the spin about the axis counts.
    """
    u = p_from - origin
    v = p_to - origin
    u = u - (u @ axis) * axis
    v = v - (v @ axis) * axis
    return float(np.arctan2(axis @ np.cross(u, v), u @ v))


def arm_angle(points: dict[str, np.ndarray], design: dict[str, np.ndarray], mount: str) -> float:
    """How far a wishbone has rotated about its chassis pivots (rad).

    ``design`` holds the design-position points and ``points`` the solved ones.
    The chassis pivots never move, so the arm's motion is a pure rotation and
    its ball joint tells us the angle.
    """
    front, rear, ball_joint = ARM_MOUNTS[mount]
    origin = design[front]
    axis = design[rear] - design[front]
    axis = axis / np.linalg.norm(axis)
    return angle_about_axis(origin, axis, design[ball_joint], points[ball_joint])


# ---------------------------------------------------------------------------
# The rocker solve: circle (PRI on the rocker) meets sphere (pushrod length)
# ---------------------------------------------------------------------------
def rocker_angles(geom: RockerGeometry, pro: np.ndarray) -> tuple[float, float]:
    """Both rocker angles (rad) that keep the pushrod at its design length.

    Write PRI's design position relative to the pivot as ``v`` and split it
    into a part along the axis (``v_par``, never changes) and a part
    perpendicular to it (``p``, spins round). After turning by theta::

        PRI(theta) = RKP + v_par + cos(theta) p + sin(theta) (axis x p)

    Setting ``|PRI(theta) - PRO|^2 = L^2`` and expanding gives::

        A cos(theta) + B sin(theta) = C

    with ``w = RKP + v_par - PRO``, ``A = 2 w.p``, ``B = 2 w.(axis x p)``
    and ``C = L^2 - |w|^2 - |p|^2``. Writing ``A cos + B sin`` as
    ``rho cos(theta - phi)`` gives the two answers
    ``theta = phi +/- acos(C / rho)``.
    """
    a = geom.axis
    rkp, pri = geom.points["RKP"], geom.points["PRI"]
    v = pri - rkp
    v_par = (v @ a) * a
    p = v - v_par
    q = np.cross(a, p)
    w = rkp + v_par - pro
    A, B = 2 * (w @ p), 2 * (w @ q)
    C = geom.pushrod_length ** 2 - w @ w - p @ p
    rho = float(np.hypot(A, B))
    if rho < 1e-12 or abs(C) > rho:
        raise RuntimeError(
            "The pushrod can't reach the rocker at this wheel travel "
            "(the rocker would have to stretch past its toggle point)."
        )
    phi = float(np.arctan2(B, A))
    half = float(np.arccos(np.clip(C / rho, -1.0, 1.0)))
    return phi + half, phi - half


def _wrap(angle: float) -> float:
    """Angle wrapped to [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def solve_rocker(geom: RockerGeometry, pro: np.ndarray, guess: float = 0.0) -> float:
    """Rocker angle (rad, 0 = design) for a given pushrod outer position.

    Of the two mathematical answers, the one nearest ``guess`` is the real one
    (a sweep passes the previous step's angle). The other is the rocker
    flipped over to the far side of the pushrod, which can't happen without
    passing through the toggle point.
    """
    t1, t2 = rocker_angles(geom, pro)
    return min((_wrap(t1), _wrap(t2)), key=lambda t: abs(_wrap(t - guess)))


def rocker_points(geom: RockerGeometry, angle: float) -> dict[str, np.ndarray]:
    """PRI, SHR and RKP after the rocker turns by ``angle`` radians."""
    rkp = geom.points["RKP"]
    return {
        "RKP": rkp.copy(),
        "PRI": rotate_about_axis(geom.points["PRI"], rkp, geom.axis, angle),
        "SHR": rotate_about_axis(geom.points["SHR"], rkp, geom.axis, angle),
    }


# ---------------------------------------------------------------------------
# Design checks at one pose
# ---------------------------------------------------------------------------
def _angle_to_plane_deg(direction: np.ndarray, plane_normal: np.ndarray) -> float:
    """Angle between a line and a plane (0 = lying in the plane)."""
    d = direction / np.linalg.norm(direction)
    return float(np.degrees(np.arcsin(abs(d @ plane_normal))))


def rocker_checks(geom: RockerGeometry, points: dict[str, np.ndarray]) -> dict[str, float]:
    """A few numbers worth checking on any pushrod/rocker layout.

    * ``pushrod_to_rocker_arm_deg``: angle between the pushrod and the rocker
      arm (RKP->PRI). Near 90 deg the pushrod has the best leverage; near
      0 or 180 deg it's close to the toggle point, where the rocker locks.
    * ``damper_to_rocker_arm_deg``: same for the damper and arm RKP->SHR.
    * ``pushrod_out_of_plane_deg`` / ``damper_out_of_plane_deg``: how far the
      pushrod and damper lean out of the rocker's plane of rotation. A few
      degrees is fine; much more bends the rocker and its pivot bolt.
    """
    a = geom.axis
    rkp, pri, shr = points["RKP"], points["PRI"], points["SHR"]
    pushrod = pri - points["PRO"]
    damper = points["SHC"] - shr

    def between(u, v):
        return float(np.degrees(np.arccos(np.clip(
            (u @ v) / (np.linalg.norm(u) * np.linalg.norm(v)), -1, 1))))

    return {
        "pushrod_to_rocker_arm_deg": between(pushrod, pri - rkp),
        "damper_to_rocker_arm_deg": between(damper, shr - rkp),
        "pushrod_out_of_plane_deg": _angle_to_plane_deg(pushrod, a),
        "damper_out_of_plane_deg": _angle_to_plane_deg(damper, a),
    }
