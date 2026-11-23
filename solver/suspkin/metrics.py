"""Turning solved poses into the numbers a suspension engineer cares about.

All angles are in degrees and all lengths in mm. Sign conventions are written
for the LEFT wheel in SAE/ISO axes (+X forward, +Y left/outboard, +Z up):

========================  =====================================================
camber                    negative = top of tyre leans inboard (toward the car)
toe                       positive = toe-in (front of tyre points inboard)
caster                    positive = upper ball joint behind the lower one
kingpin inclination (KPI) positive = upper ball joint inboard of the lower one
scrub radius              positive = steering axis meets the ground INBOARD of
                          the contact patch
mechanical trail          positive = steering axis meets the ground AHEAD of
                          the contact patch
motion ratio              spring compression / wheel travel (dimensionless)
========================  =====================================================
"""

from __future__ import annotations

import numpy as np

from .kinematics import Pose


# --- wheel orientation -----------------------------------------------------
def camber_deg(pose: Pose) -> float:
    return float(-np.degrees(np.arcsin(pose.wheel_axis[2])))


def toe_deg(pose: Pose) -> float:
    a = pose.wheel_axis
    return float(np.degrees(np.arctan2(a[0], a[1])))


# --- steering axis -----------------------------------------------------------
def _steer_axis(pose: Pose) -> np.ndarray:
    """Vector from the lower to the upper ball joint."""
    return pose.points["UBJ"] - pose.points["LBJ"]


def caster_deg(pose: Pose) -> float:
    d = _steer_axis(pose)
    return float(np.degrees(np.arctan2(-d[0], d[2])))


def kpi_deg(pose: Pose) -> float:
    d = _steer_axis(pose)
    return float(np.degrees(np.arctan2(-d[1], d[2])))


def steer_axis_ground_point(pose: Pose) -> np.ndarray:
    """Where the steering axis pierces the ground plane under the tyre.

    The contact patch moves with the upright, so "ground" is taken as the
    horizontal plane through the contact patch at this pose.
    """
    lbj, d = pose.points["LBJ"], _steer_axis(pose)
    s = (pose.points["CP"][2] - lbj[2]) / d[2]
    return lbj + s * d


def scrub_radius_mm(pose: Pose) -> float:
    return float(pose.points["CP"][1] - steer_axis_ground_point(pose)[1])


def mechanical_trail_mm(pose: Pose) -> float:
    return float(steer_axis_ground_point(pose)[0] - pose.points["CP"][0])


# --- roll centre (front view) ------------------------------------------------
def _intersect_2d(p1, d1, p2, d2):
    """Intersection of lines p1 + a*d1 and p2 + b*d2 in 2D (None if parallel)."""
    m = np.array([d1, -d2]).T
    if abs(np.linalg.det(m)) < 1e-12:
        return None
    a, _ = np.linalg.solve(m, p2 - p1)
    return p1 + a * d1


def _arm_line_front_view(pose: Pose, front: str, rear: str, ball_joint: str):
    """2D (y, z) line for one wishbone as seen from the front.

    In 3D the wishbone pivots about the line front->rear pivot. We take the
    point on that pivot axis level (in x) with the ball joint, and draw the
    line from there to the ball joint.
    """
    f, r, bj = pose.points[front], pose.points[rear], pose.points[ball_joint]
    axis = r - f
    s = (bj[0] - f[0]) / axis[0] if abs(axis[0]) > 1e-9 else 0.5
    pivot = f + s * axis
    return pivot[1:], bj[1:] - pivot[1:]


def instant_center_front_view(pose: Pose):
    """Front-view instant centre (y, z) of the upright, or None if arms are parallel."""
    p_u, d_u = _arm_line_front_view(pose, "UAF", "UAA", "UBJ")
    p_l, d_l = _arm_line_front_view(pose, "LAF", "LAA", "LBJ")
    return _intersect_2d(p_u, d_u, p_l, d_l)


def roll_center_height_mm(pose: Pose) -> float:
    """Roll centre height for symmetric heave (both wheels moving together).

    Draw a line from the contact patch through the front-view instant centre;
    by symmetry the roll centre is where it crosses the car centreline (y = 0).
    Height is measured from the contact-patch ground plane.
    """
    cp = pose.points["CP"][1:]
    ic = instant_center_front_view(pose)
    if ic is None:  # parallel equal arms: IC at infinity, line is parallel to arms
        p_l, d_l = _arm_line_front_view(pose, "LAF", "LAA", "LBJ")
        direction = d_l
    else:
        direction = ic - cp
    if abs(direction[0]) < 1e-12:
        return float("nan")
    s = (0.0 - cp[0]) / direction[0]  # how far along the line to reach y = 0
    rc_z = cp[1] + s * direction[1]
    return float(rc_z - cp[1])  # height above the contact-patch ground plane


# --- spring / damper -------------------------------------------------------
def spring_length_mm(pose: Pose) -> float:
    return float(np.linalg.norm(pose.points["SHU"] - pose.points["SHC"]))


# --- everything at once ----------------------------------------------------
METRIC_COLUMNS = (
    "travel_mm", "camber_deg", "toe_deg", "caster_deg", "kpi_deg",
    "scrub_radius_mm", "mech_trail_mm", "roll_center_height_mm",
    "spring_length_mm", "motion_ratio",
)


def compute_metrics(poses: list[Pose]) -> dict[str, np.ndarray]:
    """Evaluate every metric over a sweep. Returns column name -> array."""
    out = {
        "travel_mm": np.array([p.travel for p in poses]),
        "camber_deg": np.array([camber_deg(p) for p in poses]),
        "toe_deg": np.array([toe_deg(p) for p in poses]),
        "caster_deg": np.array([caster_deg(p) for p in poses]),
        "kpi_deg": np.array([kpi_deg(p) for p in poses]),
        "scrub_radius_mm": np.array([scrub_radius_mm(p) for p in poses]),
        "mech_trail_mm": np.array([mechanical_trail_mm(p) for p in poses]),
        "roll_center_height_mm": np.array([roll_center_height_mm(p) for p in poses]),
        "spring_length_mm": np.array([spring_length_mm(p) for p in poses]),
    }
    # Motion ratio = -d(spring length)/d(wheel travel): positive when the
    # spring compresses as the wheel goes into bump. Needs >= 2 poses.
    if len(poses) >= 2:
        out["motion_ratio"] = -np.gradient(out["spring_length_mm"], out["travel_mm"])
    else:
        out["motion_ratio"] = np.full(len(poses), np.nan)
    return out


def summary(metrics: dict[str, np.ndarray]) -> dict[str, float]:
    """A few headline numbers: values at static ride height and the gains."""
    t = metrics["travel_mm"]
    i0 = int(np.argmin(np.abs(t)))
    slope = lambda key: float(np.gradient(metrics[key], t)[i0])  # noqa: E731
    return {
        "static_camber_deg": float(metrics["camber_deg"][i0]),
        "static_toe_deg": float(metrics["toe_deg"][i0]),
        "camber_gain_deg_per_10mm": 10 * slope("camber_deg"),
        "bump_steer_deg_per_10mm": 10 * slope("toe_deg"),
        "caster_deg": float(metrics["caster_deg"][i0]),
        "kpi_deg": float(metrics["kpi_deg"][i0]),
        "scrub_radius_mm": float(metrics["scrub_radius_mm"][i0]),
        "mech_trail_mm": float(metrics["mech_trail_mm"][i0]),
        "roll_center_height_mm": float(metrics["roll_center_height_mm"][i0]),
        "motion_ratio": float(metrics["motion_ratio"][i0]),
        "toe_range_deg": float(np.ptp(metrics["toe_deg"])),
    }
