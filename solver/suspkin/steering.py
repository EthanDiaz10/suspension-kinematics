"""Steering sweep and Ackermann.

When a car turns, the inside front wheel follows a tighter circle than the
outside one, so it needs to steer further. **Ackermann geometry** gets this
exactly right at low speed: lines along both front axles meet on the
rear-axle line. With track ``t`` and wheelbase ``L`` that means::

    cot(outer) - cot(inner) = t / L

**Ackermann percentage** compares the actual inner angle with that ideal, for
the same outer angle::

    Ackermann % = (inner - outer) / (inner_ideal - outer) * 100

* 100 %  = perfect Ackermann
* 0 %    = parallel steer (both wheels turn the same amount)
* < 0 %  = anti-Ackermann (the OUTER wheel turns more)

Race cars often run well under 100 % (sometimes anti-Ackermann), because at
speed the tyres run at slip angles and a heavily loaded outer tyre wants more
angle than low-speed geometry gives it. It's a tuning choice, not a pass/fail.

The mirror trick
----------------
We only model the LEFT corner. The right corner is its mirror image (y -> -y),
and moving the rack toward +Y for the left side is the same as moving it toward
-Y for a mirrored right side. So the right wheel at rack ``r`` is simply the
left wheel at rack ``-r``, mirrored. One steering sweep of the left corner from
``-r_max`` to ``+r_max`` therefore gives both wheels.

Signs: positive steer angle = wheel points LEFT (+Y), positive rack = rack
moves toward +Y, which steers the car left. In a left turn the inner wheel is
the left one.
"""

from __future__ import annotations

import numpy as np

from .kinematics import Pose
from .sideview import VehicleParams


def heading_deg(wheel_axis: np.ndarray) -> float:
    """Direction the wheel rolls, in the ground plane (deg, + = pointing left).

    The rolling direction is horizontal and at right angles to the spin axis.
    We take the one that points forward (+X), so it works for either side.
    """
    fwd = np.array([wheel_axis[1], -wheel_axis[0]])
    if fwd[0] < 0:
        fwd = -fwd
    return float(np.degrees(np.arctan2(fwd[1], fwd[0])))


def mirror_axis(wheel_axis: np.ndarray) -> np.ndarray:
    """The right wheel's spin axis, from the mirror image of the left one."""
    return np.array([wheel_axis[0], -wheel_axis[1], wheel_axis[2]])


def ideal_inner_deg(outer_deg, track_mm: float, wheelbase_mm: float):
    """Inner angle that perfect Ackermann gives for a given outer angle."""
    outer = np.radians(np.abs(outer_deg))
    with np.errstate(divide="ignore", invalid="ignore"):
        inner = np.arctan(1.0 / (1.0 / np.tan(outer) - track_mm / wheelbase_mm))
    return np.degrees(inner)


def ackermann_pct(inner_deg, outer_deg, track_mm: float, wheelbase_mm: float):
    """Ackermann percentage (see module docstring). NaN when not steering."""
    inner, outer = np.abs(inner_deg), np.abs(outer_deg)
    ideal = ideal_inner_deg(outer, track_mm, wheelbase_mm)
    with np.errstate(divide="ignore", invalid="ignore"):
        pct = (inner - outer) / (ideal - outer) * 100.0
    return np.where(outer > 1e-6, pct, np.nan)


STEER_COLUMNS = (
    "rack_mm", "steer_left_deg", "steer_right_deg", "inner_deg", "outer_deg",
    "ideal_inner_deg", "ackermann_pct", "turn_radius_m",
)


def steering_metrics(poses: list[Pose], vehicle: VehicleParams | None = None) -> dict[str, np.ndarray]:
    """Ackermann numbers from a steering sweep of the left corner.

    ``poses`` must be a sweep over rack positions symmetric about 0 (as made by
    :func:`suspkin.steer_sweep`), so the mirrored right wheel can be looked up.
    """
    vehicle = vehicle or VehicleParams()
    racks = np.array([p.rack for p in poses])
    by_rack = {round(r, 6): p for r, p in zip(racks, poses)}
    i0 = int(np.argmin(np.abs(racks)))
    straight = heading_deg(poses[i0].wheel_axis)  # heading with the rack centred (static toe)

    left, right = [], []
    for r, p in zip(racks, poses):
        mirror = by_rack.get(round(-r, 6))
        if mirror is None:
            raise ValueError(f"steering sweep needs rack {-r:+g} mm to mirror rack {r:+g} mm")
        left.append(heading_deg(p.wheel_axis) - straight)
        # Right wheel: mirror the left wheel at -r. Its straight-ahead heading is
        # the mirror of the left one's, i.e. -straight.
        right.append(heading_deg(mirror_axis(mirror.wheel_axis)) + straight)
    left, right = np.array(left), np.array(right)

    # Turning left (positive angles): inner = left. Turning right: inner = right.
    turning_left = left + right > 0
    inner = np.where(turning_left, left, right)
    outer = np.where(turning_left, right, left)

    # Track between the contact patches, and wheelbase, for the ideal curve.
    track = 2.0 * float(poses[i0].points["CP"][1])
    L = vehicle.wheelbase_mm
    with np.errstate(divide="ignore"):
        # Low-speed turning radius to the car's centreline at the rear axle,
        # from the mean front steer angle (the "bicycle model").
        mean = np.radians((np.abs(inner) + np.abs(outer)) / 2.0)
        radius_m = np.where(mean > 1e-6, L / np.tan(mean) / 1000.0, np.inf)

    return {
        "rack_mm": racks,
        "steer_left_deg": left,
        "steer_right_deg": right,
        "inner_deg": np.abs(inner),
        "outer_deg": np.abs(outer),
        "ideal_inner_deg": ideal_inner_deg(outer, track, L),
        "ackermann_pct": ackermann_pct(inner, outer, track, L),
        "turn_radius_m": radius_m,
    }


def steering_summary(steer: dict[str, np.ndarray]) -> dict[str, float]:
    """Headline steering numbers: lock angles and Ackermann at a few points."""
    r = steer["rack_mm"]
    i_max = int(np.argmax(r))  # full left lock
    out = {
        "max_rack_mm": float(r[i_max]),
        "inner_lock_deg": float(steer["inner_deg"][i_max]),
        "outer_lock_deg": float(steer["outer_deg"][i_max]),
        "ackermann_at_lock_pct": float(steer["ackermann_pct"][i_max]),
        "turn_radius_at_lock_m": float(steer["turn_radius_m"][i_max]),
    }
    # Steering "speed": how many degrees of (mean) wheel angle per mm of rack,
    # near straight ahead.
    i0 = int(np.argmin(np.abs(r)))
    mean = (steer["steer_left_deg"] + steer["steer_right_deg"]) / 2.0
    out["steer_deg_per_mm_rack"] = float(np.gradient(mean, r)[i0])
    # Ackermann half way to lock, where a lot of real cornering happens.
    half = int(np.argmin(np.abs(r - r[i_max] / 2.0)))
    out["ackermann_half_lock_pct"] = float(steer["ackermann_pct"][half])
    return out
