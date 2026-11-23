"""Side-view geometry: the side-view instant centre, anti-dive and anti-squat.

The idea
--------
Look at the car from the side. As the wheel moves up and down, the upright
(and everything bolted to it) swings about a point called the **side-view
instant centre** (SVIC). Under braking, the brake torque and the tyre's
braking force are fed into the chassis through the wishbones. If the SVIC is
placed well, part of the weight transfer goes through the links instead of the
springs, so the nose dives less. That fraction is **anti-dive**.

How we find it
--------------
The solver already gives every point at every wheel travel. Two points on the
upright (the wheel centre WC and the contact patch CP) move in side view with
velocities ``(dx/dz, 1)`` per mm of travel. A rigid body turning about a point
moves every point at right angles to the line joining it to that point, so
the SVIC is where the two lines through WC and CP, each at right angles to
that point's velocity, cross. This uses the full 3D solution, not a hand
construction from projected arm lines, so it stays right even with angled
pivot axes, caster and steering effects.

Anti percentages (Milliken, *Race Car Vehicle Dynamics*, ch. 17)
-----------------------------------------------------------------
With ``tan(theta)`` the side-view slope of the line from a reference point to
the SVIC (positive when the SVIC is on the side that helps; behind the wheel
for a front axle, ahead of it for a rear axle), wheelbase ``L`` and CG height
``h``::

    front anti-dive  % = tan(theta) * (L / h) * front_brake_bias       * 100
    rear  anti-lift  % = tan(theta) * (L / h) * (1 - front_brake_bias) * 100
    rear  anti-squat % = tan(theta) * (L / h)                          * 100
    front anti-lift  % = tan(theta) * (L / h)                          * 100   (driven front axle)

The reference point is the **contact patch** for outboard brakes (the brake
torque goes into the upright) and the **wheel centre** for inboard brakes and
for drive torque from a chassis-mounted differential (the torque goes
straight into the chassis, only the wheel-centre force goes through the links).

100% means the links carry all the load transfer and the car doesn't pitch at
all. Most cars run far less (front anti-dive of 10-40% is common), because
too much makes the steering and brakes feel wooden and can cause hop.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Used when a hardpoints file has no "vehicle" block. A made-up but plausible
# FSAE-sized car.
DEFAULT_VEHICLE = {
    "axle": "front",          # which axle this corner is on: "front" or "rear"
    "wheelbase_mm": 1550.0,
    "cg_height_mm": 290.0,
    "front_brake_bias": 0.65,  # fraction of brake force on the front axle
    "brakes": "outboard",      # "outboard" (in the wheel) or "inboard" (on the chassis)
    "driven": False,           # does this axle put down drive torque?
}


@dataclass
class VehicleParams:
    """Whole-car numbers the side-view and steering metrics need."""

    axle: str = "front"
    wheelbase_mm: float = 1550.0
    cg_height_mm: float = 290.0
    front_brake_bias: float = 0.65
    brakes: str = "outboard"
    driven: bool = False

    @classmethod
    def from_dict(cls, raw: dict | None) -> "VehicleParams":
        cfg = dict(DEFAULT_VEHICLE)
        cfg.update({k: v for k, v in (raw or {}).items() if not k.startswith("_")})
        v = cls(
            axle=str(cfg["axle"]),
            wheelbase_mm=float(cfg["wheelbase_mm"]),
            cg_height_mm=float(cfg["cg_height_mm"]),
            front_brake_bias=float(cfg["front_brake_bias"]),
            brakes=str(cfg["brakes"]),
            driven=bool(cfg["driven"]),
        )
        if v.axle not in ("front", "rear"):
            raise ValueError(f"vehicle axle must be 'front' or 'rear', not {v.axle!r}")
        if v.brakes not in ("outboard", "inboard"):
            raise ValueError(f"vehicle brakes must be 'outboard' or 'inboard', not {v.brakes!r}")
        return v

    @classmethod
    def from_hardpoints(cls, hp) -> "VehicleParams":
        """Read the optional ``"vehicle"`` block of a hardpoints file."""
        return cls.from_dict(hp.extra.get("vehicle"))


def side_view_ic(wc, cp, v_wc, v_cp):
    """Side-view instant centre (x, z) from two points and their velocities.

    ``wc``/``cp`` are (x, z) positions and ``v_wc``/``v_cp`` their (dx, dz)
    velocities. Returns None when the velocities are parallel (the upright is
    translating, so the SVIC is infinitely far away).
    """
    # Line through each point, at right angles to its velocity.
    n1 = np.array([-v_wc[1], v_wc[0]])
    n2 = np.array([-v_cp[1], v_cp[0]])
    m = np.array([n1, -n2]).T
    if abs(np.linalg.det(m)) < 1e-12:
        return None
    a, _ = np.linalg.solve(m, np.asarray(cp) - np.asarray(wc))
    return np.asarray(wc) + a * n1


def anti_tan(dx_dz: float, axle: str) -> float:
    """``tan(theta)`` for the anti formulas, from a point's side-view path.

    The point moves at right angles to the line joining it to the SVIC, so the
    slope of that line equals ``dx/dz`` of the point's own path. For a front
    corner, moving FORWARD in bump means the SVIC is behind and above (good,
    positive). For a rear corner it's moving REARWARD in bump.
    """
    return dx_dz if axle == "front" else -dx_dz


def anti_percentages(dx_dz_cp: np.ndarray, dx_dz_wc: np.ndarray,
                     vehicle: VehicleParams) -> dict[str, np.ndarray]:
    """Anti-dive / anti-lift / anti-squat (%) from the side-view paths.

    Returns ``anti_brake_pct`` (anti-dive at the front, anti-lift at the rear)
    and ``anti_drive_pct`` (anti-squat at the rear, anti-lift at the front;
    NaN if this axle isn't driven).
    """
    k = vehicle.wheelbase_mm / vehicle.cg_height_mm * 100.0
    brake_ref = dx_dz_cp if vehicle.brakes == "outboard" else dx_dz_wc
    brake_share = vehicle.front_brake_bias if vehicle.axle == "front" else 1 - vehicle.front_brake_bias
    brake = anti_tan(brake_ref, vehicle.axle) * k * brake_share
    if vehicle.driven:
        # Drive torque comes from a chassis-mounted diff, so it is reacted at
        # the wheel centre. (A forward drive force at the wheel centre pushes
        # the front wheel up into the body when the wheel moves forward in
        # bump, which holds the nose down: the same sign as anti-dive.)
        drive = anti_tan(dx_dz_wc, vehicle.axle) * k
    else:
        drive = np.full_like(np.asarray(brake, dtype=float), np.nan)
    return {"anti_brake_pct": np.asarray(brake, dtype=float),
            "anti_drive_pct": np.asarray(drive, dtype=float)}
