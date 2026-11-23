"""The kinematic solver: where is the upright for a given wheel travel?

How it works
------------
The upright is a rigid body, so its position is fully described by 6 numbers:

* a **rotation vector** ``r`` (3 numbers). Its direction is the axis to rotate
  about and its length is the angle in radians. Unlike Euler angles it has no
  gimbal lock and no "which order do I rotate in?" confusion.
* a **translation** ``t`` (3 numbers, mm).

Every upright point is moved with  ``p_new = R(r) @ (p_design - WC) + WC + t``,
i.e. we rotate about the design wheel centre and then shift. Rotating about the
wheel centre (instead of the origin) keeps the numbers well scaled.

We need 6 equations to pin down 6 unknowns:

1-5. The five links are rigid, so their lengths must equal the design lengths:
     upper arm front & rear legs, lower arm front & rear legs, and the tie rod.
6.   The wheel centre height is prescribed: ``WC_z = WC_z_design + travel``.

``scipy.optimize.least_squares`` drives all six residuals to (numerically) zero.
For a sweep we start each step from the previous solution, so the solver only
ever has to take a small step and converges in a few iterations.

If the hardpoints have a pushrod and rocker, a second, much smaller stage runs
after the upright solve (see ``rocker.py``): it moves the pushrod's outer end
with the wishbone, then turns the rocker to keep the pushrod length fixed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .hardpoints import Hardpoints
from .rocker import ARM_MOUNTS, arm_angle, rotate_about_axis, rocker_points, solve_rocker

# (chassis end, upright end) for each rigid link.
LINKS = (
    ("UAF", "UBJ"),  # upper wishbone, front leg
    ("UAA", "UBJ"),  # upper wishbone, rear leg
    ("LAF", "LBJ"),  # lower wishbone, front leg
    ("LAA", "LBJ"),  # lower wishbone, rear leg
    ("RTI", "RTO"),  # tie rod
)


@dataclass
class Pose:
    """The solved suspension at one wheel-travel / steering position."""

    travel: float                 # wheel centre vertical travel, mm (+ = bump)
    rack: float                   # steering rack displacement, mm (+ = toward +Y)
    points: dict[str, np.ndarray] # every hardpoint (chassis + moved upright)
    wheel_axis: np.ndarray        # unit spin axis, pointing outboard
    state: np.ndarray             # the 6 solver unknowns [rx, ry, rz, tx, ty, tz]
    max_residual: float           # worst constraint error, mm (should be ~1e-9)
    rocker_angle: float = float("nan")  # rocker rotation from design, rad (nan = no rocker)


class DoubleWishbone:
    """Kinematic model of one double-wishbone corner."""

    def __init__(self, hp: Hardpoints):
        self.hp = hp
        self.center = hp.upright["WC"].copy()  # rotation centre for the upright
        # Design link lengths: these must stay constant at every pose.
        self.link_lengths = {
            link: float(np.linalg.norm(hp.upright[link[1]] - hp.chassis[link[0]]))
            for link in LINKS
        }

    # --- moving the upright -------------------------------------------------
    def _transform(self, state: np.ndarray, p: np.ndarray) -> np.ndarray:
        """Move one design upright point (or a stack of them) by ``state``."""
        R = Rotation.from_rotvec(state[:3])
        return R.apply(p - self.center) + self.center + state[3:]

    def upright_points(self, state: np.ndarray) -> dict[str, np.ndarray]:
        return {name: self._transform(state, p) for name, p in self.hp.upright.items()}

    def chassis_points(self, rack: float = 0.0) -> dict[str, np.ndarray]:
        """Chassis points; the tie-rod inner end slides sideways with the rack."""
        pts = {k: v.copy() for k, v in self.hp.chassis.items()}
        pts["RTI"] = pts["RTI"] + np.array([0.0, rack, 0.0])
        return pts

    # --- the equations we solve ---------------------------------------------
    def residuals(self, state: np.ndarray, travel: float, rack: float = 0.0) -> np.ndarray:
        """Six numbers that are all zero when the pose is valid (units: mm)."""
        up = self.upright_points(state)
        ch = self.chassis_points(rack)
        res = [
            np.linalg.norm(up[u] - ch[c]) - self.link_lengths[(c, u)]
            for c, u in LINKS
        ]
        res.append(up["WC"][2] - (self.center[2] + travel))
        return np.array(res)

    # --- the pushrod / rocker stage ---------------------------------------------
    def _pushrod_outer(self, state: np.ndarray, points: dict[str, np.ndarray]) -> np.ndarray:
        """Where the pushrod's outer end (PRO) is, given the solved upright."""
        rk = self.hp.rocker
        pro = rk.points["PRO"]
        if rk.mount == "upright":
            return self._transform(state, pro)  # rides on the upright
        # On a wishbone: spin PRO about the arm's pivot axis by the arm's angle.
        front, rear, _bj = ARM_MOUNTS[rk.mount]
        design = {**self.hp.chassis, **self.hp.upright}
        axis = design[rear] - design[front]
        axis = axis / np.linalg.norm(axis)
        angle = arm_angle(points, design, rk.mount)
        return rotate_about_axis(pro, design[front], axis, angle)

    def solve_rocker_stage(self, state: np.ndarray, points: dict[str, np.ndarray],
                           guess: float = 0.0) -> tuple[float, dict[str, np.ndarray]]:
        """Rocker angle (rad) and the PRO, PRI, RKP, SHR points for one pose."""
        pro = self._pushrod_outer(state, points)
        angle = solve_rocker(self.hp.rocker, pro, guess)
        return angle, {"PRO": pro, **rocker_points(self.hp.rocker, angle)}

    def solve(self, travel: float, rack: float = 0.0, guess: np.ndarray | None = None,
              rocker_guess: float = 0.0) -> Pose:
        """Find the upright pose for one wheel travel and rack position."""
        x0 = np.zeros(6) if guess is None else np.asarray(guess, dtype=float)
        sol = least_squares(
            self.residuals, x0, args=(travel, rack),
            xtol=1e-14, ftol=1e-14, gtol=1e-14, x_scale="jac",
        )
        max_res = float(np.max(np.abs(sol.fun)))
        if max_res > 1e-6:
            raise RuntimeError(
                f"Solver did not converge at travel={travel} mm, rack={rack} mm "
                f"(worst residual {max_res:.3g} mm). The geometry may lock up or "
                "a link may be too short to reach."
            )
        state = sol.x
        R = Rotation.from_rotvec(state[:3])
        points = {**self.chassis_points(rack), **self.upright_points(state)}
        rocker_angle = float("nan")
        if self.hp.rocker is not None:
            rocker_angle, extra = self.solve_rocker_stage(state, points, rocker_guess)
            points.update(extra)
        return Pose(
            travel=float(travel),
            rack=float(rack),
            points=points,
            wheel_axis=R.apply(self.hp.wheel_axis),
            state=state,
            max_residual=max_res,
            rocker_angle=rocker_angle,
        )

    def _march(self, values, solve_one) -> list[Pose]:
        """Solve a sorted list of values, marching outward from the one nearest 0.

        Each solve is warm-started from its neighbour, which is much more
        robust than solving every point from scratch. ``solve_one(value, prev)``
        does one solve (``prev`` is the neighbouring Pose, or None).
        """
        values = np.sort(np.asarray(values, dtype=float))
        start = int(np.argmin(np.abs(values)))
        poses: dict[int, Pose] = {start: solve_one(values[start], None)}
        for direction in (+1, -1):
            i = start + direction
            while 0 <= i < len(values):
                poses[i] = solve_one(values[i], poses[i - direction])
                i += direction
        return [poses[i] for i in range(len(values))]

    @staticmethod
    def _warm(prev: Pose | None) -> dict:
        """Starting guesses for the next solve, taken from its neighbour."""
        if prev is None:
            return {}
        rk = 0.0 if np.isnan(prev.rocker_angle) else prev.rocker_angle
        return {"guess": prev.state, "rocker_guess": rk}

    def sweep(self, travels, rack: float = 0.0) -> list[Pose]:
        """Solve a list of wheel travels, returned sorted from droop to bump."""
        return self._march(travels, lambda t, prev: self.solve(t, rack, **self._warm(prev)))

    def steer_sweep(self, racks, travel: float = 0.0) -> list[Pose]:
        """Solve a list of rack positions at one wheel travel, sorted low to high.

        Positive rack moves the rack toward +Y (left), which steers the car left.
        """
        return self._march(racks, lambda r, prev: self.solve(travel, r, **self._warm(prev)))


def bump_sweep(hp: Hardpoints, travel_mm: float = 50.0, step_mm: float = 1.0,
               rack: float = 0.0) -> list[Pose]:
    """Convenience wrapper: sweep from -travel_mm (droop) to +travel_mm (bump)."""
    n = int(round(2 * travel_mm / step_mm)) + 1
    return DoubleWishbone(hp).sweep(np.linspace(-travel_mm, travel_mm, n), rack)


def steer_sweep(hp: Hardpoints, rack_mm: float = 25.0, step_mm: float = 1.0,
                travel: float = 0.0) -> list[Pose]:
    """Convenience wrapper: sweep the rack from -rack_mm to +rack_mm."""
    n = int(round(2 * rack_mm / step_mm)) + 1
    return DoubleWishbone(hp).steer_sweep(np.linspace(-rack_mm, rack_mm, n), travel)
