"""Roll sweep: both front corners as the body rolls in a corner.

The idea
--------
In a corner the body leans (rolls) toward the outside of the turn. Seen from
the chassis, the outside wheel moves up (bump) and the inside wheel moves down
(droop). So a roll sweep is just two bump solves at once, one per side, moving
in opposite directions.

We only model the LEFT corner. The right corner is its mirror image
(y -> -y), exactly as in ``steering.py``: the right wheel at travel ``+s`` is
the left wheel solved at ``+s`` and then mirrored. So one left-corner solve at
``-s`` (left wheel drooping) and one at ``+s`` (mirrored to be the right wheel
in bump) give the whole axle.

Which travel gives which roll angle?
------------------------------------
Assumption: the two wheels move **equal and opposite** amounts relative to the
chassis (left ``-s``, right ``+s``). With the same spring on each side, that
keeps the total spring force roughly constant, i.e. the car is rolling, not
heaving. It's the usual way kinematics programs define a roll sweep.

The ground is the line through the two contact patches. Body roll is the
angle between that line and the chassis's own horizontal, so for a target
roll angle we search for the travel ``s`` that tilts the contact-patch line by
exactly that much (``scipy.optimize.brentq``, a robust 1D root finder). The
contact patches also move sideways a little as the wheels travel, so ``s`` is
close to, but not exactly, ``(track / 2) * tan(roll)``.

Signs
-----
Positive roll follows the right-hand rule about +X (forward): the LEFT side
of the body rises and the right side drops. That's the body leaning RIGHT,
which happens in a LEFT turn, the same "left is positive" convention as the
steering sweep. So at positive roll:

* the right wheel is the **outside** wheel and is in bump;
* the left wheel is the **inside** wheel and is in droop.

All "relative to ground" numbers are the chassis-frame numbers rotated by the
roll angle, so they're what a camera on the track would see.

What comes out (``roll_metrics`` / ``roll_summary``)
---------------------------------------------------
* **Roll camber**: camber change relative to the GROUND per degree of roll.
  If the wheel simply leaned with the body it would be +1 deg/deg on the
  outside wheel (it would gain positive camber, which costs grip). Camber
  gain in bump fights that. ``camber_compensation_pct`` says how much:
  0% = the wheel leans with the body, 100% = it stays at the same camber to
  the road. Double wishbones are typically 20-50%.
* **Roll steer**: change of steer angle per degree of roll (deg/deg, average of
  both wheels, + = steers left). In a left turn (positive roll) a NEGATIVE
  value means the front wheels steer out of the turn: roll understeer.
* **Roll centre under roll**: the classic front-view construction. Draw a line
  from each contact patch through that side's instant centre; the roll centre
  is where the two lines cross. With the car rolled the two sides aren't
  symmetric any more, so it moves sideways as well as up/down. A roll centre
  that stays near the middle of the car is easier to tune with springs.
* **Wheel and damper travel per degree of roll**.
* **Roll stiffness** from the springs alone (no anti-roll bar, rigid tyres),
  from the spring energy. With spring rate ``k`` and spring compressions
  ``x_L``, ``x_R`` the energy is ``U = k (x_L^2 + x_R^2) / 2``, and the roll
  stiffness is ``d^2U / d(roll)^2``. For small roll this is the textbook
  ``k * MR^2 * track^2 / 2`` (per radian), but working from the actual damper
  lengths means a rising-rate rocker is captured too.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation

from .hardpoints import Hardpoints
from .kinematics import DoubleWishbone, Pose
from .arb import bar_twist_deg
from .metrics import _arm_line_front_view, camber_deg, instant_center_front_view, spring_length_mm
from .sideview import VehicleParams
from .steering import heading_deg, mirror_axis

FLIP = np.array([1.0, -1.0, 1.0])  # mirror y -> -y (left corner -> right corner)


def mirror_point(p: np.ndarray) -> np.ndarray:
    """A left-corner point mirrored to the right-hand side of the car."""
    return np.asarray(p) * FLIP


def roll_rotation(roll_rad: float) -> Rotation:
    """Rotation from the chassis frame to the ground frame (about +X)."""
    return Rotation.from_rotvec([roll_rad, 0.0, 0.0])


@dataclass
class RollPose:
    """Both corners solved at one body roll angle.

    ``left`` is the left corner at travel ``-travel``. ``right_src`` is the left
    corner solved at ``+travel``; mirror it to get the right corner (see
    :meth:`right_points`). Every point is in the CHASSIS frame; use
    :meth:`to_ground` to see them as they sit on the road.
    """

    roll_deg: float     # body roll angle (+ = body leans right, as in a left turn)
    travel: float       # right-wheel bump = left-wheel droop, mm (chassis frame)
    left: Pose
    right_src: Pose
    arb_rate_nm_per_deg: float = 0.0  # anti-roll bar torsional rate (0 = no bar)

    @property
    def roll_rad(self) -> float:
        return float(np.radians(self.roll_deg))

    def right_points(self) -> dict[str, np.ndarray]:
        """Every right-corner point (chassis frame)."""
        return {k: mirror_point(v) for k, v in self.right_src.points.items()}

    def to_ground(self, p: np.ndarray) -> np.ndarray:
        """Chassis-frame point(s) -> ground frame.

        Ground frame: z = 0 on the road (through both contact patches), y = 0
        half way between the contact patches, x unchanged.
        """
        R = roll_rotation(self.roll_rad)
        cp_l = R.apply(self.left.points["CP"])
        cp_r = R.apply(mirror_point(self.right_src.points["CP"]))
        offset = -(cp_l + cp_r) / 2.0
        offset[0] = 0.0
        return R.apply(p) + offset


def ground_roll_rad(left: Pose, right_src: Pose) -> float:
    """Body roll implied by the two contact patches (rad, + = body leans right).

    In the chassis frame the road is the line through both contact patches.
    If the right patch sits higher than the left one (right wheel in bump),
    the body is leaning right, which is positive roll.
    """
    cp_l = left.points["CP"]
    cp_r = mirror_point(right_src.points["CP"])
    return float(np.arctan2(cp_r[2] - cp_l[2], cp_l[1] - cp_r[1]))


class RollModel:
    """Solves the axle at any roll angle, using one left-corner model."""

    def __init__(self, hp: Hardpoints):
        self.hp = hp
        self.corner = DoubleWishbone(hp)
        self.half_track = float(hp.upright["CP"][1])
        self._last: dict[str, Pose] = {}  # warm starts for the next solve

    def _solve_side(self, side: str, travel: float) -> Pose:
        """One left-corner solve, warm-started from the last one on this side."""
        pose = self.corner.solve(travel, **DoubleWishbone._warm(self._last.get(side)))
        self._last[side] = pose
        return pose

    def pair(self, travel: float) -> tuple[Pose, Pose]:
        """Left corner in droop (-travel) and the source of the right corner (+travel)."""
        return self._solve_side("left", -travel), self._solve_side("right", travel)

    def solve(self, roll_deg: float) -> RollPose:
        """Both corners at one body roll angle (deg)."""
        target = np.radians(roll_deg)
        rate = self.hp.arb.rate_nm_per_deg if self.hp.arb is not None else 0.0
        if abs(target) < 1e-12:
            return RollPose(float(roll_deg), 0.0, *self.pair(0.0), arb_rate_nm_per_deg=rate)

        def error(s: float) -> float:  # how far off the target roll we are
            return ground_roll_rad(*self.pair(s)) - target

        # First guess: rotate the design track by the roll angle. Then search a
        # bracket around it, widening until the answer is inside.
        guess = self.half_track * np.tan(target)
        width = 0.2 * abs(guess) + 1.0
        for _ in range(6):
            lo, hi = guess - width, guess + width
            if error(lo) * error(hi) < 0:
                break
            width *= 2.0
        else:
            raise RuntimeError(f"Couldn't find the wheel travel for {roll_deg:+g} deg of roll.")
        s = brentq(error, lo, hi, xtol=1e-10, rtol=1e-14)
        return RollPose(float(roll_deg), float(s), *self.pair(s), arb_rate_nm_per_deg=rate)

    def sweep(self, angles) -> list[RollPose]:
        """Solve a list of roll angles (deg), sorted from most negative to most positive.

        Like the bump sweep, it marches outward from 0 so each solve starts
        next to its neighbour.
        """
        angles = np.sort(np.asarray(angles, dtype=float))
        start = int(np.argmin(np.abs(angles)))
        out = {start: self.solve(angles[start])}
        for direction in (+1, -1):
            # Each direction starts from the pose nearest zero roll.
            self._last = {"left": out[start].left, "right": out[start].right_src}
            i = start + direction
            while 0 <= i < len(angles):
                out[i] = self.solve(angles[i])
                i += direction
        return [out[i] for i in range(len(angles))]


def roll_sweep(hp: Hardpoints, roll_deg: float = 3.0, step_deg: float = 0.25) -> list[RollPose]:
    """Convenience wrapper: sweep body roll from -roll_deg to +roll_deg."""
    n = int(round(2 * roll_deg / step_deg)) + 1
    return RollModel(hp).sweep(np.linspace(-roll_deg, roll_deg, n))


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def _force_line(pose: Pose) -> tuple[np.ndarray, np.ndarray]:
    """Front-view line from the contact patch through the instant centre (y, z).

    Returns (point, direction). With parallel arms the instant centre is at
    infinity, so the line runs parallel to the arms instead.
    """
    cp = pose.points["CP"][1:]
    ic = instant_center_front_view(pose)
    if ic is None:
        _p, direction = _arm_line_front_view(pose, "LAF", "LAA", "LBJ")
    else:
        direction = ic - cp
    return cp, direction


def roll_center_chassis(rp: RollPose) -> np.ndarray | None:
    """Roll centre (y, z) in the chassis frame, or None if the lines are parallel."""
    p_l, d_l = _force_line(rp.left)
    p_r, d_r = _force_line(rp.right_src)
    p_r, d_r = p_r * FLIP[1:], d_r * FLIP[1:]  # mirror the right side's line
    m = np.array([d_l, -d_r]).T
    if abs(np.linalg.det(m)) < 1e-12:
        return None
    a, _ = np.linalg.solve(m, p_r - p_l)
    return p_l + a * d_l


def wheel_angles_ground(rp: RollPose) -> tuple[np.ndarray, np.ndarray]:
    """Both spin axes (pointing outboard) as seen from the road."""
    R = roll_rotation(rp.roll_rad)
    left = R.apply(rp.left.wheel_axis)
    right = R.apply(mirror_axis(rp.right_src.wheel_axis))
    return left, right


ROLL_COLUMNS = (
    "roll_deg", "travel_left_mm", "travel_right_mm",
    "camber_left_deg", "camber_right_deg", "steer_left_deg", "steer_right_deg",
    "rc_height_mm", "rc_lateral_mm", "damper_left_mm", "damper_right_mm",
    "chassis_heave_mm", "track_mm", "bar_twist_deg",
    "spring_roll_moment_nm", "bar_roll_moment_nm", "total_roll_moment_nm",
)


def roll_metrics(poses: list[RollPose], vehicle: VehicleParams | None = None) -> dict[str, np.ndarray]:
    """Everything worth plotting over a roll sweep. Returns column -> array.

    Columns (all relative to the ground unless they say otherwise):

    * ``camber_left_deg`` / ``camber_right_deg``: each wheel's camber to the
      road, with the usual sign for that wheel (negative = top leans in).
    * ``steer_*_deg``: steer angle from straight ahead (+ = pointing left).
    * ``rc_height_mm`` / ``rc_lateral_mm``: roll centre above the road, and
      sideways from half way between the contact patches (+ = left).
    * ``travel_*_mm``: wheel travel relative to the chassis (+ = bump).
    * ``damper_*_mm``: damper compression from ride height (+ = shorter).
    * ``chassis_heave_mm``: how far the body rises (+) or sinks as it rolls
      (jacking).
    * ``bar_twist_deg``: anti-roll bar twist (0 without a bar).
    * ``*_roll_moment_nm``: roll moment the springs, the bar, and both
      together push back with.
    """
    vehicle = vehicle or VehicleParams()
    roll = np.array([p.roll_deg for p in poses])
    i0 = int(np.argmin(np.abs(roll)))
    # Straight ahead = the design wheel heading (static toe), so steer angles
    # are 0 at ride height with no roll.
    straight = heading_deg(poses[i0].left.wheel_axis)

    out: dict[str, list] = {c: [] for c in ROLL_COLUMNS}
    design_len = spring_length_mm(poses[i0].left)  # same both sides at zero roll
    for rp in poses:
        a_l, a_r = wheel_angles_ground(rp)
        out["roll_deg"].append(rp.roll_deg)
        out["travel_left_mm"].append(-rp.travel)
        out["travel_right_mm"].append(rp.travel)
        out["camber_left_deg"].append(_camber_from_axis(a_l))
        # Mirror the right axis back to "left-wheel" form so the same camber
        # sign convention applies (negative = top of tyre leans inboard).
        out["camber_right_deg"].append(_camber_from_axis(mirror_axis(a_r)))
        out["steer_left_deg"].append(heading_deg(a_l) - straight)
        out["steer_right_deg"].append(heading_deg(a_r) + straight)

        rc = roll_center_chassis(rp)
        if rc is None:
            out["rc_height_mm"].append(np.nan)
            out["rc_lateral_mm"].append(np.nan)
        else:
            rc_g = rp.to_ground(np.array([0.0, rc[0], rc[1]]))
            out["rc_height_mm"].append(rc_g[2])
            out["rc_lateral_mm"].append(rc_g[1])

        out["damper_left_mm"].append(design_len - spring_length_mm(rp.left))
        out["damper_right_mm"].append(design_len - spring_length_mm(rp.right_src))
        out["chassis_heave_mm"].append(rp.to_ground(np.zeros(3))[2])
        cp_l = rp.to_ground(rp.left.points["CP"])
        cp_r = rp.to_ground(mirror_point(rp.right_src.points["CP"]))
        out["track_mm"].append(cp_l[1] - cp_r[1])
        if np.isnan(rp.left.arb_angle):
            out["bar_twist_deg"].append(0.0)  # no bar
        else:
            out["bar_twist_deg"].append(bar_twist_deg(rp.left.arb_angle, rp.right_src.arb_angle))
        for key in ("spring_roll_moment_nm", "bar_roll_moment_nm", "total_roll_moment_nm"):
            out[key].append(0.0)  # filled in below

    res = {k: np.array(v, dtype=float) for k, v in out.items()}
    res["chassis_heave_mm"] -= res["chassis_heave_mm"][i0]  # 0 at no roll

    # Roll moment = slope of the stored energy against roll angle (in rad).
    # Springs: U = k (x_L^2 + x_R^2) / 2, in N mm, so divide by 1000 for N m (J).
    # Bar: torque = rate x twist, so U = rate x twist^2 / 2, with twist turned
    # into radians once so U comes out in J.
    k = vehicle.spring_rate_n_per_mm
    rate = poses[i0].arb_rate_nm_per_deg
    u_springs = 0.5 * k * (res["damper_left_mm"] ** 2 + res["damper_right_mm"] ** 2) / 1000.0
    u_bar = 0.5 * rate * res["bar_twist_deg"] ** 2 * np.pi / 180.0
    for key, energy in (("spring_roll_moment_nm", u_springs), ("bar_roll_moment_nm", u_bar),
                        ("total_roll_moment_nm", u_springs + u_bar)):
        if len(poses) >= 3:
            res[key] = np.gradient(energy, np.radians(roll), edge_order=2)
        else:
            res[key] = np.full(len(poses), np.nan)
    res["arb_rate_nm_per_deg"] = np.full(len(poses), rate)  # kept for roll_summary
    return res


def _camber_from_axis(axis: np.ndarray) -> float:
    """Camber (deg) from a left-wheel spin axis, same formula as metrics.camber_deg."""
    return float(-np.degrees(np.arcsin(axis[2])))


def roll_summary(roll: dict[str, np.ndarray], vehicle: VehicleParams | None = None,
                 whole_car: bool | None = None) -> dict[str, float]:
    """Headline roll numbers, mostly slopes at zero roll (per degree of roll).

    ``whole_car``: also add roll per g and the load transfer split, using the
    rear_* placeholders in the vehicle block. Default: only for a front file.

    At positive roll (a left turn) the right wheel is the outside one.
    """
    vehicle = vehicle or VehicleParams()
    r = roll["roll_deg"]
    i0 = int(np.argmin(np.abs(r)))
    i_max = int(np.argmax(r))
    slope = lambda key: float(np.gradient(roll[key], r)[i0])  # noqa: E731  (per deg)

    outer_camber = slope("camber_right_deg")
    wheel_per_deg = slope("travel_right_mm")
    damper_per_deg = slope("damper_right_mm")
    mr = damper_per_deg / wheel_per_deg
    k = vehicle.spring_rate_n_per_mm
    # Roll stiffness = d2U/d(roll)^2 = k (x_L'^2 + x_R'^2), with x' per RADIAN.
    per_rad = 180.0 / np.pi
    x_l = slope("damper_left_mm") * per_rad
    x_r = damper_per_deg * per_rad
    k_roll_nmm_per_rad = k * (x_l ** 2 + x_r ** 2)
    k_springs = k_roll_nmm_per_rad / 1000.0 / per_rad          # N mm/rad -> N m/deg
    # Bar: K = rate x (d twist / d roll)^2, both in degrees, gives N m/deg.
    twist_per_deg = slope("bar_twist_deg")
    rate = float(roll["arb_rate_nm_per_deg"][i0]) if "arb_rate_nm_per_deg" in roll else 0.0
    k_bar = rate * twist_per_deg ** 2
    out = {
        "roll_camber_outer_deg_per_deg": outer_camber,
        "roll_camber_inner_deg_per_deg": slope("camber_left_deg"),
        "camber_compensation_pct": (1.0 - outer_camber) * 100.0,
        "outer_camber_at_max_roll_deg": float(roll["camber_right_deg"][i_max]),
        "roll_steer_deg_per_deg": 0.5 * (slope("steer_left_deg") + slope("steer_right_deg")),
        "rc_height_mm": float(roll["rc_height_mm"][i0]),
        "rc_height_at_max_roll_mm": float(roll["rc_height_mm"][i_max]),
        "rc_lateral_shift_mm_per_deg": slope("rc_lateral_mm"),
        "wheel_travel_mm_per_deg": wheel_per_deg,
        "damper_travel_mm_per_deg": damper_per_deg,
        "motion_ratio": mr,
        "chassis_heave_at_max_roll_mm": float(roll["chassis_heave_mm"][i_max]),
        "spring_rate_n_per_mm": k,
        "wheel_rate_n_per_mm": k * mr ** 2,
        "roll_stiffness_springs_nm_per_deg": k_springs,
        "bar_twist_deg_per_deg_roll": twist_per_deg,
        "arb_rate_nm_per_deg": rate,
        "roll_stiffness_bar_nm_per_deg": k_bar,
        "roll_stiffness_nm_per_deg": k_springs + k_bar,
    }
    if whole_car is None:
        whole_car = vehicle.axle == "front"
    if not whole_car:
        # The whole-car numbers below use the rear_* placeholders of a FRONT
        # file. For a rear corner, use fullcar.py (``suspkin car``) instead,
        # which combines a real front and a real rear file.
        return out
    # Whole car: how far it actually rolls, and what the outside tyre's camber
    # is at the design cornering load.
    car = whole_car_roll(out["roll_stiffness_nm_per_deg"], out["rc_height_mm"],
                         float(roll["track_mm"][i0]), vehicle)
    out.update(car)
    out.update(camber_at_roll(roll, car["roll_at_design_g_deg"], vehicle.target_outside_camber_deg))
    return out


def camber_at_roll(roll: dict[str, np.ndarray], roll_deg: float,
                   target_deg: float) -> dict[str, float]:
    """Outside-tyre camber at a given body roll, and the static camber for a target.

    Static camber just rotates the wheel on the upright, so it shifts the
    whole camber curve up or down by the same amount (a test checks this).
    So the static camber that puts the outside tyre at ``target_deg`` is the
    current static camber plus the error.
    """
    r = roll["roll_deg"]
    i0 = int(np.argmin(np.abs(r)))
    if r.min() <= roll_deg <= r.max():
        camber_now = float(np.interp(roll_deg, r, roll["camber_right_deg"]))
    else:
        camber_now = float("nan")  # the sweep doesn't reach that far: use a bigger --roll
    static_now = float(roll["camber_right_deg"][i0])
    return {
        "outer_camber_at_design_g_deg": camber_now,
        "static_camber_for_target_deg": static_now + target_deg - camber_now,
    }


G = 9.81  # m/s^2


def whole_car_roll(front_k_nm_per_deg: float, front_rc_mm: float, front_track_mm: float,
                   vehicle: VehicleParams) -> dict[str, float]:
    """Roll per g and the front/rear load transfer split, from ONE front file.

    The rear axle's roll stiffness, roll centre and track come from the
    ``rear_*`` numbers in the front file's ``vehicle`` block. To use a real
    rear corner instead, see ``fullcar.py`` (``suspkin car``), which calls
    :func:`load_transfer` with both axles' solved numbers.
    """
    v = vehicle
    return load_transfer(
        front_k_nm_per_deg, v.rear_roll_stiffness_nm_per_deg,
        front_rc_mm, v.rear_rc_height_mm,
        front_track_mm, v.rear_track_mm, v,
    )


def load_transfer(k_front_nm_per_deg: float, k_rear_nm_per_deg: float,
                  rc_front_mm: float, rc_rear_mm: float,
                  track_front_mm: float, track_rear_mm: float,
                  vehicle: VehicleParams) -> dict[str, float]:
    """Roll per g and the front/rear split of lateral load transfer.

    Steady cornering at lateral acceleration ``A`` (in g). Following Milliken
    (*Race Car Vehicle Dynamics*, ch. 18), with the whole car's weight
    ``W = m g`` acting at the CG:

    * The **roll axis** joins the front and rear roll centres. Under the CG it
      is at ``h_ra = rc_f + (rc_r - rc_f) * a / L`` (``a`` = front axle to CG).
    * The body rolls about it with moment ``W * A * (h - h_ra)``, resisted by
      both axles' roll stiffness, so::

          roll per g = W (h - h_ra) / (K_front + K_rear)

    * Each axle's **lateral load transfer** (the weight moving from the inside
      to the outside tyre) has two parts: its share of the roll moment, which
      goes through the springs and bars in proportion to roll stiffness, and
      the part that goes straight through the links at its roll centre::

          dW_front = (W A / t_f) * ( (h - h_ra) K_f / (K_f + K_r) + w_f rc_f )

    The **front share** of the total load transfer (often called TLLTD) is
    a main handling balance knob: more on the front = more understeer. A common
    starting point is a few percent above the front weight share.

    Simplifications: everything is treated as sprung mass and the tyres are
    rigid. The load transfer numbers returned are per g (multiply by ``A``).
    """
    v = vehicle
    per_rad = 180.0 / np.pi
    k_f = k_front_nm_per_deg * per_rad                       # N m / rad
    k_r = k_rear_nm_per_deg * per_rad
    L, h = v.wheelbase_mm / 1000.0, v.cg_height_mm / 1000.0  # m
    a = (1.0 - v.front_weight_fraction) * L                  # front axle to CG
    rc_f, rc_r = rc_front_mm / 1000.0, rc_rear_mm / 1000.0
    h_ra = rc_f + (rc_r - rc_f) * a / L
    arm = h - h_ra                                           # roll moment arm, m
    W = v.mass_kg * G
    gradient_deg = W * arm / (k_f + k_r) * per_rad           # deg per g
    wf = v.front_weight_fraction
    # Split each axle's load transfer into its two parts (N per g).
    dw_f_elastic = W / (track_front_mm / 1000.0) * arm * k_f / (k_f + k_r)
    dw_f_geometric = W / (track_front_mm / 1000.0) * wf * rc_f
    dw_r_elastic = W / (track_rear_mm / 1000.0) * arm * k_r / (k_f + k_r)
    dw_r_geometric = W / (track_rear_mm / 1000.0) * (1.0 - wf) * rc_r
    dw_f = dw_f_elastic + dw_f_geometric
    dw_r = dw_r_elastic + dw_r_geometric
    return {
        "roll_axis_height_at_cg_mm": h_ra * 1000.0,
        "roll_moment_arm_mm": arm * 1000.0,
        "roll_gradient_deg_per_g": gradient_deg,
        "roll_at_design_g_deg": gradient_deg * v.design_lateral_g,
        "front_roll_stiffness_share_pct": 100.0 * k_f / (k_f + k_r),
        "front_load_transfer_share_pct": 100.0 * dw_f / (dw_f + dw_r),
        "front_weight_pct": 100.0 * wf,
        "front_load_transfer_n_per_g": dw_f,
        "rear_load_transfer_n_per_g": dw_r,
        "front_geometric_share_pct": 100.0 * dw_f_geometric / dw_f,
        "rear_geometric_share_pct": 100.0 * dw_r_geometric / dw_r,
    }
