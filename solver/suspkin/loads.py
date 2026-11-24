"""Phase 8: load cases, the force in every link, and tube / rod end sizing.

The chain, from the road to the tubes
-------------------------------------
1. **Load cases** (``data/load_cases.json``): the car accelerates sideways,
   forwards / backwards or hits a bump, given in g.
2. **Tyre loads.** Weight moves around the car. From the mass, weight split,
   CG height and wheelbase, and each axle's share of the lateral load transfer
   (``suspkin car``, ``fullcar.py``), we get every tyre's vertical load ``Fz``.
   Every tyre is assumed to use the same share of its grip, so its sideways
   and fore/aft forces are just ``a_y * Fz`` and ``a_x * Fz``.
3. **Link forces.** With rod ends and spherical bearings at both ends, each
   wishbone leg, the tie rod (toe link) and the pushrod can only push or pull
   along its own length: a *two-force member*. So the unknowns are one number
   per link, and the upright's static equilibrium (forces and moments add up
   to zero) gives them. See :func:`solve_link_forces`.
4. **Sizing.** Each tube is checked for yield in tension and buckling in
   compression (Euler for long, slender tubes, Johnson for short, stocky ones)
   and each rod end against its rated load. The result is a safety factor per
   member, and the lightest catalogue tube that would still pass.

Sign convention: a link force is **positive in tension** (the link pulls the
upright toward the chassis) and negative in compression.

The pushrod is bolted to a wishbone, not to the upright
-------------------------------------------------------
On this car the pushrod sits on a bracket on the lower arm, near the ball
joint. Then the lower arm isn't made of two two-force members any more: it's
a rigid triangle that pivots on the chassis and is held up by the pushrod.
We handle that exactly:

* The upright gets 6 equations (3 forces, 3 moments). Its unknowns are the two
  upper-arm legs, the tie rod, and the full 3D force at the lower ball joint
  (3 numbers): 6 unknowns.
* One more unknown, the pushrod force, and one more equation: the lower arm
  must not spin about its chassis pivot axis, so the moments about that axis
  of the ball joint force and the pushrod force cancel.

7 equations, 7 unknowns, one linear solve. The lower arm's own leg forces then
come from the ball joint and pushrod forces, treating the pushrod bracket as if
it were at the ball joint. The small part of the load that this can't put
along the legs (the bracket is a few cm inboard) is bending at the bracket;
it is reported as ``bracket_bending_force_n`` so you can see it's small.

If the pushrod is on the upright (``"pushrod_mount": "upright"``), or there is
no rocker and the damper goes straight to the upright (``SHU``), the upright
simply has six two-force members and a 6 x 6 solve.

Where the tyre forces act
-------------------------
Vertical and sideways forces act at the contact patch ``CP`` (the sideways
force a pneumatic trail behind it, which is the tyre's self-aligning torque). Braking acts at
``CP`` too when the brakes are outboard (the caliper on the upright takes the
brake torque); with inboard brakes it acts at the wheel centre. Drive force on
a driven axle with a chassis-mounted diff acts at the wheel centre ``WC``:
the halfshaft takes the drive torque into the chassis, so only the push at the
hub goes through the upright. This matches the anti-dive / anti-squat
reference points in ``sideview.py``.

Simplifications
---------------
Loads are applied at the design (ride height) geometry, the tyre's
self-aligning torque comes only from a fixed pneumatic trail, its overturning
and rolling resistance moments are ignored and parts are rigid.

Phase 9: the rocker, damper and anti-roll bar
---------------------------------------------
The pushrod force above is the *total* spring force at the wheel: springs plus
anti-roll bar, because the bar's drop link hangs off the same rocker. The
rocker can only spin about its pivot axis, so it gives ONE equation (moments
about the axis add up to zero) for TWO unknowns, the damper force and the
drop link force. The second equation comes from how the car moves:

* **Heave** (both wheels of an axle loaded the same way, e.g. braking or a
  bump): the bar doesn't twist, so it carries nothing. The damper (spring)
  takes the whole pushrod moment.
* **Roll** (one side up, the other down): spring and bar deflect together, so
  they share the load in proportion to their roll stiffness. ``suspkin car``
  gives both (``*_roll_stiffness_springs/bar_nm_per_deg``).

A cornering case has an inside and an outside wheel, so its pushrod forces
split into a heave part (the average of the two) and a roll part (half the
difference). The heave part goes to the spring; the roll part is shared.
See :func:`solve_rocker_loads`.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .fullcar import CarResult
from .hardpoints import Hardpoints
from .roll import G
from .sideview import VehicleParams

# Every link we report, in print order: (name, member group for sizing).
MEMBER_GROUPS = {
    "upper_front": "upper_arm",
    "upper_rear": "upper_arm",
    "lower_front": "lower_arm",
    "lower_rear": "lower_arm",
    "tie_rod": "tie_rod",
    "pushrod": "pushrod",
}
GROUP_LABELS = {"upper_arm": "upper A-arm", "lower_arm": "lower A-arm",
                "tie_rod": "tie rod", "pushrod": "pushrod"}

# (member name, upright end, other end) for the wishbone legs and tie rod.
LEGS = {
    "upper_front": ("UBJ", "UAF"),
    "upper_rear": ("UBJ", "UAA"),
    "lower_front": ("LBJ", "LAF"),
    "lower_rear": ("LBJ", "LAA"),
}
ARMS = {  # which legs make up each arm, and its ball joint
    "lower_arm": ("lower_front", "lower_rear", "LBJ"),
    "upper_arm": ("upper_front", "upper_rear", "UBJ"),
}


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


# ---------------------------------------------------------------------------
# 1. Load cases and tyre loads
# ---------------------------------------------------------------------------

@dataclass
class LoadCase:
    """One manoeuvre. Accelerations in g; + longitudinal = speeding up."""

    name: str
    lateral_g: float = 0.0       # size of the cornering acceleration (either direction)
    longitudinal_g: float = 0.0  # + = accelerating, - = braking
    vertical_g: float = 1.0      # multiplies every tyre's vertical load (3 = 3 g bump)
    note: str = ""


@dataclass
class TyreLoad:
    """Forces from the road on one tyre, in the LEFT corner's axes (+Y = outboard).

    Only the left corners are modelled. The right-hand tyre in a left turn is
    the mirror image of the left one in a right turn, so a left corner covers
    both the inside and the outside wheel: "inside" = left turn (sideways force
    toward +Y, outboard), "outside" = right turn (toward -Y, inboard).
    """

    case: str
    axle: str                   # "front" / "rear"
    side: str                   # "inside", "outside" or "-" (no cornering)
    fx: float                   # N, + = forward (drive), - = rearward (braking)
    fy: float                   # N, + = toward +Y
    fz: float                   # N, + = road pushes the tyre up
    fx_at: str = "CP"           # where fx acts: "CP" or "WC" (see module docstring)
    lifted: bool = False        # this wheel would be off the ground
    trail_mm: float = 0.0       # pneumatic trail: fy acts this far behind CP

    @property
    def label(self) -> str:
        return self.case if self.side == "-" else f"{self.case} ({self.side})"

    def applied(self, hp: Hardpoints) -> list[tuple[np.ndarray, np.ndarray]]:
        """The tyre forces as (point, force vector) pairs on this corner's upright."""
        cp, wc = hp.upright["CP"], hp.upright["WC"]
        # A rolling tyre's sideways force acts a little BEHIND the centre of
        # the contact patch (the pneumatic trail). That offset is the tyre's
        # self-aligning torque, and it's what loads the tie rod / toe link in
        # a corner.
        behind = cp - np.array([self.trail_mm, 0.0, 0.0])
        out = [(behind, np.array([0.0, self.fy, 0.0])), (cp, np.array([0.0, 0.0, self.fz]))]
        out.append((wc if self.fx_at == "WC" else cp, np.array([self.fx, 0.0, 0.0])))
        return out


def tyre_loads(case: LoadCase, car: VehicleParams, lt_front_n_per_g: float,
               lt_rear_n_per_g: float, rear: VehicleParams | None = None,
               pneumatic_trail_mm: float = 0.0) -> list[TyreLoad]:
    """Tyre forces on the front-left and rear-left corners for one load case.

    ``car`` is the front file's vehicle block (car-level numbers), ``rear`` the
    rear file's (its brakes / driven flags). ``lt_*_n_per_g`` is each axle's
    lateral load transfer per g of cornering (from ``suspkin car``).
    ``pneumatic_trail_mm`` puts the sideways force that far behind the
    contact patch (see :meth:`TyreLoad.applied`).
    """
    rear = rear or car
    W = car.mass_kg * G
    L, h = car.wheelbase_mm, car.cg_height_mm
    ay, ax = abs(case.lateral_g), case.longitudinal_g
    # Longitudinal weight transfer: speeding up moves load to the rear,
    # braking to the front. Total = W * a_x * h / L, shared by the two wheels.
    dx = W * ax * h / L
    axles = {
        "front": (W * car.front_weight_fraction / 2.0 - dx / 2.0, lt_front_n_per_g, car),
        "rear": (W * (1.0 - car.front_weight_fraction) / 2.0 + dx / 2.0, lt_rear_n_per_g, rear),
    }
    sides = ("outside", "inside") if ay > 0 else ("-",)
    out = []
    for axle, (base, lt, veh) in axles.items():
        # Vertical load on each side of this axle, before any bump factor.
        fz = {s: base + (lt * ay if s == "outside" else -lt * ay if s == "inside" else 0.0)
              for s in sides}
        fz = {s: max(v, 0.0) for s, v in fz.items()}  # a tyre can't pull the road
        lifted = {s: base + (-lt * ay if s == "inside" else 0.0) < 0 for s in sides}
        axle_fz = sum(fz.values()) * (2 if sides == ("-",) else 1)  # both wheels of the axle
        for s in sides:
            # Fore/aft force. Braking: every tyre at the same friction use.
            # Driving: only a driven axle pushes, shared by tyre load.
            if ax < 0:
                fx = ax * fz[s]
                fx_at = "CP" if veh.brakes == "outboard" else "WC"
            elif ax > 0 and veh.driven:
                fx = ax * W * fz[s] / axle_fz if axle_fz > 0 else 0.0
                fx_at = "WC"  # chassis-mounted diff: drive torque goes into the chassis
            else:
                fx, fx_at = 0.0, "CP"
            # Sideways: toward the inside of the turn. In the left corner's
            # axes that's +Y for the inside wheel (left turn) and -Y for the
            # outside wheel (right turn, mirrored).
            fy = ay * fz[s] * (1.0 if s == "inside" else -1.0 if s == "outside" else 0.0)
            k = case.vertical_g
            out.append(TyreLoad(case.name, axle, s, k * fx, k * fy, k * fz[s], fx_at, lifted[s],
                                pneumatic_trail_mm))
    return out


def ideal_front_brake_bias(car: VehicleParams, decel_g: float) -> float:
    """Front share of braking force that locks all four tyres at the same time.

    It equals the front axle's share of the vertical load while braking at
    ``decel_g``: static front weight plus the load transfer ``a h / L``.
    """
    return car.front_weight_fraction + decel_g * car.cg_height_mm / car.wheelbase_mm


# ---------------------------------------------------------------------------
# 2. Link forces from the upright's equilibrium
# ---------------------------------------------------------------------------

@dataclass
class LinkForces:
    """Solved forces for one corner under one set of tyre loads."""

    members: dict[str, float]                 # N, + = tension
    joint_forces: dict[str, np.ndarray]       # force ON the upright at each joint, N
    chassis_forces: dict[str, np.ndarray]     # force ON the chassis at each pickup, N
    lengths: dict[str, float]                 # member length centre to centre, mm
    bracket_bending_force_n: float = 0.0      # lumping error at the pushrod bracket
    pushrod_on: str = "lower_arm"


def _pushrod_ends(hp: Hardpoints) -> tuple[str, np.ndarray, np.ndarray]:
    """(what the pushrod is bolted to, outer end, inner end) at the design position."""
    if hp.rocker is None:
        # No rocker: the damper goes straight from the upright (SHU) to the
        # chassis (SHC) and is the sixth link.
        return "upright", hp.upright["SHU"], hp.chassis["SHC"]
    rk = hp.rocker
    return rk.mount, rk.points["PRO"], rk.points["PRI"]


def solve_link_forces(hp: Hardpoints,
                      applied: list[tuple[np.ndarray, np.ndarray]]) -> LinkForces:
    """Force in every link from the loads on the upright (design position).

    ``applied`` is a list of (point, force vector in N) acting on the upright,
    e.g. from :meth:`TyreLoad.applied`. See the module docstring for the method.
    """
    pts = {**hp.chassis, **hp.upright}
    c = hp.upright["WC"]  # take moments about the wheel centre
    mount, pro, pri = _pushrod_ends(hp)
    e_push = _unit(pri - pro)  # a pushrod in tension pulls its outer end toward PRI

    # Two-force members that push straight on the upright:
    # (name, point on the upright, unit vector from there toward the other end).
    direct = [("tie_rod", pts["RTO"], _unit(pts["RTI"] - pts["RTO"]))]
    if mount == "upright":
        arm = None
        direct += [(n, pts[u], _unit(pts[o] - pts[u])) for n, (u, o) in LEGS.items()]
        direct.append(("pushrod", pro, e_push))
    else:
        arm = mount                                    # the arm carrying the pushrod
        other = "upper_arm" if arm == "lower_arm" else "lower_arm"
        for n in ARMS[other][:2]:
            u, o = LEGS[n]
            direct.append((n, pts[u], _unit(pts[o] - pts[u])))

    # Build A x = b. Rows 0-2: forces on the upright add to zero. Rows 3-5:
    # moments about WC add to zero. Row 6 (arm-mounted pushrod only): the arm's
    # moments about its own pivot axis add to zero.
    n_rows = 6 if arm is None else 7
    A = np.zeros((n_rows, n_rows))
    b = np.zeros(n_rows)
    for col, (_, p, e) in enumerate(direct):
        A[0:3, col] = e
        A[3:6, col] = np.cross(p - c, e)
    if arm is not None:
        front_leg, rear_leg, bj_name = ARMS[arm]
        bj = pts[bj_name]
        f_piv, r_piv = pts[LEGS[front_leg][1]], pts[LEGS[rear_leg][1]]
        hinge = _unit(r_piv - f_piv)
        j = len(direct)  # columns j..j+2 = ball joint force on the upright; j+3 = pushrod
        A[0:3, j:j + 3] = np.eye(3)
        for k in range(3):
            A[3:6, j + k] = np.cross(bj - c, np.eye(3)[k])
        # The arm feels MINUS the ball joint force at bj, and the pushrod pull
        # (tension P along e_push) at PRO. Moment about the hinge axis:
        #   hinge . ((bj - f) x (-F))  +  P * hinge . ((pro - f) x e_push) = 0
        # and hinge . (r x F) = F . (hinge x r).
        A[6, j:j + 3] = -np.cross(hinge, bj - f_piv)
        A[6, j + 3] = hinge @ np.cross(pro - f_piv, e_push)
    for p, f in applied:
        b[0:3] -= f
        b[3:6] -= np.cross(p - c, f)
    x = np.linalg.solve(A, b)

    members: dict[str, float] = {}
    joint: dict[str, np.ndarray] = {"UBJ": np.zeros(3), "LBJ": np.zeros(3), "RTO": np.zeros(3)}
    chassis: dict[str, np.ndarray] = {}
    for (name, p, e), t in zip(direct, x):
        members[name] = float(t)
        if name == "pushrod":
            joint["PRO"] = t * e
            continue
        up_end = "RTO" if name == "tie_rod" else LEGS[name][0]
        ch_end = "RTI" if name == "tie_rod" else LEGS[name][1]
        joint[up_end] = joint[up_end] + t * e
        chassis[ch_end] = -t * e

    bending = 0.0
    if arm is not None:
        front_leg, rear_leg, bj_name = ARMS[arm]
        f_bj = x[j:j + 3]                     # force on the upright at the ball joint
        p = float(x[j + 3])                   # pushrod tension
        members["pushrod"] = p
        joint[bj_name] = f_bj
        # The arm's legs. Load on the arm's outer node = -f_bj from the upright
        # plus the pushrod pull, with the bracket treated as if it were at the
        # ball joint. Each leg in tension T pulls the node toward its pivot:
        #   node_load + T_f e_f + T_r e_r = 0     (least squares: 3 eqs, 2 unknowns)
        node = -f_bj + p * e_push
        e_f = _unit(pts[LEGS[front_leg][1]] - bj)
        e_r = _unit(pts[LEGS[rear_leg][1]] - bj)
        E = np.column_stack([e_f, e_r])
        t_legs, *_ = np.linalg.lstsq(E, -node, rcond=None)
        bending = float(np.linalg.norm(node + E @ t_legs))
        for name, t, e in ((front_leg, t_legs[0], e_f), (rear_leg, t_legs[1], e_r)):
            members[name] = float(t)
            chassis[LEGS[name][1]] = -t * e
    # The pushrod's inner end pushes on the rocker (or chassis) the opposite way.
    chassis["PRI" if hp.rocker is not None else "SHC"] = -members["pushrod"] * e_push

    lengths = {n: float(np.linalg.norm(pts[o] - pts[u])) for n, (u, o) in LEGS.items()}
    lengths["tie_rod"] = float(np.linalg.norm(pts["RTI"] - pts["RTO"]))
    lengths["pushrod"] = float(np.linalg.norm(pri - pro))
    ordered = {k: members[k] for k in MEMBER_GROUPS}
    return LinkForces(ordered, joint, chassis, lengths, bending, mount)


# ---------------------------------------------------------------------------
# 3. Tubes and rod ends
# ---------------------------------------------------------------------------

@dataclass
class Material:
    name: str = "AISI 4130 chromoly, normalised"
    yield_mpa: float = 435.0
    e_gpa: float = 205.0
    density_kg_m3: float = 7850.0


@dataclass
class Tube:
    """A round tube: outside diameter and wall thickness in mm."""

    od_mm: float
    wall_mm: float

    @property
    def area_mm2(self) -> float:
        ri = self.od_mm / 2 - self.wall_mm
        return float(np.pi * ((self.od_mm / 2) ** 2 - ri ** 2))

    @property
    def inertia_mm4(self) -> float:  # second moment of area, for buckling
        ri = self.od_mm / 2 - self.wall_mm
        return float(np.pi / 4 * ((self.od_mm / 2) ** 4 - ri ** 4))

    @property
    def gyration_mm(self) -> float:  # radius of gyration r = sqrt(I / A)
        return float(np.sqrt(self.inertia_mm4 / self.area_mm2))

    def mass_per_m(self, mat: Material) -> float:
        return self.area_mm2 * 1e-6 * mat.density_kg_m3  # kg per metre

    def __str__(self) -> str:
        return f"{self.od_mm:g} x {self.wall_mm:g}"


def tension_capacity(tube: Tube, mat: Material) -> float:
    """Pull (N) that yields the tube: yield stress x cross-section area."""
    return mat.yield_mpa * tube.area_mm2


def buckling_capacity(tube: Tube, mat: Material, length_mm: float, k: float = 1.0) -> float:
    """Push (N) that buckles the tube, Euler or Johnson.

    Slenderness = K L / r. Long, slender tubes buckle elastically (Euler):
    ``P = pi^2 E A / (K L / r)^2``. Short, stocky ones would come out of Euler
    stronger than the material can take, so below the crossover slenderness
    ``sqrt(2 pi^2 E / Sy)`` the Johnson parabola is used instead:
    ``P = A (Sy - (Sy K L / (2 pi r))^2 / E)``. Both meet at the crossover.
    """
    E = mat.e_gpa * 1000.0  # MPa
    sy = mat.yield_mpa
    slender = k * length_mm / tube.gyration_mm
    crossover = np.sqrt(2 * np.pi ** 2 * E / sy)
    if slender >= crossover:
        return float(np.pi ** 2 * E * tube.area_mm2 / slender ** 2)
    return float(tube.area_mm2 * (sy - (sy * slender / (2 * np.pi)) ** 2 / E))


@dataclass
class MemberSpec:
    tube: Tube
    rod_end: str
    rod_end_rating_n: float


@dataclass
class LoadsConfig:
    """Everything in ``data/load_cases.json``."""

    cases: list[LoadCase]
    required_sf: float = 1.5
    material: Material = field(default_factory=Material)
    k_factor: float = 1.0
    members: dict[str, MemberSpec] = field(default_factory=dict)
    ball_joint_rating_n: float = 25000.0
    catalogue: list[Tube] = field(default_factory=list)
    pneumatic_trail_mm: float = 0.0
    rocker_bearing_spacing_mm: float = 40.0

    @classmethod
    def load(cls, path: str | Path) -> "LoadsConfig":
        raw = json.loads(Path(path).read_text())
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: dict) -> "LoadsConfig":
        cases = [LoadCase(c["name"], float(c.get("lateral_g", 0.0)),
                          float(c.get("longitudinal_g", 0.0)), float(c.get("vertical_g", 1.0)),
                          c.get("_why", "")) for c in raw["cases"]]
        m = {k: v for k, v in raw.get("material", {}).items() if not k.startswith("_")}
        members = {}
        for g, spec in raw["members"].items():
            if g not in GROUP_LABELS:
                raise ValueError(f"unknown member group {g!r} (expected one of {list(GROUP_LABELS)})")
            members[g] = MemberSpec(Tube(float(spec["od_mm"]), float(spec["wall_mm"])),
                                    spec.get("rod_end", "rod end"), float(spec["rod_end_rating_n"]))
        missing = set(GROUP_LABELS) - set(members)
        if missing:
            raise ValueError(f"'members' is missing {sorted(missing)}")
        return cls(
            cases=cases,
            required_sf=float(raw.get("required_safety_factor", 1.5)),
            material=Material(**m) if m else Material(),
            k_factor=float(raw.get("effective_length_factor", 1.0)),
            members=members,
            ball_joint_rating_n=float(raw.get("ball_joint_rating_n", 25000.0)),
            catalogue=[Tube(float(od), float(w)) for od, w in raw.get("tube_catalogue", [])],
            pneumatic_trail_mm=float(raw.get("pneumatic_trail_mm", 0.0)),
            rocker_bearing_spacing_mm=float(raw.get("rocker_bearing_spacing_mm", 40.0)),
        )


# ---------------------------------------------------------------------------
# 4. Putting it together
# ---------------------------------------------------------------------------

@dataclass
class CornerResult:
    axle: str
    hp: Hardpoints
    tyres: list[TyreLoad]                 # one per load case variant
    forces: list[LinkForces]              # same order as tyres

    @property
    def labels(self) -> list[str]:
        return [t.label for t in self.tyres]

    def member_table(self) -> dict[str, np.ndarray]:
        """member -> force in each load case (N, + = tension)."""
        return {m: np.array([f.members[m] for f in self.forces]) for m in MEMBER_GROUPS}


@dataclass
class MemberCheck:
    """Worst case and safety factors for one link of one corner."""

    axle: str
    member: str
    group: str
    length_mm: float
    tube: Tube
    max_tension_n: float
    tension_case: str
    max_compression_n: float          # as a positive number
    compression_case: str
    sf_tension: float                 # yield / worst pull
    sf_buckling: float                # buckling load / worst push
    sf_rod_end: float                 # rod end rating / worst |force|

    @property
    def sf(self) -> float:
        return min(self.sf_tension, self.sf_buckling, self.sf_rod_end)


@dataclass
class LoadsResult:
    config: LoadsConfig
    corners: dict[str, CornerResult]
    checks: list[MemberCheck]
    ball_joints: dict[str, tuple[float, str, float]]  # "front UBJ" -> (max N, case, SF)
    lightest: dict[str, tuple[Tube | None, float]]    # group -> (lightest passing tube, its SF)
    notes: list[str]
    rockers: dict[str, list["RockerLoads"]] = field(default_factory=dict)  # axle -> per case
    rocker_static: dict[str, "RockerLoads"] = field(default_factory=dict)   # axle -> ride height
    bar_shares: dict[str, float] = field(default_factory=dict)             # axle -> bar roll share


def _sf(capacity: float, load: float) -> float:
    return float("inf") if load <= 1e-9 else capacity / load


def check_member(axle: str, member: str, length: float, forces: np.ndarray, labels: list[str],
                 spec: MemberSpec, cfg: LoadsConfig, tube: Tube | None = None) -> MemberCheck:
    """Safety factors of one link over all load cases."""
    tube = tube or spec.tube
    i_t, i_c = int(np.argmax(forces)), int(np.argmin(forces))
    t_max, c_max = max(forces[i_t], 0.0), max(-forces[i_c], 0.0)
    return MemberCheck(
        axle, member, MEMBER_GROUPS[member], length, tube,
        float(t_max), labels[i_t] if t_max > 0 else "-",
        float(c_max), labels[i_c] if c_max > 0 else "-",
        _sf(tension_capacity(tube, cfg.material), t_max),
        _sf(buckling_capacity(tube, cfg.material, length, cfg.k_factor), c_max),
        _sf(spec.rod_end_rating_n, float(np.max(np.abs(forces)))),
    )


def analyse_loads(car: CarResult, cfg: LoadsConfig) -> LoadsResult:
    """Every load case on the front-left and rear-left corners, then the sizing checks."""
    v = car.car
    n = car.numbers
    corners: dict[str, CornerResult] = {}
    for axle, hp in (("front", car.front.hp), ("rear", car.rear.hp)):
        corners[axle] = CornerResult(axle, hp, [], [])
    for case in cfg.cases:
        for t in tyre_loads(case, v, n["front_load_transfer_n_per_g"], n["rear_load_transfer_n_per_g"],
                            rear=car.rear.vehicle, pneumatic_trail_mm=cfg.pneumatic_trail_mm):
            c = corners[t.axle]
            c.tyres.append(t)
            c.forces.append(solve_link_forces(c.hp, t.applied(c.hp)))

    checks: list[MemberCheck] = []
    for axle, c in corners.items():
        table = c.member_table()
        for m, f in table.items():
            g = MEMBER_GROUPS[m]
            checks.append(check_member(axle, m, c.forces[0].lengths[m], f, c.labels,
                                       cfg.members[g], cfg))

    ball_joints = {}
    for axle, c in corners.items():
        for bj in ("UBJ", "LBJ"):
            mags = np.array([np.linalg.norm(f.joint_forces[bj]) for f in c.forces])
            i = int(np.argmax(mags))
            ball_joints[f"{axle} {bj}"] = (float(mags[i]), c.labels[i],
                                           _sf(cfg.ball_joint_rating_n, float(mags[i])))

    # Lightest catalogue tube per group that passes yield and buckling in every
    # member of that group on both corners (rod ends are sized separately).
    lightest: dict[str, tuple[Tube | None, float]] = {}
    for g in GROUP_LABELS:
        best = (None, 0.0)
        for tube in sorted(cfg.catalogue, key=lambda t: t.area_mm2):
            sfs = []
            for axle, c in corners.items():
                for m, f in c.member_table().items():
                    if MEMBER_GROUPS[m] == g:
                        mc = check_member(axle, m, c.forces[0].lengths[m], f, c.labels,
                                          cfg.members[g], cfg, tube)
                        sfs.append(min(mc.sf_tension, mc.sf_buckling))
            if min(sfs) >= cfg.required_sf:
                best = (tube, min(sfs))
                break
        lightest[g] = best

    notes = []
    for axle, c in corners.items():
        for t in c.tyres:
            if t.lifted:
                notes.append(f"{axle} {t.label}: this tyre lifts off (load set to 0)")
        worst = max(f.bracket_bending_force_n for f in c.forces)
        biggest = max(abs(x) for f in c.forces for x in f.members.values())
        notes.append(f"{axle}: pushrod bracket bending force up to {worst:.0f} N "
                     f"({100 * worst / biggest:.1f}% of the biggest link force)")
    brakes = [cs for cs in cfg.cases if cs.longitudinal_g < 0]
    if brakes:
        decel = max(-cs.longitudinal_g for cs in brakes)
        ideal = ideal_front_brake_bias(v, decel)
        notes.append(f"ideal front brake bias at {decel:g} g is {100 * ideal:.0f}% "
                     f"(the vehicle block has {100 * v.front_brake_bias:.0f}%)")
    # Phase 9: rocker, damper and bar drop link (corners with a rocker only).
    rockers, static, shares = {}, {}, {}
    for axle, c in corners.items():
        if c.hp.rocker is None:
            continue
        shares[axle] = bar_roll_share(car, axle)
        rockers[axle] = rocker_loads_for_corner(c, shares[axle], cfg.rocker_bearing_spacing_mm)
        static[axle] = static_rocker_loads(car, axle, cfg.rocker_bearing_spacing_mm)
        spring = getattr(car.front if axle == "front" else car.rear, "vehicle").spring_rate_n_per_mm
        i = int(np.argmin([r.damper_n for r in rockers[axle]]))
        if spring > 0:
            extra = (static[axle].damper_n - rockers[axle][i].damper_n) / spring
            notes.append(f"{axle}: biggest damper push {-rockers[axle][i].damper_n:.0f} N in "
                         f"{c.labels[i]}; on the {spring:g} N/mm spring alone that's {extra:.0f} mm "
                         f"of damper travel past ride height (a bump stop takes it if the damper "
                         f"runs out of stroke first)")
    return LoadsResult(cfg, corners, checks, ball_joints, lightest, notes, rockers, static, shares)


def upright_loads_dict(result: LoadsResult) -> dict:
    """Joint forces on each upright for every case, for an FEA check (e.g. Fusion).

    Forces are ON the upright, in N, in the corner's own axes (same as its
    hardpoints file), at the design position.
    """
    out = {
        "_note": ("Forces ON the upright (N) at each joint, per load case, from `suspkin loads`. "
                  "Axes and points as in the hardpoints file (mm, +X forward, +Y left/outboard, +Z up). "
                  "For a stress check: put these forces on the joint holes (UBJ, LBJ, RTO, and PRO "
                  "if the pushrod is on the upright) and fix the wheel bearing bore; the bore's "
                  "reaction is then the tyre load. Tyre forces are listed under 'tyre' for reference."),
        "units": {"force": "N", "length": "mm"},
        "corners": {},
    }
    for axle, c in result.corners.items():
        pts = {k: [round(float(x), 3) for x in v] for k, v in c.hp.upright.items()}
        cases = {}
        for t, f in zip(c.tyres, c.forces):
            cases[t.label] = {
                "tyre": {"fx_n": round(t.fx, 1), "fy_n": round(t.fy, 1), "fz_n": round(t.fz, 1),
                         "fx_acts_at": t.fx_at},
                "joint_forces_on_upright_n": {k: [round(float(x), 1) for x in v]
                                              for k, v in f.joint_forces.items()},
                "link_forces_n_tension_positive": {k: round(v, 1) for k, v in f.members.items()},
            }
        out["corners"][axle] = {"name": c.hp.name, "upright_points_mm": pts, "cases": cases}
        if axle in result.rockers:
            out["corners"][axle]["rocker"] = rocker_loads_dict(result, axle)
    return out


def rocker_loads_dict(result: LoadsResult, axle: str) -> dict:
    """Forces ON the rocker per case, for its FEA (see ``RockerLoads``)."""
    c = result.corners[axle]
    hp = c.hp
    pts = {**hp.rocker.points, "SHC": hp.chassis["SHC"]}
    if hp.arb is not None:
        pts.update(hp.arb.points)
    r2 = lambda v: [round(float(x), 1) for x in v]  # noqa: E731
    cases = {"static (ride height)": result.rocker_static[axle]}
    cases.update({lab: r for lab, r in zip(c.labels, result.rockers[axle])})
    return {
        "_note": ("Forces ON the rocker (N) at PRI (pushrod), SHR (damper), ARL (bar drop link) and "
                  "RKP (pivot reaction); they add up to zero. Damper / link / pushrod forces are "
                  "+ in tension. The bar's share of the roll part of the load is bar_roll_share."),
        "axis": [round(float(x), 4) for x in hp.rocker.axis],
        "points_mm": {k: [round(float(x), 3) for x in v] for k, v in pts.items()},
        "bar_roll_share": round(result.bar_shares[axle], 4),
        "bearing_spacing_mm": result.config.rocker_bearing_spacing_mm,
        "cases": {lab: {
            "pushrod_n": round(r.pushrod_n, 1), "damper_n": round(r.damper_n, 1),
            "arb_link_n": round(r.arb_link_n, 1), "bar_torque_nm": round(r.bar_torque_nm, 2),
            "forces_on_rocker_n": {k: r2(v) for k, v in r.forces_on_rocker.items()},
            "pivot_bearing_radial_n": [round(x, 1) for x in r.bearing_radial_n],
            "pivot_axial_n": round(r.pivot_axial_n, 1),
        } for lab, r in cases.items()},
    }


# ---------------------------------------------------------------------------
# 5. Phase 9: rocker, damper, anti-roll bar drop link and rocker pivot
# ---------------------------------------------------------------------------

@dataclass
class RockerLoads:
    """Forces on one rocker in one load case (left corner's axes, N and N mm).

    ``forces_on_rocker`` holds the force ON the rocker at each of its joints:
    PRI (pushrod), SHR (damper), ARL (bar drop link, if it's on the rocker) and
    RKP (the pivot's reaction, so all four add up to zero). Those are exactly
    what an FEA of the rocker needs: fix the pivot bore, load the other holes,
    or (better, it's a free body) apply all of them with inertia relief.
    """

    pushrod_n: float                     # + = tension (same as the link table)
    damper_n: float                      # + = tension; a loaded spring is in compression
    arb_link_n: float                    # + = tension
    bar_roll_share: float                # bar's share of the ROLL part (0 if no bar on the rocker)
    forces_on_rocker: dict[str, np.ndarray]
    pivot_moment_nmm: np.ndarray         # moment the pivot resists, perpendicular to its axis
    bearing_radial_n: tuple[float, float]  # the two pivot bearings, for a given spacing
    pivot_axial_n: float                 # thrust along the pivot axis
    bar_torque_nm: float                 # torque in the anti-roll bar (this side's arm)

    @property
    def pivot_force_n(self) -> float:
        return float(np.linalg.norm(self.forces_on_rocker["RKP"]))


def _moment_about_axis(point: np.ndarray, force_dir: np.ndarray, origin: np.ndarray,
                       axis: np.ndarray) -> float:
    """Moment about a fixed axis of a unit force at ``point`` (N mm per N)."""
    return float(axis @ np.cross(point - origin, force_dir))


def solve_rocker_loads(hp: Hardpoints, pushrod_heave_n: float, pushrod_roll_n: float,
                       bar_roll_share: float, bearing_spacing_mm: float = 40.0) -> RockerLoads:
    """Damper, drop link and pivot loads on the rocker (design position).

    ``pushrod_heave_n + pushrod_roll_n`` is the pushrod force (+ = tension) in
    this case; ``bar_roll_share`` is the bar's fraction of the axle's elastic
    roll stiffness (0 = no bar). The rocker's moment balance about its axis::

        M_pushrod + D m_damper + Q m_link = 0

    with ``D`` taking the heave part plus ``(1 - share)`` of the roll part and
    ``Q`` taking ``share`` of the roll part.

    Then the pivot: it carries everything else. Its force is minus the sum of
    the three joint forces, and the moment of those forces about RKP (the part
    across the axis; the part along it is zero by construction) is split
    between two bearings ``bearing_spacing_mm`` apart, one either side of RKP.
    """
    rk = hp.rocker
    if rk is None:
        raise ValueError("this corner has no rocker block")
    a, rkp = rk.axis, rk.points["RKP"]
    pts = rk.points
    shc = hp.chassis["SHC"]
    e_push = _unit(pts["PRI"] - pts["PRO"])   # pushrod tension pulls PRI toward PRO: -P e_push
    e_damp = _unit(shc - pts["SHR"])          # damper tension pulls SHR toward SHC: +D e_damp

    p_total = pushrod_heave_n + pushrod_roll_n
    m_push = _moment_about_axis(pts["PRI"], -e_push, rkp, a)
    m_damp = _moment_about_axis(pts["SHR"], e_damp, rkp, a)

    bar_on_rocker = hp.arb is not None and hp.arb.link_mount == "rocker"
    share = bar_roll_share if bar_on_rocker else 0.0
    # Moment the spring and the bar must each supply (opposite to the pushrod's).
    m_spring = -m_push * (pushrod_heave_n + (1.0 - share) * pushrod_roll_n)
    m_bar = -m_push * share * pushrod_roll_n
    damper = m_spring / m_damp

    forces = {"PRI": -p_total * e_push, "SHR": damper * e_damp}
    link, bar_torque = 0.0, 0.0
    if bar_on_rocker:
        arl, ara, arp = (hp.arb.points[k] for k in ("ARL", "ARA", "ARP"))
        e_link = _unit(ara - arl)             # drop link tension pulls ARL toward ARA
        m_link = _moment_about_axis(arl, e_link, rkp, a)
        link = m_bar / m_link if abs(m_link) > 1e-9 else 0.0
        forces["ARL"] = link * e_link
        # The same link pulls ARA toward ARL: that's the torque twisting the bar.
        bar_axis = np.array([0.0, 1.0, 0.0])
        bar_torque = link * _moment_about_axis(ara, -e_link, arp, bar_axis) / 1000.0

    total_f = sum(forces.values())
    where = {**pts, **(hp.arb.points if hp.arb else {})}
    total_m = sum(np.cross(where[k] - rkp, f) for k, f in forces.items())
    forces["RKP"] = -total_f
    m_perp = total_m - (total_m @ a) * a      # the pivot must resist -m_perp
    # Bearings at RKP -/+ (s/2) a carry F1, F2 (across the axis):
    #   F1 + F2 = -total_f(perp),   (s/2) a x (F2 - F1) = -m_perp
    f_perp = -(total_f - (total_f @ a) * a)
    diff = np.cross(-2.0 * m_perp / bearing_spacing_mm, a)  # F2 - F1
    f1, f2 = (f_perp - diff) / 2.0, (f_perp + diff) / 2.0
    return RockerLoads(p_total, float(damper), float(link), share, forces, -m_perp, (float(np.linalg.norm(f1)), float(np.linalg.norm(f2))),
                       float(-(total_f @ a)), float(bar_torque))


def bar_roll_share(car: CarResult, axle: str) -> float:
    """The anti-roll bar's share of one axle's elastic roll stiffness (0-1)."""
    n = car.numbers
    k_s = n[f"{axle}_roll_stiffness_springs_nm_per_deg"]
    k_b = n[f"{axle}_roll_stiffness_bar_nm_per_deg"]
    return float(k_b / (k_s + k_b)) if k_s + k_b > 0 else 0.0


def rocker_loads_for_corner(corner: "CornerResult", share: float,
                            bearing_spacing_mm: float = 40.0) -> list[RockerLoads]:
    """Rocker loads for every case of one corner (same order as ``corner.tyres``).

    The inside and outside wheel of the same cornering case are the two wheels
    of one axle in one turn (the right-hand one mirrored), so their pushrod
    forces split into heave = average and roll = half the difference.
    """
    if corner.hp.rocker is None:
        return []
    by_case: dict[str, dict[str, float]] = {}
    for t, f in zip(corner.tyres, corner.forces):
        by_case.setdefault(t.case, {})[t.side] = f.members["pushrod"]
    out = []
    for t, f in zip(corner.tyres, corner.forces):
        sides = by_case[t.case]
        p = f.members["pushrod"]
        if t.side in ("inside", "outside") and len(sides) == 2:
            heave = (sides["inside"] + sides["outside"]) / 2.0
        else:
            heave = p
        out.append(solve_rocker_loads(corner.hp, heave, p - heave, share, bearing_spacing_mm))
    return out


def static_rocker_loads(car: CarResult, axle: str, bearing_spacing_mm: float = 40.0) -> RockerLoads:
    """Rocker loads with the car just sitting at ride height (1 g, no cornering)."""
    hp = car.front.hp if axle == "front" else car.rear.hp
    n = car.numbers
    t = [t for t in tyre_loads(LoadCase("static"), car.car, n["front_load_transfer_n_per_g"],
                               n["rear_load_transfer_n_per_g"], rear=car.rear.vehicle)
         if t.axle == axle][0]
    p = solve_link_forces(hp, t.applied(hp)).members["pushrod"]
    return solve_rocker_loads(hp, p, 0.0, 0.0, bearing_spacing_mm)
