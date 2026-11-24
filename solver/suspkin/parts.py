"""Phase 9: upright, rocker and pushrod bracket, built around the hardpoints.

What this module does
---------------------
1. **Geometry.** Each part is made of simple solids: cylinders (bosses,
   housings, holes) and boxes (the arms joining them). They're placed straight
   from the hardpoints, so if a hardpoint moves, the part follows. The same
   list of solids is used three ways:

   * the Fusion script ``cad/fusion/SuspensionParts`` builds them as real
     Fusion bodies (then you add fillets and tidy up by hand),
   * ``suspkin parts --step`` writes STEP files with gmsh, for Ansys or
     Fusion import (see ``fea.py``),
   * ``suspkin parts --fea`` meshes and solves them right here as a quick
     cross-check before the "real" Fusion / Ansys study.

2. **Loads.** From ``suspkin loads`` (``loads.py``): the force at every joint
   hole of the upright, the rocker and the pushrod bracket, per load case,
   plus the brake caliper's reaction on the upright (below).

3. **Hand calculations.** Every arm is treated as a short cantilever beam
   from its root (where it meets the housing / pivot boss) to its hole, and
   every hole as a lug (bearing pressure and tear-out). That's how you'd size
   the parts on paper; the FEA then shows the stress raisers (holes, corners)
   the beam formulas can't.

Sizes come from ``data/parts.json``. Axes, units and points are the corner's
hardpoint file's (mm, N, SAE: +X forward, +Y left = outboard, +Z up).

How the upright is held in a stress study
-----------------------------------------
The usual (and simplest) setup: **fix the bearing bore** and **load the joint
holes** (UBJ, LBJ, steering arm) with the forces from the link solve. The bore
then reacts exactly the tyre load, because the joint forces balance it.

One catch: the wheel bearing can't carry torque about the axle, so under
braking the brake torque goes through the **caliper lugs**, not the bore.
So in braking cases the caliper lugs get a force too: the pads hold the disc
with a tangential force ``F_pad`` at the pad radius such that its moment about
the axle cancels the tyre's (road force at the contact patch). The upright
feels ``-F_pad``, split over the two lugs. With the bore fixed, the bore then
takes everything else, which is exactly the bearing load.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .hardpoints import Hardpoints
from .kinematics import DoubleWishbone
from .loads import LoadsResult


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _perp(v: np.ndarray, to: np.ndarray) -> np.ndarray:
    """``v`` with its component along ``to`` removed, normalised."""
    to = _unit(to)
    return _unit(v - (v @ to) * to)


# ---------------------------------------------------------------------------
# 1. Building blocks
# ---------------------------------------------------------------------------

@dataclass
class Cylinder:
    """Solid cylinder from ``p1`` to ``p2`` (centres of its end faces)."""

    name: str
    p1: np.ndarray
    p2: np.ndarray
    radius: float

    @property
    def axis(self) -> np.ndarray:
        return _unit(self.p2 - self.p1)

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.p2 - self.p1))

    def to_dict(self) -> dict:
        return {"type": "cylinder", "name": self.name, "p1": _r(self.p1), "p2": _r(self.p2),
                "radius": round(self.radius, 4)}

    def sample(self, n: int = 24) -> np.ndarray:
        """Points round both end-face rims (for clearance checks)."""
        a = self.axis
        u = _perp(np.array([1.0, 0.3, 0.2]), a)
        w = np.cross(a, u)
        t = np.linspace(0, 2 * np.pi, n, endpoint=False)
        ring = self.radius * (np.outer(np.cos(t), u) + np.outer(np.sin(t), w))
        return np.vstack([self.p1 + ring, self.p2 + ring])


@dataclass
class Box:
    """Solid box: centre, a length direction and a width direction.

    The third (``height``) direction is length x width.
    """

    name: str
    center: np.ndarray
    length_dir: np.ndarray
    width_dir: np.ndarray
    length: float
    width: float
    height: float

    def __post_init__(self):
        self.length_dir = _unit(self.length_dir)
        self.width_dir = _perp(self.width_dir, self.length_dir)

    @property
    def height_dir(self) -> np.ndarray:
        return np.cross(self.length_dir, self.width_dir)

    def to_dict(self) -> dict:
        return {"type": "box", "name": self.name, "center": _r(self.center),
                "length_dir": _r(self.length_dir, 6), "width_dir": _r(self.width_dir, 6),
                "length": round(self.length, 4), "width": round(self.width, 4),
                "height": round(self.height, 4)}

    def sample(self) -> np.ndarray:
        """The 8 corners."""
        out = []
        for i in (-0.5, 0.5):
            for j in (-0.5, 0.5):
                for k in (-0.5, 0.5):
                    out.append(self.center + i * self.length * self.length_dir
                               + j * self.width * self.width_dir + k * self.height * self.height_dir)
        return np.array(out)


def _r(v, nd: int = 4) -> list[float]:
    return [round(float(x), nd) for x in v]


@dataclass
class Hole:
    """A loaded or supported hole: where a force goes in or the part is held.

    The FEA finds the hole's surface as the mesh faces at ``radius`` from the
    axis line, between ``p1`` and ``p2``.
    """

    name: str
    p1: np.ndarray
    p2: np.ndarray
    radius: float
    description: str = ""

    @property
    def center(self) -> np.ndarray:
        return (self.p1 + self.p2) / 2

    def to_dict(self) -> dict:
        return {"name": self.name, "p1": _r(self.p1), "p2": _r(self.p2),
                "radius": round(self.radius, 4), "description": self.description}


@dataclass
class Part:
    """One part: solids added together, then holes cut out."""

    name: str
    material: str
    solids: list            # Cylinder / Box, unioned
    cuts: list[Cylinder]    # subtracted after the union
    holes: dict[str, Hole]  # named holes for loads / supports
    support: str            # the hole (or "faces:...") that's held fixed in a stress study
    notes: list[str] = field(default_factory=list)
    flat_supports: list[dict] = field(default_factory=list)  # planes, e.g. weld faces

    def to_dict(self) -> dict:
        return {"name": self.name, "material": self.material,
                "solids": [s.to_dict() for s in self.solids],
                "cuts": [c.to_dict() for c in self.cuts],
                "holes": {k: h.to_dict() for k, h in self.holes.items()},
                "support": self.support, "flat_supports": self.flat_supports,
                "notes": self.notes}


@dataclass
class PartsConfig:
    """Everything in ``data/parts.json`` (kept as plain dicts: it's mostly sizes)."""

    raw: dict

    @classmethod
    def load(cls, path: str | Path) -> "PartsConfig":
        return cls(json.loads(Path(path).read_text()))

    @property
    def required_sf(self) -> float:
        return float(self.raw.get("required_safety_factor", 2.0))

    def material(self, name: str) -> dict:
        return self.raw["materials"][name]

    def __getitem__(self, key):
        return self.raw[key]


# ---------------------------------------------------------------------------
# 2. Geometry
# ---------------------------------------------------------------------------

def wheel_frame(hp: Hardpoints) -> dict[str, np.ndarray]:
    """Wheel centre, spin axis (outboard) and two in-plane directions.

    ``fwd`` is forward in the wheel plane, ``up`` is up in it.
    """
    pose = DoubleWishbone(hp).solve(0.0)
    a = _unit(pose.wheel_axis)
    fwd = _perp(np.array([1.0, 0.0, 0.0]), a)
    up = np.cross(fwd, a)
    if up[2] < 0:
        up = -up
    return {"WC": hp.upright["WC"].copy(), "axis": a, "fwd": fwd, "up": up}


def _hub_face_axial(hp: Hardpoints) -> float:
    return float(hp.extra.get("packaging", {}).get("hub_face_axial_mm", -20.0))


def build_upright(hp: Hardpoints, cfg: PartsConfig, axle: str) -> Part:
    """Bearing housing on the spin axis, bosses at UBJ / LBJ / steering arm / caliper, arms between."""
    u = cfg["upright"]
    wf = wheel_frame(hp)
    wc, a = wf["WC"], wf["axis"]
    b = u["bearing"]
    r_bore = b["bore_mm"] / 2
    r_house = r_bore + b["housing_wall_mm"]
    outer = _hub_face_axial(hp) - b["gap_to_hub_face_mm"]       # axial position of outboard end
    inner = outer - b["housing_length_mm"]
    h1, h2 = wc + inner * a, wc + outer * a
    solids: list = [Cylinder("bearing housing", h1, h2, r_house)]
    cuts = [Cylinder("bearing bore", h1 - 1 * a, h2 + 1 * a, r_bore)]
    holes = {"bore": Hole("bore", h1, h2, r_bore, b["name"])}

    def foot(p: np.ndarray) -> np.ndarray:
        """Point on the axle line level with p, kept within the housing's length."""
        s = float(np.clip((p - wc) @ a, inner, outer))
        return wc + s * a

    def arm(name: str, target: np.ndarray, width: float, depth: float, label: str) -> Box:
        f = foot(target)
        length_dir = _unit(target - f)
        depth_dir = _perp(a, length_dir)
        width_dir = np.cross(depth_dir, length_dir)
        length = float(np.linalg.norm(target - f))
        return Box(label, (f + target) / 2, length_dir, width_dir, length, width, depth)

    def root_web(target: np.ndarray, dims: dict, label: str) -> Box | None:
        """Optional wider block where an arm meets the housing (a crude taper).

        Runs from the axle line out to ``root_length_mm`` beyond the housing's
        surface, ``root_width_mm`` wide and as deep as the arm.
        """
        if "root_width_mm" not in dims:
            return None
        f = foot(target)
        length_dir = _unit(target - f)
        depth_dir = _perp(a, length_dir)
        width_dir = np.cross(depth_dir, length_dir)
        length = r_house + dims.get("root_length_mm", 15.0)
        return Box(label, f + length / 2 * length_dir, length_dir, width_dir, length,
                   dims["root_width_mm"], dims["depth_mm"])

    x = np.array([1.0, 0.0, 0.0])
    for bj in ("UBJ", "LBJ"):
        s = u["ball_joints"][bj]
        p = hp.upright[bj]
        half = s["width_mm"] / 2
        solids.append(Cylinder(f"{bj} boss", p - half * x, p + half * x, s["boss_od_mm"] / 2))
        cuts.append(Cylinder(f"{bj} bearing seat", p - (half + 1) * x, p + (half + 1) * x, s["bore_mm"] / 2))
        holes[bj] = Hole(bj, p - half * x, p + half * x, s["bore_mm"] / 2, s["name"])
        dims = u["arms"][bj]
        solids.append(arm(bj, p, dims["width_mm"], dims["depth_mm"], f"{bj} arm"))
        web = root_web(p, dims, f"{bj} arm root")
        if web is not None:
            solids.append(web)

    # Steering arm (front) / toe link arm (rear): bolt roughly vertical.
    st = u["steering_arm"]
    p = hp.upright["RTO"]
    z = np.array([0.0, 0.0, 1.0])
    hh = st["boss_height_mm"] / 2
    label = "steering arm" if axle == "front" else "toe link arm"
    solids.append(Cylinder(f"{label} boss", p - hh * z, p + hh * z, st["boss_od_mm"] / 2))
    cuts.append(Cylinder(f"{label} bolt hole", p - (hh + 1) * z, p + (hh + 1) * z, st["bolt_mm"] / 2))
    holes["RTO"] = Hole("RTO", p - hh * z, p + hh * z, st["bolt_mm"] / 2, f"{label} rod end bolt")
    dims = u["arms"]["RTO"]
    solids.append(arm("RTO", p, dims["width_mm"], dims["depth_mm"], label))
    web = root_web(p, dims, f"{label} root")
    if web is not None:
        solids.append(web)

    # Caliper lugs: bolts parallel to the axle, at the outboard end of the housing.
    c = u["caliper"]
    half_angle = np.arcsin(c["bolt_spacing_mm"] / 2 / c["bolt_radius_mm"])
    for k, sign in (("CAL1", -1), ("CAL2", 1)):
        th = np.radians(c["position_deg"]) + sign * half_angle
        radial = np.cos(th) * wf["fwd"] + np.sin(th) * wf["up"]
        centre_plane = wc + c["bolt_radius_mm"] * radial
        p1, p2 = centre_plane + (outer - c["boss_length_mm"]) * a, centre_plane + outer * a
        solids.append(Cylinder(f"caliper lug {k[-1]}", p1, p2, c["boss_od_mm"] / 2))
        cuts.append(Cylinder(f"caliper bolt {k[-1]}", p1 - a, p2 + a, c["bolt_mm"] / 2))
        holes[k] = Hole(k, p1, p2, c["bolt_mm"] / 2, "brake caliper bolt")
        dims = u["arms"]["caliper"]
        mid = (p1 + p2) / 2
        bx = arm(k, mid, dims["width_mm"], dims["depth_mm"], f"caliper arm {k[-1]}")
        # Keep the caliper arm's depth inside the lug's length.
        bx.height = min(bx.height, c["boss_length_mm"])
        solids.append(bx)

    return Part(f"{axle} upright", u["material"], solids, cuts, holes, support="bore",
                notes=["Fillet every arm-to-housing and arm-to-boss edge (3-5 mm) before "
                       "the FEA: sharp inside corners are stress raisers.",
                       "Bearing shoulders / circlip grooves and the spacer are not modelled."])


def build_rocker(hp: Hardpoints, cfg: PartsConfig, axle: str) -> Part:
    """Two plates (double shear) joined by the pivot tube."""
    r = cfg["rocker"]
    rk = hp.rocker
    a, rkp = rk.axis, rk.points["RKP"]
    t, gap = r["plate_thickness_mm"], r["plate_gap_mm"]
    piv = r["pivot"]
    span = gap / 2 + t                     # pivot tube runs from -span to +span along the axis
    solids: list = [Cylinder("pivot tube", rkp - span * a, rkp + span * a, piv["boss_od_mm"] / 2)]
    cuts = [Cylinder("pivot bore", rkp - (span + 1) * a, rkp + (span + 1) * a, piv["bore_mm"] / 2)]
    holes = {"RKP": Hole("RKP", rkp - span * a, rkp + span * a, piv["bore_mm"] / 2, piv["name"])}
    pts = {"PRI": rk.points["PRI"], "SHR": rk.points["SHR"]}
    if hp.arb is not None and hp.arb.link_mount == "rocker":
        pts["ARL"] = hp.arb.points["ARL"]
    labels = {"PRI": "pushrod", "SHR": "damper", "ARL": "ARB drop link"}
    hb = r["holes"]
    notes = []
    for side, s in (("front", 1), ("rear", -1)):
        off = s * (gap / 2 + t / 2) * a    # plate mid-plane offset from the hardpoint plane
        for k, p in pts.items():
            along = (p - rkp) @ a
            if abs(along) > gap / 2 and side == "front":
                notes.append(f"{k} is {along:.1f} mm off the rocker's mid-plane: the plates "
                             f"are centred on RKP, so check the rod end fits.")
            q = p - along * a              # into RKP's plane
            solids.append(Cylinder(f"{labels[k]} boss ({side} plate)", q + off - t / 2 * a,
                                   q + off + t / 2 * a, hb["boss_od_mm"] / 2))
            length_dir = _unit(q - rkp)
            length = float(np.linalg.norm(q - rkp))
            solids.append(Box(f"{labels[k]} arm ({side} plate)", (q + rkp) / 2 + off, length_dir,
                              np.cross(a, length_dir), length, r["arm_width_mm"], t))
    for k, p in pts.items():
        q = p - ((p - rkp) @ a) * a
        cuts.append(Cylinder(f"{labels[k]} bolt hole", q - (span + 1) * a, q + (span + 1) * a,
                             hb["bolt_mm"] / 2))
        holes[k] = Hole(k, q - span * a, q + span * a, hb["bolt_mm"] / 2, f"{labels[k]} bolt")
    return Part(f"{axle} rocker", r["material"], solids, cuts, holes, support="RKP",
                notes=notes + ["Round the plate outline (convex hull of the bosses) in CAD if "
                               "you want the classic rocker look; the arms are the load paths."])


def pushrod_bolt_axis(hp: Hardpoints) -> np.ndarray:
    """The pushrod bracket's bolt axis: along the wishbone's hinge (it swings about it)."""
    mount = hp.rocker.mount
    f, r_ = ("LAF", "LAA") if mount == "lower_arm" else ("UAF", "UAA")
    return _unit(hp.chassis[f] - hp.chassis[r_])


def build_bracket(hp: Hardpoints, cfg: PartsConfig, axle: str) -> Part:
    """Two welded tabs on the wishbone carrying the pushrod's outer rod end."""
    br = cfg["pushrod_bracket"]
    pro = hp.rocker.points["PRO"]
    a = pushrod_bolt_axis(hp)
    down = _perp(np.array([0.0, 0.0, -1.0]), a)
    t, gap, e, h = br["tab_thickness_mm"], br["tab_gap_mm"], br["edge_distance_mm"], br["tab_height_below_hole_mm"]
    solids: list = []
    flats = []
    for side, s in (("front", 1), ("rear", -1)):
        off = s * (gap / 2 + t / 2) * a
        solids.append(Cylinder(f"tab end ({side})", pro + off - t / 2 * a, pro + off + t / 2 * a, e))
        solids.append(Box(f"tab ({side})", pro + off + h / 2 * down, down, np.cross(a, down), h, 2 * e, t))
        bottom = pro + off + h * down
        flats.append({"name": f"weld ({side} tab)", "point": _r(bottom), "normal": _r(down, 6)})
    span = gap / 2 + t
    cuts = [Cylinder("pushrod bolt hole", pro - (span + 1) * a, pro + (span + 1) * a, br["bolt_mm"] / 2)]
    holes = {"PRO": Hole("PRO", pro - span * a, pro + span * a, br["bolt_mm"] / 2, "pushrod rod end bolt")}
    return Part(f"{axle} pushrod bracket", br["material"], solids, cuts, holes, support="weld",
                flat_supports=flats,
                notes=["The tabs' bottom edges are welded to the wishbone tubes / ball joint "
                       "housing: held fixed in the stress study. The weld itself isn't checked."])


def build_parts(hp: Hardpoints, cfg: PartsConfig, axle: str) -> dict[str, Part]:
    out = {"upright": build_upright(hp, cfg, axle)}
    if hp.rocker is not None:
        out["rocker"] = build_rocker(hp, cfg, axle)
        if hp.rocker.mount in ("lower_arm", "upper_arm"):
            out["pushrod_bracket"] = build_bracket(hp, cfg, axle)
    return out


def rim_clearance(hp: Hardpoints, part: Part) -> dict:
    """How close the upright's solids come to the rim (radially, inside the rim's width).

    Checks the rims of every cylinder and the corners of every box against the
    rim envelope from the hardpoints file. Negative = the part is in the rim.
    """
    pk = hp.extra.get("packaging", {})
    r_rim = float(pk.get("rim_inner_radius_mm", 117.0))
    half = float(pk.get("rim_half_width_mm", 89.0))
    wf = wheel_frame(hp)
    worst = (np.inf, "")
    for s in part.solids:
        pts = s.sample()
        rel = pts - wf["WC"]
        ax = rel @ wf["axis"]
        rad = np.linalg.norm(rel - np.outer(ax, wf["axis"]), axis=1)
        inside = np.abs(ax) <= half
        if inside.any():
            gap = float(np.min(r_rim - rad[inside]))
            if gap < worst[0]:
                worst = (gap, s.name)
    return {"rim_inner_radius_mm": r_rim, "min_gap_mm": worst[0], "closest": worst[1]}


# ---------------------------------------------------------------------------
# 3. Loads on each part
# ---------------------------------------------------------------------------

def caliper_forces(hp: Hardpoints, part: Part, cfg: PartsConfig, tyre) -> dict[str, np.ndarray]:
    """Force on each caliper lug of the upright from braking (see module docstring).

    The pads grip the disc at the pad radius, halfway between the two lugs.
    The caliper is a rigid bracket, so its two bolts share that pad force the
    way a bolt group does: each takes half of it directly, plus an equal and
    opposite pair from the moment of the pad force about the bolts' midpoint.
    The lug forces on the upright then have exactly the brake torque about the
    axle, and the net pad force.
    """
    if tyre.fx >= 0 or tyre.fx_at != "CP":
        return {}
    wf = wheel_frame(hp)
    wc, a = wf["WC"], wf["axis"]
    torque = sum(a @ np.cross(p - wc, f) for p, f in tyre.applied(hp))  # tyre's moment about the axle
    l1, l2 = part.holes["CAL1"].center, part.holes["CAL2"].center
    mid = (l1 + l2) / 2
    radial = _perp(mid - wc, a)
    r_pad = cfg["upright"]["caliper"]["pad_radius_mm"]
    pad = wc + ((mid - wc) @ a) * a + r_pad * radial      # where the pads grip the disc
    tangent = np.cross(a, radial)
    f_pad = -torque / r_pad * tangent                     # disc's force from the pads: cancels the tyre's moment
    # The caliper feels -f_pad from the disc at `pad`; its bolts hold it.
    f_cal = -f_pad
    m = a @ np.cross(pad - mid, f_cal)                    # moment about the bolts' midpoint
    out = {}
    for name, lug in (("CAL1", l1), ("CAL2", l2)):
        d = lug - mid
        d -= (d @ a) * a
        # Bolt force on the caliper: -(f_cal/2) - moment share; the upright gets the opposite.
        on_caliper = -f_cal / 2 - m / (2 * (d @ d)) * np.cross(a, d)
        out[name] = -on_caliper
    return out


@dataclass
class PartLoadCase:
    label: str
    forces: dict[str, np.ndarray]           # hole name -> force ON the part, N


def part_load_cases(result: LoadsResult, axle: str, parts: dict[str, Part],
                    cfg: PartsConfig) -> dict[str, list[PartLoadCase]]:
    """Per part, the forces on its holes for every load case."""
    c = result.corners[axle]
    hp = c.hp
    out: dict[str, list[PartLoadCase]] = {k: [] for k in parts}
    for i, (t, lf) in enumerate(zip(c.tyres, c.forces)):
        up = {k: lf.joint_forces[k] for k in ("UBJ", "LBJ", "RTO")}
        up.update(caliper_forces(hp, parts["upright"], cfg, t))
        out["upright"].append(PartLoadCase(t.label, up))
        if "rocker" in parts:
            rl = result.rockers[axle][i]
            out["rocker"].append(PartLoadCase(t.label, {k: v for k, v in rl.forces_on_rocker.items()
                                                        if k != "RKP"}))
        if "pushrod_bracket" in parts:
            rk = hp.rocker.points
            e_push = _unit(rk["PRI"] - rk["PRO"])
            out["pushrod_bracket"].append(PartLoadCase(t.label, {"PRO": lf.members["pushrod"] * e_push}))
    return out


# ---------------------------------------------------------------------------
# 4. Hand calculations
# ---------------------------------------------------------------------------

@dataclass
class HandCheck:
    part: str
    item: str
    stress_mpa: float
    case: str
    sf: float
    how: str


def rect_section_stress(force: np.ndarray, lever: np.ndarray, l_dir: np.ndarray, w_dir: np.ndarray,
                        h_dir: np.ndarray, w: float, h: float) -> float:
    """Von Mises stress at the root of a rectangular cantilever (conservative).

    ``force`` acts at the tip, ``lever`` = tip - root. Section: ``w`` along
    ``w_dir``, ``h`` along ``h_dir``; the beam runs along ``l_dir``.

    * axial: N / (w h)
    * bending about w_dir (stress varies across h): M / (w h^2 / 6), and vice versa
    * torsion: T / (alpha b c^2), b >= c, alpha = 1 / (3 + 1.8 c / b) (Roark)

    The normal stresses are added as if they peak at the same corner, and
    combined with the peak torsion shear: sqrt(sigma^2 + 3 tau^2).
    """
    m = np.cross(lever, force)
    n = force @ l_dir
    sigma = abs(n) / (w * h) + abs(m @ w_dir) / (w * h ** 2 / 6) + abs(m @ h_dir) / (h * w ** 2 / 6)
    b_, c_ = max(w, h), min(w, h)
    tau = abs(m @ l_dir) / ((1 / (3 + 1.8 * c_ / b_)) * b_ * c_ ** 2)
    return float(np.sqrt(sigma ** 2 + 3 * tau ** 2))


def lug_stresses(force: float, bore: float, outer_r: float, thickness: float) -> tuple[float, float]:
    """(bearing pressure, net-section tension) for a pin / bearing in a round boss.

    bearing pressure = F / (d t); net section = F / (2 t (R - d/2)): the two
    ligaments either side of the hole carry the load in tension.
    """
    return force / (bore * thickness), force / (2 * thickness * (outer_r - bore / 2))


def _worst(values: list[float], labels: list[str]) -> tuple[float, str]:
    i = int(np.argmax(values))
    return float(values[i]), labels[i]


def hand_checks(hp: Hardpoints, parts: dict[str, Part], loads: dict[str, list[PartLoadCase]],
                result: LoadsResult, cfg: PartsConfig) -> list[HandCheck]:
    out: list[HandCheck] = []
    # --- upright arms and bosses -------------------------------------------
    up = parts["upright"]
    sy = cfg.material(up.material)["yield_mpa"]
    labels = [lc.label for lc in loads["upright"]]
    housing = up.solids[0]
    wf = wheel_frame(hp)
    arms = {s.name: s for s in up.solids if isinstance(s, Box)}
    rto = "steering arm" if any(x.name.startswith("steering arm") for x in up.solids) else "toe link arm"
    arm_for = {"UBJ": "UBJ arm", "LBJ": "LBJ arm", "RTO": rto,
               "CAL1": "caliper arm 1", "CAL2": "caliper arm 2"}
    for hole, arm_name in arm_for.items():
        bx = arms[arm_name]
        tip = up.holes[hole].center
        # Root: where the arm leaves the housing's outer surface.
        rel = tip - wf["WC"]
        along = rel @ wf["axis"]
        radial = rel - along * wf["axis"]
        root = wf["WC"] + along * wf["axis"] + housing.radius * _unit(radial)
        vals = [rect_section_stress(lc.forces.get(hole, np.zeros(3)), tip - root, bx.length_dir,
                                    bx.width_dir, bx.height_dir, bx.width, bx.height)
                for lc in loads["upright"]]
        s, case = _worst(vals, labels)
        out.append(HandCheck(up.name, f"{arm_name} at the housing", s, case, sy / s if s else np.inf,
                             f"cantilever {np.linalg.norm(tip - root):.0f} mm long, "
                             f"{bx.width:g} x {bx.height:g} mm section"))
    for hole in ("UBJ", "LBJ", "RTO"):
        boss_name = f"{hole} boss" if hole != "RTO" else f"{rto} boss"
        boss = next(s for s in up.solids if s.name == boss_name)
        h = up.holes[hole]
        mags = [float(np.linalg.norm(lc.forces[hole])) for lc in loads["upright"]]
        f, case = _worst(mags, labels)
        bear, net = lug_stresses(f, 2 * h.radius, boss.radius, boss.length)
        out.append(HandCheck(up.name, f"{hole} boss (net section)", net, case, sy / net,
                             f"{f:.0f} N, bore {2 * h.radius:.1f}, boss OD {2 * boss.radius:g}, "
                             f"{boss.length:g} thick; bearing pressure {bear:.0f} MPa"))

    # --- rocker ------------------------------------------------------------
    if "rocker" in parts:
        rp = parts["rocker"]
        rc = cfg["rocker"]
        sy = cfg.material(rp.material)["yield_mpa"]
        t, b = rc["plate_thickness_mm"], rc["arm_width_mm"]
        rk = hp.rocker
        a, rkp = rk.axis, rk.points["RKP"]
        r_piv = rc["pivot"]["boss_od_mm"] / 2
        labels = [lc.label for lc in loads["rocker"]]
        for k in [h for h in rp.holes if h != "RKP"]:
            tip = rp.holes[k].center
            l_dir = _unit(tip - rkp - ((tip - rkp) @ a) * a)
            root = rkp + r_piv * l_dir
            # Each plate takes half of the force.
            vals = [rect_section_stress(lc.forces.get(k, np.zeros(3)) / 2, tip - root, l_dir,
                                        np.cross(a, l_dir), a, b, t) for lc in loads["rocker"]]
            s, case = _worst(vals, labels)
            out.append(HandCheck(rp.name, f"{k} arm at the pivot boss (per plate)", s, case,
                                 sy / s if s else np.inf,
                                 f"cantilever {np.linalg.norm(tip - root):.0f} mm, {b:g} x {t:g} mm plate"))
            mags = [float(np.linalg.norm(lc.forces.get(k, np.zeros(3)))) / 2 for lc in loads["rocker"]]
            f, case = _worst(mags, labels)
            if f > 0:
                bear, net = lug_stresses(f, rc["holes"]["bolt_mm"], rc["holes"]["boss_od_mm"] / 2, t)
                out.append(HandCheck(rp.name, f"{k} hole (net section, per plate)", net, case, sy / net,
                                     f"{f:.0f} N per plate; bearing pressure {bear:.0f} MPa"))
        # Pivot bearings: radial load vs static rating, with this rocker's real spacing.
        # Bearing centres: one bearing pressed into each end of the pivot tube.
        spacing = rc["plate_gap_mm"] + 2 * t - rc["pivot"].get("bearing_width_mm", 8.0)
        worst_b, worst_case = 0.0, "-"
        for lab, rl in zip(result.corners[hp_axle(result, hp)].labels, result.rockers[hp_axle(result, hp)]):
            f_rkp = rl.forces_on_rocker["RKP"]
            f_perp = f_rkp - (f_rkp @ a) * a
            diff = np.cross(2.0 * rl.pivot_moment_nmm / spacing, a)
            fb = max(np.linalg.norm((f_perp - diff) / 2), np.linalg.norm((f_perp + diff) / 2))
            if fb > worst_b:
                worst_b, worst_case = float(fb), lab
        c0 = rc["pivot"]["bearing_static_rating_n"]
        out.append(HandCheck(rp.name, "pivot bearing (radial load vs static rating C0)", worst_b, worst_case,
                             c0 / worst_b, f"{rc['pivot']['name']}, C0 {c0:g} N, {spacing:g} mm apart "
                             f"(stress column is the load in N)"))
        bolt = rc["pivot"]["bolt_mm"]
        piv = max(rl.pivot_force_n for rl in result.rockers[hp_axle(result, hp)])
        tau = piv / (2 * np.pi * bolt ** 2 / 4)
        sy_bolt = 640.0  # grade 8.8 bolt yield, MPa
        out.append(HandCheck(rp.name, "pivot bolt (double shear)", tau, "worst", 0.577 * sy_bolt / tau,
                             f"M{bolt:g} grade 8.8, shear yield 0.577 x 640 MPa"))

    # --- pushrod bracket ---------------------------------------------------
    if "pushrod_bracket" in parts:
        bp = parts["pushrod_bracket"]
        br = cfg["pushrod_bracket"]
        sy = cfg.material(bp.material)["yield_mpa"]
        labels = [lc.label for lc in loads["pushrod_bracket"]]
        a = pushrod_bolt_axis(hp)
        down = _perp(np.array([0.0, 0.0, -1.0]), a)
        t, e, h = br["tab_thickness_mm"], br["edge_distance_mm"], br["tab_height_below_hole_mm"]
        vals = [rect_section_stress(lc.forces["PRO"] / 2, -h * down, -down, np.cross(a, down), a, 2 * e, t)
                for lc in loads["pushrod_bracket"]]
        s, case = _worst(vals, labels)
        out.append(HandCheck(bp.name, "tab at the weld (per tab)", s, case, sy / s,
                             f"{h:g} mm tall, {2 * e:g} x {t:g} mm section"))
        mags = [float(np.linalg.norm(lc.forces["PRO"])) / 2 for lc in loads["pushrod_bracket"]]
        f, case = _worst(mags, labels)
        bear, net = lug_stresses(f, br["bolt_mm"], e, t)
        out.append(HandCheck(bp.name, "tab hole (net section, per tab)", net, case, sy / net,
                             f"{f:.0f} N per tab; bearing pressure {bear:.0f} MPa"))
    return out


def hp_axle(result: LoadsResult, hp: Hardpoints) -> str:
    return next(k for k, c in result.corners.items() if c.hp is hp)


# ---------------------------------------------------------------------------
# 5. Files
# ---------------------------------------------------------------------------

def parts_dict(hp: Hardpoints, parts: dict[str, Part], loads: dict[str, list[PartLoadCase]],
               cfg: PartsConfig) -> dict:
    """Everything the Fusion script, the STEP export and the FEA need for one corner."""
    out = {"corner": hp.name, "units": {"length": "mm", "force": "N"}, "parts": {}}
    for k, p in parts.items():
        d = p.to_dict()
        d["material_props"] = cfg.material(p.material)
        d["load_cases"] = {lc.label: {h: _r(f, 1) for h, f in lc.forces.items()} for lc in loads[k]}
        out["parts"][k] = d
    return out
