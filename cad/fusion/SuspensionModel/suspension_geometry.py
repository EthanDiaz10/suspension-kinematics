"""Pure-Python geometry for the Fusion suspension script.

Everything in this file is plain Python (only ``json`` and ``math``), with no
``adsk`` (Fusion) imports and no numpy. That has two benefits:

1. Fusion ships its own Python interpreter without numpy, so this runs there.
2. The same file can be imported by ``pytest`` on a normal PC or in GitHub
   Actions, so the maths gets tested even though Fusion itself can't run in CI.

``SuspensionModel.py`` (the Fusion entry point) does the CAD calls and asks
this module for every number it needs.

Conventions (same as the solver): millimetres, SAE/ISO vehicle axes
(+X forward, +Y left, +Z up), front-left corner. Points are ``(x, y, z)``
tuples. Fusion's API works in centimetres, so use :func:`to_fusion_cm` at the
very last moment, right before handing a point to Fusion.
"""

from __future__ import annotations

import json
import math

# ---------------------------------------------------------------------------
# Hardpoint names (must match solver/suspkin/hardpoints.py)
# ---------------------------------------------------------------------------
CHASSIS_POINTS = ("UAF", "UAA", "LAF", "LAA", "RTI", "SHC")
UPRIGHT_POINTS = ("UBJ", "LBJ", "RTO", "WC", "CP")
# Pushrod and rocker (Phase 4): pushrod outer/inner, rocker pivot, damper end
# on the rocker. Designs without a rocker use the old direct-acting point SHU.
ROCKER_POINTS = ("PRO", "PRI", "RKP", "SHR")
DIRECT_ACTING_POINT = "SHU"
# Every point name we know, in drawing order.
ALL_POINTS = CHASSIS_POINTS + UPRIGHT_POINTS + (DIRECT_ACTING_POINT,) + ROCKER_POINTS

MM_PER_CM = 10.0


# ---------------------------------------------------------------------------
# Small vector helpers. Points and directions are 3-tuples of floats.
# ---------------------------------------------------------------------------
def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    """Vector from b to a (a - b)."""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    """Same direction as ``a`` but length 1."""
    n = norm(a)
    if n < 1e-12:
        raise ValueError("cannot normalise a zero-length vector")
    return scale(a, 1.0 / n)


def distance(a, b):
    return norm(sub(a, b))


def lerp(a, b, t):
    """Point a fraction ``t`` of the way from a to b (t=0 -> a, t=1 -> b)."""
    return add(a, scale(sub(b, a), t))


# ---------------------------------------------------------------------------
# Reading the JSON files
# ---------------------------------------------------------------------------
def wheel_axis_from_angles(camber_deg, toe_deg):
    """Unit wheel spin axis pointing outboard (+Y), left wheel.

    Same formula as ``suspkin.hardpoints.wheel_axis_from_angles``: negative
    camber tips the outboard end of the axle up, positive toe (toe-in) swings
    it forward.
    """
    c = math.radians(camber_deg)
    t = math.radians(toe_deg)
    return (math.sin(t) * math.cos(c), math.cos(t) * math.cos(c), -math.sin(c))


def _as_point(name, value, source):
    """Turn a JSON ``[x, y, z]`` list into a tuple, with a helpful error."""
    try:
        x, y, z = (float(v) for v in value)
    except (TypeError, ValueError):
        raise ValueError(f"{source}: point '{name}' should be [x, y, z] in mm, got {value!r}")
    return (x, y, z)


def has_rocker(points):
    """True if the design has a pushrod and rocker (else: direct-acting damper)."""
    return all(n in points for n in ROCKER_POINTS)


def _check_points(points, source):
    missing = [n for n in CHASSIS_POINTS + UPRIGHT_POINTS if n not in points]
    if not has_rocker(points) and DIRECT_ACTING_POINT not in points:
        missing += [n for n in ROCKER_POINTS if n not in points]
        missing.append(DIRECT_ACTING_POINT + " (or a rocker block)")
    if missing:
        raise ValueError(f"{source}: missing hardpoints {missing}")


def _rocker_info(raw_axis, mount, source):
    """Unit rocker axis (or None) and the pushrod mount name."""
    axis = unit(_as_point("rocker axis", raw_axis, source)) if raw_axis is not None else None
    return axis, (mount or "lower_arm")


def _axle_info(raw, source):
    """Which axle the corner is on, and the wheelbase, from the vehicle block.

    Files without one (Phase 1-6 front files are fine) count as a front corner.
    """
    vehicle = raw.get("vehicle") or {}
    axle = str(vehicle.get("axle", "front"))
    if axle not in ("front", "rear"):
        raise ValueError(f"{source}: vehicle 'axle' must be 'front' or 'rear', not {axle!r}")
    return {"axle": axle, "wheelbase_mm": float(vehicle.get("wheelbase_mm", 0.0))}


def car_position_offset(model):
    """(dx, dy, dz) in mm that moves this corner to its place on the whole car.

    Each hardpoints file has its origin at ITS OWN axle line (so the front and
    rear files read the same way). On the car the origin is the FRONT axle
    line, so a rear corner sits one wheelbase further back (-X).
    """
    if model["axle"] == "rear":
        if model["wheelbase_mm"] <= 0:
            raise ValueError("a rear corner needs vehicle.wheelbase_mm to be placed on the car")
        return (-model["wheelbase_mm"], 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def place_on_car(model):
    """A copy of ``model`` with every point moved to its place on the whole car.

    Only points move (directions such as the wheel and rocker axes don't
    change when you slide something), so all angles and lengths stay the same.
    """
    off = car_position_offset(model)
    placed = dict(model)
    placed["points"] = {k: add(v, off) for k, v in model["points"].items()}
    return placed


def parse_model(raw, travel_mm=0.0, source="JSON"):
    """Turn the contents of a JSON file into one set of points to draw.

    Two file types are accepted:

    * a **hardpoints file** (``data/*.json``, has ``"chassis"`` and
      ``"upright"`` groups). This is the design (static) position.
    * a **poses file** (``results/*_poses.json`` from ``suspkin sweep``, has a
      ``"poses"`` list). The pose whose travel is closest to ``travel_mm`` is
      used, so you can draw the corner at full bump or droop.

    Returns a dict with ``name``, ``points`` (name -> (x, y, z) mm),
    ``wheel_axis`` (unit vector), ``travel_mm``, ``kind``, ``rocker_axis``
    (unit vector, or None without a rocker), ``pushrod_mount``, ``axle``
    ("front" or "rear", from the vehicle block) and ``wheelbase_mm``.
    """
    if not isinstance(raw, dict):
        raise ValueError(f"{source}: expected a JSON object at the top level")

    units = raw.get("units", "mm")
    if units != "mm":
        raise ValueError(f"{source}: units are '{units}', this script expects 'mm'")

    if "poses" in raw:
        poses = raw["poses"]
        if not poses:
            raise ValueError(f"{source}: the 'poses' list is empty")
        # Pick the pose nearest the requested wheel travel.
        pose = min(poses, key=lambda p: abs(float(p.get("travel_mm", 0.0)) - travel_mm))
        points = {k: _as_point(k, v, source) for k, v in pose["points"].items()}
        _check_points(points, source)
        axis, mount = _rocker_info(raw.get("rocker_axis"), raw.get("pushrod_mount"), source)
        return {
            **_axle_info(raw, source),
            "name": raw.get("name") or "suspension",
            "kind": "poses",
            "travel_mm": float(pose.get("travel_mm", 0.0)),
            "points": points,
            "wheel_axis": unit(_as_point("wheel_axis", pose["wheel_axis"], source)),
            "rocker_axis": axis if has_rocker(points) else None,
            "pushrod_mount": mount,
        }

    if "chassis" in raw and "upright" in raw:
        points = {}
        for group in ("chassis", "upright"):
            for k, v in raw[group].items():
                points[k] = _as_point(k, v, source)
        rocker = raw.get("rocker") or {}
        for k in ROCKER_POINTS:
            if k in rocker:
                points[k] = _as_point(k, rocker[k], source)
        _check_points(points, source)
        axis, mount = _rocker_info(rocker.get("axis"), rocker.get("pushrod_mount"), source)
        if has_rocker(points) and axis is None:
            raise ValueError(f"{source}: the 'rocker' block needs an 'axis' direction")
        return {
            **_axle_info(raw, source),
            "name": raw.get("name") or "suspension",
            "kind": "hardpoints",
            "travel_mm": 0.0,
            "points": points,
            "wheel_axis": wheel_axis_from_angles(
                float(raw.get("static_camber_deg", 0.0)),
                float(raw.get("static_toe_deg", 0.0)),
            ),
            "rocker_axis": axis if has_rocker(points) else None,
            "pushrod_mount": mount,
        }

    raise ValueError(
        f"{source}: not a hardpoints file (needs 'chassis' and 'upright') "
        f"or a poses file (needs 'poses')"
    )


def read_json(path):
    """Read a JSON file, turning a syntax error into a readable message."""
    with open(path, "r", encoding="utf-8") as f:
        try:
            return json.load(f)
        except json.JSONDecodeError as e:
            raise ValueError(f"{path}: not valid JSON ({e})")


def is_poses_file(raw):
    """True for ``*_poses.json`` sweep output (it has a choice of wheel travel)."""
    return isinstance(raw, dict) and "poses" in raw


def available_travels(raw):
    """Wheel travels (mm) stored in a poses file, e.g. [-50, -40, ..., 50]."""
    return [float(p.get("travel_mm", 0.0)) for p in raw.get("poses", [])]


def load_model(path, travel_mm=0.0):
    """Read a hardpoints or poses JSON file. See :func:`parse_model`."""
    return parse_model(read_json(path), travel_mm=travel_mm, source=str(path))


# ---------------------------------------------------------------------------
# Units and axes for Fusion
# ---------------------------------------------------------------------------
def to_fusion_cm(p, up_axis="Z"):
    """Convert an SAE point in mm to Fusion model coordinates in cm.

    ``up_axis="Z"`` (the default) keeps SAE axes as they are, so the STEP file
    comes out in the same coordinates as the JSON. Set Fusion's preference
    *General > Default modeling orientation* to **Z up** so the view matches.

    ``up_axis="Y"`` maps SAE -> Fusion Y-up as Fusion X = SAE X,
    Fusion Y = SAE Z, Fusion Z = -SAE Y. That is still a right-handed system,
    so nothing gets mirrored.
    """
    x, y, z = (c / MM_PER_CM for c in p)
    if up_axis == "Z":
        return (x, y, z)
    if up_axis == "Y":
        return (x, z, -y)
    raise ValueError(f"up_axis must be 'Z' or 'Y', not {up_axis!r}")


# ---------------------------------------------------------------------------
# What to draw
# ---------------------------------------------------------------------------
# Each link: (display name, inboard point, outboard point, group).
# "inboard" ends get a rod end and "outboard" ends a ball joint, except the
# pushrod on a rocker, which has rod ends at both ends (see end_styles).
ARM_LINKS = (
    ("Upper arm front leg", "UAF", "UBJ", "upper_arm"),
    ("Upper arm rear leg", "UAA", "UBJ", "upper_arm"),
    ("Lower arm front leg", "LAF", "LBJ", "lower_arm"),
    ("Lower arm rear leg", "LAA", "LBJ", "lower_arm"),
    ("Tie rod", "RTI", "RTO", "tie_rod"),
)
# Pushrod from the rocker (inboard) down to its bracket on the arm (outboard).
PUSHROD_LINK = ("Pushrod", "PRI", "PRO", "pushrod")
# Old direct-acting spring/damper (files without a rocker block).
DIRECT_ACTING_LINK = ("Pushrod", "SHC", "SHU", "pushrod")

# For A-arm inner rod ends, the bolt axis runs along the arm's pivot axis
# (front pivot -> rear pivot). Other rod ends just point their bolt as close
# to vertical as possible.
PIVOT_AXES = {
    "upper_arm": ("UAF", "UAA"),
    "lower_arm": ("LAF", "LAA"),
}


def skeleton_links(points, axle="front"):
    """Every tube to build for this design: the arms, tie rod and pushrod.

    On a rear corner (``axle="rear"``) the tie rod is a fixed **toe link**: the
    same tube between the same points (RTI on the chassis, RTO on the upright),
    just named for what it is.
    """
    links = ARM_LINKS + ((PUSHROD_LINK,) if has_rocker(points) else (DIRECT_ACTING_LINK,))
    if axle == "rear":
        links = tuple((TOE_LINK_NAME,) + lk[1:] if lk[3] == "tie_rod" else lk for lk in links)
    return links


TOE_LINK_NAME = "Toe link"


def part_name(name, axle="front"):
    """A part's display name for this axle ("Tie rod" becomes "Toe link" at the rear)."""
    return TOE_LINK_NAME if (axle == "rear" and name == "Tie rod") else name


def end_styles(points, link):
    """("rod_end" | "ball", same) for the (inboard, outboard) end of a link.

    A pushrod on a rocker has rod ends at both ends; everything else has a rod
    end at the chassis and a ball joint at the upright.
    """
    if link[3] == "pushrod" and has_rocker(points):
        return ("rod_end", "rod_end")
    return ("rod_end", "ball")


def kingpin_line(points, above_ubj_mm=40.0):
    """Ends of the kingpin (steering) axis line for the sketch.

    The axis runs through LBJ and UBJ. It is drawn from where it meets the
    ground (the horizontal plane through the contact patch, same definition
    the solver uses for scrub radius) to a little above the upper ball joint,
    so you can see the scrub radius and trail directly in the model.
    """
    lbj, ubj = points["LBJ"], points["UBJ"]
    d = unit(sub(ubj, lbj))
    if abs(d[2]) < 1e-9:
        raise ValueError("kingpin axis is horizontal; check UBJ and LBJ")
    s = (points["CP"][2] - lbj[2]) / d[2]       # distance along the axis to the ground
    ground = add(lbj, scale(d, s))
    top = add(ubj, scale(d, above_ubj_mm))
    return ground, top


def kingpin_angles_deg(points):
    """(caster, KPI) in degrees from the ball joints. Matches suspkin.metrics.

    Shown in Fusion's finish message as a sanity check that the right file was
    read: compare with ``suspkin sweep`` output.
    """
    d = unit(sub(points["UBJ"], points["LBJ"]))
    caster = math.degrees(math.atan2(-d[0], d[2]))
    kpi = math.degrees(math.atan2(-d[1], d[2]))
    return caster, kpi


def trim_segment(a, b, trim_a, trim_b):
    """Shorten segment a->b by ``trim_a`` mm at a and ``trim_b`` mm at b.

    Used so a tube stops where it meets the ball joint or rod end instead of
    running through its centre.
    """
    length = distance(a, b)
    if trim_a + trim_b >= length:
        raise ValueError(
            f"segment is {length:.1f} mm long, too short to trim {trim_a + trim_b:.1f} mm"
        )
    d = unit(sub(b, a))
    return add(a, scale(d, trim_a)), sub(b, scale(d, trim_b))


def perpendicular_axis(direction, hint=(0.0, 0.0, 1.0)):
    """Unit vector perpendicular to ``direction``, as close to ``hint`` as possible.

    Removes the part of ``hint`` that lies along ``direction`` (one step of
    Gram-Schmidt). If ``hint`` is parallel to ``direction``, falls back to X.
    """
    d = unit(direction)
    h = sub(hint, scale(d, dot(hint, d)))
    if norm(h) < 1e-6:
        h = sub((1.0, 0.0, 0.0), scale(d, d[0]))
    return unit(h)


def rod_end_axis(points, link, at=None, rocker_axis=None, pushrod_mount="lower_arm"):
    """Bolt direction of the rod end at one end of ``link``.

    ``at`` is the point name of that end (default: the inboard end). The bolt
    is always at right angles to the link, and as close as possible to:

    * the arm's pivot axis, for A-arm inner rod ends;
    * the rocker's axis, for the pushrod's rocker end (PRI) and the damper;
    * the mounting arm's pivot axis, for the pushrod's outer end (PRO);
    * vertical, for everything else.
    """
    _name, a, b, group = link
    at = at or a
    direction = sub(points[b], points[a])
    if group in PIVOT_AXES:
        front, rear = PIVOT_AXES[group]
        hint = sub(points[rear], points[front])
    elif at in ("PRI", "SHR", "SHC") and rocker_axis is not None:
        hint = rocker_axis
    elif at == "PRO" and pushrod_mount in PIVOT_AXES:
        front, rear = PIVOT_AXES[pushrod_mount]
        hint = sub(points[rear], points[front])
    else:
        hint = (0.0, 0.0, 1.0)
    return perpendicular_axis(direction, hint)


def tube_segments(points, ball_radius_mm, rod_end_radius_mm, overlap_mm=1.0, axle="front"):
    """Trimmed tube centre lines, one per link.

    Each tube is shortened so it just reaches into its end fittings
    (``overlap_mm`` of overlap so the solids touch in the model).
    Returns a list of dicts: ``name``, ``group``, ``start``, ``end`` (mm).
    """
    out = []
    for link in skeleton_links(points, axle):
        name, a, b, group = link
        radius = {"rod_end": rod_end_radius_mm, "ball": ball_radius_mm}
        style_a, style_b = end_styles(points, link)
        start, end = trim_segment(
            points[a],
            points[b],
            max(radius[style_a] - overlap_mm, 0.0),
            max(radius[style_b] - overlap_mm, 0.0),
        )
        out.append({"name": name, "group": group, "start": start, "end": end})
    return out


def link_lengths(points, axle="front"):
    """Centre-to-centre length of every link in mm (handy for a cut list).

    With a rocker, the damper's eye-to-eye length is included too.
    """
    out = {name: distance(points[a], points[b]) for name, a, b, _g in skeleton_links(points, axle)}
    if has_rocker(points):
        out["Damper (eye to eye)"] = distance(points["SHC"], points["SHR"])
    return out


# ---------------------------------------------------------------------------
# Rocker and damper (Phase 4)
# ---------------------------------------------------------------------------
def rocker_axis_line(points, rocker_axis, half_length_mm=30.0):
    """Ends of a short line along the rocker pivot axis, centred on RKP."""
    half = scale(rocker_axis, half_length_mm)
    return sub(points["RKP"], half), add(points["RKP"], half)


def rocker_bars(points):
    """The rocker drawn as a triangle of bars: pivot->pushrod, pivot->damper,
    pushrod->damper. Returns (name, start, end) tuples in mm."""
    return [
        ("Rocker arm to pushrod", points["RKP"], points["PRI"]),
        ("Rocker arm to damper", points["RKP"], points["SHR"]),
        ("Rocker web", points["PRI"], points["SHR"]),
    ]


def rocker_plane_error_deg(points, rocker_axis):
    """How far PRI and SHR sit out of the plane through RKP at right angles to
    the axis, as angles seen from the pivot (deg). Should be ~0 for a flat
    rocker plate; the solver doesn't need it, it's a sanity check."""
    out = []
    for name in ("PRI", "SHR"):
        v = sub(points[name], points["RKP"])
        out.append(math.degrees(math.asin(min(1.0, abs(dot(unit(v), rocker_axis))))))
    return max(out)


def damper_segments(points, body_length_mm, eye_radius_mm):
    """Damper split into a fat body (chassis end) and a thin shaft (rocker end).

    Returns ``(body_start, body_end, shaft_end)``: the body runs from just past
    the chassis eye for ``body_length_mm`` (capped at 70 % of the length so the
    shaft always shows), and the shaft from there to just short of the rocker
    eye.
    """
    shc, shr = points["SHC"], points["SHR"]
    length = distance(shc, shr)
    d = unit(sub(shr, shc))
    body_start = add(shc, scale(d, eye_radius_mm))
    body_end = add(shc, scale(d, eye_radius_mm + min(body_length_mm, 0.7 * length)))
    shaft_end = sub(shr, scale(d, eye_radius_mm))
    return body_start, body_end, shaft_end


def wheel_cylinder(points, wheel_axis, width_mm):
    """Inner and outer face centres of a simple tyre, for a visual reference.

    The tyre is centred on WC along the spin axis. Its outer radius is the
    loaded radius (WC to CP).
    """
    half = scale(wheel_axis, width_mm / 2.0)
    radius = distance(points["WC"], points["CP"])
    return sub(points["WC"], half), add(points["WC"], half), radius
