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
UPRIGHT_POINTS = ("UBJ", "LBJ", "RTO", "SHU", "WC", "CP")
ALL_POINTS = CHASSIS_POINTS + UPRIGHT_POINTS

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


def _check_points(points, source):
    missing = [n for n in ALL_POINTS if n not in points]
    if missing:
        raise ValueError(f"{source}: missing hardpoints {missing}")


def parse_model(raw, travel_mm=0.0, source="JSON"):
    """Turn the contents of a JSON file into one set of points to draw.

    Two file types are accepted:

    * a **hardpoints file** (``data/*.json``, has ``"chassis"`` and
      ``"upright"`` groups). This is the design (static) position.
    * a **poses file** (``results/*_poses.json`` from ``suspkin sweep``, has a
      ``"poses"`` list). The pose whose travel is closest to ``travel_mm`` is
      used, so you can draw the corner at full bump or droop.

    Returns a dict with ``name``, ``points`` (name -> (x, y, z) mm),
    ``wheel_axis`` (unit vector), ``travel_mm`` and ``kind``.
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
        return {
            "name": raw.get("name") or "suspension",
            "kind": "poses",
            "travel_mm": float(pose.get("travel_mm", 0.0)),
            "points": points,
            "wheel_axis": unit(_as_point("wheel_axis", pose["wheel_axis"], source)),
        }

    if "chassis" in raw and "upright" in raw:
        points = {}
        for group in ("chassis", "upright"):
            for k, v in raw[group].items():
                points[k] = _as_point(k, v, source)
        _check_points(points, source)
        return {
            "name": raw.get("name") or "suspension",
            "kind": "hardpoints",
            "travel_mm": 0.0,
            "points": points,
            "wheel_axis": wheel_axis_from_angles(
                float(raw.get("static_camber_deg", 0.0)),
                float(raw.get("static_toe_deg", 0.0)),
            ),
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
# "inboard" ends get a rod end, "outboard" ends a ball joint (see LINK_ENDS).
SKELETON_LINKS = (
    ("Upper arm front leg", "UAF", "UBJ", "upper_arm"),
    ("Upper arm rear leg", "UAA", "UBJ", "upper_arm"),
    ("Lower arm front leg", "LAF", "LBJ", "lower_arm"),
    ("Lower arm rear leg", "LAA", "LBJ", "lower_arm"),
    ("Tie rod", "RTI", "RTO", "tie_rod"),
    # Direct-acting spring/damper placeholder until the pushrod/rocker phase.
    ("Pushrod", "SHC", "SHU", "pushrod"),
)

# For A-arm inner rod ends, the bolt axis runs along the arm's pivot axis
# (front pivot -> rear pivot). Other rod ends just point their bolt as close
# to vertical as possible.
PIVOT_AXES = {
    "upper_arm": ("UAF", "UAA"),
    "lower_arm": ("LAF", "LAA"),
}


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


def rod_end_axis(points, link):
    """Bolt direction of the rod end at the inboard end of ``link``."""
    _name, a, b, group = link
    direction = sub(points[b], points[a])
    if group in PIVOT_AXES:
        front, rear = PIVOT_AXES[group]
        hint = sub(points[rear], points[front])
    else:
        hint = (0.0, 0.0, 1.0)
    return perpendicular_axis(direction, hint)


def tube_segments(points, ball_radius_mm, rod_end_radius_mm, overlap_mm=1.0):
    """Trimmed tube centre lines, one per link.

    Each tube is shortened so it just reaches into its end fittings
    (``overlap_mm`` of overlap so the solids touch in the model).
    Returns a list of dicts: ``name``, ``group``, ``start``, ``end`` (mm).
    """
    out = []
    for link in SKELETON_LINKS:
        name, a, b, group = link
        start, end = trim_segment(
            points[a],
            points[b],
            max(rod_end_radius_mm - overlap_mm, 0.0),
            max(ball_radius_mm - overlap_mm, 0.0),
        )
        out.append({"name": name, "group": group, "start": start, "end": end})
    return out


def link_lengths(points):
    """Centre-to-centre length of every link in mm (handy for a cut list)."""
    return {name: distance(points[a], points[b]) for name, a, b, _g in SKELETON_LINKS}


def wheel_cylinder(points, wheel_axis, width_mm):
    """Inner and outer face centres of a simple tyre, for a visual reference.

    The tyre is centred on WC along the spin axis. Its outer radius is the
    loaded radius (WC to CP).
    """
    half = scale(wheel_axis, width_mm / 2.0)
    radius = distance(points["WC"], points["CP"])
    return sub(points["WC"], half), add(points["WC"], half), radius
