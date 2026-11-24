"""Phase 10: the whole car's suspension as one list of simple solids.

``suspkin assembly`` reads the front and rear hardpoints files and writes
``results/car_assembly.json``: every part of all four corners, both anti-roll
bars, the steering rack and the wheels, as plain cylinders, boxes, spheres and
rings in car coordinates. The Fusion script ``cad/fusion/CarAssembly`` then just
draws them, so all the geometry (mirroring, placing the rear one wheelbase
back, rod end directions) happens here, where it is tested.

Car coordinates
---------------
SAE axes as everywhere else (+X forward, +Y left, +Z up, mm), origin on the
ground at the car's centre line on the FRONT axle line. Each hardpoints file
has its origin at its own axle line, so the rear corner moves one wheelbase
back (-X). The right-hand corners are the left ones mirrored (y -> -y), which
is exactly how the solver treats them too.

Every solid is a dict (the same format as ``suspkin parts`` uses for
``cad/fusion/SuspensionParts``, plus ``sphere`` and ``torus``)::

    {"type": "cylinder", "p1": [...], "p2": [...], "radius": r}
    {"type": "box", "center": [...], "length_dir": [...], "width_dir": [...],
     "length": l, "width": w, "height": h}
    {"type": "sphere", "center": [...], "radius": r}
    {"type": "torus", "center": [...], "axis": [...], "major_radius": R, "minor_radius": r}

A **body** is ``{"name", "look", "solids": [...], "cuts": [...]}``: the solids
are joined, then the cuts removed. ``look`` names an appearance (steel,
aluminium, rubber, ...) that the Fusion script maps to one from its library.
A **component** is ``{"name", "material", "bodies": [...]}`` and a **group**
(one corner, one axle's anti-roll bar, ...) is ``{"name", "components": [...]}``.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .hardpoints import Hardpoints
from .parts import PartsConfig, build_parts, wheel_frame

X = np.array([1.0, 0.0, 0.0])
Y = np.array([0.0, 1.0, 0.0])
Z = np.array([0.0, 0.0, 1.0])
MIRROR = np.array([1.0, -1.0, 1.0])   # y -> -y


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _perp(v, to) -> np.ndarray:
    """``v`` with its component along ``to`` removed, normalised."""
    to = _unit(to)
    v = np.asarray(v, dtype=float)
    w = v - (v @ to) * to
    if np.linalg.norm(w) < 1e-6:      # v parallel to `to`: pick any perpendicular
        w = X - (X @ to) * to
        if np.linalg.norm(w) < 1e-6:
            w = Y - (Y @ to) * to
    return _unit(w)


def _r(v, nd: int = 4) -> list[float]:
    return [round(float(x), nd) for x in v]


# ---------------------------------------------------------------------------
# Solid helpers (each returns the dict described in the module docstring)
# ---------------------------------------------------------------------------

def cylinder(p1, p2, radius: float) -> dict:
    return {"type": "cylinder", "p1": _r(p1), "p2": _r(p2), "radius": round(float(radius), 4)}


def eye(centre, axis, radius: float, width: float) -> dict:
    """A short cylinder centred on a point (rod end eye, boss, bearing)."""
    half = _unit(axis) * width / 2
    return cylinder(np.asarray(centre) - half, np.asarray(centre) + half, radius)


def box(center, length_dir, width_dir, length: float, width: float, height: float) -> dict:
    l = _unit(length_dir)
    w = _perp(width_dir, l)
    return {"type": "box", "center": _r(center), "length_dir": _r(l, 6), "width_dir": _r(w, 6),
            "length": round(float(length), 4), "width": round(float(width), 4),
            "height": round(float(height), 4)}


def sphere(center, radius: float) -> dict:
    return {"type": "sphere", "center": _r(center), "radius": round(float(radius), 4)}


def torus(center, axis, major: float, minor: float) -> dict:
    return {"type": "torus", "center": _r(center), "axis": _r(_unit(axis), 6),
            "major_radius": round(float(major), 4), "minor_radius": round(float(minor), 4)}


def body(name: str, look: str, solids: list, cuts: list | None = None) -> dict:
    return {"name": name, "look": look, "solids": solids, "cuts": cuts or []}


def tube_body(name: str, look: str, a, b, od: float, wall: float, trim_a=0.0, trim_b=0.0) -> dict:
    """A tube from a to b, shortened by trim_a / trim_b at the ends. wall 0 = solid bar."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = _unit(b - a)
    p1, p2 = a + trim_a * d, b - trim_b * d
    cuts = []
    if wall > 0:
        cuts = [cylinder(p1 - d, p2 + d, od / 2 - wall)]
    return body(name, look, [cylinder(p1, p2, od / 2)], cuts)


def mirror_solid(s: dict) -> dict:
    """The same solid reflected in the car's centre plane (y -> -y)."""
    out = dict(s)
    for key in ("p1", "p2", "center", "length_dir", "width_dir", "axis"):
        if key in s:
            out[key] = _r(np.asarray(s[key]) * MIRROR, 6 if key.endswith(("dir", "axis")) else 4)
    return out


def mirror_group(group: dict, name: str) -> dict:
    return {"name": name, "components": [
        {**c, "bodies": [{**b, "solids": [mirror_solid(s) for s in b["solids"]],
                          "cuts": [mirror_solid(s) for s in b["cuts"]]} for b in c["bodies"]]}
        for c in group["components"]]}


def shift_solid(s: dict, offset) -> dict:
    """The same solid moved by ``offset`` (directions don't change)."""
    out = dict(s)
    for key in ("p1", "p2", "center"):
        if key in s:
            out[key] = _r(np.asarray(s[key]) + offset)
    return out


def _part_bodies(part, look: str) -> list[dict]:
    """A Phase 9 part (upright, rocker, bracket) as one body: solids joined, holes cut."""
    return [body(part.name, look, [s.to_dict() for s in part.solids],
                 [c.to_dict() for c in part.cuts])]


def _clean(d: dict) -> dict:
    """Drop the 'name' key that Phase 9 solids carry (bodies have names, solids don't need one)."""
    return {k: v for k, v in d.items() if k != "name"}


# ---------------------------------------------------------------------------
# One corner (left side, in the hardpoints file's own coordinates)
# ---------------------------------------------------------------------------

def rod_end_axis(link_dir, hint) -> np.ndarray:
    """Bolt direction of a rod end: at right angles to its link, as close to ``hint`` as possible."""
    return _perp(hint, link_dir)


def corner_group(hp: Hardpoints, axle: str, cfg: dict, parts_cfg: PartsConfig) -> dict:
    """Every part of one (left) corner, in the hardpoints file's coordinates."""
    c, u = hp.chassis, hp.upright
    t, re = cfg["tubes"], cfg["rod_end"]
    parts = build_parts(hp, parts_cfg, axle)
    pu = parts_cfg["upright"]
    comps = []

    # --- A-arms: two tube legs, rod end eyes on the chassis, clevis at the ball joint
    for key, label, front, rear, bj in (("upper_arm", "Upper A-arm", "UAF", "UAA", "UBJ"),
                                        ("lower_arm", "Lower A-arm", "LAF", "LAA", "LBJ")):
        bjp = u[bj]
        bjs = pu["ball_joints"][bj]
        boss_r = bjs["boss_od_mm"] / 2
        pivot_axis = c[rear] - c[front]
        bodies = []
        for leg, inner in (("front leg", front), ("rear leg", rear)):
            bodies.append(tube_body(f"{label} {leg}", "steel", c[inner], bjp, t["arm"]["od"],
                                    t["arm"]["wall"], trim_a=re["radius"] - 1, trim_b=boss_r))
            axis = rod_end_axis(bjp - c[inner], pivot_axis)
            bodies.append(body(f"Rod end {inner}", "rod_end", [eye(c[inner], axis, re["radius"], re["width"])]))
        # Clevis: a plate either side of the upright's boss (the bearing axis is X).
        cl = cfg["clevis"]
        off = bjs["width_mm"] / 2 + cl["gap_to_boss"] + cl["plate_thickness"] / 2
        for side, s in (("front", 1), ("rear", -1)):
            centre = bjp + s * off * X
            bodies.append(body(f"Clevis {bj} ({side})", "steel",
                               [eye(centre, X, boss_r, cl["plate_thickness"])],
                               [eye(centre, X, bjs["bore_mm"] / 2 - 4, cl["plate_thickness"] + 2)]))
        comps.append({"name": label, "material": "4130", "bodies": bodies})

    # --- Tie rod (front) / toe link (rear): rod ends at both ends
    tie = "Tie rod" if axle == "front" else "Toe link"
    d = u["RTO"] - c["RTI"]
    comps.append({"name": tie, "material": "4130", "bodies": [
        tube_body(tie, "steel", c["RTI"], u["RTO"], t["tie_rod"]["od"], t["tie_rod"]["wall"],
                  trim_a=re["radius"] - 1, trim_b=re["radius"] - 1),
        body("Rod end RTI", "rod_end", [eye(c["RTI"], rod_end_axis(d, Z), re["radius"], re["width"])]),
        body("Rod end RTO", "rod_end", [eye(u["RTO"], rod_end_axis(d, Z), re["radius"], re["width"])]),
    ]})

    # --- Upright (Phase 9 part)
    comps.append({"name": "Upright", "material": parts["upright"].material,
                  "bodies": _part_bodies(parts["upright"], "aluminium")})

    if hp.rocker is not None:
        rk = hp.rocker
        p = rk.points
        # Pushrod: rod ends bolted parallel to the rocker axis (top) and the arm's hinge (bottom)
        d = p["PRO"] - p["PRI"]
        bracket_axis = (c["LAF"] - c["LAA"]) if rk.mount == "lower_arm" else (c["UAF"] - c["UAA"])
        comps.append({"name": "Pushrod", "material": "4130", "bodies": [
            tube_body("Pushrod", "steel", p["PRI"], p["PRO"], t["pushrod"]["od"], t["pushrod"]["wall"],
                      trim_a=re["radius"] - 1, trim_b=re["radius"] - 1),
            body("Rod end PRI", "rod_end", [eye(p["PRI"], rod_end_axis(d, rk.axis), re["radius"], re["width"])]),
            body("Rod end PRO", "rod_end", [eye(p["PRO"], rod_end_axis(d, bracket_axis), re["radius"], re["width"])]),
        ]})
        if "pushrod_bracket" in parts:
            comps.append({"name": "Pushrod bracket", "material": parts["pushrod_bracket"].material,
                          "bodies": _part_bodies(parts["pushrod_bracket"], "steel")})
        comps.append({"name": "Rocker", "material": parts["rocker"].material,
                      "bodies": _part_bodies(parts["rocker"], "aluminium")})
        comps.append(damper_component(c["SHC"], p["SHR"], rk.axis, cfg))

    if hp.arb is not None:
        # The drop link belongs to the corner; the bar itself is one part per axle (arb_group).
        a = hp.arb.points
        dl, dre = cfg["tubes"]["drop_link"], cfg["drop_link_rod_end"]
        d = a["ARA"] - a["ARL"]
        comps.append({"name": "ARB drop link", "material": "4130", "bodies": [
            tube_body("ARB drop link", "steel", a["ARL"], a["ARA"], dl["od"], dl["wall"],
                      trim_a=dre["radius"] - 1, trim_b=dre["radius"] - 1),
            body("Rod end ARL", "rod_end", [eye(a["ARL"], rod_end_axis(d, X), dre["radius"], dre["width"])]),
            body("Rod end ARA", "rod_end", [eye(a["ARA"], rod_end_axis(d, Y), dre["radius"], dre["width"])]),
        ]})

    comps.append(wheel_component(hp, cfg))
    for comp in comps:
        for b in comp["bodies"]:
            b["solids"] = [_clean(s) for s in b["solids"]]
            b["cuts"] = [_clean(s) for s in b["cuts"]]
    return {"name": f"{axle.capitalize()} left corner", "components": comps}


def damper_component(shc, shr, rocker_axis, cfg: dict) -> dict:
    """Damper body at the chassis end, shaft to the rocker, coil spring round the body."""
    dm, re = cfg["damper"], cfg["rod_end"]
    shc, shr = np.asarray(shc, float), np.asarray(shr, float)
    length = float(np.linalg.norm(shr - shc))
    d = _unit(shr - shc)
    body_start = shc + re["radius"] * d
    body_end = shc + (re["radius"] + min(dm["body_length"], 0.7 * length)) * d
    shaft_end = shr - re["radius"] * d
    eye_axis = rod_end_axis(d, rocker_axis)
    bodies = [
        body("Damper body", "damper", [cylinder(body_start, body_end, dm["body_radius"])]),
        body("Damper shaft", "chrome", [cylinder(body_end, shaft_end, dm["shaft_radius"])]),
        body("Damper eye SHC", "rod_end", [eye(shc, eye_axis, re["radius"], re["width"])]),
        body("Damper eye SHR", "rod_end", [eye(shr, eye_axis, re["radius"], re["width"])]),
    ]
    # Coil spring: rings between a perch near the body start and one near the shaft end.
    n = int(dm["spring_turns"])
    s0 = re["radius"] + 8.0
    s1 = length - re["radius"] - 8.0
    rings = [torus(shc + s * d, d, dm["spring_outer_radius"] - dm["spring_wire_radius"],
                   dm["spring_wire_radius"]) for s in np.linspace(s0, s1, n)]
    bodies.append(body("Spring", "spring", rings))
    for name, s in (("Spring perch (body)", s0 - 6), ("Spring perch (shaft)", s1 + 6)):
        bodies.append(body(name, "aluminium", [eye(shc + s * d, d, dm["spring_outer_radius"] + 2, 5.0)],
                           [eye(shc + s * d, d, dm["shaft_radius"] + 1, 7.0)]))
    return {"name": "Damper", "material": "", "bodies": bodies}


def wheel_component(hp: Hardpoints, cfg: dict) -> dict:
    """Rim (barrel, hub and spokes) and tyre, on the wheel's spin axis."""
    w = cfg["wheel"]
    wf = wheel_frame(hp)
    wc, a, fwd, up = wf["WC"], wf["axis"], wf["fwd"], wf["up"]
    loaded_r = float(np.linalg.norm(hp.upright["WC"] - hp.upright["CP"]))
    half_t, half_r = w["tyre_width"] / 2, w["rim_width"] / 2
    tyre = body("Tyre", "rubber", [cylinder(wc - half_t * a, wc + half_t * a, loaded_r)],
                [cylinder(wc - (half_t + 1) * a, wc + (half_t + 1) * a, w["rim_bead_radius"])])
    barrel = body("Rim barrel", "rim", [cylinder(wc - half_r * a, wc + half_r * a, w["rim_bead_radius"])],
                  [cylinder(wc - (half_r + 1) * a, wc + (half_r + 1) * a, w["rim_inner_radius"])])
    hub_axial = float(hp.extra.get("packaging", {}).get("hub_face_axial_mm", -20.0))
    hub_c = wc + (hub_axial + w["hub_thickness"] / 2) * a
    spokes = [cylinder(hub_c - w["hub_thickness"] / 2 * a, hub_c + w["hub_thickness"] / 2 * a, w["hub_radius"])]
    for k in range(int(w["spokes"])):
        th = 2 * np.pi * k / w["spokes"] + np.pi / 2
        radial = np.cos(th) * up + np.sin(th) * fwd
        r0, r1 = w["hub_radius"] - 5, w["rim_inner_radius"] + 2
        spokes.append(box(hub_c + (r0 + r1) / 2 * radial, radial, np.cross(a, radial),
                          r1 - r0, w["spoke_width"], w["spoke_thickness"]))
    centre = body("Rim centre", "rim", spokes,
                  [cylinder(hub_c - 10 * a, hub_c + 10 * a, 20.0)])
    return {"name": "Wheel", "material": "", "bodies": [barrel, centre, tyre]}


# ---------------------------------------------------------------------------
# Parts shared by both sides of an axle
# ---------------------------------------------------------------------------

def arb_group(hp: Hardpoints, axle: str, cfg: dict) -> dict | None:
    """The anti-roll bar: torsion bar across the car, a blade arm each side, two bearings."""
    if hp.arb is None:
        return None
    a, ab, tb = hp.arb.points, cfg["arb"], cfg["tubes"]["arb_bar"]
    arp_l, ara_l = a["ARP"], a["ARA"]
    arp_r, ara_r = arp_l * MIRROR, ara_l * MIRROR
    bodies = [tube_body("Torsion bar", "steel", arp_r - ab["arm_thickness"] / 2 * Y,
                        arp_l + ab["arm_thickness"] / 2 * Y, tb["od"], tb["wall"])]
    for side, arp, ara in (("left", arp_l, ara_l), ("right", arp_r, ara_r)):
        arm_dir = ara - arp
        length = float(np.linalg.norm(arm_dir)) + ab["arm_width"]   # half a width past each end
        centre = (arp + ara) / 2
        bodies.append(body(f"Bar arm ({side})", "steel",
                           [box(centre, arm_dir, Z, length, ab["arm_width"], ab["arm_thickness"])]))
        inboard = -np.sign(arp[1]) * Y
        bodies.append(body(f"Bar bearing ({side})", "aluminium",
                           [eye(arp + inboard * (ab["bearing_inset"] + ab["bearing_length"] / 2), Y,
                                ab["bearing_radius"], ab["bearing_length"])],
                           [eye(arp + inboard * (ab["bearing_inset"] + ab["bearing_length"] / 2), Y,
                                tb["od"] / 2 + 0.5, ab["bearing_length"] + 2)]))
    return {"name": f"{axle.capitalize()} anti-roll bar",
            "components": [{"name": f"{axle.capitalize()} anti-roll bar", "material": "4130",
                            "bodies": bodies}]}


def steering_rack_group(hp: Hardpoints, cfg: dict) -> dict:
    """Rack housing on the line between the inner tie rod ends, rods out to them, and a pinion."""
    sr = cfg["steering_rack"]
    rti_l = hp.chassis["RTI"]
    rti_r = rti_l * MIRROR
    mid = (rti_l + rti_r) / 2
    h = sr["housing_half_length"]
    bodies = [body("Rack housing", "aluminium", [cylinder(mid - h * Y, mid + h * Y, sr["housing_radius"])]),
              body("Rack rod", "chrome", [cylinder(rti_r, rti_l, sr["rod_radius"])])]
    # Pinion housing sticking up and back toward the steering column.
    pin_dir = _unit(np.array([-0.5, 0.0, 1.0]))
    p0 = mid + 0.25 * h * Y
    bodies.append(body("Pinion housing", "aluminium",
                       [cylinder(p0, p0 + sr["pinion_length"] * pin_dir, sr["pinion_radius"])]))
    return {"name": "Steering rack", "components": [{"name": "Steering rack", "material": "", "bodies": bodies}]}


# ---------------------------------------------------------------------------
# The whole car
# ---------------------------------------------------------------------------

def _shift_group(group: dict, offset) -> dict:
    offset = np.asarray(offset, float)
    if not offset.any():
        return group
    return {"name": group["name"], "components": [
        {**c, "bodies": [{**b, "solids": [shift_solid(s, offset) for s in b["solids"]],
                          "cuts": [shift_solid(s, offset) for s in b["cuts"]]} for b in c["bodies"]]}
        for c in group["components"]]}


def axle_offset(hp: Hardpoints, axle: str) -> np.ndarray:
    """Move from this file's own origin (its axle line) to car coordinates (front axle line)."""
    if axle == "front":
        return np.zeros(3)
    wb = float(hp.extra.get("vehicle", {}).get("wheelbase_mm", 0.0))
    if wb <= 0:
        raise ValueError("the rear file needs vehicle.wheelbase_mm to place it on the car")
    return np.array([-wb, 0.0, 0.0])


def build_assembly(front: Hardpoints, rear: Hardpoints, cfg: dict, parts_cfg: PartsConfig) -> dict:
    """The whole car as groups of components (see the module docstring)."""
    groups = []
    summary = {}
    for axle, hp in (("front", front), ("rear", rear)):
        off = axle_offset(hp, axle)
        left = corner_group(hp, axle, cfg, parts_cfg)
        axle_groups = [left, mirror_group(left, f"{axle.capitalize()} right corner")]
        arb = arb_group(hp, axle, cfg)
        if arb is not None:
            axle_groups.append(arb)
        if axle == "front":
            axle_groups.append(steering_rack_group(hp, cfg))
        groups += [_shift_group(g, off) for g in axle_groups]
        summary[axle] = {
            "track_mm": round(2 * float(hp.upright["CP"][1]), 1),
            "axle_x_mm": round(float(off[0]), 1),
            "static_camber_deg": hp.static_camber_deg,
        }
    wb = -axle_offset(rear, "rear")[0]
    frame = None
    if "frame" in cfg:
        from .frame import frame_group, frame_stats   # imported here: frame.py uses this module
        group = frame_group(front, rear, cfg, wb)
        groups.append(group)
        frame = {k: round(float(v), 2) for k, v in frame_stats(group, cfg).items()}
    n_bodies = sum(len(c["bodies"]) for g in groups for c in g["components"])
    return {
        "name": "FSAE-style car: suspension assembly",
        "units": "mm",
        "coordinate_system": "SAE/ISO vehicle axes: +X forward, +Y left, +Z up. Origin on the ground, "
                             "on the car's centre line, at the FRONT axle line.",
        "wheelbase_mm": wb,
        "axles": summary,
        "body_count": n_bodies,
        "frame": frame,
        "groups": groups,
    }


def load_config(path: str | Path) -> dict:
    return json.loads(Path(path).read_text())


# ---------------------------------------------------------------------------
# Checks used by the tests and printed by the CLI
# ---------------------------------------------------------------------------

def solid_points(s: dict) -> np.ndarray:
    """A few points on/in a solid (for bounding boxes and simple checks)."""
    if s["type"] == "cylinder":
        return np.array([s["p1"], s["p2"]], float)
    if s["type"] in ("sphere", "torus"):
        return np.array([s["center"]], float)
    if s["type"] == "box":
        c, l, w = (np.asarray(s[k], float) for k in ("center", "length_dir", "width_dir"))
        h = np.cross(l, w)
        return np.array([c + i * s["length"] * l + j * s["width"] * w + k * s["height"] * h
                         for i in (-.5, .5) for j in (-.5, .5) for k in (-.5, .5)])
    raise ValueError(f"unknown solid type {s['type']!r}")


def group_bounds(group: dict) -> tuple[np.ndarray, np.ndarray]:
    pts = np.vstack([solid_points(s) for c in group["components"] for b in c["bodies"]
                     for s in b["solids"]])
    return pts.min(axis=0), pts.max(axis=0)
