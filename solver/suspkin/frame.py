"""Phase 10 step 2: an FSAE-style tube space frame around the suspension pickups.

The frame is a list of **nodes** (points where tubes meet) and **members**
(a tube between two nodes). Only the left half is described; the right half
is its mirror image, and members that cross the centre line join a left node
to its own mirror.

Where the nodes come from:

* Every A-arm pickup of both axles (UAF, UAA, LAF, LAA) sits on a short tab
  just outboard of a frame node, so each wishbone pivots right at a joint of the
  frame, where the load goes into tubes along their length instead of bending
  one. (The lower rails also sit a little below the pickups so the tie rods,
  toe links and steering rack pass over them.)
* The rocker and damper hang off a **top cross tube** behind the rockers, at
  the damper's chassis-end height. A strut from its end down to the upper
  wishbone rail carries the rocker pivot on a short tab.
* The bulkhead, hoops, cockpit and rear end come from the ``frame`` block in
  ``data/assembly.json`` (made-up, FSAE-like positions).

This is a packaging and visual model: it shows where the loads go into the
chassis and gives the renders a car, but it is not a rules-checked or
stress-checked frame.
"""

from __future__ import annotations

import numpy as np

from .assembly import MIRROR, _unit, body, box, cylinder, tube_body
from .hardpoints import Hardpoints

# (name, from node, to node, tube size). A trailing "'" means the mirrored
# (right-hand) node, so ("FB_lo", "FB_lo'") is a cross member.
MEMBERS = [
    # --- front bulkhead (a square at the very front)
    ("Bulkhead lower", "FB_lo", "FB_lo'", "primary"),
    ("Bulkhead top", "FB_hi", "FB_hi'", "primary"),
    ("Bulkhead side lower", "FB_lo", "FB_up", "primary"),
    ("Bulkhead side upper", "FB_up", "FB_hi", "primary"),
    ("Bulkhead diagonal", "FB_lo", "FB_hi'", "secondary"),
    # --- front suspension box: rails through the wishbone pickups
    ("Front lower rail 1", "FB_lo", "F_LAF", "primary"),
    ("Front lower rail 2", "F_LAF", "F_LAA", "primary"),
    ("Front lower rail 3", "F_LAA", "FH_lo", "primary"),
    ("Front upper rail 1", "FB_up", "F_UAF", "primary"),
    ("Front upper rail 2", "F_UAF", "F_RS", "primary"),
    ("Front upper rail 3", "F_RS", "F_UAA", "primary"),
    ("Front upper rail 4", "F_UAA", "FH_up", "primary"),
    ("Front top rail 1", "FB_hi", "F_TOPO", "primary"),
    ("Front top rail 2", "F_TOPO", "FH_mid", "primary"),
    ("Front rocker strut", "F_TOP", "F_RS", "primary"),
    ("Front rocker cross tube end", "F_TOP", "F_TOPO", "primary"),
    ("Front rocker cross tube", "F_TOP", "F_TOP'", "primary"),
    ("Front pickup upright (front)", "F_LAF", "F_UAF", "secondary"),
    ("Front pickup upright (rear)", "F_LAA", "F_UAA", "secondary"),
    ("Front box diagonal 1", "FB_lo", "F_UAF", "secondary"),
    ("Front box diagonal 2", "F_LAA", "FH_up", "secondary"),
    ("Front box diagonal 3", "F_LAA", "F_UAF", "secondary"),
    ("Front top diagonal", "FB_up", "FB_hi", "secondary"),
    ("Front top diagonal 2", "F_UAF", "FB_hi", "secondary"),
    ("Front floor cross (front pickups)", "F_LAF", "F_LAF'", "secondary"),
    ("Front floor cross (rear pickups)", "F_LAA", "F_LAA'", "secondary"),
    ("Front floor diagonal", "FB_lo", "F_LAA'", "secondary"),
    # --- front hoop and its braces
    ("Front hoop lower", "FH_lo", "FH_up", "hoop"),
    ("Front hoop side", "FH_up", "FH_mid", "hoop"),
    ("Front hoop upper", "FH_mid", "FH_top", "hoop"),
    ("Front hoop top", "FH_top", "FH_top'", "hoop"),
    ("Front hoop floor", "FH_lo", "FH_lo'", "primary"),
    ("Front hoop brace", "FH_top", "FB_hi", "primary"),
    # --- cockpit sides: lower rail, side impact tube and a diagonal between
    ("Cockpit lower rail 1", "FH_lo", "C_lo", "primary"),
    ("Cockpit lower rail 2", "C_lo", "MH_lo", "primary"),
    ("Side impact tube 1", "FH_up", "C_up", "primary"),
    ("Side impact tube 2", "C_up", "MH_up", "primary"),
    ("Side impact vertical", "C_lo", "C_up", "secondary"),
    ("Side impact diagonal 1", "FH_up", "C_lo", "primary"),
    ("Side impact diagonal 2", "C_lo", "MH_up", "primary"),
    ("Cockpit floor cross", "C_lo", "C_lo'", "secondary"),
    ("Cockpit floor diagonal", "FH_lo", "C_lo'", "secondary"),
    # --- main hoop and its braces back to the rear box
    ("Main hoop lower", "MH_lo", "MH_up", "hoop"),
    ("Main hoop side", "MH_up", "MH_mid", "hoop"),
    ("Main hoop upper", "MH_mid", "MH_top", "hoop"),
    ("Main hoop top", "MH_top", "MH_top'", "hoop"),
    ("Main hoop floor", "MH_lo", "MH_lo'", "primary"),
    ("Main hoop shoulder cross", "MH_mid", "MH_mid'", "secondary"),
    ("Main hoop brace", "MH_top", "R_TOPO", "primary"),
    ("Main hoop brace support", "MH_mid", "R_TOPO", "secondary"),
    # --- rear suspension box
    ("Rear lower rail 1", "MH_lo", "R_LAF", "primary"),
    ("Rear lower rail 2", "R_LAF", "R_LAA", "primary"),
    ("Rear lower rail 3", "R_LAA", "RE_lo", "primary"),
    ("Rear upper rail 1", "MH_up", "R_UAF", "primary"),
    ("Rear upper rail 2", "R_UAF", "R_RS", "primary"),
    ("Rear upper rail 3", "R_RS", "R_UAA", "primary"),
    ("Rear upper rail 4", "R_UAA", "RE_hi", "primary"),
    ("Rear rocker strut", "R_TOP", "R_RS", "primary"),
    ("Rear rocker cross tube", "R_TOP", "R_TOP'", "primary"),
    ("Rear rocker cross tube end", "R_TOP", "R_TOPO", "primary"),
    ("Rear top rail", "R_TOPO", "RE_hi", "primary"),
    ("Rear top diagonal", "MH_up", "R_TOPO", "secondary"),
    ("Rear pickup upright (front)", "R_LAF", "R_UAF", "secondary"),
    ("Rear pickup upright (rear)", "R_LAA", "R_UAA", "secondary"),
    ("Rear end upright", "RE_lo", "RE_hi", "primary"),
    ("Rear end lower cross", "RE_lo", "RE_lo'", "primary"),
    ("Rear end top cross", "RE_hi", "RE_hi'", "primary"),
    ("Rear box diagonal 1", "MH_lo", "R_UAF", "secondary"),
    ("Rear box diagonal 2", "R_LAA", "RE_hi", "secondary"),
    ("Rear box diagonal 3", "R_LAF", "R_UAA", "secondary"),
    ("Rear floor cross (front pickups)", "R_LAF", "R_LAF'", "secondary"),
    ("Rear floor cross (rear pickups)", "R_LAA", "R_LAA'", "secondary"),
    ("Rear floor diagonal", "MH_lo", "R_LAA'", "secondary"),
]


def _yz(x: float, yz) -> np.ndarray:
    return np.array([x, float(yz[0]), float(yz[1])])


def pickup_node(hp: Hardpoints, name: str, offset_x: float, fcfg: dict) -> np.ndarray:
    """Frame node next to a wishbone pickup: the pickup sits on a tab
    ``pickup_tab_mm`` = [inboard, down] from it.

    Putting the nodes inboard keeps the frame's side tubes out of the way of
    the wishbones as they leave their pickups; putting the lower rail a little
    below the pickups keeps it clear of the tie rods, toe links and steering
    rack, which run out at about the lower pickups' height.
    """
    inboard, down = fcfg["pickup_tab_mm"]["lower" if name.startswith("L") else "upper"]
    return hp.chassis[name] + [offset_x, -inboard, -down]


def rocker_nodes(hp: Hardpoints, offset_x: float, fcfg: dict, uaf, uaa) -> tuple[np.ndarray, np.ndarray]:
    """Top cross tube end (TOP) and rocker strut foot (RS), in car coordinates.

    TOP sits ``behind_rocker_mm`` behind the rocker pivot, at the damper's
    chassis-end height. The strut runs from TOP straight down to the upper
    rail (the line between the upper pickup nodes ``uaf`` and ``uaa``), and a
    tab from the strut carries the rocker pivot.
    """
    rm = fcfg["rocker_mount"]
    rkp = hp.rocker.points["RKP"] + [offset_x, 0, 0]
    shc = hp.chassis["SHC"] + [offset_x, 0, 0]
    x = rkp[0] - rm["behind_rocker_mm"]
    top = np.array([x, rm["top_node_y"], shc[2]])
    t = (x - uaf[0]) / (uaa[0] - uaf[0])
    return top, uaf + t * (uaa - uaf)


def frame_nodes(front: Hardpoints, rear: Hardpoints, fcfg: dict, wheelbase: float) -> dict[str, np.ndarray]:
    """Every left-side node, in car coordinates (mm)."""
    n: dict[str, np.ndarray] = {}
    fb = fcfg["front_bulkhead"]
    n["FB_lo"], n["FB_up"], n["FB_hi"] = (_yz(fb["x"], fb[k]) for k in ("lower", "upper", "top"))
    for prefix, hp, off in (("F", front, 0.0), ("R", rear, -wheelbase)):
        for k in ("UAF", "UAA", "LAF", "LAA"):
            n[f"{prefix}_{k}"] = pickup_node(hp, k, off, fcfg)
        n[f"{prefix}_TOP"], n[f"{prefix}_RS"] = rocker_nodes(hp, off, fcfg, n[f"{prefix}_UAF"],
                                                             n[f"{prefix}_UAA"])
        n[f"{prefix}_TOPO"] = n[f"{prefix}_TOP"] * [1, 0, 1] + [0, fcfg["rocker_mount"]["top_rail_y"], 0]
    fh = fcfg["front_hoop"]
    for k, yz in zip(("FH_lo", "FH_up", "FH_mid", "FH_top"), fh["points"]):
        n[k] = _yz(fh["x"], yz)
    c = fcfg["cockpit"]
    n["C_lo"], n["C_up"] = _yz(c["x"], c["lower"]), _yz(c["x"], c["upper"])
    mh = fcfg["main_hoop"]
    for k, yz in zip(("MH_lo", "MH_up", "MH_mid", "MH_top"), mh["points"]):
        n[k] = _yz(mh["x"], yz)
    re = fcfg["rear_end"]
    x = -wheelbase + re["x_behind_rear_axle"]
    n["RE_lo"], n["RE_hi"] = _yz(x, re["lower"]), _yz(x, re["top"])
    return n


def node(nodes: dict, name: str) -> np.ndarray:
    """A node by name; a trailing "'" means its mirror image (right side)."""
    if name.endswith("'"):
        return nodes[name[:-1]] * MIRROR
    return nodes[name]


def frame_members(nodes: dict) -> list[tuple[str, np.ndarray, np.ndarray, str]]:
    """Every tube of the whole frame: (name, start, end, size). Left, right and cross members."""
    out = []
    for name, a, b, size in MEMBERS:
        pa, pb = node(nodes, a), node(nodes, b)
        if a.endswith("'") or b.endswith("'"):
            out.append((name, pa, pb, size))             # crosses the centre line: one tube
        else:
            out.append((name + " (left)", pa, pb, size))
            out.append((name + " (right)", pa * MIRROR, pb * MIRROR, size))
    return out


def _tab(name: str, from_pt, to_pt, cfg: dict, normal) -> dict:
    """A flat plate (box) from a frame tube to a pickup, ``thickness`` along ``normal``."""
    from_pt, to_pt = np.asarray(from_pt, float), np.asarray(to_pt, float)
    d = to_pt - from_pt
    length = float(np.linalg.norm(d)) + cfg["width"] / 2
    centre = from_pt + _unit(d) * length / 2
    width_dir = np.cross(normal, _unit(d))
    return body(name, "frame", [box(centre, d, width_dir, length, cfg["width"], cfg["thickness"])])


def _closest_on_segment(p, a, b) -> np.ndarray:
    ab = b - a
    t = float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
    return a + t * ab


def mount_tabs(front: Hardpoints, rear: Hardpoints, nodes: dict, fcfg: dict, wheelbase: float,
               arb_cfg: dict | None = None) -> list[dict]:
    """Tabs from the frame to every pickup: wishbone pivots, rocker pivots, damper
    chassis ends, anti-roll bar bearings, the rear toe links and the steering rack."""
    tab = fcfg["tab"]
    out = []
    for prefix, hp, off in (("Front", front, 0.0), ("Rear", rear, -wheelbase)):
        p = prefix[0]
        o = np.array([off, 0, 0])
        rkp = hp.rocker.points["RKP"] + o
        shc = hp.chassis["SHC"] + o
        strut = (nodes[f"{p}_TOP"], nodes[f"{p}_RS"])
        cross = (nodes[f"{p}_TOP"], nodes[f"{p}_TOP"] * MIRROR)
        lower = (nodes[f"{p}_LAF"], nodes[f"{p}_LAA"])
        for side, m in (("left", np.ones(3)), ("right", MIRROR)):
            for k in ("UAF", "UAA", "LAF", "LAA"):
                out.append(_tab(f"{prefix} {k} pickup tab ({side})", nodes[f"{p}_{k}"] * m,
                                (hp.chassis[k] + o) * m, tab, np.array([1.0, 0.0, 0.0])))
            if hp.arb is not None:
                # The bar's bearing blocks hang under the rocker cross tube.
                ab = arb_cfg
                bearing = hp.arb.points["ARP"] + o - [0, ab["bearing_inset"] + ab["bearing_length"] / 2, 0]
                top = np.array([nodes[f"{p}_TOP"][0], bearing[1], nodes[f"{p}_TOP"][2]])
                out.append(_tab(f"{prefix} anti-roll bar mount ({side})", top * m, bearing * m, tab,
                                np.array([0.0, 1.0, 0.0])))
            # Rocker pivot: a pair of plates either side of the pivot boss would be
            # the real thing; one plate in the rocker's plane shows the load path.
            foot = _closest_on_segment(rkp, *strut)
            out.append(_tab(f"{prefix} rocker pivot tab ({side})", foot * m, rkp * m, tab,
                            np.array([0.0, 1.0, 0.0])))
            foot = _closest_on_segment(shc, *cross)
            out.append(_tab(f"{prefix} damper mount tab ({side})", foot * m, shc * m, tab,
                            np.array([0.0, 1.0, 0.0])))
            if prefix == "Rear":
                rti = hp.chassis["RTI"] + o
                foot = _closest_on_segment(rti, *lower)
                out.append(_tab(f"Rear toe link tab ({side})", foot * m, rti * m, tab,
                                np.array([1.0, 0.0, 0.0])))
    # Steering rack clamps: two tabs from the front floor cross tubes up to the rack housing.
    rti = front.chassis["RTI"]
    cross_x = nodes["F_LAF"][0]
    for side, s in (("left", 1), ("right", -1)):
        clamp = np.array([rti[0], s * 90.0, rti[2]])
        out.append(_tab(f"Steering rack clamp ({side})", [cross_x, s * 90.0, nodes["F_LAF"][2]],
                        clamp, tab, np.array([0.0, 1.0, 0.0])))
    return out


def frame_group(front: Hardpoints, rear: Hardpoints, cfg: dict, wheelbase: float) -> dict:
    """The space frame as one assembly group: a component of tubes and one of mounting tabs."""
    fcfg = cfg["frame"]
    nodes = frame_nodes(front, rear, fcfg, wheelbase)
    tubes = fcfg["tubes"]
    bodies = [tube_body(name, "frame", a, b, tubes[size]["od"], tubes[size]["wall"])
              for name, a, b, size in frame_members(nodes)]
    # A ball at every node tidies up where the tubes meet (stands in for the welds).
    joints = [body(f"Node {k} ({side})", "frame", [{"type": "sphere", "center": [round(float(c), 4) for c in p * m],
                                                     "radius": tubes["primary"]["od"] / 2}])
              for k, p in nodes.items() for side, m in (("left", np.ones(3)), ("right", MIRROR))]
    tabs = mount_tabs(front, rear, nodes, fcfg, wheelbase, cfg["arb"])
    return {"name": "Chassis frame", "components": [
        {"name": "Frame tubes", "material": "4130", "bodies": bodies + joints},
        {"name": "Suspension mounting tabs", "material": "4130", "bodies": tabs},
    ]}


def frame_stats(group: dict, cfg: dict) -> dict:
    """Tube count, total length and mass (4130 steel, 7850 kg/m^3)."""
    tubes = [b for b in group["components"][0]["bodies"] if not b["name"].startswith("Node")]
    length = 0.0
    mass = 0.0
    for b in tubes:
        s = b["solids"][0]
        L = float(np.linalg.norm(np.subtract(s["p2"], s["p1"])))
        r_out = s["radius"]
        r_in = b["cuts"][0]["radius"] if b["cuts"] else 0.0
        length += L
        mass += np.pi * (r_out ** 2 - r_in ** 2) * L * 7.85e-6   # mm^3 -> kg
    return {"tubes": len(tubes), "length_m": length / 1000, "mass_kg": mass}


# ---------------------------------------------------------------------------
# Clearance check: does any frame tube hit a suspension part?
# ---------------------------------------------------------------------------

def _segment_distance(p1, q1, p2, q2) -> float:
    """Shortest distance between segments p1-q1 and p2-q2 (standard clamped solution)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    denom = a * e - b * b
    s = float(np.clip((b * f - c * e) / denom, 0, 1)) if denom > 1e-12 else 0.0
    t = (b * s + f) / e
    if t < 0:
        t, s = 0.0, float(np.clip(-c / a, 0, 1))
    elif t > 1:
        t, s = 1.0, float(np.clip((b - c) / a, 0, 1))
    return float(np.linalg.norm((p1 + s * d1) - (p2 + t * d2)))


def _capsules(component: dict, look_skip=()) -> list[tuple[str, np.ndarray, np.ndarray, float]]:
    out = []
    for b in component["bodies"]:
        for sd in b["solids"]:
            if sd["type"] == "cylinder" and b.get("look") not in look_skip:
                out.append((b["name"], np.array(sd["p1"], float), np.array(sd["p2"], float), sd["radius"]))
    return out


def frame_clearances(asm: dict, near_mm: float = 3.0) -> list[dict]:
    """Every frame tube that comes within ``near_mm`` of a suspension part's
    cylinder (tubes, rod ends, damper, spring perches, rack, anti-roll bar).

    Wheels and the Phase 9 parts are left out (they're checked against the
    rim in Phase 9 and sit outboard of the frame), and so are a frame tube's
    own mounting tabs. Each part is treated as round (a capsule), which is a
    little cautious at the ends.
    """
    frame = next((g for g in asm["groups"] if g["name"] == "Chassis frame"), None)
    if frame is None:
        return []
    tubes = _capsules(frame["components"][0])
    skip_parts = {"Wheel", "Upright", "Rocker", "Pushrod bracket"}
    out = []
    for g in asm["groups"]:
        if g is frame:
            continue
        for comp in g["components"]:
            if comp["name"] in skip_parts:
                continue
            for name, p, q, r in _capsules(comp):
                for tname, tp, tq, tr in tubes:
                    gap = _segment_distance(p, q, tp, tq) - r - tr
                    if gap < near_mm:
                        out.append({"part": f"{g['name']}: {name}", "tube": tname, "gap_mm": round(gap, 1)})
    return sorted(out, key=lambda c: c["gap_mm"])
