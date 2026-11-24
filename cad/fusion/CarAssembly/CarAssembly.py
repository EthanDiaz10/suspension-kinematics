"""Autodesk Fusion script: build the whole car's suspension (Phase 10).

What it does, in order:

1. Reads ``results/car_assembly.json``, written by ``suspkin assembly``. If the
   file is in the usual place in the repo it is used straight away; otherwise
   it asks for it.
2. Opens a new Fusion design and makes one component per group (front left
   corner, front right corner, front anti-roll bar, steering rack, rear ...),
   with a component inside it for every part (upper A-arm, upright, rocker,
   damper, wheel, ...).
3. Builds each part's bodies from the simple solids in the file (cylinders,
   boxes, spheres, rings), joined and with the holes cut, and gives them a
   material and an appearance (steel, aluminium, rubber, ...) for rendering.
4. Exports everything as one STEP file next to the JSON file.

All the geometry (mirroring the right side, moving the rear one wheelbase
back, where every rod end points) is worked out in Python by
``solver/suspkin/assembly.py``, where it is tested. This script only turns the
numbers into Fusion bodies.

The design is made as a **direct** design (no timeline), because nothing in
it is parametric and it builds much faster that way. Install and run it like
the other scripts (see cad/fusion/README.md).
"""

import json
import os
import time
import traceback

import adsk.core
import adsk.fusion

# ===========================================================================
# Settings you might want to change
# ===========================================================================

# Skip the search and the file dialog by putting a full path here, e.g.
# r"C:\Users\ethan\Projects\suspension-kinematics\results\car_assembly.json"
DEFAULT_JSON = ""

# "Z" keeps SAE axes (+Z up): set Fusion's Preferences > General >
# Default modeling orientation to "Z up" so the car sits on the ground.
UP_AXIS = "Z"

EXPORT_STEP = True

# Appearance library names to try for each "look" in the file (the first one
# Fusion has wins). Change these to restyle the car; you can also drag new
# appearances onto parts afterwards (press A in the Design workspace).
APPEARANCES = {
    "steel": ["Steel - Satin", "Steel - Polished", "Steel"],
    "rod_end": ["Steel - Polished", "Chrome - Polished", "Steel - Satin"],
    "aluminium": ["Aluminum - Satin", "Aluminum - Polished", "Aluminum"],
    "damper": ["Paint - Enamel Glossy (Black)", "Plastic - Glossy (Black)", "Plastic - Matte (Black)"],
    "chrome": ["Chrome - Polished", "Chrome", "Steel - Polished"],
    "spring": ["Paint - Enamel Glossy (Red)", "Powder Coat (Red)", "Plastic - Glossy (Red)"],
    "rubber": ["Rubber - Black", "Rubber - Soft", "Plastic - Matte (Black)"],
    "rim": ["Aluminum - Anodized Glossy (Black)", "Paint - Enamel Glossy (Black)", "Plastic - Matte (Black)"],
    "frame": ["Powder Coat (Blue)", "Paint - Enamel Glossy (Blue)", "Plastic - Glossy (Blue)"],
}
APPEARANCE_LIBRARIES = ["Fusion Appearance Library", "Fusion 360 Appearance Library"]

# Physical materials (for mass in Inspect > Properties), same names as the
# parts script. Components with no material in the file keep Fusion's default.
MATERIALS = {
    "7075-T6": ["Aluminum 7075-T6", "Aluminum 7075", "Aluminum"],
    "4130": ["Steel AISI 4130", "Steel, Alloy", "Steel"],
}

MM_PER_CM = 10.0


# ===========================================================================
# Pure-Python helpers (no Fusion calls, so tests can run them anywhere)
# ===========================================================================

def map_vector(v, up_axis=UP_AXIS):
    """SAE direction -> Fusion model direction (same mapping as points, no scaling)."""
    x, y, z = v
    if up_axis == "Z":
        return (x, y, z)
    if up_axis == "Y":
        return (x, z, -y)
    raise ValueError("UP_AXIS must be 'Z' or 'Y'")


def to_cm(p, up_axis=UP_AXIS):
    """SAE point in mm -> Fusion point in cm (the API always works in cm)."""
    return map_vector([c / MM_PER_CM for c in p], up_axis)


def box_axes(length_dir, width_dir):
    """Exactly perpendicular unit length / width directions (Fusion is strict about it)."""
    def unit(v):
        n = sum(c * c for c in v) ** 0.5
        if n == 0:
            raise ValueError("zero-length box direction")
        return [c / n for c in v]
    l = unit(length_dir)
    dot = sum(a * b for a, b in zip(width_dir, l))
    w = unit([a - dot * b for a, b in zip(width_dir, l)])
    return l, w


def default_json_path(here):
    """results/car_assembly.json in the repo this script lives in (cad/fusion/CarAssembly)."""
    repo = os.path.normpath(os.path.join(here, "..", "..", ".."))
    return os.path.join(repo, "results", "car_assembly.json")


def read_assembly(path):
    """Read and sanity-check an assembly file."""
    with open(path, "r", encoding="utf-8") as fh:
        raw = json.load(fh)
    if "groups" not in raw or not isinstance(raw["groups"], list):
        raise ValueError("{} isn't an assembly file: run `suspkin assembly` to make one "
                         "(results/car_assembly.json)".format(os.path.basename(path)))
    if raw.get("units", "mm") != "mm":
        raise ValueError("expected units 'mm'")
    for g in raw["groups"]:
        for c in g["components"]:
            for b in c["bodies"]:
                if not b.get("solids"):
                    raise ValueError("body '{}' in '{}' has no solids".format(b.get("name"), c["name"]))
    return raw


# ===========================================================================
# Fusion helpers
# ===========================================================================

def set_name(obj, name):
    """Name a Fusion object, but never let naming stop the build."""
    try:
        obj.name = name
    except Exception:
        pass


def point(p_mm):
    return adsk.core.Point3D.create(*to_cm(p_mm))


def vector(v):
    return adsk.core.Vector3D.create(*map_vector(v))


def temp_solid(tbm, s):
    """One solid from the file as a temporary BRep body."""
    t = s["type"]
    if t == "cylinder":
        r = s["radius"] / MM_PER_CM
        return tbm.createCylinderOrCone(point(s["p1"]), r, point(s["p2"]), r)
    if t == "box":
        l, w = box_axes(s["length_dir"], s["width_dir"])
        obb = adsk.core.OrientedBoundingBox3D.create(
            point(s["center"]), vector(l), vector(w),
            s["length"] / MM_PER_CM, s["width"] / MM_PER_CM, s["height"] / MM_PER_CM)
        return tbm.createBox(obb)
    if t == "sphere":
        return tbm.createSphere(point(s["center"]), s["radius"] / MM_PER_CM)
    if t == "torus":
        try:
            return tbm.createTorus(point(s["center"]), vector(s["axis"]),
                                   s["major_radius"] / MM_PER_CM, s["minor_radius"] / MM_PER_CM)
        except Exception:
            # Older Fusion without createTorus: a flat washer the same size instead.
            return washer(tbm, s)
    raise ValueError("unknown solid type {!r}".format(t))


def washer(tbm, s):
    """A flat ring (outer cylinder minus inner) standing in for a torus."""
    c, a = s["center"], s["axis"]
    r_out = (s["major_radius"] + s["minor_radius"]) / MM_PER_CM
    r_in = (s["major_radius"] - s["minor_radius"]) / MM_PER_CM
    h = s["minor_radius"]
    p1 = point([c[i] - a[i] * h for i in range(3)])
    p2 = point([c[i] + a[i] * h for i in range(3)])
    ring = tbm.createCylinderOrCone(p1, r_out, p2, r_out)
    tbm.booleanOperation(ring, tbm.createCylinderOrCone(p1, r_in, p2, r_in),
                         adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return ring


def make_bodies(b):
    """Join a body's solids, then cut its holes. Returns a list of temporary bodies.

    Usually that's one body. Solids that don't touch (the coils of a spring)
    can't always be joined; those stay as extra bodies with the same name.
    """
    tbm = adsk.fusion.TemporaryBRepManager.get()
    union = adsk.fusion.BooleanTypes.UnionBooleanType
    solids = [temp_solid(tbm, s) for s in b["solids"]]
    bodies = [solids[0]]
    for other in solids[1:]:
        try:
            ok = tbm.booleanOperation(bodies[0], other, union)
        except Exception:
            ok = False
        if not ok:
            bodies.append(other)
    for c in b["cuts"]:
        for body in bodies:
            try:
                tbm.booleanOperation(body, temp_solid(tbm, c),
                                     adsk.fusion.BooleanTypes.DifferenceBooleanType)
            except Exception:
                pass   # a cut that misses this body
    return bodies


def new_component(parent, name, notes):
    """Add an empty component inside ``parent`` (falls back to ``parent`` if refused)."""
    try:
        occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        set_name(occ.component, name)
        return occ.component
    except Exception:
        if not any("one component" in n for n in notes):
            notes.append("This document type allows only one component, so all parts "
                         "were built in the root component.")
        return parent


def add_bodies(comp, named, parametric):
    """Put temporary bodies into ``comp``. Returns the real bodies, in order."""
    out = []
    if parametric:
        # Parametric designs need temporary bodies inside a Base Feature.
        base = comp.features.baseFeatures.add()
        base.startEdit()
        for _name, tb in named:
            comp.bRepBodies.add(tb, base)
        base.finishEdit()
        for i in range(base.bodies.count):
            out.append(base.bodies.item(i))
    else:
        for _name, tb in named:
            out.append(comp.bRepBodies.add(tb))
    for (name, _tb), real in zip(named, out):
        set_name(real, name)
    return out


class Looks:
    """Finds appearances and materials in Fusion's libraries once, then reuses them."""

    def __init__(self, app, design, notes):
        self.app, self.design, self.notes = app, design, notes
        self.cache, self.mat_cache = {}, {}

    def _library(self, names):
        for name in names:
            try:
                lib = self.app.materialLibraries.itemByName(name)
            except Exception:
                lib = None
            if lib:
                return lib
        return None

    def appearance(self, look):
        if look in self.cache:
            return self.cache[look]
        found = None
        lib = self._library(APPEARANCE_LIBRARIES)
        for name in APPEARANCES.get(look, []):
            try:
                existing = self.design.appearances.itemByName(name)
                if existing:
                    found = existing
                    break
                src = lib.appearances.itemByName(name) if lib else None
                if src:
                    found = self.design.appearances.addByCopy(src, name)
                    break
            except Exception:
                continue
        if found is None and look in APPEARANCES:
            self.notes.append("No appearance found for '{}' (tried {}). Drag one on by hand "
                              "(press A).".format(look, ", ".join(APPEARANCES[look])))
        self.cache[look] = found
        return found

    def material(self, key):
        if not key:
            return None
        if key in self.mat_cache:
            return self.mat_cache[key]
        found = None
        for lib in self.app.materialLibraries:
            for name in MATERIALS.get(key, []):
                try:
                    m = lib.materials.itemByName(name)
                except Exception:
                    m = None
                if m:
                    found = m
                    break
            if found:
                break
        if found is None:
            self.notes.append("No Fusion material matched {}.".format(key))
        self.mat_cache[key] = found
        return found


def export_step(design, json_path):
    path = os.path.splitext(json_path)[0] + ".step"
    options = design.exportManager.createSTEPExportOptions(path, design.rootComponent)
    design.exportManager.execute(options)
    return path


def build(json_path):
    raw = read_assembly(json_path)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    notes = []
    parametric = False
    try:
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
    except Exception:
        parametric = True
        notes.append("Fusion refused a direct design, so the bodies were built as Base "
                     "Features in a parametric design (slower, same result).")
    design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    root = design.rootComponent
    set_name(root, raw.get("name", "Car assembly"))
    looks = Looks(app, design, notes)

    counts = []
    t0 = time.time()
    for g in raw["groups"]:
        gcomp = new_component(root, g["name"], notes)
        n = 0
        for c in g["components"]:
            comp = new_component(gcomp, c["name"], notes)
            named, spec = [], []
            for b in c["bodies"]:
                for tb in make_bodies(b):
                    named.append((b["name"], tb))
                    spec.append(b)
            bodies = add_bodies(comp, named, parametric)
            mat = looks.material(c.get("material"))
            for b, real in zip(spec, bodies):
                try:
                    if mat:
                        real.material = mat
                    ap = looks.appearance(b.get("look"))
                    if ap:
                        real.appearance = ap
                except Exception:
                    pass   # looks are cosmetic: never fail the build for them
            n += len(bodies)
        counts.append("{}: {} parts, {} bodies".format(g["name"], len(g["components"]), n))
        adsk.doEvents()   # let Fusion redraw so you can watch it build
    try:
        app.activeViewport.fit()
    except Exception:
        pass
    step = export_step(design, json_path) if EXPORT_STEP else None
    return raw, counts, step, notes, time.time() - t0


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        here = os.path.dirname(os.path.abspath(__file__))
        path = DEFAULT_JSON if DEFAULT_JSON and os.path.isfile(DEFAULT_JSON) else default_json_path(here)
        if not os.path.isfile(path):
            dlg = ui.createFileDialog()
            dlg.title = "Choose the assembly file (results/car_assembly.json from `suspkin assembly`)"
            dlg.filter = "JSON files (*.json)"
            if dlg.showOpen() != adsk.core.DialogResults.DialogOK:
                return
            path = dlg.filename
        raw, counts, step, notes, secs = build(path)
        lines = ["Built: " + raw.get("name", ""), "From: " + path, ""]
        lines += counts
        lines += ["", "Wheelbase {:.0f} mm, front track {:.0f} mm, rear track {:.0f} mm".format(
            raw.get("wheelbase_mm", 0), raw["axles"]["front"]["track_mm"], raw["axles"]["rear"]["track_mm"])
            if "axles" in raw else ""]
        lines += ["Took {:.0f} s".format(secs), "", "STEP: " + (step or "not exported")]
        if notes:
            lines += [""] + notes
        ui.messageBox("\n".join(lines), "Car assembly")
    except ValueError as e:
        if ui:
            ui.messageBox("Could not build the car:\n\n{}".format(e), "Car assembly")
    except Exception:
        if ui:
            ui.messageBox("Something went wrong:\n\n{}".format(traceback.format_exc()), "Car assembly")
