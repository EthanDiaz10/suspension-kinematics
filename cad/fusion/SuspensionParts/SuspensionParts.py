"""Autodesk Fusion script: build the upright, rocker and pushrod bracket (Phase 9).

What it does, in order:

1. Asks for a parts file written by ``suspkin parts`` (for example
   ``results/optimized_front_parts.json``).
2. Opens a new Fusion design. For each part (upright, rocker, pushrod
   bracket) it makes a component, builds every cylinder and box listed in the
   file, joins them into one solid and cuts the holes. The result is a plain
   solid body (a Base Feature in the timeline), placed exactly where it sits on
   the car, so it lines up with the suspension model from ``SuspensionModel``.
3. Sets each body's physical material if Fusion's library has a match
   (7075-T6 aluminium, 4130 steel), so *Inspect > Properties* shows the mass.
4. Exports everything as one STEP file next to the parts file.

After that, the usual next steps by hand (see docs/full-breakdown/phase9-fea-guide.md):
fillet the inside corners, then run the stress study.

The parts file is plain JSON written by solver/suspkin/parts.py; this script
only turns its numbers into Fusion objects. Install and run it like the
SuspensionModel script (see cad/fusion/README.md).
"""

import json
import os
import traceback

import adsk.core
import adsk.fusion

# Skip the file dialog by putting a full path here, e.g.
# r"C:\\Users\\you\\Documents\\suspension-kinematics\\results\\optimized_front_parts.json"
DEFAULT_JSON = ""

# Same meaning as in SuspensionModel: "Z" keeps SAE axes (set Fusion's
# Preferences > General > Default modeling orientation to "Z up").
UP_AXIS = "Z"

EXPORT_STEP = True

# Fusion material library names to try for each material in parts.json.
MATERIALS = {
    "7075-T6": ["Aluminum 7075-T6", "Aluminum 7075", "Aluminum"],
    "4130": ["Steel AISI 4130", "Steel, Alloy", "Steel"],
}

MM_PER_CM = 10.0


# ===========================================================================
# Pure-Python helpers (no Fusion calls, so they can be tested anywhere)
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
    """Exactly perpendicular unit length / width directions for a box.

    The parts file rounds directions to 6 decimals, which leaves them about
    1e-7 off perpendicular. Fusion's OrientedBoundingBox3D rejects that
    ("invalid argument widthDirection"), so tidy them up first: normalise the
    length direction, remove its component from the width direction, normalise.
    """
    def unit(v):
        n = sum(c * c for c in v) ** 0.5
        if n == 0:
            raise ValueError("zero-length box direction")
        return [c / n for c in v]
    l = unit(length_dir)
    dot = sum(a * b for a, b in zip(width_dir, l))
    w = unit([a - dot * b for a, b in zip(width_dir, l)])
    return l, w


def read_parts(path):
    """Read and sanity-check a parts file."""
    with open(path) as fh:
        raw = json.load(fh)
    if "parts" not in raw or not isinstance(raw["parts"], dict):
        raise ValueError("{} isn't a parts file: run `suspkin parts` to make one "
                         "(results/*_parts.json)".format(os.path.basename(path)))
    for name, part in raw["parts"].items():
        for key in ("solids", "cuts", "material"):
            if key not in part:
                raise ValueError("part '{}' is missing '{}'".format(name, key))
    return raw


# ===========================================================================
# Fusion helpers
# ===========================================================================

def set_name(obj, name):
    try:
        obj.name = name
    except Exception:
        pass


def point(p_mm):
    return adsk.core.Point3D.create(*to_cm(p_mm))


def vector(v):
    return adsk.core.Vector3D.create(*map_vector(v))


def temp_solid(tbm, s):
    """One cylinder or box from the parts file as a temporary BRep body."""
    if s["type"] == "cylinder":
        r = s["radius"] / MM_PER_CM
        return tbm.createCylinderOrCone(point(s["p1"]), r, point(s["p2"]), r)
    if s["type"] == "box":
        l, w = box_axes(s["length_dir"], s["width_dir"])
        box = adsk.core.OrientedBoundingBox3D.create(
            point(s["center"]), vector(l), vector(w),
            s["length"] / MM_PER_CM, s["width"] / MM_PER_CM, s["height"] / MM_PER_CM)
        return tbm.createBox(box)
    raise ValueError("unknown solid type {!r}".format(s["type"]))


def part_body(part):
    """Join all solids, then cut the holes. Returns one temporary body."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    solids = [temp_solid(tbm, s) for s in part["solids"]]
    body = solids[0]
    for other in solids[1:]:
        tbm.booleanOperation(body, other, adsk.fusion.BooleanTypes.UnionBooleanType)
    for c in part["cuts"]:
        tbm.booleanOperation(body, temp_solid(tbm, c), adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return body


def new_component(root, name, notes):
    try:
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        set_name(occ.component, name)
        return occ.component
    except Exception:
        if not any("one component" in n for n in notes):
            notes.append("This document type allows only one component, so all parts "
                         "were built in the root component.")
        return root


def add_body(comp, body, name):
    base = comp.features.baseFeatures.add()
    set_name(base, name)
    base.startEdit()
    comp.bRepBodies.add(body, base)
    base.finishEdit()
    if base.bodies.count > 0:
        b = base.bodies.item(0)
        set_name(b, name)
        return b
    return None


def apply_material(app, body, material, notes):
    """Set the body's physical material from Fusion's library, if one matches."""
    if body is None:
        return
    for lib in app.materialLibraries:
        for name in MATERIALS.get(material, []):
            try:
                m = lib.materials.itemByName(name)
            except Exception:
                m = None
            if m:
                try:
                    body.material = m
                    return
                except Exception:
                    pass
    notes.append("No Fusion library material matched {}: set it by hand "
                 "(right-click the body > Physical Material).".format(material))


def export_step(design, json_path):
    path = os.path.splitext(json_path)[0] + ".step"
    options = design.exportManager.createSTEPExportOptions(path, design.rootComponent)
    design.exportManager.execute(options)
    return path


def build(json_path):
    raw = read_parts(json_path)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    root = design.rootComponent
    set_name(root, raw.get("corner", "Suspension parts"))
    notes, built = [], []
    for key, part in raw["parts"].items():
        label = part.get("name", key)
        comp = new_component(root, label, notes)
        body = add_body(comp, part_body(part), label)
        apply_material(app, body, part["material"], notes)
        built.append("{} ({}, {} solids, {} holes)".format(label, part["material"],
                                                            len(part["solids"]), len(part["cuts"])))
        adsk.doEvents()
    app.activeViewport.fit()
    step = export_step(design, json_path) if EXPORT_STEP else None
    return built, step, notes


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        path = DEFAULT_JSON
        if not (path and os.path.isfile(path)):
            dlg = ui.createFileDialog()
            dlg.title = "Choose a parts file (results/*_parts.json from `suspkin parts`)"
            dlg.filter = "JSON files (*.json)"
            if dlg.showOpen() != adsk.core.DialogResults.DialogOK:
                return
            path = dlg.filename
        built, step, notes = build(path)
        lines = ["Built:"] + ["  " + b for b in built]
        lines += ["", "STEP: " + (step or "not exported"),
                  "", "Next: fillet the inside corners (3-5 mm), then run the stress study",
                  "(docs/full-breakdown/phase9-fea-guide.md)."]
        if notes:
            lines += [""] + notes
        ui.messageBox("\n".join(lines), "Suspension parts")
    except ValueError as e:
        if ui:
            ui.messageBox("Could not build the parts:\n\n{}".format(e), "Suspension parts")
    except Exception:
        if ui:
            ui.messageBox("Something went wrong:\n\n{}".format(traceback.format_exc()),
                          "Suspension parts")
