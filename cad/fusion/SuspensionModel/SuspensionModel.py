"""Autodesk Fusion script: build the suspension corner from solver output.

What it does, in order:

1. Asks for a JSON file: a hardpoints file (``data/optimized_front.json``) or a
   sweep output file (``results/*_poses.json``, then it asks which wheel
   travel to draw).
2. Opens a new Fusion design and draws a **master 3D sketch**: every hardpoint
   as a fixed sketch point, a line for each A-arm leg, the tie rod, the
   pushrod, the rocker triangle and the damper, and the kingpin axis (from the
   ground up past the upper ball joint) and rocker pivot axis as construction
   lines.
3. Builds simple solids, one component per part: tubes along each link (Pipe
   feature, sizes are user parameters you can change later in
   *Modify > Change Parameters*), a sphere for each ball joint, a rod-end eye
   at each pivot, the rocker (as a triangle of bars around a pivot boss), the
   damper (body and shaft) and a see-through tyre for reference.
4. Exports everything as a STEP file.

How to install and run it: see ``cad/fusion/README.md``.

All the maths (reading the JSON, trimming tubes, axis directions) lives in
``suspension_geometry.py`` so it can be tested without Fusion. This file only
turns those numbers into Fusion objects.
"""

import importlib
import os
import sys
import traceback

import adsk.core
import adsk.fusion

# Make the helper module next to this file importable. Fusion runs scripts
# from its own working folder, so the script's folder isn't on the path.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import suspension_geometry as geo  # noqa: E402

# Fusion keeps modules loaded between runs. Reloading means edits to the
# helper file take effect next run, without restarting Fusion.
importlib.reload(geo)


# ===========================================================================
# Settings you might want to change
# ===========================================================================

# Skip the file dialog by putting a full path here, e.g.
# r"C:\Users\you\suspension-kinematics\data\optimized_front.json"
DEFAULT_JSON = ""

# "Z": keep SAE axes (+Z up). Set Fusion's Preferences > General >
#      Default modeling orientation to "Z up" so the view looks right.
# "Y": rotate into Fusion's Y-up default. The STEP file will then be rotated too.
UP_AXIS = "Z"

# Tube sizes become Fusion user parameters (name, value, comment). After the
# script runs you can change them in Modify > Change Parameters and the pipes
# update. 19.05 x 1.65 mm is 3/4" x 0.065" steel tube.
PARAMETERS = [
    ("arm_tube_od", "19.05 mm", "A-arm tube outside diameter"),
    ("arm_tube_wall", "1.65 mm", "A-arm tube wall thickness"),
    ("tie_rod_od", "15.875 mm", "Tie rod tube outside diameter"),
    ("tie_rod_wall", "1.65 mm", "Tie rod tube wall thickness"),
    ("pushrod_od", "15.875 mm", "Pushrod tube outside diameter"),
    ("pushrod_wall", "1.24 mm", "Pushrod tube wall thickness"),
]

# One Fusion component per part: (component name, OD parameter, wall parameter)
PARTS = {
    "upper_arm": ("Upper A-arm", "arm_tube_od", "arm_tube_wall"),
    "lower_arm": ("Lower A-arm", "arm_tube_od", "arm_tube_wall"),
    "tie_rod": ("Tie rod", "tie_rod_od", "tie_rod_wall"),
    "pushrod": ("Pushrod", "pushrod_od", "pushrod_wall"),
}

# End fittings (mm). These are simple stand-ins, not real part models.
BALL_JOINT_RADIUS_MM = 11.0     # sphere at each outboard (upright) end
ROD_END_RADIUS_MM = 11.0        # rod-end eye at each inboard (chassis) end
ROD_END_WIDTH_MM = 12.0         # eye thickness along its bolt

# Rocker and damper (only drawn when the JSON has a pushrod/rocker). Simple
# stand-ins sized like an FSAE part, so you can see the layout and clearances.
ROCKER_BAR_RADIUS_MM = 7.0      # each side of the rocker triangle
ROCKER_BOSS_RADIUS_MM = 13.0    # pivot boss around the rocker bearing
ROCKER_BOSS_LENGTH_MM = 30.0    # along the pivot axis
DAMPER_BODY_RADIUS_MM = 22.0    # damper body (about 44 mm diameter)
DAMPER_BODY_LENGTH_MM = 110.0
DAMPER_SHAFT_RADIUS_MM = 6.0

# Reference tyre (set DRAW_TYRE = False to skip it)
DRAW_TYRE = True
TYRE_WIDTH_MM = 180.0           # about 7" wide
RIM_RADIUS_MM = 127.0           # 10" rim -> 127 mm; the tyre is drawn as a ring
TYRE_OPACITY = 0.25             # 0 = invisible, 1 = solid

# Put the corner where it sits on the whole car. Each hardpoints file has its
# origin at its own axle line; with this on, a REAR corner is moved one
# wheelbase back (-X) so front and rear STEP files line up in one assembly
# (origin at the front axle). A front corner doesn't move either way.
PLACE_ON_CAR = True

# Ask where to save the STEP file at the end
EXPORT_STEP = True


# ===========================================================================
# Small Fusion helpers
# ===========================================================================

def set_name(obj, name):
    """Give a Fusion object a name, but never let naming stop the build.

    Some names can't be changed in some document types (for example the root
    component's), and a missing label isn't worth failing the whole model for.
    """
    try:
        obj.name = name
    except Exception:
        pass


def fusion_point(p_mm):
    """SAE point in mm -> Fusion Point3D in cm (the API always works in cm)."""
    x, y, z = geo.to_fusion_cm(p_mm, UP_AXIS)
    return adsk.core.Point3D.create(x, y, z)


def ask_for_json(ui):
    """Return the JSON path to use, or None if the user cancels."""
    if DEFAULT_JSON and os.path.isfile(DEFAULT_JSON):
        return DEFAULT_JSON
    dlg = ui.createFileDialog()
    dlg.title = "Choose a hardpoints file or a *_poses.json sweep file"
    dlg.filter = "JSON files (*.json)"
    dlg.isMultiSelectEnabled = False
    if dlg.showOpen() != adsk.core.DialogResults.DialogOK:
        return None
    return dlg.filename


def ask_for_travel(ui, raw):
    """For a poses file, ask which wheel travel to draw. None = cancelled."""
    travels = geo.available_travels(raw)
    prompt = (
        "This sweep has wheel travel from {:+.0f} to {:+.0f} mm.\n"
        "Which travel should be drawn? (0 = design position, + = bump)"
    ).format(min(travels), max(travels))
    value, cancelled = ui.inputBox(prompt, "Wheel travel (mm)", "0")
    if cancelled:
        return None
    return float(value)


def add_user_parameters(design):
    """Create (or update) the tube size parameters."""
    params = design.userParameters
    for name, value, comment in PARAMETERS:
        existing = params.itemByName(name)
        if existing:
            existing.expression = value
        else:
            params.add(name, adsk.core.ValueInput.createByString(value), "mm", comment)


def new_component(root, name, notes=None):
    """Add an empty component at the origin (no move, no rotation).

    Newer Fusion versions have "Part Design" documents that allow only one
    component. If adding a component is refused, everything goes into the
    root component instead (bodies keep their names, so it's still tidy).
    """
    try:
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        set_name(occ.component, name)
        return occ.component
    except Exception:
        if notes is not None and not any("one component" in n for n in notes):
            notes.append("This document type allows only one component, "
                         "so all parts were built in the root component.")
        return root


def add_solid_bodies(comp, named_bodies, feature_name):
    """Put TemporaryBRep solids into ``comp`` and name them.

    In a parametric (timeline) design, solids made with the TemporaryBRepManager
    have to go inside a Base Feature. It shows in the timeline as one item.
    """
    base = comp.features.baseFeatures.add()
    set_name(base, feature_name)
    base.startEdit()
    for _name, body in named_bodies:
        comp.bRepBodies.add(body, base)
    base.finishEdit()
    # Bodies come back in the order they were added, so name them now.
    for i, (name, _body) in enumerate(named_bodies):
        if i < base.bodies.count:
            set_name(base.bodies.item(i), name)
    return base


def make_tube_with_temp_brep(start, end, od_mm, wall_mm):
    """Fallback tube (outer cylinder minus inner cylinder) if Pipe fails."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    p0, p1 = fusion_point(start), fusion_point(end)
    r_out = od_mm / 2.0 / geo.MM_PER_CM
    r_in = (od_mm / 2.0 - wall_mm) / geo.MM_PER_CM
    outer = tbm.createCylinderOrCone(p0, r_out, p1, r_out)
    inner = tbm.createCylinderOrCone(p0, r_in, p1, r_in)
    tbm.booleanOperation(outer, inner, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return outer


# ===========================================================================
# Building the model
# ===========================================================================

def build_master_sketch(root, points, rocker_axis=None):
    """One 3D sketch with every hardpoint, link line and the kingpin axis."""
    sketch = root.sketches.add(root.xYConstructionPlane)
    set_name(sketch, "Hardpoints (master)")
    sketch.isComputeDeferred = True   # faster: solve the sketch once at the end

    # A sketch point per hardpoint. Lines below reuse these points, so their
    # ends are joined (coincident) instead of just sitting on top of each other.
    # On the root XY plane, sketch coordinates are the same as model coordinates.
    sketch_points = {}
    for name in geo.ALL_POINTS:
        if name in points:   # SHU only on old designs, rocker points only on new
            sketch_points[name] = sketch.sketchPoints.add(fusion_point(points[name]))

    lines = sketch.sketchCurves.sketchLines
    for _label, a, b, _group in geo.skeleton_links(points):
        lines.addByTwoPoints(sketch_points[a], sketch_points[b])

    if rocker_axis is not None:
        # Rocker triangle and damper line, then the pivot axis as construction.
        for a, b in (("RKP", "PRI"), ("RKP", "SHR"), ("PRI", "SHR"), ("SHR", "SHC")):
            lines.addByTwoPoints(sketch_points[a], sketch_points[b])
        a0, a1 = geo.rocker_axis_line(points, rocker_axis)
        lines.addByTwoPoints(fusion_point(a0), fusion_point(a1)).isConstruction = True

    # Kingpin (steering) axis as a construction line: ground -> above UBJ.
    ground, top = geo.kingpin_line(points)
    kingpin = lines.addByTwoPoints(fusion_point(ground), fusion_point(top))
    kingpin.isConstruction = True

    # Wheel centre to contact patch, also construction.
    wc_cp = lines.addByTwoPoints(sketch_points["WC"], sketch_points["CP"])
    wc_cp.isConstruction = True

    # Fix the hardpoints so dragging in the sketch can't move them.
    for sp in sketch_points.values():
        sp.isFixed = True

    sketch.isComputeDeferred = False
    return sketch


def build_tube(comp, seg, od_param, wall_param, od_mm, wall_mm, notes):
    """Tube along one trimmed link line. Tries the Pipe feature first."""
    # Each part draws its tube path in its own sketch so the Pipe feature and
    # its path live in the same component (simpler for Fusion and for STEP).
    sketch = comp.sketches.add(comp.xYConstructionPlane)
    set_name(sketch, seg["name"] + " path")
    line = sketch.sketchCurves.sketchLines.addByTwoPoints(
        fusion_point(seg["start"]), fusion_point(seg["end"])
    )

    try:
        path = comp.features.createPath(line, False)   # False = just this line
        pipes = comp.features.pipeFeatures
        pipe_in = pipes.createInput(path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        pipe_in.sectionType = adsk.fusion.PipeSectionTypes.CircularPipeSectionType
        # Using the parameter names means the pipe follows later parameter edits.
        pipe_in.sectionSize = adsk.core.ValueInput.createByString(od_param)
        pipe_in.isHollow = True
        pipe_in.sectionThickness = adsk.core.ValueInput.createByString(wall_param)
        pipe = pipes.add(pipe_in)
        set_name(pipe, seg["name"])
        set_name(pipe.bodies.item(0), seg["name"])
    except Exception:  # older Fusion versions, or an API change
        notes.append("Pipe feature failed for {}, used a plain tube body instead.".format(seg["name"]))
        body = make_tube_with_temp_brep(seg["start"], seg["end"], od_mm, wall_mm)
        add_solid_bodies(comp, [(seg["name"], body)], seg["name"] + " (tube)")


def eye_body(centre_mm, axis, radius_mm, width_mm):
    """A short cylinder (rod-end eye, boss) centred on a point, along ``axis``."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    half = geo.scale(axis, width_mm / 2.0)
    r = radius_mm / geo.MM_PER_CM
    return tbm.createCylinderOrCone(fusion_point(geo.sub(centre_mm, half)), r,
                                    fusion_point(geo.add(centre_mm, half)), r)


def end_fitting_bodies(model, links):
    """Ball-joint spheres and rod-end eyes for one part's links."""
    points = model["points"]
    tbm = adsk.fusion.TemporaryBRepManager.get()
    bodies = []
    done = set()
    r_ball = BALL_JOINT_RADIUS_MM / geo.MM_PER_CM
    for link in links:
        label, inboard, outboard, _group = link
        styles = geo.end_styles(points, link)
        for name, style in zip((inboard, outboard), styles):
            if name in done:
                continue   # two A-arm legs share one ball joint
            done.add(name)
            if style == "rod_end":
                # Rod end: a short cylinder centred on the pivot, along its bolt.
                axis = geo.rod_end_axis(points, link, at=name, rocker_axis=model["rocker_axis"],
                                        pushrod_mount=model["pushrod_mount"])
                bodies.append(("Rod end " + name,
                               eye_body(points[name], axis, ROD_END_RADIUS_MM, ROD_END_WIDTH_MM)))
            else:
                sphere = tbm.createSphere(fusion_point(points[name]), r_ball)
                bodies.append(("Ball joint " + name, sphere))
    return bodies


def bar_body(start_mm, end_mm, radius_mm):
    """A solid round bar between two points."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    r = radius_mm / geo.MM_PER_CM
    return tbm.createCylinderOrCone(fusion_point(start_mm), r, fusion_point(end_mm), r)


def build_rocker(root, model, notes):
    """The rocker: a triangle of bars, a pivot boss on its axis and two eyes."""
    points, axis = model["points"], model["rocker_axis"]
    bodies = [(name, bar_body(a, b, ROCKER_BAR_RADIUS_MM)) for name, a, b in geo.rocker_bars(points)]
    bodies.append(("Rocker pivot boss",
                   eye_body(points["RKP"], axis, ROCKER_BOSS_RADIUS_MM, ROCKER_BOSS_LENGTH_MM)))
    for name in ("PRI", "SHR"):
        bodies.append(("Rocker eye " + name,
                       eye_body(points[name], axis, ROD_END_RADIUS_MM, ROD_END_WIDTH_MM + 4.0)))
    comp = new_component(root, "Rocker", notes)
    add_solid_bodies(comp, bodies, "Rocker")
    if geo.rocker_plane_error_deg(points, axis) > 5.0:
        notes.append("PRI or SHR sits more than 5 deg out of the rocker's plane; "
                     "a flat rocker plate won't fit these points.")


def build_damper(root, model, notes):
    """The damper: a fat body at the chassis end, a thin shaft to the rocker."""
    points = model["points"]
    body_start, body_end, shaft_end = geo.damper_segments(
        points, DAMPER_BODY_LENGTH_MM, ROD_END_RADIUS_MM)
    link = ("Damper", "SHC", "SHR", "damper")
    bodies = [
        ("Damper body", bar_body(body_start, body_end, DAMPER_BODY_RADIUS_MM)),
        ("Damper shaft", bar_body(body_end, shaft_end, DAMPER_SHAFT_RADIUS_MM)),
    ]
    for name in ("SHC", "SHR"):
        axis = geo.rod_end_axis(points, link, at=name, rocker_axis=model["rocker_axis"])
        bodies.append(("Damper eye " + name,
                       eye_body(points[name], axis, ROD_END_RADIUS_MM, ROD_END_WIDTH_MM)))
    comp = new_component(root, "Damper", notes)
    add_solid_bodies(comp, bodies, "Damper")


def build_tyre(root, model, notes):
    """A see-through ring the size of the tyre, for visual reference."""
    inner_face, outer_face, radius = geo.wheel_cylinder(
        model["points"], model["wheel_axis"], TYRE_WIDTH_MM
    )
    tbm = adsk.fusion.TemporaryBRepManager.get()
    p0, p1 = fusion_point(inner_face), fusion_point(outer_face)
    r_out = radius / geo.MM_PER_CM
    r_in = min(RIM_RADIUS_MM, radius - 5.0) / geo.MM_PER_CM
    tyre = tbm.createCylinderOrCone(p0, r_out, p1, r_out)
    hole = tbm.createCylinderOrCone(p0, r_in, p1, r_in)
    tbm.booleanOperation(tyre, hole, adsk.fusion.BooleanTypes.DifferenceBooleanType)

    comp = new_component(root, "Tyre (reference)", notes)
    base = add_solid_bodies(comp, [("Tyre", tyre)], "Tyre")
    try:
        for i in range(base.bodies.count):
            base.bodies.item(i).opacity = TYRE_OPACITY
    except Exception:
        pass  # see-through is only cosmetic


def export_step(ui, design, json_path, model):
    """Ask where to save, then write a STEP file. Returns the path or None."""
    stem = os.path.splitext(os.path.basename(json_path))[0]
    if model["kind"] == "poses":
        stem += "_travel{:+.0f}mm".format(model["travel_mm"])
    dlg = ui.createFileDialog()
    dlg.title = "Save STEP file"
    dlg.filter = "STEP files (*.step)"
    dlg.initialDirectory = os.path.dirname(json_path)
    dlg.initialFilename = stem + ".step"
    if dlg.showSave() != adsk.core.DialogResults.DialogOK:
        return None
    path = dlg.filename
    options = design.exportManager.createSTEPExportOptions(path, design.rootComponent)
    design.exportManager.execute(options)
    return path


def build(ui, json_path):
    raw = geo.read_json(json_path)
    travel = 0.0
    if geo.is_poses_file(raw):
        travel = ask_for_travel(ui, raw)
        if travel is None:
            return None
    model = geo.parse_model(raw, travel_mm=travel, source=json_path)
    if PLACE_ON_CAR:
        model = geo.place_on_car(model)
    points = model["points"]

    # A fresh design, so the script never touches whatever you had open.
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    design.fusionUnitsManager.distanceDisplayUnits = (
        adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    )
    root = design.rootComponent
    set_name(root, model["name"])
    add_user_parameters(design)

    # 1. Master sketch
    build_master_sketch(root, points, model["rocker_axis"])
    adsk.doEvents()   # let Fusion redraw so you can watch progress

    # 2. One component per part: tubes + end fittings
    notes = []
    sizes_mm = {name: float(value.split()[0]) for name, value, _c in PARAMETERS}
    axle = model["axle"]
    segments = geo.tube_segments(points, BALL_JOINT_RADIUS_MM, ROD_END_RADIUS_MM, axle=axle)
    for group, (comp_name, od_param, wall_param) in PARTS.items():
        comp = new_component(root, geo.part_name(comp_name, axle), notes)
        links = [lk for lk in geo.skeleton_links(points, axle) if lk[3] == group]
        for seg in segments:
            if seg["group"] == group:
                build_tube(comp, seg, od_param, wall_param,
                           sizes_mm[od_param], sizes_mm[wall_param], notes)
        add_solid_bodies(comp, end_fitting_bodies(model, links), comp_name + " ends")
        adsk.doEvents()

    # 3. Rocker and damper (pushrod designs only)
    if model["rocker_axis"] is not None:
        build_rocker(root, model, notes)
        build_damper(root, model, notes)
        adsk.doEvents()

    # 4. Reference tyre
    if DRAW_TYRE:
        build_tyre(root, model, notes)

    app.activeViewport.fit()

    # 5. STEP export
    step_path = export_step(ui, design, json_path, model) if EXPORT_STEP else None
    return model, step_path, notes


def summary_text(json_path, model, step_path, notes):
    caster, kpi = geo.kingpin_angles_deg(model["points"])
    lines = [
        "Built: " + model["name"],
        "From: " + json_path,
    ]
    if model["kind"] == "poses":
        lines.append("Wheel travel: {:+.1f} mm".format(model["travel_mm"]))
    dx = geo.car_position_offset(model)[0]
    if model["axle"] == "rear":
        if PLACE_ON_CAR:
            lines.append("Rear corner, placed on the car: moved {:+.0f} mm in X".format(dx))
        else:
            lines.append("Rear corner, at its own axle line (PLACE_ON_CAR is off)")
    lines += [
        "",
        "Check against suspkin output:",
        "  caster {:.2f} deg, KPI {:.2f} deg".format(caster, kpi),
        "",
        "Link lengths, centre to centre (mm):",
    ]
    for name, length in geo.link_lengths(model["points"], model["axle"]).items():
        lines.append("  {:<24} {:7.1f}".format(name, length))
    if model["rocker_axis"] is None:
        lines.append("  (no rocker in this file: drawn as a direct-acting damper)")
    lines.append("")
    lines.append("STEP: " + (step_path if step_path else "not exported"))
    if notes:
        lines += [""] + notes
    return "\n".join(lines)


# ===========================================================================
# Fusion calls run() when you press Run in Scripts and Add-Ins
# ===========================================================================

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        json_path = ask_for_json(ui)
        if not json_path:
            return
        result = build(ui, json_path)
        if result is None:
            return
        model, step_path, notes = result
        ui.messageBox(summary_text(json_path, model, step_path, notes), "Suspension model")
    except ValueError as e:
        # Problems with the JSON file: show a short, readable message.
        if ui:
            ui.messageBox("Could not build the model:\n\n{}".format(e), "Suspension model")
    except Exception:
        # Anything else: show the full traceback so it can be reported/fixed.
        if ui:
            ui.messageBox("Something went wrong:\n\n{}".format(traceback.format_exc()),
                          "Suspension model")
