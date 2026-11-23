# Fusion script (Phase 3)

`SuspensionModel/` is an Autodesk Fusion Python script that **reads the
solver's JSON** and builds the corner in CAD, so the CAD never re-does any
maths and you never re-model by hand. Change a hardpoint, re-run the
optimiser or a sweep, run the script, and you have a fresh model and STEP file.

What it builds, in a brand-new design:

| Item | What it is |
|---|---|
| **Hardpoints (master)** sketch | A 3D sketch with every hardpoint as a fixed point, a line for each A-arm leg, the tie rod and the pushrod, the kingpin axis (construction line from the ground to above the upper ball joint) and WC to CP |
| **Upper A-arm**, **Lower A-arm**, **Tie rod**, **Pushrod** components | Tubes along each link (Pipe feature), a sphere at each ball joint and a rod-end eye at each chassis pivot |
| **Tyre (reference)** component | A see-through ring the size of the tyre, to check the joints sit inside the wheel |
| STEP file | Everything above, exported for other CAD tools or for sharing |

When it finishes it shows caster, KPI and every link length. Compare caster
and KPI with `suspkin sweep` output to confirm you loaded the right file.

## Install (once per PC)

1. Clone or `git pull` the repo so `cad/fusion/SuspensionModel/` is on your PC.
2. Optional but recommended: in Fusion, open **Preferences > General** and set
   **Default modeling orientation** to **Z up**. The solver uses +Z up, so the
   model then appears the right way up (otherwise it looks like it is lying
   on its side, though the geometry and STEP are still correct).
3. In Fusion, open **Utilities > Add-Ins > Scripts and Add-Ins** (or press
   **Shift+S**).
4. On the **Scripts** tab, click the green **+** next to *My Scripts* (newer
   versions: **+ > Script or add-in from device**), and choose the
   `cad/fusion/SuspensionModel` folder. *SuspensionModel* now appears in the list.

Because Fusion runs the script straight from the repo folder, `git pull`
updates it. There is nothing to copy.

## Run

1. In **Scripts and Add-Ins**, select **SuspensionModel** and click **Run**.
2. Pick a JSON file:
   * a hardpoints file such as `data/optimized_front.json` (design position), or
   * a sweep file such as `results/optimized_front_poses.json` from
     `suspkin sweep data/optimized_front.json`. It then asks which wheel
     travel to draw (e.g. `25` for 25 mm bump, `-25` for droop) and uses the
     nearest stored pose.
3. Watch it build (a few seconds), then choose where to save the STEP file
   (Cancel skips the export).

The script always opens a **new** design, so it never changes a file you
already have open.

## Change things after it runs

* **Tube sizes:** **Modify > Change Parameters**, edit `arm_tube_od`,
  `arm_tube_wall`, `tie_rod_od`, and so on. The tubes update. The default
  19.05 × 1.65 mm is 3/4" × 0.065" tube.
* **Everything else** (ball-joint size, tyre width, skipping the tyre,
  Y-up mode, a default JSON so it doesn't ask): edit the *Settings* block at
  the top of `SuspensionModel.py` (right-click the script > **Edit** opens it
  in VS Code) and run again.

## How the code is organised

| File | Job |
|---|---|
| `SuspensionModel.py` | Fusion entry point. Fusion calls `run()`. Only Fusion API calls live here |
| `suspension_geometry.py` | All the maths: reading the JSON, picking a pose, mm to cm, kingpin line, trimming tubes, rod-end directions. Standard library only, so it also runs in `pytest` |
| `SuspensionModel.manifest` | Tells Fusion this folder is a script |

`tests/test_fusion_geometry.py` checks the geometry against the solver
(caster, KPI and the kingpin ground point must match `suspkin.metrics`
exactly) and also runs the whole script against a fake Fusion API, which
catches typos before you ever open Fusion. It can't check what Fusion
actually draws, so if something looks wrong in the model, the traceback in
the error box is the thing to share.

Things to know:

* **Units:** Fusion's API always works in centimetres, whatever the display
  units are. The JSON is in mm, so `to_fusion_cm()` divides by 10 right before
  each point goes to Fusion. The script sets display units to mm for you.
* **Axes:** by default SAE axes go straight through (Fusion X, Y, Z = SAE X,
  Y, Z), so the STEP file uses the same coordinates as the JSON. Set
  `UP_AXIS = "Y"` to rotate into Fusion's Y-up default instead.
* **Pipe feature:** if your Fusion version's Pipe API fails, the script makes
  a plain tube body instead and says so in the finish message. Those tubes
  won't follow parameter changes.
* **Pushrod:** for now the "pushrod" is the direct-acting spring/damper line
  SHC to SHU, the same placeholder the solver uses. It will become a real
  pushrod and rocker in a later phase.
* **Fusion caches Python modules.** The script reloads its helper each run,
  so edits take effect. If something still seems stale, restart Fusion.
* **Script not showing up?** Some older Fusion versions want
  `"autodeskProduct": "Fusion360"` in the `.manifest` file instead of `"Fusion"`.
