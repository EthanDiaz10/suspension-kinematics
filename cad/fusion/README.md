# Fusion script (Phases 3 and 4)

`SuspensionModel/` is an Autodesk Fusion Python script that **reads the
solver's JSON** and builds the corner in CAD, so the CAD never re-does any
maths and you never re-model by hand. Change a hardpoint, re-run the
optimiser or a sweep, run the script, and you have a fresh model and STEP file.

What it builds, in a brand-new design:

| Item | What it is |
|---|---|
| **Hardpoints (master)** sketch | A 3D sketch with every hardpoint as a fixed point, a line for each A-arm leg, the tie rod, the pushrod, the rocker triangle and the damper, plus construction lines for the kingpin axis (from the ground to above the upper ball joint), the rocker pivot axis and WC to CP |
| **Upper A-arm**, **Lower A-arm**, **Tie rod**, **Pushrod** components | Tubes along each link (Pipe feature), a sphere at each ball joint and a rod-end eye at each pivot. The pushrod has rod ends at both ends, bolted parallel to the rocker axis at the top and to the lower-arm pivot axis at the bottom |
| **Rocker** component | A triangle of bars (pivot to pushrod, pivot to damper, and the web between), a pivot boss along the rocker axis and an eye at each end |
| **Damper** component | A body and shaft between the chassis mount (SHC) and the rocker (SHR), with an eye at each end |
| **Tyre (reference)** component | A see-through ring the size of the tyre, to check the joints sit inside the wheel |
| STEP file | Everything above, exported for other CAD tools or for sharing |

When it finishes it shows caster, KPI and every link length, including the
pushrod and the damper's eye-to-eye length. Compare caster
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
* **Pushrod and rocker:** drawn from the JSON's `"rocker"` block (hardpoints
  file) or the rocker points in each pose (`*_poses.json`), so a poses file
  shows the rocker turned to match the wheel travel you pick. Older files
  without a rocker still work: the pushrod is then drawn as the direct-acting
  SHC to SHU line, as in Phase 3. Rocker and damper sizes are in the
  *Settings* block (`ROCKER_BAR_RADIUS_MM`, `DAMPER_BODY_RADIUS_MM`, ...).
* **Rear corner (Phase 7):** pick `data/optimized_rear.json` (or a rear
  `*_poses.json`). The script reads `"axle": "rear"` from the file, names the
  tie-rod part **Toe link**, and, with `PLACE_ON_CAR = True` in the
  *Settings* block (the default), moves the whole corner one wheelbase back
  (−1550 mm in X). Each hardpoints file has its origin at its own axle line;
  this puts the rear where it sits on the car, so a front and a rear STEP
  line up when you insert both into one assembly (origin at the front axle).
  The finish message says how far it moved. Set it to `False` to keep the
  rear corner at its own origin.
* **Fusion caches Python modules.** The script reloads its helper each run,
  so edits take effect. If something still seems stale, restart Fusion.
* **Script not showing up?** Some older Fusion versions want
  `"autodeskProduct": "Fusion360"` in the `.manifest` file instead of `"Fusion"`.

## Phase 9: the upright, rocker and pushrod bracket (`SuspensionParts/`)

A second script, `SuspensionParts/`, builds the **real parts** (not just the
skeleton) from the parts file that `suspkin parts` writes:

```powershell
suspkin parts data\optimized_front.json data\optimized_rear.json
```

That writes `results\optimized_front_parts.json` and
`results\optimized_rear_parts.json`. Install the script the same way as
above (green **+**, choose the `cad/fusion/SuspensionParts` folder), run it
and pick one of those files. You get one component per part (upright,
rocker, pushrod bracket), each one solid body placed where it sits on the
car, with its material set when Fusion's library has a match, plus a STEP
file next to the parts file.

The parts are built from simple blocks and cylinders, so every inside corner
is sharp: **add 3 to 4 mm fillets** where the arms meet the housing and the
bosses before any stress study. The full workflow, including the Ansys
Student study, is in [`docs/phase9-fea-guide.md`](../../docs/phase9-fea-guide.md).
