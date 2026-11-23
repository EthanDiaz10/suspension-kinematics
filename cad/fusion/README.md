# Fusion script (Phase 3, not started yet)

This folder will hold an Autodesk Fusion Python script that **reads the solver's
output** and draws the suspension, so the CAD never has to re-do any math.

Planned flow:

1. Run a sweep: `suspkin sweep data/baseline_fsae_front.json --out results`
2. In Fusion: **Utilities > Add-Ins > Scripts and Add-Ins**, add this folder,
   and run the script. It will ask for `results/<name>_poses.json`.
3. The script creates a skeleton sketch (points + lines for each link) at a
   chosen wheel travel, and later pipes/tubes for a quick visual model.

Things to know when writing it:

* Fusion's API works in **centimetres** internally, so divide the mm values
  from the JSON by 10.
* The solver uses SAE/ISO axes (+X forward, +Y left, +Z up). Fusion's default
  is Y-up, so the script will map axes explicitly (one function, one place).
* Fusion ships its own Python, so this script must only use the standard
  library (`json`, `math`) plus `adsk.core` / `adsk.fusion`. No numpy.
