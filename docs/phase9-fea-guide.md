# Phase 9 guide: parts in Fusion, stress study in Ansys Student

This walks you through the whole Phase 9 workflow on Windows, start to finish.
You don't need to have used Ansys before.

1. **PowerShell**: make the parts, the hand calcs and a quick FEA check (`suspkin parts`)
2. **Fusion**: build the parts, add fillets, export STEP
3. **Ansys Student**: the final stress study, with a mesh convergence check
4. **Compare**: hand calcs vs the quick FEA vs Ansys, in one table

Allow an afternoon the first time. Most of it is the Ansys setup, and after
the first part the rest go quickly.

---

## What's being checked, in plain words

The links (wishbones, tie rods, pushrods) were sized in Phase 8. They're
straight tubes, so they only push or pull, and a formula is enough. The
**upright**, the **rocker** and the **pushrod bracket** are chunky shapes with
holes and corners, and their stress piles up in places a formula can't see.
That's what FEA (finite element analysis) is for: the part is split into
thousands of tiny pyramids ("elements"), and the computer works out how each
one stretches under the load.

Every number you need comes out of `suspkin parts`:

* **where the forces go**: at each joint hole, per load case
  (`results/parts_fea_loads.csv`)
* **where the part is held**: the wheel bearing bore for the upright, the
  pivot bore for the rocker, and the weld faces for the bracket
* **what to expect**: hand calcs (`results/parts_hand_calcs.csv`) and a quick
  FEA (`results/parts_fea.csv`)

**Pass mark: safety factor 2.0 on yield** (`required_safety_factor` in
`data/parts.json`). In other words, the peak stress must stay below half of
7075-T6's 503 MPa yield strength, so below about 250 MPa. That's stricter
than the 1.5 used for the tubes, because machined aluminium parts with holes
fatigue sooner.

---

## 1. PowerShell: make the parts

Open **PowerShell** and type these lines one at a time (press Enter after each):

```powershell
cd ~\Projects\suspension-kinematics
git checkout main
git pull
.\.venv\Scripts\Activate.ps1
pip install -e ".[dev,fea]"
```

Use the folder where your copy of the repo lives. `git checkout main` makes sure you're on the main branch (an old feature branch won't get the new code), and `git pull` fetches Phase 9. The `pip` line adds the three FEA packages (gmsh,
scikit-fem, pyamg). It only needs doing once.

Now run the loads (they changed slightly in Phase 9) and then the parts:

```powershell
suspkin loads data\optimized_front.json data\optimized_rear.json
suspkin parts data\optimized_front.json data\optimized_rear.json --step --fea
```

The second command takes **20 to 40 minutes** on a laptop (the two uprights
are most of it; leave it running). If you only want the parts and the hand
calcs, leave off `--step --fea` and it finishes in seconds. It prints:

* **Rim clearance** of the upright: it should say `ok`.
* **Hand calculations**: one line per check, all `ok`.
* For each part: its **volume and mass**, the **STEP file** name, and the
  **peak stress per load case** from the quick FEA.

Everything is written to the `results\` folder:

| File | What it is |
|---|---|
| `optimized_front_parts.json`, `optimized_rear_parts.json` | The part shapes and loads, for the Fusion script |
| `optimized_front_upright.step` etc. | One STEP file per part (arm-to-housing corners already filleted), straight into Ansys if you skip Fusion |
| `parts_fea_loads.csv` | **The forces to type into Ansys**: part, case, hole, hole centre (x, y, z in mm), Fx, Fy, Fz (N) |
| `parts_hand_calcs.csv` | Hand calcs: beam bending at each arm root, lug checks, bearing and bolt checks |
| `parts_fea.csv` | Quick FEA results |
| `optimized_front+optimized_rear_rocker_loads.csv` | Damper, ARB drop link and rocker pivot forces (from `suspkin loads`) |

Optional: add `--converge` to re-solve each part's worst case on three
meshes (section 3.6 explains why). It roughly doubles the time, so you can
limit it to some parts, for example `--only front_rocker rear_rocker`.

> Open the CSV files in Excel (double-click). To see the upright's worst case,
> filter the `case` column to `braking_cornering (outside)`.

---

## 2. Fusion: build, fillet, export

### 2.1 Install the script (once)

1. In Fusion, press **Shift+S** (Utilities > Add-Ins > Scripts and Add-Ins).
2. On the **Scripts** tab, click the green **+** (newer versions: **+ > Script
   or add-in from device**) and choose the folder
   `cad\fusion\SuspensionParts` inside your copy of the repo.

### 2.2 Build the parts

1. In **Scripts and Add-Ins**, select **SuspensionParts** and click **Run**.
2. Choose `results\optimized_front_parts.json`.
3. A new design opens with three components: **front upright**, **front
   rocker** and **front pushrod bracket**. They sit exactly where they go on
   the car, so they line up with the `SuspensionModel` script's model.
4. A message lists what was built and where the STEP file went.

Do the same with `optimized_rear_parts.json` for the rear.

### 2.3 Add fillets (this matters for the stress)

The script joins plain blocks and cylinders, so every inside corner is
perfectly sharp. A sharp inside corner is a **stress raiser**: in theory the
stress there is infinite, and the finer you mesh it, the higher the number
climbs (you'll see this in the convergence check). Real parts always have a
radius there. So:

1. **Modify > Fillet** (or press **F**).
2. Click the edges where each **arm meets the bearing housing**, and where
   each **arm meets its boss**. Hold **Ctrl** to pick several at once.
3. Radius **4 mm**. Click **OK**. If Fusion refuses an edge, try 3 mm, or
   skip that edge.
4. Do the same on the rocker (where the arms meet the pivot tube, 3 mm) and
   on the bracket (where the tab meets its round end, 2 mm).
5. **Inspect > Properties** on each body shows its **mass**. Write it down for
   the report. Without fillets the front upright is about 400 g.

### 2.4 Export STEP for Ansys

For each part: right-click the **body** in the browser, choose **Save As STEP**
(or use **File > Export**, type **STEP**), and save it to `results\` as, for
example, `front_upright_filleted.step`.

> If anything in Fusion goes wrong, you can skip it: the STEP files from
> step 1 go straight into Ansys. They already have the most important
> fillets (where the arms meet the bearing housing / pivot tube, radius
> `fillet_mm` in `data/parts.json`); the arm-to-boss corners stay sharp, so
> read section 3.6 if a peak sits there.

---

## 3. Ansys Student: the stress study

**Install**: ansys.com/academic/students, then **Ansys Student**. It's free.
The download is big, so give it time. When the installer asks, make sure
**Mechanical** (Structural) is included.

**The node limit**: older Ansys Student versions allowed **128,000 nodes**
(mesh points). 2026 R1 solved a 905,000-node mesh of the upright without
complaint, so the limit may be higher in your version. A single part fits
easily either way, as long as you mesh it as below.

We'll do the **front upright** in full. The rocker and the bracket follow the
same steps, and their differences are listed in 3.8.

### 3.1 Start a project

1. Open **Ansys Workbench** (Start menu > Ansys 202x R2 > Workbench).
2. From the **Toolbox** on the left, drag **Static Structural** into the
   big empty area (the Project Schematic). A block with numbered rows appears.
3. **File > Save As**: `results\ansys\phase9.wbpj` inside your copy of the repo.

### 3.2 Add the material (7075-T6)

Ansys's default material is structural steel, which is wrong for this part.

1. Double-click **Engineering Data** (row 2).
2. In the table, click the empty row at the bottom ("Click here to add a new
   material") and type **Aluminium 7075-T6**.
3. From the **Toolbox** on the left, drag these onto the new material and
   fill in the values (watch the units next to each value):
   * **Physical Properties > Density**: `2810` kg m^-3
   * **Linear Elastic > Isotropic Elasticity**: Young's Modulus `71.7` GPa, Poisson's Ratio `0.33`
   * **Strength > Tensile Yield Strength**: `503` MPa
   * **Strength > Tensile Ultimate Strength**: `572` MPa
4. Do the same for **Steel 4130** (bracket): density 7850, E 205 GPa, Poisson 0.29, yield 435 MPa, ultimate 670 MPa.
5. Click **Project** (tab at the top) to go back.

### 3.3 Import the part

1. Right-click **Geometry** (row 3), then **Import Geometry > Browse**, and
   pick `front_upright_filleted.step` (or `optimized_front_upright.step`).
2. Double-click **Model** (row 4). **Mechanical** opens. This is where the
   rest happens.
3. At the top, **Units > Metric (mm, kg, N, s, mV, mA)**. This is
   important: every number below is in mm and N.
4. In the tree on the left, expand **Geometry** and click the body. In the
   **Details** panel below, set **Material > Assignment** to **Aluminium 7075-T6**.

The STEP keeps the car's axes: **X forward, Y left (outboard for this
corner), Z up**, in mm. Ansys's global coordinate system is the same, so the
force components in the CSV can be typed in as they are.

### 3.4 Mesh

1. Click **Mesh** in the tree. In Details: **Element Order > Quadratic**,
   **Element Size > 3 mm**.
2. Right-click **Mesh > Generate Mesh**. When it finishes, **Details >
   Statistics** shows **Nodes**. That number must stay below 128,000.
   At 3 mm the upright has roughly 60,000 to 80,000.
3. Better mesh where it matters, without using up the node budget: right-click
   **Mesh > Insert > Sizing**, click the fillet faces where the arms meet the
   housing (Ctrl to add more), click **Apply** next to Geometry, and set
   **Element Size 1.5 mm**. Generate again and check the node count.

### 3.5 Supports and loads

**Hold the part at the bearing bore.**

1. Right-click **Static Structural** (under Model) > **Insert > Fixed Support**.
2. Use the **face select** tool (the cursor with a square), click the inside
   face of the big bore (the bearing seat), and click **Apply**. If the bore
   is split into several faces, Ctrl-click them all.

Why fix the bore? The upright's loads come in at the ball joints and the tie
rod and go out through the wheel bearing. Holding the bore and pushing on the
joints is the same thing seen from the wheel's side. The joint forces were
solved so that they exactly balance the tyre load, so the bore ends up taking
exactly the tyre load.

**Push on each joint hole with a Remote Force.** Do this once per row of
`parts_fea_loads.csv` for your case (start with `front upright` /
`braking_cornering (outside)`, the worst case):

1. Right-click **Static Structural > Insert > Remote Force**.
2. **Geometry**: click the inside face of the hole (for example the lower ball
   joint's bore), then **Apply**.
3. **Location**: X, Y, Z = the hole centre from the CSV (`x_mm`, `y_mm`, `z_mm`).
4. **Define By > Components**: X, Y, Z = `fx_n`, `fy_n`, `fz_n` from the CSV.
5. **Behavior > Deformable**. This spreads the force over the hole like a
   real bolt or bearing would, without making the hole artificially stiff.

For the front upright's worst case that's 5 forces: UBJ, LBJ, RTO (steering
arm), CAL1 and CAL2 (the two caliper bolts carry the brake torque, because the
wheel bearing can't). Here they are, from the current design:

| Hole | Centre x, y, z (mm) | Fx, Fy, Fz (N) |
|---|---|---|
| UBJ | -8.0, 555.0, 327.8 | -1153, -793, -204 |
| LBJ | 5.0, 575.0, 142.5 | 2598, 3395, -1018 |
| RTO | 68.0, 585.0, 150.0 | -9, -1166, -83 |
| CAL1 | -75.5, 578.0, 259.0 | 164, -64, 2102 |
| CAL2 | -75.5, 580.0, 195.1 | -164, -64, 2102 |

(If the CSV disagrees, the CSV wins: it's regenerated every time you run
`suspkin parts`.)

### 3.6 Solve, and check the mesh is fine enough

1. Right-click **Solution > Insert > Stress > Equivalent (von-Mises)**, and also
   **Insert > Deformation > Total**.
2. Click **Solve** (green lightning bolt at the top). It takes under a minute.
3. Click **Equivalent Stress**. The coloured plot shows where the stress is,
   with the maximum labelled. Use **Max** and **Probe** at the top to read values.

**Is the mesh fine enough? Mesh convergence.** The answer from a coarse mesh
is too low, because big elements are too stiff to bend properly. Refine the
mesh until the answer stops changing:

| Run | Element size (body / fillets) | Nodes | Peak stress (MPa) | Change from previous |
|---|---|---|---|---|
| 1 | 4 mm / 2 mm | | | |
| 2 | 3 mm / 1.5 mm | | | |
| 3 | 2.5 mm / 1 mm | | | |

Change the sizes in **Mesh** and **Mesh > Sizing**, then click **Solve** again,
which re-meshes automatically. Fill in the table. Converged means **under 5%
change** between the last two runs. (Ansys can also do this for you: right-click
**Equivalent Stress > Insert > Convergence**, set 5%, and **Solution > Max
Refinement Loops = 3**. Watch the node limit.)

Two traps, and how to tell them apart:

* **The peak sits on the edge of the fixed bore.** A fixed support is
  infinitely stiff, which real bearings aren't, so stress piles up right at
  its edge and keeps rising as you refine. That is a *singularity*, not a real
  stress. Ignore it and read the peak a few mm away: right-click **Equivalent
  Stress > Insert > Scope... ** or simply **Probe** just outside it. The quick FEA
  in `suspkin parts` does this automatically ("peak away from the support"
  excludes the first 4 mm).
* **The peak sits in a sharp inside corner** (no fillet). Same story: it keeps
  climbing with every refinement. Add the fillet in Fusion and re-import.
  With a fillet the peak converges.

If the peak converges and it's away from the bore, it's real.

### 3.7 Record the results

For each part, for the worst case:

* the **converged peak von Mises stress** and where it is (screenshot)
* the **safety factor** = 503 / peak (7075-T6) or 435 / peak (4130). Ansys can show
  this too: **Solution > Insert > Stress Tool > Max Equivalent Stress**, then
  look at **Safety Factor**.
* the **max deformation** (mm)
* the **mesh convergence table**

Then check the other cases quickly (same mesh, just change the forces), at
least `cornering (outside)` and `braking`. **Tip**: right-click **Static Structural**
in the Workbench Project Schematic, choose **Duplicate**, and change only the forces.

Screenshots for the report: the mesh (close-up of a fillet), the stress plot
with the max labelled, and the deformed shape (**Result > Deformation scale >
5x auto**).

### 3.8 The rocker and the bracket

Same steps, with these differences:

| | Rocker | Pushrod bracket |
|---|---|---|
| Material | Aluminium 7075-T6 | Steel 4130 |
| STEP | `optimized_front_rocker.step` | `optimized_front_pushrod_bracket.step` |
| Fixed Support | Inside faces of the **pivot bore** (both ends of the tube) | The **bottom faces** of both tabs (where they're welded to the wishbone) |
| Remote Forces | PRI (pushrod), SHR (damper), ARL (ARB drop link, cornering cases only) | PRO (pushrod), on the hole faces of **both** tabs |
| Worst case | `bump` | `bump` |
| Element size | 2.5 mm | 1.5 mm (it's small: 3 mm plate) |

---

## 4. Compare: hand calc vs quick FEA vs Ansys

This table is the most convincing part of the report. Three methods, built
independently, should tell the same story.

| Part, location, case | Hand calc (MPa) | Quick FEA (MPa) | Ansys (MPa) | Ansys / hand calc |
|---|---|---|---|---|
| Front upright, LBJ arm root, braking_cornering (outside) | see `parts_hand_calcs.csv` | see `parts_fea.csv` | | |
| Front upright, steering arm root, braking_cornering (outside) | | | | |
| Front rocker, damper arm at pivot, bump | | | | |
| Front bracket, tab at weld, bump | | | | |

How to read it:

* **Ansys roughly equal to the quick FEA** (within about 10 to 15%): good. Both
  are FE of the same shape with the same loads, so they should agree. The quick
  FEA has no fillets, so near sharp corners it reads higher.
* **FEA higher than the hand calc**: expected. The beam formula gives the
  *nominal* stress across the section. The FEA adds the *stress
  concentration* at holes and corners. The ratio is the stress concentration
  factor **Kt**: 1.3 to 2 is typical for a filleted corner, and about 3 for
  an open hole. A ratio of 5 or more means the load path isn't what the hand
  calc assumed, so look at the plot to see why.
* **FEA lower than the hand calc**: the hand calcs are deliberately pessimistic
  (they add every stress at the same corner), so that's fine.

---

## If something goes wrong

| Problem | Fix |
|---|---|
| `pip install` fails on gmsh | `python -m pip install --upgrade pip`, then try again. Still failing? Skip `--step --fea`: the hand calcs and the parts files don't need them, and Fusion can make the STEP. |
| `suspkin` not recognised | Activate the venv first: `.\.venv\Scripts\Activate.ps1` |
| Ansys says "node limit exceeded" | Bigger body element size (4 mm), keep the 1.5 mm sizing only on the fillets |
| Ansys: "insufficient supports / rigid body motion" | The Fixed Support didn't stick to a face: click it and check **Geometry** says 1 Face (or more) |
| The stress plot is all one colour | **Units** isn't mm: check 3.3 step 3, and that the forces are in N |
| Fusion: fillet fails | Smaller radius, or fillet fewer edges at once |
| Deformation about 10x the quick FEA, stress about the same | Young's modulus typo (7.17 instead of 71.7 GPa). Fix it in Engineering Data, **Refresh Project**, solve again |
| Huge stress (thousands of MPa) after **Replace Geometry** | Ansys re-matched a support or force to the wrong face. Click each one and check what's highlighted; re-pick the bore and the holes. Also re-check the material |
| The peak keeps rising every refinement | It's on a sharp edge (a singularity). Fillet it, or read the stress a couple of mm away with **Probe**. See [`phase9-ansys-results.md`](phase9-ansys-results.md) for a worked example |
