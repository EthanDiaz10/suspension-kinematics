# suspension-kinematics

A personal simulation, analysis and design project: a parametric
**double-wishbone suspension kinematics solver** in Python, with an Autodesk
Fusion script (planned) that draws the result in CAD. The reference geometry
uses Formula SAE-sized dimensions because they are well documented and
realistic for a small open-wheel car.

Change a hardpoint in a JSON file, re-run one command, and get camber, toe,
caster, KPI, scrub, roll centre and motion ratio over the full wheel travel,
with no manual remodelling. Or let the optimiser move the hardpoints for you,
toward targets you set, while keeping the joints inside a 10" rim.

![Bump sweep of the baseline front corner](docs/images/bump_sweep.png)

<p align="center"><img src="docs/images/front_view.png" width="520" alt="Front view at droop, static and bump"></p>

## Quick start

```bash
git clone https://github.com/EthanDiaz10/suspension-kinematics.git
cd suspension-kinematics
python -m venv .venv
# Windows:      .venv\Scripts\activate
# macOS/Linux:  source .venv/bin/activate
pip install -e ".[dev]"

suspkin sweep data/baseline_fsae_front.json   # writes CSV, JSON and plots to results/
suspkin check data/baseline_fsae_front.json   # do the joints fit inside the rim?
suspkin optimize data/baseline_fsae_front.json data/optimize_front.json   # ~2 minutes
pytest                                         # run the tests
```

Output for the baseline:

```
static_camber_deg                     -1.000
camber_gain_deg_per_10mm              -0.371
bump_steer_deg_per_10mm               +0.001
caster_deg                            +3.542
kpi_deg                               +5.440
scrub_radius_mm                      +23.095
mech_trail_mm                        +12.738
roll_center_height_mm                +28.825
rc_migration_vs_chassis_mm_per_mm     -0.383
motion_ratio                          +0.793
```

Or from Python / a notebook (see `notebooks/01_bump_sweep.ipynb`):

```python
import suspkin as sk
hp = sk.load_hardpoints("data/baseline_fsae_front.json")
hp.chassis["UAF"][2] += 10           # try a design change
metrics = sk.compute_metrics(sk.bump_sweep(hp, travel_mm=50))
sk.plot_sweep(metrics, "my_change.png")
```

## Repository layout

| Folder | What lives there |
|---|---|
| `solver/suspkin/` | The Python package: hardpoint loading, the solver, metrics, rim packaging check, optimiser, plots, CLI |
| `data/` | Hardpoint sets and optimiser settings as JSON. Copy the baseline to start a new design |
| `tests/` | `pytest` checks (link lengths preserved, sign conventions, rim clashes, optimiser) |
| `notebooks/` | Jupyter notebooks for exploring designs |
| `cad/fusion/` | Fusion script that reads solver output (Phase 3) |
| `docs/images/` | Plots used in this README |

The solver and the CAD script only talk through files (`*_poses.json`), so
each can change without breaking the other.

## Conventions

* **Units:** millimetres and degrees.
* **Axes (SAE/ISO):** +X forward, +Y left, +Z up. Origin on the ground, on the
  car centreline, at the front axle line. We model the **front-left** corner.
* **Signs:** negative camber = top of tyre leans in; positive toe = toe-in;
  positive caster = upper ball joint behind lower; positive scrub = steering
  axis hits the ground inboard of the contact patch.

### Hardpoints

| Chassis (fixed) | | Upright (moves as one rigid body) | |
|---|---|---|---|
| `UAF` / `UAA` | upper arm front / aft pivot | `UBJ` | upper ball joint |
| `LAF` / `LAA` | lower arm front / aft pivot | `LBJ` | lower ball joint |
| `RTI` | tie rod inner (rack) | `RTO` | tie rod outer (steering arm) |
| `SHC` | spring/damper chassis end | `SHU` | spring/damper upright end |
| | | `WC` / `CP` | wheel centre / contact patch |

The baseline is a made-up but realistic FSAE-sized front corner, not a real car,
with −1° static camber. `data/optimized_front.json` is the same corner after
running the optimiser (see below).

### Rim packaging

The ball joints and tie rod end sit inside the wheel rim, so the rim limits
where they can go. Each hardpoints file has a `"packaging"` block describing a
10" rim. `suspkin check` reports, for each joint, how far it is from the rim
barrel (radial margin) and from the hub mounting face. Because the upright and
wheel move together, this only needs checking at the design position.

## How the solver works

The upright is a rigid body, so its pose is 6 numbers: a **rotation vector**
(3, avoids Euler-angle gimbal lock) and a **translation** (3). Six equations
pin them down:

1. to 5. each of the five links (4 wishbone legs and the tie rod) keeps its design length;
6. the wheel centre sits at the requested height (design + travel).

`scipy.optimize.least_squares` solves these at every step from −50 to +50 mm,
starting each step from the previous answer. The tests check that every link
length is preserved to within 0.000001 mm at every pose.

**Current simplifications:** the spring/damper is modelled as direct acting
(the pushrod and rocker come later), the contact patch moves rigidly with the
upright (no tyre deflection), and the roll centre is for pure heave of a
symmetric car.

## Optimisation (Phase 2)

![Before vs after optimisation](docs/images/optimization_comparison.png)

`suspkin optimize` moves chosen hardpoints to hit targets. Everything is set in
`data/optimize_front.json`:

* **Variables:** which points move, along which axis, and how far (mm). For
  example `"upper inner pivots z"` moves both upper-arm chassis pivots up or
  down together by up to 25 mm.
* **Goals:** each has a `target`, a `scale` (how far off counts as one unit
  of bad) and a `weight` (how much you care). The score is
  `sum(weight × ((achieved − target) / scale)²)` and lower is better.
* **Hard limits:** the variable bounds, and every joint must fit in the rim.

| Goal | Meaning | Target | Baseline | Optimised |
|---|---|---|---|---|
| `rc_migration` | roll-centre movement relative to the chassis, per mm of travel | 0 | −0.38 | −0.05 |
| `rc_height` | static roll-centre height (mm) | 30 | 28.8 | 28.0 |
| `camber_gain` | camber change per 10 mm of bump (deg) | −0.40 | −0.37 | −0.45 |
| `bump_steer` | toe change over ±50 mm (deg) | 0 | 0.009 | 0.019 |
| `bj_spread` | distance between the ball joints (mm), for joint and arm loads | 210 | 211 | 195 |
| `kpi` | kingpin inclination (deg) | 5.5 | 5.4 | 5.9 |

**Why roll-centre movement is measured against the chassis.** The centre of
gravity rides on the chassis. If the roll centre stays put on the chassis, the
lever arm between them (which sets how much the car rolls) stays the same as
the car rises and falls. Measured from the ground instead, even a perfectly
fixed roll centre "drops" 1 mm per mm of bump, which is why the
"height above ground" plot still slopes after optimising.

**Things to know:**

* The optimiser (`scipy.optimize.minimize`, SLSQP) walks downhill from the
  starting geometry. It finds a good design nearby, not necessarily the best
  one possible. Different bounds or a different starting design can give a
  different answer.
* "Iteration limit reached" just means it used its full 100-iteration budget.
  The score had already dropped from 235 to about 10.
* Variables that finish **at a bound** are printed with a marker. The
  optimiser wanted to go further, so think about whether that bound is real
  (packaging, chassis tubes) or can be widened.
* It only knows about the goals you give it, and goals pull against each
  other. Without the `bj_spread` and `kpi` goals, the same run cut RC
  migration to −0.006 but squeezed the ball joints from 211 to 172 mm apart
  and raised KPI to 6.6°. With them it settles at −0.05 with the joints
  195 mm apart. The upper ball joint is already against the rim, so the
  lower one is the only joint that can move, and lowering it fights the RC
  goal. The weights decide where that balance lands.

## Working on more than one PC

Git keeps every PC in sync through GitHub. Set up each PC once:

```bash
git clone https://github.com/EthanDiaz10/suspension-kinematics.git
cd suspension-kinematics
python -m venv .venv && pip install -e ".[dev]"   # activate .venv first, as above
```

Then every session follows the same routine:

```bash
git pull                      # 1. START: get what you did on the other PC
# ... edit, run, test ...
git add -A
git commit -m "Raise upper arm pivots 10 mm"
git push                      # 2. FINISH: send it up for the other PC
```

Tips:

* Always `git pull` before you start and `git push` before you walk away. That
  alone avoids almost all conflicts.
* `.venv/` and `results/` are not synced on purpose (see `.gitignore`). Each PC
  builds its own environment, and results can be regenerated in seconds.
* If `git pull` says there is a conflict, open the file, keep the lines you
  want, delete the `<<<<<<<`, `=======` and `>>>>>>>` markers, then
  `git add` the file and `git commit`.
* For a risky experiment, use a branch: `git switch -c try-longer-upper-arm`.
  Merge it back if you like it, delete it if not.

## Roadmap

- [x] **Phase 1:** repo scaffold, baseline hardpoints, bump sweep, camber/toe and other metrics, tests
- [x] **Phase 2:** optimisation, nudging chosen hardpoints to hit targets (camber gain, bump steer, roll centre height and migration) within 10" rim packaging limits
- [ ] Optimiser goal for scrub radius and mechanical trail
- [ ] Steering sweep and Ackermann percentage
- [ ] Pushrod and rocker stage (motion ratio through the rocker)
- [ ] Anti-dive / anti-squat (side-view geometry)
- [ ] Roll sweep (left and right corners moving opposite ways)
- [ ] **Phase 3:** Fusion script that draws the skeleton and tubes from `*_poses.json`

## License

MIT, see [LICENSE](LICENSE).
