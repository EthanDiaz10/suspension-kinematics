# Formula SAE Suspension Design: Hardpoints to Full-Car CAD

**Ethan Diaz** · Personal engineering project · Python, Autodesk Fusion, Ansys Student

<p align="center"><img src="docs/images/renders/front_corner.png" width="720" alt="The whole car rendered in Fusion, front-left wheel removed to show the corner"></p>

## Overview

I designed the complete front and rear suspension for a made-up Formula SAE style
car, from the first pivot points to a full 3D car in CAD. To do it, I built a Python
tool that calculates how the suspension moves and how loads flow through it. That
let me change a design, re-run it in seconds, and justify every choice with numbers.
Fusion scripts then build the whole car from the same files.

The car isn't real, and I'm not on a team. I sized it like an FSAE car because those
cars are well documented and realistic. The point was to work through the whole
design process myself.

**Full write-up:** [design report](docs/design-report.md)

## Objectives

I set my own design brief from typical FSAE numbers: a 280 kg car with driver, a
1550 mm wheelbase, 10 inch wheels and a 1.5 g design cornering load. The targets were:

- Outside tyre at -1° camber in a 1.5 g corner, where the tyre grips best
- A low roll centre that stays put on the chassis, so the handling stays consistent
- Near-zero bump steer, plus 20 to 30% anti-dive and anti-squat
- Mild understeer balance and a 2.5 to 3 Hz ride frequency
- Every part strong enough (safety factor 1.5 on tubes, 2 on machined parts) and
  every joint packaged inside the wheel rim

## Approach

**1. Built a kinematics solver.** I modelled the upright as a rigid body held by five
fixed-length links, and solved its position at each step of wheel travel with least
squares. From that the tool reports camber, toe, roll centre, caster, kingpin angle,
scrub and Ackermann through bump, roll and steering.

<p align="center"><img src="docs/images/report/front_sweep.png" width="680" alt="Front corner over ±50 mm of wheel travel"></p>

**2. Optimised the geometry.** My hand-drawn first design had a roll centre that moved
0.38 mm for every mm of bump. I wrote an optimiser that moves chosen hardpoints
toward the targets while keeping every joint inside the rim. It cut that movement to
0.04 mm per mm. It also tried to squeeze the ball joints together, which would have
doubled the arm loads, so I added a goal to stop it.

**3. Added the pushrod, steering and side-view geometry.** I designed a pushrod and
rocker for each damper (motion ratio 0.78, slightly rising). I moved the wishbone
pivots to get 23% anti-dive and 25% anti-squat, and the tie rods to turn -16%
Ackermann into +17%.

**4. Tuned roll and balance.** With springs alone the car leaned toward oversteer:
the front carried 39% of the load transfer against a 47% weight share. I sized
anti-roll bars to bring that to 52% (mild understeer) at 0.84° of roll per g. A
static camber of -1.75° puts the outside tyres at -1.0° at 1.5 g, right on target.

**5. Calculated loads and sized the members.** For seven load cases (braking,
cornering, combined braking and cornering, acceleration and a 3 g bump), I solved the
force in every link and checked each tube and rod end for yield and buckling.

**6. Designed and stress-checked the parts.** I built the uprights and rockers around
the hardpoints and checked them three ways: hand calcs, a finite-element model I set
up in Python, and Ansys Student.

**7. Built the whole car in CAD.** I wrote Fusion scripts that build all four corners,
dampers, anti-roll bars, steering rack, wheels and a 126-tube space frame. A
clearance check confirms no frame tube hits the suspension.

## Results

| | |
|---|---|
| Camber gain, front / rear | -0.38 / -0.36° per 10 mm of bump |
| Bump steer | 0.002° or less over ±50 mm |
| Roll centre height, front / rear | 17 / 25 mm, nearly fixed to the chassis |
| Anti-dive / anti-squat | 23% / 25% |
| Ackermann | +17% at full lock |
| Ride frequency | 2.85 Hz front, 2.83 Hz rear |
| Roll gradient | 0.84° per g |
| Front share of load transfer | 52% (47% front weight: mild understeer) |
| Weakest link | Front lower wishbone rod end, safety factor 3.8 (required 1.5) |
| Front upright | ~150 MPa in Ansys, safety factor 3.4 (required 2) |
| Space frame | 126 tubes, about 34 kg, no clashes with the suspension |

Every target was met. Two findings I'd fix next: the front inside tyre lifts at about
1.9 g, and the 65% front brake bias should move toward the ideal 77%.

## What I learned

- **Design is a set of trade-offs.** Lowering the upper ball joint 6 mm made a real
  bearing housing fit inside the rim, but it cost some camber gain. I had to win that
  back with static camber.
- **An optimiser only knows what you tell it.** Left alone, it found a "better" design
  that would have doubled the arm loads.
- **FEA results need judgement.** In Ansys, a sharp inside corner on the upright gave a
  peak stress that kept rising as I refined the mesh (157 to 320 MPa), because a sharp
  corner has no true stress value. Adding a real fillet brought it to about 150 MPa.
- **Check the maths independently.** 184 automated tests confirm things that must be
  true, such as constant link lengths and virtual work balancing.
- **Strength isn't the only requirement.** The arms could be half the weight and still
  pass, but I kept the 3/4 in tube for stiffness and damage tolerance.

## Skills and tools

- **Vehicle dynamics:** suspension kinematics, roll centres, anti-dive and anti-squat,
  Ackermann, load transfer, ride and roll rates
- **Structures:** free-body load paths, tube yield and buckling, hand calcs, FEA with
  mesh convergence
- **Software:** Python (NumPy, SciPy, Matplotlib, gmsh, scikit-fem), pytest, GitHub
  Actions, git
- **CAD and CAE:** Autodesk Fusion (scripted modelling, rendering, STEP export), Ansys
  Student

## Run it yourself

```bash
git clone https://github.com/EthanDiaz10/suspension-kinematics.git
cd suspension-kinematics
python -m venv .venv
# Windows: .venv\Scripts\activate    macOS/Linux: source .venv/bin/activate
pip install -e ".[dev]"

suspkin sweep data/optimized_front.json                              # bump sweep and plots
suspkin car data/optimized_front.json data/optimized_rear.json       # whole car in roll
suspkin loads data/optimized_front.json data/optimized_rear.json     # link forces, tube sizing
suspkin assembly data/optimized_front.json data/optimized_rear.json  # whole car for Fusion
pytest
```

Results go to `results/`. Then run `cad/fusion/CarAssembly` in Fusion to build the
car ([how](cad/fusion/README.md)).

| Folder | Contents |
|---|---|
| `solver/suspkin/` | The Python package: solver, optimiser, roll and loads models, parts, frame |
| `data/` | Hardpoints, optimiser settings, load cases and part sizes (JSON) |
| `cad/fusion/` | Fusion scripts that build the corner, the parts and the whole car |
| `docs/` | Design report, images, and the [full breakdown](docs/full-breakdown/) |

**Want every detail?** The [full breakdown](docs/full-breakdown/) has every command,
every hardpoint, each phase step by step, and the Ansys study.

## License

MIT, see [LICENSE](LICENSE).
