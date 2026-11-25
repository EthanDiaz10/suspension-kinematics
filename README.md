# suspension-kinematics

A complete front and rear suspension design for a made-up Formula SAE style car,
taken from the first geometry points to a full car in CAD. Everything is driven by
one Python tool, so a design change is a number edit and a re-run, not a remodel.

<p align="center"><img src="docs/images/renders/front_corner.png" width="720" alt="The whole car rendered in Fusion, front-left wheel removed to show the corner"></p>

**Read the [design report](docs/design-report.md)** for the finished design and why
it is the way it is.

## The problem

A suspension's behaviour is set almost entirely by where its pivot points
(hardpoints) are. Move one by a few millimetres and the camber, steering feel, roll
and loads all change. Doing that by hand in CAD is slow, so the goal was a tool
where the hardpoints live in a JSON file, every number is calculated from them, and
the CAD is rebuilt from the same file. That tool was then used to design a complete
car's suspension and justify every choice.

## What we did and found

**1. Kinematics solver.** The tool moves the wheel through ±50 mm of bump, through
body roll and through steering. At every step it reports camber, toe, roll centre,
caster, kingpin angle, scrub and Ackermann. The first, hand-drawn front corner had
reasonable camber gain, but its roll centre moved 0.38 mm for every mm of bump. That
would make the car's handling change as it rises and falls.

<p align="center"><img src="docs/images/report/front_sweep.png" width="720" alt="Front corner over ±50 mm of wheel travel"></p>

**2. Optimiser.** It moves chosen hardpoints toward targets while keeping every joint
inside the 10 inch rim. It cut the roll centre movement from 0.38 to 0.04 mm per mm.
Left alone, it also squeezed the ball joints closer together, which would have
doubled the arm loads, so a ball joint spread goal was added to stop it. The rear
corner got its own run, adding goals for anti-squat and toe change.

**3. Pushrods, steering and side view.** A pushrod and rocker drive each damper, with
a motion ratio of about 0.78 that rises gently through bump. Moving the
wishbones' front pivots up or down gave 23% anti-dive at the front and 25% anti-squat at the
rear, which cuts pitch under braking and acceleration. Moving the outer tie rod end
outboard turned the steering from -16% to +17% Ackermann, which suits tight FSAE
corners.

**4. Roll and balance.** With springs alone, the front took only 39% of the
cornering load transfer, less than its 47% weight share, so the car would lean
toward oversteer. Anti-roll bars brought it to 52% (mild understeer) and set the roll
to 0.84° per g. Static camber of -1.75° puts the outside tyres at almost exactly the
-1° target at 1.5 g. One finding to fix: the front inside tyre lifts at about 1.9 g.

**5. Loads and tube sizing.** For braking, cornering, combined braking and
cornering, acceleration and a 3 g bump, the tool works out the force in every link
(up to 5.2 kN) and checks each tube and rod end for yield and buckling. Everything
passes. The weakest part is a front rod end at a safety factor of 3.8 against the
1.5 required. A second finding: the 65% front brake bias is too low, and about 77%
would be ideal at 1.6 g.

**6. Uprights and rockers.** These parts are generated around the hardpoints and
checked three ways: hand calcs, a built-in FE model, and Ansys Student. Making the
upper ball joint housing a realistic size meant lowering that joint 6 mm to fit inside
the rim. The Ansys study showed that a sharp inside corner gives a stress that never
settles as the mesh is refined. A proper fillet brought it to about 150 MPa, a safety
factor of 3.4.

**7. The whole car in CAD.** The same numbers build the complete car in Fusion: four
corners, dampers, anti-roll bars, steering rack, wheels, and a 126-tube space frame of
about 34 kg. Every wishbone pickup sits at a frame node, and a clearance check
confirms no tube comes within 3 mm of a suspension part.

## Key results

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

## How it works

The upright is treated as a rigid body held by five fixed-length links. Its
position at each step of wheel travel is found by least squares (to about 1e-13 mm).
The pushrod, rocker and anti-roll bar are solved in a second stage. The solver and
the Fusion scripts only talk through JSON files, so either side can change without
breaking the other. 184 tests check the maths against things that must be true,
such as constant link lengths and virtual work balancing.

## Quick start

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

## Repository layout

| Folder | Contents |
|---|---|
| `solver/suspkin/` | The Python package: solver, optimiser, roll and loads models, parts, frame, CLI |
| `data/` | Hardpoints, optimiser settings, load cases and part sizes (JSON) |
| `cad/fusion/` | Fusion scripts that build the corner, the parts and the whole car |
| `tests/` | `pytest` checks |
| `docs/` | Design report, images, and the [full breakdown](docs/full-breakdown/) |

## More detail

The [full breakdown](docs/full-breakdown/) folder has everything else: every command,
every hardpoint, each phase of the project step by step, and the Ansys study.

## License

MIT, see [LICENSE](LICENSE).
