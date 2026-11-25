# suspension-kinematics

A complete front and rear suspension design for a made-up Formula SAE style car,
taken from the first geometry points to a full car in CAD. Everything is driven by
one Python tool, so a design change is a number edit and a re-run, not a remodel.

<p align="center"><img src="docs/images/renders/front_corner.png" width="720" alt="The whole car rendered in Fusion, front-left wheel removed to show the corner"></p>

**Read the [design report](docs/design-report.md)** for the finished design and why
it is the way it is.

## What it does

The suspension is defined by a few dozen pivot points (hardpoints) in a JSON file.
From those, the `suspkin` Python package:

1. **Solves the kinematics.** It moves the wheel through bump, roll and steering and
   reports camber, toe, roll centre, caster, kingpin angle, scrub, Ackermann and the
   pushrod and rocker motion ratio.
2. **Optimises the geometry.** It moves chosen hardpoints toward targets you set,
   while keeping every joint inside the 10 inch wheel rim.
3. **Models the whole car.** It covers springs, anti-roll bars, roll per g, the
   front/rear load transfer split, anti-dive and anti-squat.
4. **Sizes the parts.** It finds the force in every link under braking, cornering
   and bumps, checks every tube and rod end, and designs and stress-checks the
   uprights and rockers (hand calcs, built-in FE, and Ansys Student).
5. **Builds the CAD.** Fusion scripts draw the whole car from the same numbers:
   four corners, dampers, anti-roll bars, steering rack, wheels and a tube space
   frame, exported as STEP.

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
| `docs/` | Design report, technical guide and images |

## More detail

* [Design report](docs/design-report.md): the design, the choices behind it and the results.
* [Technical guide](docs/technical-guide.md): every command, the conventions, and what
  each phase of the project added.
* [Full design notes](docs/design-details.md): every hardpoint and table.
* [Ansys results](docs/phase9-ansys-results.md): the front upright stress study.

## License

MIT, see [LICENSE](LICENSE).
