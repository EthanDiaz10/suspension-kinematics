"""Command line tool. Examples:

    suspkin sweep data/baseline_fsae_front.json
    suspkin steer data/baseline_fsae_front.json --rack 25
    suspkin roll data/optimized_front.json --roll 3
    suspkin car data/optimized_front.json data/optimized_rear.json
    suspkin loads data/optimized_front.json data/optimized_rear.json
    suspkin check data/baseline_fsae_front.json
    suspkin optimize data/baseline_fsae_front.json data/optimize_front.json
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from .export import write_metrics_csv, write_metrics_table, write_poses_json
from .hardpoints import load_hardpoints, save_hardpoints
from .kinematics import DoubleWishbone, bump_sweep, steer_sweep
from .metrics import compute_metrics, summary
from .optimize import OptimizeConfig, optimize
from .packaging import check_packaging
from .fullcar import full_car, placeholder_check
from .loads import GROUP_LABELS, MEMBER_GROUPS, LoadsConfig, analyse_loads, upright_loads_dict
from .plotting import (plot_comparison, plot_front_view, plot_full_car, plot_loads, plot_roll,
                       plot_roll_front_view, plot_steering, plot_sweep)
from .roll import ROLL_COLUMNS, roll_metrics, roll_summary, roll_sweep
from .rocker import rocker_checks
from .sideview import VehicleParams
from .steering import STEER_COLUMNS, steering_metrics, steering_summary


# Whole-car keys in roll_summary, printed as their own group.
CAR_KEYS = ("roll_axis_height_at_cg_mm", "roll_moment_arm_mm", "roll_gradient_deg_per_g",
            "roll_at_design_g_deg", "front_roll_stiffness_share_pct",
            "front_load_transfer_share_pct", "front_weight_pct",
            "front_load_transfer_n_per_g", "rear_load_transfer_n_per_g",
            "front_geometric_share_pct", "rear_geometric_share_pct",
            "outer_camber_at_design_g_deg", "static_camber_for_target_deg")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="suspkin", description="Suspension kinematics tools",
                                     epilog=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    s = sub.add_parser("sweep", help="run a bump sweep and write CSV, JSON and plots")
    s.add_argument("hardpoints", help="hardpoints JSON file")
    s.add_argument("--travel", type=float, default=50.0, help="± travel in mm (default 50)")
    s.add_argument("--step", type=float, default=1.0, help="step in mm (default 1)")
    s.add_argument("--rack", type=float, default=0.0, help="steering rack offset in mm")
    s.add_argument("--out", default="results", help="output folder (default results/)")

    st = sub.add_parser("steer", help="steering sweep: steer angles and Ackermann, CSV and plot")
    st.add_argument("hardpoints", help="hardpoints JSON file")
    st.add_argument("--rack", type=float, default=25.0, help="± rack travel in mm (default 25)")
    st.add_argument("--step", type=float, default=1.0, help="step in mm (default 1)")
    st.add_argument("--travel", type=float, default=0.0, help="wheel travel to steer at, mm (default 0)")
    st.add_argument("--out", default="results", help="output folder (default results/)")

    ro = sub.add_parser("roll", help="roll sweep: both corners as the body rolls, CSV and plots")
    ro.add_argument("hardpoints", help="hardpoints JSON file")
    ro.add_argument("--roll", type=float, default=3.0, help="± body roll in degrees (default 3)")
    ro.add_argument("--step", type=float, default=0.25, help="step in degrees (default 0.25)")
    ro.add_argument("--spring-rate", type=float, default=None,
                    help="spring rate in N/mm for the roll stiffness (default: the file's "
                         "vehicle.spring_rate_n_per_mm, else 35)")
    ro.add_argument("--arb-rate", type=float, default=None,
                    help="anti-roll bar rate in N m/deg (default: the file's arb block; "
                         "0 to try it without the bar)")
    ro.add_argument("--out", default="results", help="output folder (default results/)")

    ca = sub.add_parser("car", help="whole car: front + rear roll stiffness, load transfer, roll per g")
    ca.add_argument("front", help="front hardpoints JSON file (car-level numbers are read from it)")
    ca.add_argument("rear", help="rear hardpoints JSON file")
    ca.add_argument("--roll", type=float, default=3.0, help="± body roll to sweep each axle (default 3)")
    ca.add_argument("--step", type=float, default=0.25, help="step in degrees (default 0.25)")
    ca.add_argument("--out", default="results", help="output folder (default results/)")

    lo = sub.add_parser("loads", help="load cases: tyre loads, force in every link, tube and rod end checks")
    lo.add_argument("front", help="front hardpoints JSON file (car-level numbers are read from it)")
    lo.add_argument("rear", help="rear hardpoints JSON file")
    lo.add_argument("--cases", default="data/load_cases.json",
                    help="load cases, tube sizes and ratings (default data/load_cases.json)")
    lo.add_argument("--out", default="results", help="output folder (default results/)")

    o = sub.add_parser("optimize", help="nudge hardpoints toward the targets in a config file")
    o.add_argument("hardpoints", help="starting hardpoints JSON file")
    o.add_argument("config", help="optimiser config JSON (variables, goals)")
    o.add_argument("--out", default="results", help="output folder (default results/)")
    o.add_argument("--quiet", action="store_true", help="don't print each iteration")

    c = sub.add_parser("check", help="check that the joints fit inside the rim")
    c.add_argument("hardpoints", help="hardpoints JSON file")

    args = parser.parse_args(argv)
    {"sweep": run_sweep, "steer": run_steer, "roll": run_roll, "car": run_car, "loads": run_loads, "optimize": run_optimize,
     "check": run_check}[args.command](args)


def run_check(args) -> None:
    hp = load_hardpoints(args.hardpoints)
    print(f"{'point':6s} {'axial':>8s} {'radial':>8s} {'radial margin':>14s} {'hub margin':>11s}")
    for c in check_packaging(hp):
        print(f"{c.name:6s} {c.axial_mm:8.1f} {c.radial_mm:8.1f} {c.radial_margin_mm:14.1f} "
              f"{c.axial_margin_mm:11.1f}  {'ok' if c.ok else 'CLASH'}")


def run_optimize(args) -> None:
    hp = load_hardpoints(args.hardpoints)
    cfg = OptimizeConfig.load(args.config)
    print(f"Optimising {hp.name} ({len(cfg.variables)} variables). This takes a minute or two.")
    result = optimize(hp, cfg, verbose=not args.quiet)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stem = Path(args.hardpoints).stem
    save_hardpoints(result.hardpoints, out / f"{stem}_optimized.json")
    vehicle = VehicleParams.from_hardpoints(hp)
    runs = {
        "before": compute_metrics(bump_sweep(hp, 50, 1), vehicle),
        "after": compute_metrics(bump_sweep(result.hardpoints, 50, 1), vehicle),
    }
    plot_comparison(runs, out / f"{stem}_optimized_comparison.png", title=f"{hp.name}: before vs after")

    print(f"\nStopped: {result.message} ({result.evaluations} sweeps)")
    print(f"Score {result.score_before:.3g} -> {result.score_after:.3g}  (lower is better)\n")
    print(f"{'goal':14s} {'target':>8s} {'before':>9s} {'after':>9s}")
    for name, g in cfg.goals.items():
        print(f"{name:14s} {g.target:8.3f} {result.before[name]:9.3f} {result.after[name]:9.3f}")
    print("\nHardpoint offsets (mm):")
    for name, off in result.offsets.items():
        flag = "  <- at its bound" if name in result.at_bounds else ""
        print(f"  {name:24s} {off:+7.2f}{flag}")
    print(f"\nWrote {out / (stem + '_optimized.json')} and the comparison plot.")


def run_sweep(args) -> None:
    hp = load_hardpoints(args.hardpoints)
    vehicle = VehicleParams.from_hardpoints(hp)
    poses = bump_sweep(hp, args.travel, args.step, args.rack)
    metrics = compute_metrics(poses, vehicle)

    out = Path(args.out)
    stem = Path(args.hardpoints).stem
    write_metrics_csv(metrics, out / f"{stem}_sweep.csv")
    write_poses_json(poses, out / f"{stem}_poses.json", name=hp.name, rocker=hp.rocker,
                     vehicle=vehicle)
    plot_sweep(metrics, out / f"{stem}_sweep.png", title=hp.name)
    plot_front_view(poses, out / f"{stem}_front_view.png")

    print(f"{hp.name}: {len(poses)} poses, worst residual "
          f"{max(p.max_residual for p in poses):.1e} mm")
    for k, v in summary(metrics, vehicle).items():
        print(f"  {k:36s} {v:+9.3f}")
    if hp.rocker is not None:
        print("Rocker layout at ride height:")
        static = DoubleWishbone(hp).solve(0.0)
        for k, v in rocker_checks(hp.rocker, static.points).items():
            print(f"  {k:36s} {v:9.1f}")
    print(f"Wrote results to {out.resolve()}")


def run_steer(args) -> None:
    hp = load_hardpoints(args.hardpoints)
    vehicle = VehicleParams.from_hardpoints(hp)
    if vehicle.axle == "rear":
        raise SystemExit("This is a rear corner: its toe link is fixed, so there is no steering "
                         "to sweep. Use `suspkin sweep` for its toe change in bump.")
    poses = steer_sweep(hp, args.rack, args.step, args.travel)
    steer = steering_metrics(poses, vehicle)

    out = Path(args.out)
    stem = Path(args.hardpoints).stem
    write_metrics_table(steer, STEER_COLUMNS, out / f"{stem}_steer.csv")
    plot_steering(steer, out / f"{stem}_steer.png", title=f"{hp.name}: steering")

    print(f"{hp.name}: steering ±{args.rack:g} mm rack at {args.travel:+g} mm wheel travel")
    for k, v in steering_summary(steer).items():
        print(f"  {k:36s} {v:+9.3f}")
    print("  (Ackermann: 100% = ideal, 0% = parallel steer, below 0 = anti-Ackermann)")
    print(f"Wrote results to {out.resolve()}")


def run_roll(args) -> None:
    hp = load_hardpoints(args.hardpoints)
    vehicle = VehicleParams.from_hardpoints(hp)
    if args.spring_rate is not None:
        vehicle.spring_rate_n_per_mm = args.spring_rate
    if args.arb_rate is not None:
        if hp.arb is None:
            raise SystemExit("--arb-rate needs an 'arb' block in the hardpoints file")
        hp.arb.rate_nm_per_deg = args.arb_rate
    poses = roll_sweep(hp, args.roll, args.step)
    roll = roll_metrics(poses, vehicle)

    out = Path(args.out)
    stem = Path(args.hardpoints).stem
    write_metrics_table(roll, ROLL_COLUMNS, out / f"{stem}_roll.csv")
    plot_roll(roll, out / f"{stem}_roll.png", title=f"{hp.name}: roll sweep")
    plot_roll_front_view(poses[-1], out / f"{stem}_roll_front_view.png",
                         arb_mount=hp.arb.link_mount if hp.arb else "rocker")

    print(f"{hp.name}: body roll ±{args.roll:g} deg "
          "(+ = leans right, as in a left turn; the right wheel is then the outside one)")
    summ = roll_summary(roll, vehicle)
    # Print in two groups, so the whole-car numbers stand out.
    for k, v in summ.items():
        if k not in CAR_KEYS:
            print(f"  {k:36s} {v:+9.3f}")
    print("  (camber compensation: 0% = wheel leans with the body, 100% = stays upright to the road)")
    if vehicle.axle == "front":
        print("  (roll steer: below 0 = front wheels steer out of the turn = roll understeer)")
        print(f"Whole car at {vehicle.design_lateral_g:g} g, using the rear_* numbers in this file's "
              "vehicle block (rigid tyres). For a real rear corner use `suspkin car`:")
        for k in CAR_KEYS:
            print(f"  {k:36s} {summ[k]:+9.3f}")
        print(f"  (static_camber_for_target: the static camber that puts the outside tyre at "
              f"{vehicle.target_outside_camber_deg:+g} deg at {vehicle.design_lateral_g:g} g)")
    else:
        print("  (roll steer: above 0 = rear wheels steer into the turn = roll oversteer)")
        print("Whole-car numbers (roll per g, load transfer split) need both axles: "
              "suspkin car <front.json> <rear.json>")
    print(f"Wrote results to {out.resolve()}")



# Groups of whole-car numbers, printed in this order with a heading each.
CAR_GROUPS = (
    ("Each axle", ("front_roll_stiffness_springs_nm_per_deg", "front_roll_stiffness_bar_nm_per_deg",
                   "front_roll_stiffness_nm_per_deg", "rear_roll_stiffness_springs_nm_per_deg",
                   "rear_roll_stiffness_bar_nm_per_deg", "rear_roll_stiffness_nm_per_deg",
                   "front_rc_height_mm", "rear_rc_height_mm", "front_track_mm", "rear_track_mm",
                   "front_wheel_rate_n_per_mm", "rear_wheel_rate_n_per_mm",
                   "front_ride_frequency_hz", "rear_ride_frequency_hz")),
    ("Body roll", ("roll_axis_height_at_cg_mm", "roll_moment_arm_mm", "roll_gradient_deg_per_g",
                   "roll_at_design_g_deg")),
    ("Balance", ("front_weight_pct", "front_roll_stiffness_share_pct",
                 "front_load_transfer_share_pct", "front_geometric_share_pct",
                 "rear_geometric_share_pct")),
    ("Tyres at the design load", ("front_outer_camber_at_design_g_deg",
                                  "front_static_camber_for_target_deg",
                                  "rear_outer_camber_at_design_g_deg",
                                  "rear_static_camber_for_target_deg",
                                  "front_inside_wheel_load_n", "front_outside_wheel_load_n",
                                  "rear_inside_wheel_load_n", "rear_outside_wheel_load_n")),
)


def run_car(args) -> None:
    front = load_hardpoints(args.front)
    rear = load_hardpoints(args.rear)
    try:
        result = full_car(front, rear, args.roll, args.step)
    except ValueError as e:
        raise SystemExit(str(e))
    n = result.numbers
    car = result.car

    out = Path(args.out)
    stem = f"{Path(args.front).stem}+{Path(args.rear).stem}"
    plot_full_car(result, out / f"{stem}_car.png")

    print(f"Whole car: {front.name} + {rear.name}")
    print(f"  {car.mass_kg:g} kg, {100 * car.front_weight_fraction:g}% on the front, CG "
          f"{car.cg_height_mm:g} mm high, design load {car.design_lateral_g:g} g (rigid tyres)")
    for heading, keys in CAR_GROUPS:
        print(heading + ":")
        for k in keys:
            print(f"  {k:40s} {n[k]:+9.3f}")
    print("Reading it:")
    print("  front_load_transfer_share above front_weight = the car leans toward understeer "
          "(a few % above is a common start)")
    print("  geometric share = load transfer that goes through the links at the roll centre, "
          "not the springs (it acts instantly)")
    print(f"  static_camber_for_target = static camber that puts that outside tyre at "
          f"{car.target_outside_camber_deg:+g} deg at {car.design_lateral_g:g} g")
    for axle in ("front", "rear"):
        if n[f"{axle}_inside_wheel_load_n"] < 0:
            print(f"  WARNING: the {axle} inside wheel lifts before {car.design_lateral_g:g} g")
    stale = {k: v for k, v in placeholder_check(result).items() if abs(v[0] - v[1]) > 0.5}
    if stale:
        print(f"Note: {args.front} still has rear placeholders that differ from the real rear "
              "(only `suspkin roll` on the front file uses them):")
        for k, (old, new) in stale.items():
            print(f"  vehicle.{k}: {old:g} -> {new:.1f}")
    print(f"Wrote {(out / (stem + '_car.png')).resolve()}")


def run_loads(args) -> None:
    front = load_hardpoints(args.front)
    rear = load_hardpoints(args.rear)
    try:
        cfg = LoadsConfig.load(args.cases)
        car = full_car(front, rear)
    except (OSError, ValueError, KeyError) as e:
        raise SystemExit(f"{type(e).__name__}: {e}")
    result = analyse_loads(car, cfg)
    v = car.car
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stem = f"{Path(args.front).stem}+{Path(args.rear).stem}"

    print(f"Load cases on {front.name} + {rear.name}")
    print(f"  {v.mass_kg:g} kg, {100 * v.front_weight_fraction:g}% front, CG {v.cg_height_mm:g} mm, "
          f"pneumatic trail {cfg.pneumatic_trail_mm:g} mm, required safety factor {cfg.required_sf:g}")

    # 1. Tyre loads (left corners; "outside" = the left wheel in a right turn).
    print("\nTyre forces on the left corners, N (Fx + = forward, Fy + = outboard, Fz + = up):")
    print(f"  {'axle':5s} {'case':30s} {'Fx':>7s} {'Fy':>7s} {'Fz':>7s}")
    for axle, c in result.corners.items():
        for t in c.tyres:
            flag = "  (lifts off)" if t.lifted else ""
            print(f"  {axle:5s} {t.label:30s} {t.fx:7.0f} {t.fy:7.0f} {t.fz:7.0f}{flag}")

    # 2. Link forces, one table per corner.
    for axle, c in result.corners.items():
        print(f"\n{axle.capitalize()}-left link forces, N (+ = tension, - = compression):")
        cols = list(MEMBER_GROUPS)
        heads = [("toe_link" if (m == "tie_rod" and axle == "rear") else m) for m in cols]
        print(f"  {'case':30s} " + " ".join(f"{h:>11s}" for h in heads))
        for t, f in zip(c.tyres, c.forces):
            print(f"  {t.label:30s} " + " ".join(f"{f.members[m]:11.0f}" for m in cols))

    # 3. Margins.
    print("\nSafety factors (capacity / worst load; '-' = never loaded that way):")
    print(f"  {'axle':5s} {'member':12s} {'tube':>13s} {'length':>7s} {'max pull':>9s} {'max push':>9s} "
          f"{'yield SF':>9s} {'buckle SF':>10s} {'rod end SF':>11s}  tube's worst case")
    for m in result.checks:
        name = "toe_link" if (m.member == "tie_rod" and m.axle == "rear") else m.member
        def fmt(x):
            return f"{x:.1f}" if x != float("inf") else "-"
        worst = m.compression_case if m.sf_buckling <= m.sf_tension else m.tension_case
        ok = "ok" if m.sf >= cfg.required_sf else "FAIL"
        print(f"  {m.axle:5s} {name:12s} {str(m.tube):>13s} {m.length_mm:7.0f} {m.max_tension_n:9.0f} "
              f"{m.max_compression_n:9.0f} {fmt(m.sf_tension):>9s} {fmt(m.sf_buckling):>10s} "
              f"{fmt(m.sf_rod_end):>11s}  {worst} {ok}")
    print(f"  Ball joints (rating {cfg.ball_joint_rating_n:g} N):")
    for k, (f, case, sf) in result.ball_joints.items():
        print(f"    {k:10s} {f:7.0f} N in {case:30s} SF {sf:.1f}")

    print("\nLightest catalogue tube that still passes (yield and buckling, both corners):")
    for g, (tube, sf) in result.lightest.items():
        spec = cfg.members[g]
        now = spec.tube
        if tube is None:
            print(f"  {GROUP_LABELS[g]:12s} none in the catalogue passes")
            continue
        saving = 100 * (1 - tube.area_mm2 / now.area_mm2)
        print(f"  {GROUP_LABELS[g]:12s} {str(tube):>13s} mm (SF {sf:.1f}), now {now} mm: "
              f"{saving:.0f}% lighter")
    print("  (Arms are usually kept above the bare minimum for stiffness, damage tolerance and welding.)")

    print("\nNotes:")
    for note in result.notes:
        print(f"  {note}")

    # Files.
    with open(out / f"{stem}_link_forces.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["axle", "case", "fx_n", "fy_n", "fz_n", *[f"{m}_n" for m in MEMBER_GROUPS],
                    "bracket_bending_force_n"])
        for axle, c in result.corners.items():
            for t, f in zip(c.tyres, c.forces):
                w.writerow([axle, t.label, f"{t.fx:.1f}", f"{t.fy:.1f}", f"{t.fz:.1f}",
                            *[f"{f.members[m]:.1f}" for m in MEMBER_GROUPS],
                            f"{f.bracket_bending_force_n:.1f}"])
    with open(out / f"{stem}_margins.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["axle", "member", "od_mm", "wall_mm", "length_mm", "max_tension_n", "tension_case",
                    "max_compression_n", "compression_case", "sf_tension", "sf_buckling", "sf_rod_end"])
        for m in result.checks:
            w.writerow([m.axle, m.member, m.tube.od_mm, m.tube.wall_mm, f"{m.length_mm:.1f}",
                        f"{m.max_tension_n:.1f}", m.tension_case, f"{m.max_compression_n:.1f}",
                        m.compression_case, f"{m.sf_tension:.2f}", f"{m.sf_buckling:.2f}",
                        f"{m.sf_rod_end:.2f}"])
    (out / f"{stem}_upright_loads.json").write_text(json.dumps(upright_loads_dict(result), indent=2))
    plot_loads(result, out / f"{stem}_loads.png")
    print(f"\nWrote link forces, margins, upright loads (for a Fusion stress check) and a plot "
          f"to {out.resolve()}")
    if any(m.sf < cfg.required_sf for m in result.checks):
        print("WARNING: at least one member is below the required safety factor (see FAIL above)")


if __name__ == "__main__":
    main()
