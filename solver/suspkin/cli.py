"""Command line tool. Examples:

    suspkin sweep data/baseline_fsae_front.json
    suspkin steer data/baseline_fsae_front.json --rack 25
    suspkin check data/baseline_fsae_front.json
    suspkin optimize data/baseline_fsae_front.json data/optimize_front.json
"""

from __future__ import annotations

import argparse
from pathlib import Path

from .export import write_metrics_csv, write_metrics_table, write_poses_json
from .hardpoints import load_hardpoints, save_hardpoints
from .kinematics import DoubleWishbone, bump_sweep, steer_sweep
from .metrics import compute_metrics, summary
from .optimize import OptimizeConfig, optimize
from .packaging import check_packaging
from .plotting import plot_comparison, plot_front_view, plot_steering, plot_sweep
from .rocker import rocker_checks
from .sideview import VehicleParams
from .steering import STEER_COLUMNS, steering_metrics, steering_summary


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

    o = sub.add_parser("optimize", help="nudge hardpoints toward the targets in a config file")
    o.add_argument("hardpoints", help="starting hardpoints JSON file")
    o.add_argument("config", help="optimiser config JSON (variables, goals)")
    o.add_argument("--out", default="results", help="output folder (default results/)")
    o.add_argument("--quiet", action="store_true", help="don't print each iteration")

    c = sub.add_parser("check", help="check that the joints fit inside the rim")
    c.add_argument("hardpoints", help="hardpoints JSON file")

    args = parser.parse_args(argv)
    {"sweep": run_sweep, "steer": run_steer, "optimize": run_optimize,
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
    write_poses_json(poses, out / f"{stem}_poses.json", name=hp.name, rocker=hp.rocker)
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


if __name__ == "__main__":
    main()
