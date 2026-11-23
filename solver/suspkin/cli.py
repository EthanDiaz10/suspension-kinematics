"""Command line entry point:  ``suspkin sweep data/baseline_fsae_front.json``"""

from __future__ import annotations

import argparse
from pathlib import Path

from .export import write_metrics_csv, write_poses_json
from .hardpoints import load_hardpoints
from .kinematics import bump_sweep
from .metrics import compute_metrics, summary
from .plotting import plot_front_view, plot_sweep


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="suspkin", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    s = sub.add_parser("sweep", help="run a bump sweep and write CSV, JSON and plots")
    s.add_argument("hardpoints", help="hardpoints JSON file")
    s.add_argument("--travel", type=float, default=50.0, help="± travel in mm (default 50)")
    s.add_argument("--step", type=float, default=1.0, help="step in mm (default 1)")
    s.add_argument("--rack", type=float, default=0.0, help="steering rack offset in mm")
    s.add_argument("--out", default="results", help="output folder (default results/)")
    args = parser.parse_args(argv)

    hp = load_hardpoints(args.hardpoints)
    poses = bump_sweep(hp, args.travel, args.step, args.rack)
    metrics = compute_metrics(poses)

    out = Path(args.out)
    stem = Path(args.hardpoints).stem
    write_metrics_csv(metrics, out / f"{stem}_sweep.csv")
    write_poses_json(poses, out / f"{stem}_poses.json", name=hp.name)
    plot_sweep(metrics, out / f"{stem}_sweep.png", title=hp.name)
    plot_front_view(poses, out / f"{stem}_front_view.png")

    print(f"{hp.name}: {len(poses)} poses, worst residual "
          f"{max(p.max_residual for p in poses):.1e} mm")
    for k, v in summary(metrics).items():
        print(f"  {k:28s} {v:+9.3f}")
    print(f"Wrote results to {out.resolve()}")


if __name__ == "__main__":
    main()
