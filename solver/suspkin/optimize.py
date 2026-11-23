"""Phase 2: let the computer nudge hardpoints toward target behaviour.

The idea
--------
1. Pick a few **design variables**. Each one moves one or more hardpoints along
   one axis by an offset in mm, within bounds you choose. Example: "move both
   upper-arm chassis pivots up or down by up to 25 mm".
2. For any set of offsets, build the geometry, run a bump sweep and measure it.
3. Score it. Each goal contributes ``weight * ((achieved - target) / scale)**2``.
   ``scale`` is "how far off counts as one unit of bad", so goals in different
   units (mm, degrees) can be added up fairly. ``weight`` says how much you care.
4. ``scipy.optimize.minimize`` (method SLSQP) changes the offsets to make the
   score as small as possible, while keeping
     * every offset inside its bounds, and
     * every rim-packaging margin >= 0 (a hard limit, never traded away).

Goals (all set in the config file, e.g. ``data/optimize_front.json``):

==================  ===========================================================
rc_migration        roll-centre movement RELATIVE TO THE CHASSIS per mm of wheel
                    travel (mm/mm), least-squares slope over the sweep.
                    Target 0 = RC stays put on the chassis, so the roll moment
                    arm (CG height - RC height) is constant as the car heaves.
rc_height           static roll-centre height above the ground (mm)
camber_gain         camber change per 10 mm of bump at ride height (deg/10 mm)
bump_steer          toe range over the whole sweep (deg). Target 0.
bj_spread           distance between the upper and lower ball joints (mm).
                    Closer joints mean higher loads in the arms and joints.
kpi                 static kingpin inclination (deg). More KPI means more
                    camber change when steering and heavier steering.
==================  ===========================================================

SLSQP is a *local* method: it walks downhill from the starting geometry and
finds the best design nearby, not necessarily the best design possible.
Different starting geometries or bounds can give different answers.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from .hardpoints import Hardpoints
from .kinematics import bump_sweep
from .metrics import compute_metrics, rc_migration
from .packaging import packaging_margins

AXES = {"x": 0, "y": 1, "z": 2}
GOALS = ("rc_migration", "rc_height", "camber_gain", "bump_steer", "bj_spread", "kpi")
FAILED_SCORE = 1e6  # score for a geometry the solver can't assemble


@dataclass
class Variable:
    """Moves ``points`` together along ``axis`` by an offset within ``bounds``."""

    name: str
    points: list[str]
    axis: str
    bounds: tuple[float, float]


@dataclass
class Goal:
    target: float
    scale: float
    weight: float


@dataclass
class OptimizeConfig:
    variables: list[Variable]
    goals: dict[str, Goal]
    travel_mm: float = 50.0
    step_mm: float = 10.0  # coarser than plots: the sweep runs thousands of times
    max_iter: int = 100

    @classmethod
    def load(cls, path: str | Path) -> "OptimizeConfig":
        raw = json.loads(Path(path).read_text())
        variables = [
            Variable(v["name"], list(v["points"]), v["axis"], tuple(v["bounds"]))
            for v in raw["variables"]
        ]
        goals = {k: Goal(**raw["goals"][k]) for k in GOALS if k in raw["goals"]}
        sweep = raw.get("sweep", {})
        return cls(
            variables, goals,
            travel_mm=sweep.get("travel_mm", 50.0),
            step_mm=sweep.get("step_mm", 10.0),
            max_iter=raw.get("max_iter", 100),
        )


def apply_offsets(hp: Hardpoints, variables: list[Variable], offsets) -> Hardpoints:
    """Return a copy of ``hp`` with every variable's offset applied."""
    new = hp.copy()
    for var, off in zip(variables, offsets):
        for name in var.points:
            group = new.chassis if name in new.chassis else new.upright
            group[name][AXES[var.axis]] += off
    return new


def measure(hp: Hardpoints, travel_mm: float = 50.0, step_mm: float = 10.0) -> dict[str, float]:
    """The achieved value of every goal for one geometry."""
    m = compute_metrics(bump_sweep(hp, travel_mm, step_mm))
    t = m["travel_mm"]
    i0 = int(np.argmin(np.abs(t)))
    return {
        "rc_migration": rc_migration(m),
        "rc_height": float(m["roll_center_height_mm"][i0]),
        "camber_gain": float(10 * np.gradient(m["camber_deg"], t)[i0]),
        "bump_steer": float(np.ptp(m["toe_deg"])),
        "bj_spread": float(np.linalg.norm(hp.upright["UBJ"] - hp.upright["LBJ"])),
        "kpi": float(m["kpi_deg"][i0]),
    }


def score(achieved: dict[str, float], goals: dict[str, Goal]) -> float:
    """Sum of weight * (scaled error)^2 over all goals. Lower is better."""
    return float(sum(
        g.weight * ((achieved[name] - g.target) / g.scale) ** 2
        for name, g in goals.items()
    ))


@dataclass
class OptimizeResult:
    hardpoints: Hardpoints      # the optimised geometry
    offsets: dict[str, float]   # variable name -> chosen offset (mm)
    before: dict[str, float]    # goal values for the starting geometry
    after: dict[str, float]     # goal values for the optimised geometry
    score_before: float
    score_after: float
    evaluations: int
    message: str                # the optimiser's own stopping reason
    at_bounds: list[str]        # variables that ended at a bound (worth widening?)


def optimize(hp: Hardpoints, cfg: OptimizeConfig, verbose: bool = False) -> OptimizeResult:
    """Run the optimisation starting from ``hp`` (all offsets zero)."""
    bounds = [v.bounds for v in cfg.variables]

    def objective(x):
        try:
            return score(measure(apply_offsets(hp, cfg.variables, x), cfg.travel_mm, cfg.step_mm),
                         cfg.goals)
        except RuntimeError:  # links can't reach: tell the optimiser "very bad"
            return FAILED_SCORE

    def constraint(x):  # every entry must stay >= 0
        return packaging_margins(apply_offsets(hp, cfg.variables, x))

    iteration = [0]

    def report(x):
        iteration[0] += 1
        print(f"  iteration {iteration[0]:3d}: score {objective(x):.4g}")

    x0 = np.zeros(len(cfg.variables))
    sol = minimize(
        objective, x0, method="SLSQP", bounds=bounds,
        constraints=[{"type": "ineq", "fun": constraint}],
        callback=report if verbose else None,
        options={
            "eps": 1e-3,       # finite-difference step (mm) for estimating slopes
            "ftol": 1e-9,
            "maxiter": cfg.max_iter,
        },
    )
    # SLSQP can end very slightly past a bound; clip back inside.
    x = np.clip(sol.x, [b[0] for b in bounds], [b[1] for b in bounds])
    best = apply_offsets(hp, cfg.variables, x)
    best.name = f"{hp.name} (optimised)"
    before = measure(hp, cfg.travel_mm, cfg.step_mm)
    after = measure(best, cfg.travel_mm, cfg.step_mm)
    return OptimizeResult(
        hardpoints=best,
        offsets={v.name: float(xi) for v, xi in zip(cfg.variables, x)},
        before=before,
        after=after,
        score_before=score(before, cfg.goals),
        score_after=score(after, cfg.goals),
        evaluations=int(sol.nfev),
        message=str(sol.message),
        at_bounds=[v.name for v, xi in zip(cfg.variables, x)
                   if min(abs(xi - v.bounds[0]), abs(xi - v.bounds[1])) < 0.05],
    )
