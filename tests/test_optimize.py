"""Tests for the rim packaging check and the optimiser.

The optimiser tests use a small, fast problem (2 variables, coarse sweep) so
the whole suite still runs in seconds.
"""

from pathlib import Path

import numpy as np
import pytest

import suspkin as sk
from suspkin.optimize import Goal, OptimizeConfig, Variable, apply_offsets, measure, optimize
from suspkin.packaging import check_packaging, packaging_margins, packaging_violation_mm

DATA = Path(__file__).resolve().parents[1] / "data"


@pytest.fixture
def hp():
    return sk.load_hardpoints(DATA / "baseline_fsae_front.json")


# --- packaging -------------------------------------------------------------
def test_baseline_fits_inside_rim(hp):
    assert all(c.ok for c in check_packaging(hp))
    assert packaging_violation_mm(hp) == 0.0


def test_raising_upper_ball_joint_hits_rim(hp):
    """The baseline UBJ is <1 mm from the barrel; 10 mm higher must clash."""
    hp.upright["UBJ"][2] += 10
    ubj = next(c for c in check_packaging(hp) if c.name == "UBJ")
    assert not ubj.ok and ubj.radial_margin_mm < 0
    assert packaging_violation_mm(hp) > 0
    assert np.min(packaging_margins(hp)) < 0


def test_point_past_hub_face_is_flagged(hp):
    hp.upright["LBJ"][1] = hp.upright["WC"][1] - 5  # only 5 mm inboard of centre plane
    lbj = next(c for c in check_packaging(hp) if c.name == "LBJ")
    assert lbj.axial_margin_mm < 0


def test_point_inboard_of_rim_has_no_radial_limit(hp):
    hp.upright["RTO"][1] = hp.upright["WC"][1] - 150  # beyond the rim's inner edge
    hp.upright["RTO"][2] += 200                     # far from the axis
    rto = next(c for c in check_packaging(hp) if c.name == "RTO")
    assert rto.ok and rto.radial_margin_mm == float("inf")


# --- optimiser -------------------------------------------------------------
def test_apply_offsets_moves_only_the_named_points(hp):
    var = Variable("upper pivots z", ["UAF", "UAA"], "z", (-10, 10))
    moved = apply_offsets(hp, [var], [5.0])
    assert moved.chassis["UAF"][2] == hp.chassis["UAF"][2] + 5
    assert moved.chassis["UAA"][2] == hp.chassis["UAA"][2] + 5
    np.testing.assert_array_equal(moved.chassis["LAF"], hp.chassis["LAF"])
    assert hp.chassis["UAF"][2] == 290.0  # original left untouched


def test_optimizer_reduces_rc_migration_within_bounds_and_rim(hp):
    cfg = OptimizeConfig(
        variables=[
            Variable("upper pivots z", ["UAF", "UAA"], "z", (-25, 25)),
            Variable("lower pivots y", ["LAF", "LAA"], "y", (-20, 20)),
            Variable("lower ball joint z", ["LBJ"], "z", (-10, 20)),
        ],
        goals={"rc_migration": Goal(target=0.0, scale=0.05, weight=1.0)},
        travel_mm=30, step_mm=15, max_iter=15,
    )
    result = optimize(hp, cfg)
    assert abs(result.after["rc_migration"]) < 0.5 * abs(result.before["rc_migration"])
    assert result.score_after < result.score_before
    for var in cfg.variables:
        lo, hi = var.bounds
        assert lo - 1e-9 <= result.offsets[var.name] <= hi + 1e-9
    assert all(c.ok for c in check_packaging(result.hardpoints))


def test_committed_optimised_geometry_is_better_and_fits(hp):
    """data/optimized_front.json must beat the baseline and fit in the rim."""
    cfg = OptimizeConfig.load(DATA / "optimize_front.json")
    opt = sk.load_hardpoints(DATA / "optimized_front.json")
    before, after = measure(hp), measure(opt)
    assert abs(after["rc_migration"]) < 0.2 * abs(before["rc_migration"])
    assert after["bump_steer"] < 0.05
    assert after["bj_spread"] > 185  # ball joints didn't collapse together (187 after Phase 9 lowered the UBJ 6 mm for the rim)
    assert after["kpi"] < 6.2
    assert all(c.ok for c in check_packaging(opt))
    from suspkin.optimize import score
    assert score(after, cfg.goals) < score(before, cfg.goals)


# --- Phase 4 goals and rocker variables --------------------------------------
def test_measure_reports_motion_ratio_and_anti_dive(hp):
    achieved = measure(hp)
    assert 0.5 < achieved["motion_ratio"] < 1.0
    assert np.isfinite(achieved["anti_dive"])


def test_rocker_points_can_be_optimiser_variables(hp):
    """Lengthening the rocker's damper arm raises the motion ratio."""
    var = Variable("damper arm z", ["SHR", "SHC"], "z", (-10, 10))
    moved = apply_offsets(hp, [var], [8.0])
    assert moved.rocker.points["SHR"][2] == hp.rocker.points["SHR"][2] + 8
    assert hp.rocker.points["SHR"][2] == 474.2  # original left untouched
    assert measure(moved)["motion_ratio"] > measure(hp)["motion_ratio"]
