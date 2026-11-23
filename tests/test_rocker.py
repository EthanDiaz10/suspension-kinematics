"""Tests for the pushrod / rocker stage.

Same idea as the main solver tests: every rigid part must keep its shape at
every pose, otherwise the motion ratio is wrong.
"""

import json
from itertools import combinations
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import brentq

import suspkin as sk
from suspkin.rocker import angle_about_axis, rocker_angles, rotate_about_axis

DATA = Path(__file__).resolve().parents[1] / "data" / "baseline_fsae_front.json"
TOL_MM = 1e-6


@pytest.fixture(scope="module")
def hp():
    return sk.load_hardpoints(DATA)


@pytest.fixture(scope="module")
def sweep(hp):
    return sk.bump_sweep(hp, travel_mm=50, step_mm=2)


def dist(a, b):
    return float(np.linalg.norm(a - b))


def test_baseline_has_a_rocker_and_no_direct_acting_point(hp):
    assert hp.rocker is not None and hp.rocker.mount == "lower_arm"
    assert "SHU" not in hp.upright


def test_pushrod_length_preserved(hp, sweep):
    for pose in sweep:
        assert dist(pose.points["PRI"], pose.points["PRO"]) == pytest.approx(
            hp.rocker.pushrod_length, abs=TOL_MM)


def test_pushrod_outer_end_rides_on_the_lower_arm(hp, sweep):
    """PRO keeps its distance to both arm pivots and to the ball joint."""
    design = {**hp.chassis, **hp.upright, **hp.rocker.points}
    for pose in sweep:
        for other in ("LAF", "LAA", "LBJ"):
            assert dist(pose.points["PRO"], pose.points[other]) == pytest.approx(
                dist(design["PRO"], design[other]), abs=TOL_MM)


def test_rocker_turns_as_a_rigid_body_about_its_axis(hp, sweep):
    rk = hp.rocker
    for pose in sweep[::5]:
        for a, b in combinations(("RKP", "PRI", "SHR"), 2):
            assert dist(pose.points[a], pose.points[b]) == pytest.approx(
                dist(rk.points[a], rk.points[b]), abs=TOL_MM)
        # A pure rotation about the axis never moves a point ALONG the axis.
        for name in ("PRI", "SHR"):
            along = (pose.points[name] - rk.points["RKP"]) @ rk.axis
            assert along == pytest.approx((rk.points[name] - rk.points["RKP"]) @ rk.axis, abs=TOL_MM)


def test_design_position_has_zero_rocker_angle(hp):
    pose = sk.DoubleWishbone(hp).solve(0.0)
    assert pose.rocker_angle == pytest.approx(0.0, abs=1e-9)
    for name in ("PRO", "PRI", "SHR"):
        np.testing.assert_allclose(pose.points[name], hp.rocker.points[name], atol=1e-6)


def test_closed_form_matches_a_numerical_root_find(hp, sweep):
    """Solve the rocker angle a second, independent way (bracketed root find
    on the pushrod length) and check the damper lengths agree."""
    rk = hp.rocker
    for pose in sweep[::10]:
        pro = pose.points["PRO"]

        def stretch(theta):
            pri = rotate_about_axis(rk.points["PRI"], rk.points["RKP"], rk.axis, theta)
            return dist(pri, pro) - rk.pushrod_length

        guess = pose.rocker_angle
        theta = brentq(stretch, guess - 0.2, guess + 0.2, xtol=1e-14)
        assert theta == pytest.approx(pose.rocker_angle, abs=1e-9)


def test_both_closed_form_roots_satisfy_the_pushrod(hp):
    rk = hp.rocker
    pro = rk.points["PRO"] + np.array([0.0, -2.0, 15.0])
    for theta in rocker_angles(rk, pro):
        pri = rotate_about_axis(rk.points["PRI"], rk.points["RKP"], rk.axis, theta)
        assert dist(pri, pro) == pytest.approx(rk.pushrod_length, abs=1e-9)


def test_bump_compresses_the_damper_and_rate_rises(hp, sweep):
    metrics = sk.compute_metrics(sweep)
    assert np.all(metrics["motion_ratio"] > 0.4) and np.all(metrics["motion_ratio"] < 1.2)
    s = sk.summary(metrics)
    assert s["motion_ratio_change_droop_to_bump"] > 0  # baseline is mildly progressive


def test_motion_ratio_matches_virtual_work_by_finite_difference(hp):
    """Motion ratio straight from two tiny solves either side of ride height."""
    model = sk.DoubleWishbone(hp)
    h = 1e-3
    lo, hi = model.solve(-h), model.solve(h)
    mr_fd = -(sk.metrics.spring_length_mm(hi) - sk.metrics.spring_length_mm(lo)) / (2 * h)
    metrics = sk.compute_metrics(sk.bump_sweep(hp, 2, 0.5))
    assert metrics["motion_ratio"][4] == pytest.approx(mr_fd, rel=1e-4)


def test_upright_mounted_pushrod_moves_with_the_upright(hp):
    hp2 = hp.copy()
    hp2.rocker.mount = "upright"
    for pose in sk.bump_sweep(hp2, 30, 10):
        for other in ("UBJ", "LBJ", "WC"):
            assert dist(pose.points["PRO"], pose.points[other]) == pytest.approx(
                dist(hp2.rocker.points["PRO"], hp2.upright[other]), abs=TOL_MM)


def test_pushrod_that_cannot_reach_raises(hp):
    """A tiny rocker arm (10 mm) can only take up ~20 mm of pushrod motion, so
    big wheel travel must fail loudly instead of returning nonsense."""
    hp2 = hp.copy()
    rkp = hp2.rocker.points["RKP"]
    hp2.rocker.points["PRI"] = rkp + 0.2 * (hp2.rocker.points["PRI"] - rkp)
    model = sk.DoubleWishbone(hp2)
    model.solve(2.0)  # small travel is still fine
    with pytest.raises(RuntimeError, match="pushrod"):
        model.solve(40.0)


def test_angle_about_axis_round_trip():
    origin, axis = np.zeros(3), np.array([0.0, 0.0, 1.0])
    p = np.array([10.0, 0.0, 5.0])
    q = rotate_about_axis(p, origin, axis, 0.3)
    assert angle_about_axis(origin, axis, p, q) == pytest.approx(0.3)


def test_rocker_checks_on_baseline(hp):
    static = sk.DoubleWishbone(hp).solve(0.0)
    checks = sk.rocker_checks(hp.rocker, static.points)
    assert checks["pushrod_to_rocker_arm_deg"] == pytest.approx(90.0, abs=2.0)
    assert checks["pushrod_out_of_plane_deg"] < 1.0
    assert checks["damper_out_of_plane_deg"] < 1.0


def test_hardpoints_round_trip_keeps_the_rocker(hp, tmp_path):
    out = tmp_path / "hp.json"
    sk.save_hardpoints(hp, out)
    again = sk.load_hardpoints(out)
    assert again.rocker.mount == hp.rocker.mount
    np.testing.assert_allclose(again.rocker.axis, hp.rocker.axis)
    for k, v in hp.rocker.points.items():
        np.testing.assert_allclose(again.rocker.points[k], v)


def test_old_direct_acting_files_still_work(tmp_path):
    """A Phase 1 style file: SHU on the upright and no rocker block."""
    raw = json.loads(DATA.read_text())
    del raw["rocker"]
    raw["chassis"]["SHC"] = [0.0, 300.0, 470.0]
    raw["upright"]["SHU"] = [0.0, 545.0, 145.0]
    path = tmp_path / "old.json"
    path.write_text(json.dumps(raw))
    hp = sk.load_hardpoints(path)
    assert hp.rocker is None
    metrics = sk.compute_metrics(sk.bump_sweep(hp, 20, 2))
    assert np.all(np.isnan(metrics["rocker_angle_deg"]))
    assert np.all(metrics["motion_ratio"] > 0)


def test_file_without_rocker_or_shu_is_rejected(tmp_path):
    raw = json.loads(DATA.read_text())
    del raw["rocker"]
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="SHU"):
        sk.load_hardpoints(path)
