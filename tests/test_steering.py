"""Tests for the steering sweep and Ackermann."""

from pathlib import Path

import numpy as np
import pytest

import suspkin as sk
from suspkin.hardpoints import Hardpoints
from suspkin.steering import ackermann_pct, heading_deg, ideal_inner_deg

DATA = Path(__file__).resolve().parents[1] / "data" / "baseline_fsae_front.json"


@pytest.fixture(scope="module")
def hp():
    return sk.load_hardpoints(DATA)


@pytest.fixture(scope="module")
def steer(hp):
    return sk.steering_metrics(sk.steer_sweep(hp, rack_mm=20, step_mm=2),
                               sk.VehicleParams.from_hardpoints(hp))


def test_ackermann_formula_end_points():
    t, L = 1220.0, 1550.0
    outer = np.array([5.0, 15.0, 25.0])
    ideal = ideal_inner_deg(outer, t, L)
    np.testing.assert_allclose(ackermann_pct(ideal, outer, t, L), 100.0)
    np.testing.assert_allclose(ackermann_pct(outer, outer, t, L), 0.0)    # parallel
    assert np.all(ackermann_pct(outer - 1, outer, t, L) < 0)              # anti
    # Perfect Ackermann: both axle lines meet on the rear axle line.
    o, i = np.radians(15.0), np.radians(ideal[1])
    assert 1 / np.tan(o) - 1 / np.tan(i) == pytest.approx(t / L)


def test_centred_rack_is_straight_ahead(steer):
    i0 = int(np.argmin(np.abs(steer["rack_mm"])))
    assert steer["steer_left_deg"][i0] == pytest.approx(0.0, abs=1e-9)
    assert steer["steer_right_deg"][i0] == pytest.approx(0.0, abs=1e-9)


def test_positive_rack_steers_both_wheels_left(steer):
    left_turn = steer["rack_mm"] > 0
    assert np.all(steer["steer_left_deg"][left_turn] > 0)
    assert np.all(steer["steer_right_deg"][left_turn] > 0)


def test_left_and_right_turns_are_mirror_images(steer):
    np.testing.assert_allclose(steer["steer_left_deg"], -steer["steer_right_deg"][::-1], atol=1e-9)


class RightCorner(Hardpoints):
    """Hardpoints for an explicitly mirrored RIGHT corner (spin axis to -Y)."""

    @property
    def wheel_axis(self):
        a = super().wheel_axis
        return np.array([a[0], -a[1], a[2]])


def test_mirror_trick_matches_a_real_right_corner(hp, steer):
    """Build the right corner for real (every y flipped) and steer it with the
    same rack motion. Its wheel angles must match the mirrored left corner."""
    flip = np.array([1.0, -1.0, 1.0])
    right = RightCorner(
        name="right", chassis={k: v * flip for k, v in hp.chassis.items()},
        upright={k: v * flip for k, v in hp.upright.items()},
        static_camber_deg=hp.static_camber_deg, static_toe_deg=hp.static_toe_deg,
    )
    model = sk.DoubleWishbone(right)
    straight = heading_deg(model.solve(0.0, rack=0.0).wheel_axis)
    for r, expected in zip(steer["rack_mm"], steer["steer_right_deg"]):
        pose = model.solve(0.0, rack=r)  # rack moves toward +Y for both sides
        assert heading_deg(pose.wheel_axis) - straight == pytest.approx(expected, abs=1e-6)


def test_baseline_is_slightly_anti_ackermann(steer):
    """The baseline's steering arm sits inboard of the kingpin with a front
    rack, which gives a little anti-Ackermann. Documented in docs/full-breakdown/technical-guide.md."""
    s = sk.steering_summary(steer)
    assert -30 < s["ackermann_at_lock_pct"] < 0
    assert 15 < s["inner_lock_deg"] < 30


def test_moving_the_steering_arm_outboard_adds_ackermann(hp, steer):
    """With the rack ahead of the axle, the outer tie-rod end further outboard
    angles the steering arm out, which is the classic way to add Ackermann."""
    hp2 = hp.copy()
    hp2.upright["RTO"][1] += 15.0
    s2 = sk.steering_summary(sk.steering_metrics(sk.steer_sweep(hp2, 20, 2)))
    assert s2["ackermann_at_lock_pct"] > sk.steering_summary(steer)["ackermann_at_lock_pct"]


def test_cli_steer_writes_outputs(tmp_path):
    from suspkin.cli import main

    main(["steer", str(DATA), "--rack", "10", "--step", "2", "--out", str(tmp_path)])
    for suffix in ("_steer.csv", "_steer.png"):
        assert (tmp_path / f"{DATA.stem}{suffix}").exists()


def test_optimised_design_has_pro_ackermann_and_anti_dive():
    """The Phase 4 changes applied to data/optimized_front.json."""
    opt = sk.load_hardpoints(DATA.parent / "optimized_front.json")
    v = sk.VehicleParams.from_hardpoints(opt)
    steer = sk.steering_summary(sk.steering_metrics(sk.steer_sweep(opt, 25, 1), v))
    assert steer["ackermann_at_lock_pct"] > 10
    bump = sk.summary(sk.compute_metrics(sk.bump_sweep(opt, 50, 2), v), v)
    assert 15 < bump["anti_dive_pct"] < 30
    assert bump["toe_range_deg"] < 0.02
    assert all(c.ok for c in sk.check_packaging(opt))
