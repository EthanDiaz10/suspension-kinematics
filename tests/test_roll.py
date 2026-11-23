"""Tests for the roll sweep (both corners, body rolling)."""

from pathlib import Path

import numpy as np
import pytest

import suspkin as sk
from suspkin.roll import RollModel, ground_roll_rad, mirror_point, roll_center_chassis

DATA = Path(__file__).resolve().parents[1] / "data"
BASELINE = DATA / "baseline_fsae_front.json"
OPTIMIZED = DATA / "optimized_front.json"


class RightCorner(sk.Hardpoints):
    """A real right-hand corner: its spin axis points outboard, toward -Y."""

    @property
    def wheel_axis(self):
        return mirror_point(super().wheel_axis)


@pytest.fixture(scope="module")
def hp():
    return sk.load_hardpoints(BASELINE)


@pytest.fixture(scope="module")
def poses(hp):
    return sk.roll_sweep(hp, roll_deg=2.0, step_deg=0.5)


@pytest.fixture(scope="module")
def roll(hp, poses):
    return sk.roll_metrics(poses, sk.VehicleParams.from_hardpoints(hp))


def test_sweep_hits_the_requested_roll_angles(poses):
    for rp in poses:
        assert np.degrees(ground_roll_rad(rp.left, rp.right_src)) == pytest.approx(rp.roll_deg, abs=1e-8)
        # Both contact patches sit on the road in the ground frame.
        assert rp.to_ground(rp.left.points["CP"])[2] == pytest.approx(0.0, abs=1e-8)
        assert rp.to_ground(rp.right_points()["CP"])[2] == pytest.approx(0.0, abs=1e-8)


def test_zero_roll_matches_the_bump_sweep(hp, roll):
    """With no roll it's just the static car: same camber both sides and the
    same roll centre as the symmetric construction in metrics.py."""
    i0 = int(np.argmin(np.abs(roll["roll_deg"])))
    static = sk.DoubleWishbone(hp).solve(0.0)
    assert roll["camber_left_deg"][i0] == pytest.approx(hp.static_camber_deg, abs=1e-9)
    assert roll["camber_right_deg"][i0] == pytest.approx(hp.static_camber_deg, abs=1e-9)
    assert roll["steer_left_deg"][i0] == pytest.approx(0.0, abs=1e-9)
    assert roll["rc_height_mm"][i0] == pytest.approx(sk.metrics.roll_center_height_mm(static), abs=1e-6)
    assert roll["rc_lateral_mm"][i0] == pytest.approx(0.0, abs=1e-6)


def test_left_and_right_turns_mirror_each_other(roll):
    """Rolling the other way swaps the wheels and flips the roll centre sideways."""
    np.testing.assert_allclose(roll["camber_left_deg"], roll["camber_right_deg"][::-1], atol=1e-8)
    np.testing.assert_allclose(roll["steer_left_deg"], -roll["steer_right_deg"][::-1], atol=1e-8)
    np.testing.assert_allclose(roll["rc_height_mm"], roll["rc_height_mm"][::-1], atol=1e-6)
    np.testing.assert_allclose(roll["rc_lateral_mm"], -roll["rc_lateral_mm"][::-1], atol=1e-6)
    np.testing.assert_allclose(roll["spring_roll_moment_nm"], -roll["spring_roll_moment_nm"][::-1],
                               rtol=1e-6, atol=1e-6)


def test_positive_roll_puts_the_right_wheel_in_bump(roll):
    i = int(np.argmax(roll["roll_deg"]))
    assert roll["travel_right_mm"][i] > 0 > roll["travel_left_mm"][i]
    assert roll["damper_right_mm"][i] > 0 > roll["damper_left_mm"][i]
    assert roll["spring_roll_moment_nm"][i] > 0  # the springs resist the roll


def test_mirror_trick_matches_a_real_right_corner(hp, poses):
    """Build the right corner for real (every y flipped) and solve it at the
    same bump travel: its points must match the mirrored left corner."""
    flip = np.array([1.0, -1.0, 1.0])
    right = RightCorner(
        name="right", chassis={k: v * flip for k, v in hp.chassis.items()},
        upright={k: v * flip for k, v in hp.upright.items()},
        static_camber_deg=hp.static_camber_deg, static_toe_deg=hp.static_toe_deg,
    )
    rp = poses[-1]
    real = sk.DoubleWishbone(right).solve(rp.travel)
    for name in ("UBJ", "LBJ", "RTO", "WC", "CP"):
        np.testing.assert_allclose(real.points[name], rp.right_points()[name], atol=1e-6)
    np.testing.assert_allclose(real.wheel_axis, mirror_point(rp.right_src.wheel_axis), atol=1e-9)


def test_parallel_equal_arms_lean_with_the_body():
    """Parallel, equal, horizontal arms: the upright doesn't tilt relative to
    the chassis, so the wheel leans exactly as much as the body (roll camber
    +1 deg/deg on the outside wheel, 0% compensation)."""
    hp = sk.load_hardpoints(BASELINE)
    for k in ("UAF", "UAA"):
        hp.chassis[k][1:] = [250.0, hp.upright["UBJ"][2]]
    for k in ("LAF", "LAA"):
        hp.chassis[k][1:] = [250.0, hp.upright["LBJ"][2]]
    hp.upright["UBJ"][1] = hp.upright["LBJ"][1]
    # Tie rod parallel to the arms too, so there's no bump steer to tilt things.
    hp.chassis["RTI"][1:] = [250.0, hp.upright["RTO"][2]]
    s = sk.roll_summary(sk.roll_metrics(sk.roll_sweep(hp, 1.0, 0.5)))
    assert s["roll_camber_outer_deg_per_deg"] == pytest.approx(1.0, abs=0.02)
    assert s["camber_compensation_pct"] == pytest.approx(0.0, abs=2.0)


def test_roll_stiffness_matches_the_textbook_formula(hp, roll):
    """Springs only: K = k * MR^2 * track^2 / 2 per radian, for small roll."""
    v = sk.VehicleParams.from_hardpoints(hp)
    s = sk.roll_summary(roll, v)
    bump = sk.summary(sk.compute_metrics(sk.bump_sweep(hp, 2, 0.5), v), v)
    track = 2 * hp.upright["CP"][1]
    k_rad = v.spring_rate_n_per_mm * bump["motion_ratio"] ** 2 * track ** 2 / 2  # N mm / rad
    expected = k_rad / 1000 * np.pi / 180                                       # N m / deg
    assert s["roll_stiffness_nm_per_deg"] == pytest.approx(expected, rel=0.01)
    assert s["motion_ratio"] == pytest.approx(bump["motion_ratio"], rel=0.01)
    # Roughly half the track per radian of roll.
    assert s["wheel_travel_mm_per_deg"] == pytest.approx(track / 2 * np.pi / 180, rel=0.02)


def test_roll_stiffness_scales_with_spring_rate(hp, poses):
    soft = sk.roll_summary(sk.roll_metrics(poses, sk.VehicleParams(spring_rate_n_per_mm=20)),
                           sk.VehicleParams(spring_rate_n_per_mm=20))
    stiff = sk.roll_summary(sk.roll_metrics(poses, sk.VehicleParams(spring_rate_n_per_mm=40)),
                            sk.VehicleParams(spring_rate_n_per_mm=40))
    assert stiff["roll_stiffness_nm_per_deg"] == pytest.approx(2 * soft["roll_stiffness_nm_per_deg"])


def test_roll_centre_lines_meet_at_the_reported_point(poses):
    """Each contact patch, its instant centre and the roll centre are on one line."""
    rp = poses[-1]
    rc = roll_center_chassis(rp)
    for pose, flip in ((rp.left, 1.0), (rp.right_src, -1.0)):
        cp = pose.points["CP"][1:] * [flip, 1.0]
        ic = sk.metrics.instant_center_front_view(pose) * [flip, 1.0]
        a, b = ic - cp, rc - cp
        assert a[0] * b[1] - a[1] * b[0] == pytest.approx(0.0, abs=1e-6 * np.linalg.norm(a) * np.linalg.norm(b))


def test_optimised_design_compensates_camber_and_keeps_roll_centre_central():
    """Numbers quoted in the README for data/optimized_front.json."""
    opt = sk.load_hardpoints(OPTIMIZED)
    v = sk.VehicleParams.from_hardpoints(opt)
    s = sk.roll_summary(sk.roll_metrics(sk.roll_sweep(opt, 3.0, 0.5), v), v)
    base = sk.roll_summary(sk.roll_metrics(sk.roll_sweep(sk.load_hardpoints(BASELINE), 3.0, 0.5), v), v)
    assert 40 < s["camber_compensation_pct"] < 55
    assert abs(s["roll_steer_deg_per_deg"]) < 0.01
    # The optimiser's roll-centre migration goal pays off in roll too.
    assert abs(s["rc_lateral_shift_mm_per_deg"]) < abs(base["rc_lateral_shift_mm_per_deg"]) / 3


def test_cli_roll_writes_outputs(tmp_path):
    from suspkin.cli import main

    main(["roll", str(BASELINE), "--roll", "1", "--step", "0.5", "--spring-rate", "50",
          "--out", str(tmp_path)])
    for suffix in ("_roll.csv", "_roll.png", "_roll_front_view.png"):
        assert (tmp_path / f"{BASELINE.stem}{suffix}").exists()


def test_roll_model_solves_a_single_angle(hp):
    rp = RollModel(hp).solve(1.5)
    assert rp.travel > 0
    assert np.degrees(ground_roll_rad(rp.left, rp.right_src)) == pytest.approx(1.5, abs=1e-8)
