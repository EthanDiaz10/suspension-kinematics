"""Tests for the anti-roll bar and the whole-car roll numbers."""

from pathlib import Path

import numpy as np
import pytest

import suspkin as sk
from suspkin.arb import BAR_AXIS, ARBGeometry
from suspkin.rocker import RockerGeometry, angle_about_axis
from suspkin.roll import roll_metrics, roll_summary, roll_sweep, whole_car_roll

DATA = Path(__file__).resolve().parents[1] / "data"
OPTIMIZED = DATA / "optimized_front.json"
BASELINE = DATA / "baseline_fsae_front.json"
FLIP = np.array([1.0, -1.0, 1.0])


@pytest.fixture(scope="module")
def hp():
    return sk.load_hardpoints(OPTIMIZED)


@pytest.fixture(scope="module")
def vehicle(hp):
    return sk.VehicleParams.from_hardpoints(hp)


@pytest.fixture(scope="module")
def bump(hp):
    return sk.bump_sweep(hp, 30, 5)


@pytest.fixture(scope="module")
def roll(hp, vehicle):
    return roll_metrics(roll_sweep(hp, 2.0, 0.25), vehicle)


def test_optimised_file_has_a_bar(hp):
    assert hp.arb is not None and hp.arb.link_mount == "rocker"
    assert hp.arb.rate_nm_per_deg > 0


def test_drop_link_length_and_arm_are_rigid(hp, bump):
    for p in bump:
        P = p.points
        assert np.linalg.norm(P["ARA"] - P["ARL"]) == pytest.approx(hp.arb.drop_link_length, abs=1e-6)
        # The arm end turns about the bar axis: same distance from ARP, same y.
        assert np.linalg.norm(P["ARA"] - P["ARP"]) == pytest.approx(
            np.linalg.norm(hp.arb.points["ARA"] - hp.arb.points["ARP"]), abs=1e-9)
        assert P["ARA"][1] == pytest.approx(hp.arb.points["ARA"][1], abs=1e-9)
        # ARL is bolted to the rocker, so it keeps its distance to the rocker's points.
        assert np.linalg.norm(P["ARL"] - P["PRI"]) == pytest.approx(
            np.linalg.norm(hp.arb.points["ARL"] - hp.rocker.points["PRI"]), abs=1e-6)


def test_arm_angle_matches_the_arm_end(hp, bump):
    for p in bump:
        angle = angle_about_axis(hp.arb.points["ARP"], BAR_AXIS, hp.arb.points["ARA"], p.points["ARA"])
        assert angle == pytest.approx(p.arb_angle, abs=1e-9)


def test_bar_does_not_change_the_wheel_kinematics(hp, bump):
    """The bar hangs off the rocker but pushes nothing around kinematically."""
    no_bar = hp.copy()
    no_bar.arb = None
    for a, b in zip(bump, sk.bump_sweep(no_bar, 30, 5)):
        for name in ("UBJ", "LBJ", "WC", "PRI", "SHR"):
            np.testing.assert_allclose(a.points[name], b.points[name], atol=1e-9)


def test_bar_does_not_twist_in_heave_or_at_zero_roll(roll):
    i0 = int(np.argmin(np.abs(roll["roll_deg"])))
    assert roll["bar_twist_deg"][i0] == pytest.approx(0.0, abs=1e-12)
    # Twist is odd in roll: left and right turns twist it opposite ways.
    np.testing.assert_allclose(roll["bar_twist_deg"], -roll["bar_twist_deg"][::-1], atol=1e-9)


def test_mirror_trick_for_the_bar(hp, bump):
    """Build a real right corner (rocker and bar mirrored) and solve it at the
    same travel: its arm angle about +Y equals the left arm's angle, so the
    twist is angle(-s) - angle(+s) with no sign flip."""

    class RightCorner(sk.Hardpoints):
        @property
        def wheel_axis(self):
            return super().wheel_axis * FLIP

    rk = hp.rocker
    right = RightCorner(
        name="right", chassis={k: v * FLIP for k, v in hp.chassis.items()},
        upright={k: v * FLIP for k, v in hp.upright.items()},
        static_camber_deg=hp.static_camber_deg, static_toe_deg=hp.static_toe_deg,
        rocker=RockerGeometry(rk.mount, rk.axis * FLIP, {k: v * FLIP for k, v in rk.points.items()}),
        arb=ARBGeometry(hp.arb.link_mount, hp.arb.rate_nm_per_deg,
                        {k: v * FLIP for k, v in hp.arb.points.items()}),
    )
    model = sk.DoubleWishbone(right)
    for p in bump:
        real = model.solve(p.travel)
        np.testing.assert_allclose(real.points["ARA"], p.points["ARA"] * FLIP, atol=1e-6)
        assert real.arb_angle == pytest.approx(p.arb_angle, abs=1e-8)


def test_bar_stiffness_is_rate_times_twist_ratio_squared(roll, vehicle):
    s = roll_summary(roll, vehicle)
    expected = s["arb_rate_nm_per_deg"] * s["bar_twist_deg_per_deg_roll"] ** 2
    assert s["roll_stiffness_bar_nm_per_deg"] == pytest.approx(expected)
    assert s["roll_stiffness_nm_per_deg"] == pytest.approx(
        s["roll_stiffness_springs_nm_per_deg"] + s["roll_stiffness_bar_nm_per_deg"])
    # Same number from the slope of the total roll moment curve.
    i0 = int(np.argmin(np.abs(roll["roll_deg"])))
    slope = np.gradient(roll["total_roll_moment_nm"], roll["roll_deg"])[i0]
    assert slope == pytest.approx(s["roll_stiffness_nm_per_deg"], rel=0.01)


def test_zero_rate_bar_adds_nothing(hp, vehicle):
    h = hp.copy()
    h.arb.rate_nm_per_deg = 0.0
    s = roll_summary(roll_metrics(roll_sweep(h, 1.0, 0.5), vehicle), vehicle)
    assert s["roll_stiffness_bar_nm_per_deg"] == 0.0
    assert s["roll_stiffness_nm_per_deg"] == pytest.approx(s["roll_stiffness_springs_nm_per_deg"])


def test_whole_car_hand_calculation():
    v = sk.VehicleParams(mass_kg=300, cg_height_mm=300, front_weight_fraction=0.5,
                         rear_track_mm=1200, rear_roll_stiffness_nm_per_deg=500,
                         rear_rc_height_mm=0.0, design_lateral_g=1.0)
    car = whole_car_roll(500.0, 0.0, 1200.0, v)
    # Roll centres on the ground: moment arm = CG height, split by stiffness.
    W = 300 * 9.81
    assert car["roll_moment_arm_mm"] == pytest.approx(300.0)
    assert car["roll_gradient_deg_per_g"] == pytest.approx(W * 0.3 / (1000 * 180 / np.pi) * 180 / np.pi)
    assert car["front_load_transfer_share_pct"] == pytest.approx(50.0)
    stiffer = whole_car_roll(800.0, 0.0, 1200.0, v)
    assert stiffer["front_load_transfer_share_pct"] > 50.0
    assert stiffer["roll_gradient_deg_per_g"] < car["roll_gradient_deg_per_g"]


def test_suggested_static_camber_hits_the_target(hp, vehicle):
    """Static camber shifts the whole camber-in-roll curve, so the suggestion works."""
    s = roll_summary(roll_metrics(roll_sweep(hp, 2.0, 0.25), vehicle), vehicle)
    h = hp.copy()
    h.static_camber_deg = s["static_camber_for_target_deg"]
    s2 = roll_summary(roll_metrics(roll_sweep(h, 2.0, 0.25), vehicle), vehicle)
    assert s2["outer_camber_at_design_g_deg"] == pytest.approx(vehicle.target_outside_camber_deg, abs=0.02)


def test_optimised_design_numbers(hp, roll, vehicle):
    """Numbers quoted in docs/technical-guide.md for data/optimized_front.json."""
    s = roll_summary(roll, vehicle)
    assert 49 < s["front_load_transfer_share_pct"] < 53       # a little above 47% front weight
    assert 0.7 < s["roll_gradient_deg_per_g"] < 0.9
    assert s["outer_camber_at_design_g_deg"] == pytest.approx(-1.0, abs=0.1)
    assert all(c.ok for c in sk.check_packaging(hp))


def test_hardpoints_round_trip_keeps_the_bar(hp, tmp_path):
    sk.save_hardpoints(hp, tmp_path / "hp.json")
    back = sk.load_hardpoints(tmp_path / "hp.json")
    assert back.arb.rate_nm_per_deg == hp.arb.rate_nm_per_deg
    for k, v in hp.arb.points.items():
        np.testing.assert_allclose(back.arb.points[k], v)


def test_bar_on_a_rocker_needs_a_rocker(tmp_path):
    import json
    raw = json.loads(OPTIMIZED.read_text())
    del raw["rocker"]
    raw["upright"]["SHU"] = [0.0, 520.0, 160.0]  # direct-acting damper instead
    (tmp_path / "hp.json").write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="rocker"):
        sk.load_hardpoints(tmp_path / "hp.json")


def test_bar_can_hang_off_the_lower_arm(hp):
    h = hp.copy()
    h.arb.link_mount = "lower_arm"
    h.arb.points["ARL"] = np.array([5.0, 400.0, 140.0])
    h.arb.points["ARA"] = np.array([5.0, 400.0, 230.0])
    h.arb.points["ARP"] = np.array([-75.0, 400.0, 230.0])
    poses = sk.bump_sweep(h, 20, 5)
    for p in poses:
        assert np.linalg.norm(p.points["ARA"] - p.points["ARL"]) == pytest.approx(90.0, abs=1e-6)
    assert poses[-1].arb_angle * poses[0].arb_angle < 0  # bump and droop turn it opposite ways


def test_cli_arb_rate_override(tmp_path, capsys):
    from suspkin.cli import main

    main(["roll", str(OPTIMIZED), "--roll", "1", "--step", "0.5", "--arb-rate", "0",
          "--out", str(tmp_path)])
    out = capsys.readouterr().out
    line = next(ln for ln in out.splitlines() if "roll_stiffness_bar_nm_per_deg" in ln)
    assert float(line.split()[-1]) == 0.0
    with pytest.raises(SystemExit):
        main(["roll", str(BASELINE), "--arb-rate", "5", "--out", str(tmp_path)])
