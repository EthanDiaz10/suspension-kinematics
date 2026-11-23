"""Tests for the side-view instant centre and anti-dive / anti-squat."""

from pathlib import Path

import numpy as np
import pytest

import suspkin as sk
from suspkin.metrics import _intersect_2d
from suspkin.sideview import VehicleParams

DATA = Path(__file__).resolve().parents[1] / "data" / "baseline_fsae_front.json"


@pytest.fixture
def hp():
    return sk.load_hardpoints(DATA)


def tilt_pivots(hp, upper_mm, lower_mm):
    """Lower the FRONT inner pivots and raise the REAR ones by these amounts."""
    hp.chassis["UAF"][2] -= upper_mm
    hp.chassis["UAA"][2] += upper_mm
    hp.chassis["LAF"][2] -= lower_mm
    hp.chassis["LAA"][2] += lower_mm
    return hp


def static_metrics(hp, vehicle=None):
    m = sk.compute_metrics(sk.bump_sweep(hp, 10, 0.5), vehicle or VehicleParams.from_hardpoints(hp))
    i0 = int(np.argmin(np.abs(m["travel_mm"])))
    return m, i0


def test_level_pivot_axes_give_no_anti_dive(hp):
    """Pivot axes parallel to X: the wheel moves straight up in side view."""
    for k in ("UAF", "UAA"):
        hp.chassis[k][2] = 285.0
    for k in ("LAF", "LAA"):
        hp.chassis[k][2] = 112.5
    hp.upright["UBJ"][0] = hp.upright["LBJ"][0] = 0.0  # no caster either
    m, i0 = static_metrics(hp)
    assert m["anti_brake_pct"][i0] == pytest.approx(0.0, abs=0.5)


def test_pivots_low_at_the_front_give_anti_dive(hp):
    m, i0 = static_metrics(tilt_pivots(hp, 15, 10))
    assert m["anti_brake_pct"][i0] > 5.0


def test_pivots_high_at_the_front_give_pro_dive(hp):
    m, i0 = static_metrics(tilt_pivots(hp, -15, -10))
    assert m["anti_brake_pct"][i0] < -5.0


def test_anti_dive_matches_the_svic_line(hp):
    """anti-dive % = tan(line from contact patch to SVIC) * L/h * bias."""
    hp = tilt_pivots(hp, 15, 10)
    v = VehicleParams.from_hardpoints(hp)
    poses = sk.bump_sweep(hp, 10, 0.5)
    m = sk.compute_metrics(poses, v)
    i0 = int(np.argmin(np.abs(m["travel_mm"])))
    cp = poses[i0].points["CP"]
    tan = (m["svic_z_mm"][i0] - cp[2]) / (cp[0] - m["svic_x_mm"][i0])
    expected = tan * v.wheelbase_mm / v.cg_height_mm * v.front_brake_bias * 100
    assert m["anti_brake_pct"][i0] == pytest.approx(expected, rel=1e-6)


def test_svic_close_to_the_classic_graphical_construction(hp):
    """Textbook method: in side view, draw a line through each ball joint
    parallel to its arm's pivot axis; the SVIC is where they cross. That is an
    approximation (it ignores the 3D tilt of the arms), so allow a few percent."""
    hp = tilt_pivots(hp, 15, 10)
    v = VehicleParams.from_hardpoints(hp)
    poses = sk.bump_sweep(hp, 10, 0.5)
    m = sk.compute_metrics(poses, v)
    i0 = int(np.argmin(np.abs(m["travel_mm"])))
    P = poses[i0].points
    xz = [0, 2]
    ic = _intersect_2d(P["UBJ"][xz], (P["UAA"] - P["UAF"])[xz],
                       P["LBJ"][xz], (P["LAA"] - P["LAF"])[xz])
    cp = P["CP"]
    tan = (ic[1] - cp[2]) / (cp[0] - ic[0])
    graphical = tan * v.wheelbase_mm / v.cg_height_mm * v.front_brake_bias * 100
    assert m["anti_brake_pct"][i0] == pytest.approx(graphical, rel=0.05)


def test_inboard_brakes_use_the_wheel_centre(hp):
    hp = tilt_pivots(hp, 15, 10)
    out_m, i0 = static_metrics(hp, VehicleParams(brakes="outboard"))
    in_m, _ = static_metrics(hp, VehicleParams(brakes="inboard"))
    assert in_m["anti_brake_pct"][i0] != pytest.approx(out_m["anti_brake_pct"][i0], rel=1e-3)


def test_rear_axle_flips_the_sense_and_uses_rear_bias(hp):
    """The same geometry on a rear axle: what helps the front hurts the rear."""
    hp = tilt_pivots(hp, 15, 10)
    front, i0 = static_metrics(hp, VehicleParams(axle="front", front_brake_bias=0.6))
    rear, _ = static_metrics(hp, VehicleParams(axle="rear", front_brake_bias=0.6, driven=True))
    ratio = rear["anti_brake_pct"][i0] / front["anti_brake_pct"][i0]
    assert ratio == pytest.approx(-0.4 / 0.6, rel=1e-9)
    assert not np.isnan(rear["anti_drive_pct"][i0])
    assert np.isnan(front["anti_drive_pct"][i0])  # front not driven


def test_bad_vehicle_settings_are_rejected():
    with pytest.raises(ValueError):
        VehicleParams.from_dict({"axle": "middle"})
    with pytest.raises(ValueError):
        VehicleParams.from_dict({"brakes": "drum"})
