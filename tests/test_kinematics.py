"""Tests for the solver. Run them with:  pytest

The most important check: at every solved pose, every link must still be
exactly its design length. If that ever fails, every metric is wrong.
"""

from itertools import combinations
from pathlib import Path

import numpy as np
import pytest

import suspkin as sk
from suspkin.kinematics import LINKS

DATA = Path(__file__).resolve().parents[1] / "data" / "baseline_fsae_front.json"
TOL_MM = 1e-6  # a millionth of a millimetre


@pytest.fixture(scope="module")
def hp():
    return sk.load_hardpoints(DATA)


@pytest.fixture(scope="module")
def sweep(hp):
    return sk.bump_sweep(hp, travel_mm=50, step_mm=2)


def test_link_lengths_preserved_through_sweep(hp, sweep):
    model = sk.DoubleWishbone(hp)
    for pose in sweep:
        for chassis_pt, upright_pt in LINKS:
            length = np.linalg.norm(pose.points[upright_pt] - pose.points[chassis_pt])
            assert length == pytest.approx(model.link_lengths[(chassis_pt, upright_pt)], abs=TOL_MM)


def test_link_lengths_preserved_while_steering(hp):
    model = sk.DoubleWishbone(hp)
    for rack in (-15.0, 15.0):
        for pose in model.sweep(np.linspace(-40, 40, 9), rack=rack):
            for c, u in LINKS:
                length = np.linalg.norm(pose.points[u] - pose.points[c])
                assert length == pytest.approx(model.link_lengths[(c, u)], abs=TOL_MM)


def test_wheel_center_hits_requested_travel(hp, sweep):
    z0 = hp.upright["WC"][2]
    for pose in sweep:
        assert pose.points["WC"][2] == pytest.approx(z0 + pose.travel, abs=TOL_MM)


def test_upright_stays_rigid(hp, sweep):
    """Distances between any two upright points never change."""
    names = list(hp.upright)
    for pose in sweep[:: len(sweep) // 5]:
        for a, b in combinations(names, 2):
            design = np.linalg.norm(hp.upright[a] - hp.upright[b])
            moved = np.linalg.norm(pose.points[a] - pose.points[b])
            assert moved == pytest.approx(design, abs=TOL_MM)


def test_zero_travel_is_design_position(hp):
    pose = sk.DoubleWishbone(hp).solve(0.0)
    for name, p in hp.upright.items():
        np.testing.assert_allclose(pose.points[name], p, atol=1e-6)


def test_sweep_is_sorted_and_complete(sweep):
    travels = [p.travel for p in sweep]
    assert travels == sorted(travels)
    assert travels[0] == -50 and travels[-1] == 50 and len(travels) == 51


def test_unreachable_geometry_raises(hp):
    """Asking for far more travel than the links allow must fail loudly."""
    with pytest.raises(RuntimeError):
        sk.DoubleWishbone(hp).solve(1000.0)


def test_hardpoints_round_trip(hp, tmp_path):
    out = tmp_path / "hp.json"
    sk.save_hardpoints(hp, out)
    again = sk.load_hardpoints(out)
    for group in ("chassis", "upright"):
        for k, v in getattr(hp, group).items():
            np.testing.assert_allclose(getattr(again, group)[k], v)


def test_missing_point_is_reported(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text('{"chassis": {}, "upright": {}}')
    with pytest.raises(ValueError, match="missing"):
        sk.load_hardpoints(bad)
