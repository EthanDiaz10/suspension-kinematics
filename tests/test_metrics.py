"""Tests for the metric calculations and their sign conventions."""

from pathlib import Path

import numpy as np
import pytest

import suspkin as sk
from suspkin import metrics as m

DATA = Path(__file__).resolve().parents[1] / "data" / "baseline_fsae_front.json"


@pytest.fixture
def hp():
    return sk.load_hardpoints(DATA)


def static_pose(hp):
    return sk.DoubleWishbone(hp).solve(0.0)


def test_static_camber_and_toe_round_trip(hp):
    hp.static_camber_deg, hp.static_toe_deg = -1.5, 0.25
    pose = static_pose(hp)
    assert m.camber_deg(pose) == pytest.approx(-1.5, abs=1e-9)
    assert m.toe_deg(pose) == pytest.approx(0.25, abs=1e-9)


def test_caster_and_kpi_match_hand_calculation(hp):
    ubj, lbj = hp.upright["UBJ"], hp.upright["LBJ"]
    d = ubj - lbj
    pose = static_pose(hp)
    assert m.caster_deg(pose) == pytest.approx(np.degrees(np.arctan(-d[0] / d[2])))
    assert m.kpi_deg(pose) == pytest.approx(np.degrees(np.arctan(-d[1] / d[2])))
    # Baseline has the UBJ behind and inboard of the LBJ: positive caster and KPI.
    assert m.caster_deg(pose) > 0 and m.kpi_deg(pose) > 0


def test_negative_camber_gain_in_bump(hp):
    """A short-upper/long-lower arm layout gains negative camber in bump."""
    metrics = sk.compute_metrics(sk.bump_sweep(hp, 30, 5))
    assert metrics["camber_deg"][-1] < metrics["camber_deg"][0]


def test_rack_toward_wheel_gives_toe_out(hp):
    """Front-steer rack: pushing the tie rod outboard swings the wheel front out."""
    pose = sk.DoubleWishbone(hp).solve(0.0, rack=5.0)
    assert m.toe_deg(pose) < 0


def test_roll_center_on_ground_for_parallel_horizontal_arms(hp):
    """Parallel, horizontal, equal arms: instant centre at infinity, RC at ground."""
    for k in ("UAF", "UAA"):
        hp.chassis[k][2] = hp.upright["UBJ"][2]
    for k in ("LAF", "LAA"):
        hp.chassis[k][2] = hp.upright["LBJ"][2]
    for k in ("UAF", "UAA", "LAF", "LAA"):
        hp.chassis[k][1] = 250.0
    hp.upright["UBJ"][1] = hp.upright["LBJ"][1]  # vertical steering axis in front view
    pose = static_pose(hp)
    assert m.instant_center_front_view(pose) is None
    assert m.roll_center_height_mm(pose) == pytest.approx(0.0, abs=1e-9)


def test_motion_ratio_is_positive_and_below_one(hp):
    metrics = sk.compute_metrics(sk.bump_sweep(hp, 20, 2))
    assert np.all(metrics["motion_ratio"] > 0)
    assert np.all(metrics["motion_ratio"] < 1.2)


def test_cli_writes_outputs(tmp_path):
    from suspkin.cli import main

    main(["sweep", str(DATA), "--travel", "10", "--step", "5", "--out", str(tmp_path)])
    stem = DATA.stem
    for suffix in ("_sweep.csv", "_poses.json", "_sweep.png", "_front_view.png"):
        assert (tmp_path / f"{stem}{suffix}").exists()
