"""Phase 9: rocker, damper, anti-roll bar drop link and pivot loads."""

from pathlib import Path

import numpy as np
import pytest
import suspkin as sk
from suspkin.cli import main
from suspkin.kinematics import DoubleWishbone
from suspkin.loads import (LoadsConfig, analyse_loads, bar_roll_share, solve_link_forces,
                           solve_rocker_loads)

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
FRONT = DATA / "optimized_front.json"
REAR = DATA / "optimized_rear.json"
CASES = DATA / "load_cases.json"


@pytest.fixture(scope="module")
def car():
    return sk.full_car(sk.load_hardpoints(FRONT), sk.load_hardpoints(REAR))


@pytest.fixture(scope="module")
def result(car):
    return analyse_loads(car, LoadsConfig.load(CASES))


def _where(hp):
    return {**hp.rocker.points, **(hp.arb.points if hp.arb else {})}


def test_rocker_is_in_equilibrium(result):
    """Forces on the rocker add to zero; its moment about the pivot is all taken by the bearings."""
    for axle, rls in result.rockers.items():
        hp = result.corners[axle].hp
        rkp, a = hp.rocker.points["RKP"], hp.rocker.axis
        where = _where(hp)
        for r in rls:
            f = r.forces_on_rocker
            assert np.allclose(sum(f.values()), 0, atol=1e-6)
            m = sum(np.cross(where[k] - rkp, v) for k, v in f.items())
            assert abs(m @ a) < 1e-6 * max(1.0, abs(r.pushrod_n)) * 1000  # free to spin: no axial moment
            assert np.allclose(m + r.pivot_moment_nmm, 0, atol=1e-6)


def _velocity(hp, name, h=0.01):
    dw = DoubleWishbone(hp)
    up, down = dw.solve(h), dw.solve(-h)
    return (up.points[name] - down.points[name]) / (2 * h)


@pytest.mark.parametrize("path", [FRONT, REAR])
def test_damper_force_matches_virtual_work(path):
    """Independent check from the kinematics: tyre work + damper work = 0 for a tiny bump.

    With the bar not sharing (one corner moving alone, share 0), the damper is
    the only thing that stores energy, so it must balance the tyre load.
    """
    hp = sk.load_hardpoints(path)
    load = np.array([-400.0, -900.0, 1500.0])
    lf = solve_link_forces(hp, [(hp.upright["CP"], load)])
    r = solve_rocker_loads(hp, lf.members["pushrod"], 0.0, 0.0)
    e_damp = hp.chassis["SHC"] - hp.rocker.points["SHR"]
    e_damp /= np.linalg.norm(e_damp)
    work = load @ _velocity(hp, "CP") + r.damper_n * e_damp @ _velocity(hp, "SHR")
    assert abs(work) < 1e-5 * np.linalg.norm(load) * np.linalg.norm(_velocity(hp, "CP"))


def test_heave_cases_dont_load_the_bar(result):
    for axle, rls in result.rockers.items():
        for t, r in zip(result.corners[axle].tyres, rls):
            if t.side == "-":
                assert r.arb_link_n == 0.0 and abs(r.bar_torque_nm) < 1e-9, t.label


def test_inside_and_outside_bar_loads_are_opposite(result):
    for axle, rls in result.rockers.items():
        by = {(t.case, t.side): r for t, r in zip(result.corners[axle].tyres, rls)}
        for (case, side), r in by.items():
            if side == "outside":
                assert by[(case, "inside")].arb_link_n == pytest.approx(-r.arb_link_n)


@pytest.mark.parametrize("axle", ["front", "rear"])
def test_bar_torque_matches_rate_times_twist(car, result, axle):
    """Bar torque from the loads = bar rate x twist from the roll model (within a few %).

    Twist = roll angle at 1.6 g x bar twist per degree of roll. The two
    numbers come from different code (statics vs. energy), so this ties them up.
    """
    ax = car.front if axle == "front" else car.rear
    i = [t.label for t in result.corners[axle].tyres].index("cornering (outside)")
    torque = abs(result.rockers[axle][i].bar_torque_nm)
    roll = car.numbers["roll_gradient_deg_per_g"] * 1.6
    twist = abs(ax.summary["bar_twist_deg_per_deg_roll"]) * roll
    assert torque == pytest.approx(ax.hp.arb.rate_nm_per_deg * twist, rel=0.05)


def test_bar_share_is_stiffness_ratio(car):
    n = car.numbers
    s = bar_roll_share(car, "front")
    assert s == pytest.approx(n["front_roll_stiffness_bar_nm_per_deg"]
                              / n["front_roll_stiffness_nm_per_deg"])
    assert 0 < s < 1


def test_static_damper_is_in_compression(result):
    for st in result.rocker_static.values():
        assert st.damper_n < 0 and st.pushrod_n < 0


def test_cli_writes_rocker_loads(tmp_path, capsys):
    main(["loads", str(FRONT), str(REAR), "--cases", str(CASES), "--out", str(tmp_path)])
    out = capsys.readouterr().out
    assert "Front rocker loads" in out and "Rear rocker loads" in out
    assert (tmp_path / "optimized_front+optimized_rear_rocker_loads.csv").exists()
    import json
    d = json.loads((tmp_path / "optimized_front+optimized_rear_upright_loads.json").read_text())
    assert "bump" in d["corners"]["front"]["rocker"]["cases"]
