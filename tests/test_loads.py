"""Phase 8: load cases, link forces and tube sizing."""

import json
from pathlib import Path

import numpy as np
import pytest
import suspkin as sk
from suspkin.cli import main
from suspkin.kinematics import DoubleWishbone
from suspkin.loads import (LoadCase, LoadsConfig, Material, Tube, analyse_loads, buckling_capacity,
                           ideal_front_brake_bias, solve_link_forces, tension_capacity, tyre_loads)
from suspkin.roll import G
from suspkin.sideview import VehicleParams

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
FRONT = DATA / "optimized_front.json"
REAR = DATA / "optimized_rear.json"
CASES = DATA / "load_cases.json"

# A general load on the tyre: sideways, braking and vertical all at once.
SOME_LOAD = np.array([-900.0, -1500.0, 1200.0])


@pytest.fixture(scope="module")
def car():
    return sk.full_car(sk.load_hardpoints(FRONT), sk.load_hardpoints(REAR))


@pytest.fixture(scope="module")
def result(car):
    return analyse_loads(car, LoadsConfig.load(CASES))


def _upright_balance(hp, lf, applied):
    """Net force and net moment (about WC) on the upright: both should be zero."""
    c = hp.upright["WC"]
    pts = {**hp.upright, **(hp.rocker.points if hp.rocker else {})}
    force = sum(f for _, f in applied) + sum(lf.joint_forces.values())
    moment = sum(np.cross(p - c, f) for p, f in applied)
    moment = moment + sum(np.cross(pts[k] - c, f) for k, f in lf.joint_forces.items())
    return force, moment


@pytest.mark.parametrize("path", [FRONT, REAR])
def test_link_forces_balance_the_tyre_load(path):
    """The link forces on the upright exactly cancel the applied load (forces and moments)."""
    hp = sk.load_hardpoints(path)
    applied = [(hp.upright["CP"], SOME_LOAD)]
    lf = solve_link_forces(hp, applied)
    force, moment = _upright_balance(hp, lf, applied)
    assert np.allclose(force, 0, atol=1e-6)
    assert np.allclose(moment, 0, atol=1e-3)  # N mm


def test_every_case_balances(result):
    for c in result.corners.values():
        for t, lf in zip(c.tyres, c.forces):
            applied = t.applied(c.hp)
            force, moment = _upright_balance(c.hp, lf, applied)
            assert np.allclose(force, 0, atol=1e-6), t.label
            assert np.allclose(moment, 0, atol=1e-3), t.label


def test_pure_vertical_load_puts_the_pushrod_in_compression():
    hp = sk.load_hardpoints(FRONT)
    lf = solve_link_forces(hp, [(hp.upright["CP"], np.array([0.0, 0.0, 1000.0]))])
    assert lf.members["pushrod"] < 0


def _velocity(hp, name, h=0.01):
    """How fast a point moves per mm of wheel travel (central difference)."""
    dw = DoubleWishbone(hp)
    up, down = dw.solve(h), dw.solve(-h)
    return (up.points[name] - down.points[name]) / (2 * h)


@pytest.mark.parametrize("path", [FRONT, REAR])
def test_pushrod_force_matches_virtual_work(path):
    """An independent check of the pushrod force, from the kinematic solver.

    Take the pushrod out and the corner can move up and down (one degree of
    freedom). For a tiny wheel movement the work done by the tyre load plus the
    work done by the pushrod force on its bracket must add up to zero, because
    every other link is rigid and its chassis end doesn't move.
    """
    hp = sk.load_hardpoints(path)
    load_cp = SOME_LOAD
    load_wc = np.array([700.0, 0.0, 0.0])
    lf = solve_link_forces(hp, [(hp.upright["CP"], load_cp), (hp.upright["WC"], load_wc)])
    rk = hp.rocker.points
    e_push = (rk["PRI"] - rk["PRO"]) / np.linalg.norm(rk["PRI"] - rk["PRO"])
    work = (load_cp @ _velocity(hp, "CP") + load_wc @ _velocity(hp, "WC")
            + lf.members["pushrod"] * e_push @ _velocity(hp, "PRO"))
    scale = np.linalg.norm(load_cp) * np.linalg.norm(_velocity(hp, "CP"))
    assert abs(work) < 1e-5 * scale


def test_pushrod_on_the_upright_uses_six_links():
    """With the pushrod bolted to the upright it is one of six two-force links."""
    hp = sk.load_hardpoints(FRONT)
    hp.rocker.mount = "upright"
    hp.rocker.points["PRO"] = np.array([5.0, 560.0, 160.0])
    applied = [(hp.upright["CP"], SOME_LOAD)]
    lf = solve_link_forces(hp, applied)
    assert lf.bracket_bending_force_n == 0.0
    force, moment = _upright_balance(hp, lf, applied)
    assert np.allclose(force, 0, atol=1e-6) and np.allclose(moment, 0, atol=1e-3)
    # Virtual work check for this layout too.
    rk = hp.rocker.points
    e_push = (rk["PRI"] - rk["PRO"]) / np.linalg.norm(rk["PRI"] - rk["PRO"])
    work = SOME_LOAD @ _velocity(hp, "CP") + lf.members["pushrod"] * e_push @ _velocity(hp, "PRO")
    assert abs(work) < 1e-5 * np.linalg.norm(SOME_LOAD)


def test_whole_corner_balance_up_to_bracket_bending():
    """What reaches the chassis equals the tyre load, apart from the small
    part the pushrod bracket carries in bending (the lumping approximation)."""
    hp = sk.load_hardpoints(FRONT)
    lf = solve_link_forces(hp, [(hp.upright["CP"], SOME_LOAD)])
    on_chassis = sum(lf.chassis_forces.values())
    assert np.linalg.norm(on_chassis - SOME_LOAD) <= lf.bracket_bending_force_n + 1e-6
    assert lf.bracket_bending_force_n < 0.1 * np.linalg.norm(SOME_LOAD)


def test_tyre_loads_add_up(car):
    """Without lift-off, the four tyres carry the car's weight and its acceleration."""
    v = car.car
    W = v.mass_kg * G
    n = car.numbers
    lt = (n["front_load_transfer_n_per_g"], n["rear_load_transfer_n_per_g"])

    def totals(case):
        loads = tyre_loads(case, v, *lt, rear=car.rear.vehicle)
        per_axle = 1 if case.lateral_g else 2   # no cornering: one entry stands for both wheels
        fz = sum(t.fz for t in loads) * per_axle
        fx = sum(t.fx for t in loads) * per_axle
        fy = sum(abs(t.fy) for t in loads)
        return loads, fz, fx, fy

    _, fz, _, fy = totals(LoadCase("c", lateral_g=1.2))
    assert fz == pytest.approx(W) and fy == pytest.approx(1.2 * W)
    _, fz, fx, _ = totals(LoadCase("b", longitudinal_g=-1.4))
    assert fz == pytest.approx(W) and fx == pytest.approx(-1.4 * W)
    loads, fz, fx, _ = totals(LoadCase("a", longitudinal_g=0.9))
    assert fx == pytest.approx(0.9 * W)
    assert all(t.fx == 0 for t in loads if t.axle == "front")   # only the rear is driven
    assert all(t.fx_at == "WC" for t in loads if t.axle == "rear")  # chassis-mounted diff
    _, fz, _, _ = totals(LoadCase("bump", vertical_g=3.0))
    assert fz == pytest.approx(3 * W)


def test_inside_tyre_matches_full_car(car):
    """At the design load the tyre loads agree with `suspkin car`."""
    v = car.car
    n = car.numbers
    loads = tyre_loads(LoadCase("c", lateral_g=v.design_lateral_g), v,
                       n["front_load_transfer_n_per_g"], n["rear_load_transfer_n_per_g"])
    inside = {t.axle: t.fz for t in loads if t.side == "inside"}
    assert inside["front"] == pytest.approx(n["front_inside_wheel_load_n"])
    assert inside["rear"] == pytest.approx(n["rear_inside_wheel_load_n"])


def test_wheel_lift_is_flagged(car):
    n = car.numbers
    loads = tyre_loads(LoadCase("c", lateral_g=2.5), car.car,
                       n["front_load_transfer_n_per_g"], n["rear_load_transfer_n_per_g"])
    front_inside = next(t for t in loads if t.axle == "front" and t.side == "inside")
    assert front_inside.lifted and front_inside.fz == 0.0


def test_buckling_euler_and_johnson():
    mat = Material()
    tube = Tube(15.875, 1.24)
    E = mat.e_gpa * 1000
    long = 1500.0
    euler = np.pi ** 2 * E * tube.inertia_mm4 / long ** 2
    assert buckling_capacity(tube, mat, long) == pytest.approx(euler)
    # Short tube: Johnson, below yield and above Euler's (impossible) answer.
    short = 150.0
    p = buckling_capacity(tube, mat, short)
    assert p < tension_capacity(tube, mat)
    # The two formulas meet at the crossover slenderness.
    crossover = np.sqrt(2 * np.pi ** 2 * E / mat.yield_mpa) * tube.gyration_mm
    assert buckling_capacity(tube, mat, crossover * 0.999999) == pytest.approx(
        buckling_capacity(tube, mat, crossover * 1.000001), rel=1e-4)


def test_ideal_brake_bias():
    v = VehicleParams(front_weight_fraction=0.47, cg_height_mm=290, wheelbase_mm=1550)
    assert ideal_front_brake_bias(v, 1.6) == pytest.approx(0.47 + 1.6 * 290 / 1550)


def test_default_design_passes(result):
    """The current tubes pass every case with the required safety factor."""
    sf = result.config.required_sf
    assert all(m.sf >= sf for m in result.checks)
    for g, (tube, tube_sf) in result.lightest.items():
        assert tube is not None and tube_sf >= sf
        assert tube.area_mm2 <= result.config.members[g].tube.area_mm2


def test_cli_loads(tmp_path, capsys):
    main(["loads", str(FRONT), str(REAR), "--cases", str(CASES), "--out", str(tmp_path)])
    out = capsys.readouterr().out
    assert "Safety factors" in out and "FAIL" not in out
    stem = "optimized_front+optimized_rear"
    for suffix in ("_link_forces.csv", "_margins.csv", "_upright_loads.json", "_loads.png"):
        assert (tmp_path / f"{stem}{suffix}").exists()
    up = json.loads((tmp_path / f"{stem}_upright_loads.json").read_text())
    assert set(up["corners"]) == {"front", "rear"}
    assert "bump" in up["corners"]["front"]["cases"]
