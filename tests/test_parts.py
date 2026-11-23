"""Phase 9: upright, rocker and pushrod bracket geometry, loads, hand calcs and FEA."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest
import suspkin as sk
from suspkin.cli import main
from suspkin.loads import LoadsConfig, analyse_loads
from suspkin.parts import (Box, Cylinder, Hole, Part, PartsConfig, build_parts, caliper_forces,
                           hand_checks, part_load_cases, parts_dict, rect_section_stress,
                           rim_clearance, wheel_frame)

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
FRONT = DATA / "optimized_front.json"
REAR = DATA / "optimized_rear.json"
CASES = DATA / "load_cases.json"
PARTS = DATA / "parts.json"
FUSION = ROOT / "cad" / "fusion" / "SuspensionParts"


def _fea_available() -> bool:
    try:  # gmsh raises OSError (not ImportError) when its system libraries are missing
        import gmsh  # noqa: F401
        import pyamg  # noqa: F401
        import skfem  # noqa: F401
        return True
    except Exception:
        return False


needs_fea = pytest.mark.skipif(not _fea_available(), reason="FEA extras not installed")


@pytest.fixture(scope="module")
def result():
    car = sk.full_car(sk.load_hardpoints(FRONT), sk.load_hardpoints(REAR))
    return analyse_loads(car, LoadsConfig.load(CASES))


@pytest.fixture(scope="module")
def cfg():
    return PartsConfig.load(PARTS)


@pytest.fixture(scope="module")
def built(result, cfg):
    out = {}
    for axle in ("front", "rear"):
        hp = result.corners[axle].hp
        parts = build_parts(hp, cfg, axle)
        out[axle] = (hp, parts, part_load_cases(result, axle, parts, cfg))
    return out


def test_every_corner_gets_three_parts(built, result):
    for hp, parts, loads in built.values():
        assert set(parts) == {"upright", "rocker", "pushrod_bracket"}
        for k in parts:
            assert len(loads[k]) == len(result.corners["front"].tyres)  # one per case variant


def test_holes_sit_on_the_hardpoints(built):
    for hp, parts, _ in built.values():
        up = parts["upright"]
        for j in ("UBJ", "LBJ", "RTO"):
            assert np.allclose(up.holes[j].center, hp.upright[j])
        rk = parts["rocker"]
        assert np.allclose(rk.holes["RKP"].center, hp.rocker.points["RKP"])
        assert np.allclose(parts["pushrod_bracket"].holes["PRO"].center, hp.rocker.points["PRO"])


def test_bearing_housing_is_on_the_spin_axis(built):
    for hp, parts, _ in built.values():
        wf = wheel_frame(hp)
        housing = parts["upright"].solids[0]
        assert np.allclose(housing.axis, wf["axis"])
        rel = housing.p1 - wf["WC"]
        assert np.linalg.norm(rel - (rel @ wf["axis"]) * wf["axis"]) < 1e-9


def test_upright_fits_inside_the_rim(built):
    """Phase 9 lowered the UBJ 6 mm so the real boss fits: at least 1 mm to the rim."""
    for hp, parts, _ in built.values():
        assert rim_clearance(hp, parts["upright"])["min_gap_mm"] > 1.0


def test_hand_calcs_pass(built, result, cfg):
    for hp, parts, loads in built.values():
        for c in hand_checks(hp, parts, loads, result, cfg):
            assert c.sf >= cfg.required_sf, (c.part, c.item, c.sf)


def test_caliper_forces_carry_the_brake_torque(built, result, cfg):
    """The lug forces' moment about the axle equals the tyre's (the bearing takes no torque)."""
    for axle, (hp, parts, _) in built.items():
        wf = wheel_frame(hp)
        for t in result.corners[axle].tyres:
            f = caliper_forces(hp, parts["upright"], cfg, t)
            if t.fx >= 0:
                assert f == {}
                continue
            torque = sum(wf["axis"] @ np.cross(p - wf["WC"], v) for p, v in t.applied(hp))
            m = sum(wf["axis"] @ np.cross(parts["upright"].holes[k].center - wf["WC"], v)
                    for k, v in f.items())
            assert m == pytest.approx(torque, rel=1e-9)
            assert abs(torque) > 1e4  # it's a real braking case (> 10 N m)


def test_rect_section_stress_matches_beam_formula():
    # 1000 N across a 20 x 10 cantilever, 50 mm long: M / Z = 50000 / (10 * 20^2 / 6) = 75 MPa
    s = rect_section_stress(np.array([0, 1000.0, 0]), np.array([50.0, 0, 0]),
                            np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]), 20, 10)
    assert s == pytest.approx(75.0)


def test_parts_json_round_trips_into_the_fusion_reader(built, cfg, tmp_path):
    sys.path.insert(0, str(FUSION))
    try:
        from unittest import mock
        with mock.patch.dict(sys.modules, {"adsk": mock.MagicMock(), "adsk.core": mock.MagicMock(),
                                           "adsk.fusion": mock.MagicMock()}):
            import importlib
            import SuspensionParts as script
            script = importlib.reload(script)
            hp, parts, loads = built["front"]
            path = tmp_path / "front_parts.json"
            path.write_text(json.dumps(parts_dict(hp, parts, loads, cfg)))
            raw = script.read_parts(str(path))
            assert set(raw["parts"]) == set(parts)
            assert script.to_cm([10.0, 20.0, 30.0], "Y") == (1.0, 3.0, -2.0)
    finally:
        sys.path.remove(str(FUSION))


def test_fusion_parts_script_runs_against_a_fake_api(monkeypatch, built, cfg, tmp_path):
    """Smoke test with Fusion's API mocked: catches typos before Ethan runs it."""
    import importlib
    from unittest import mock
    adsk = mock.MagicMock()
    for name, mod in {"adsk": adsk, "adsk.core": adsk.core, "adsk.fusion": adsk.fusion}.items():
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.syspath_prepend(str(FUSION))
    import SuspensionParts as script
    script = importlib.reload(script)
    hp, parts, loads = built["rear"]
    path = tmp_path / "rear_parts.json"
    path.write_text(json.dumps(parts_dict(hp, parts, loads, cfg)))
    monkeypatch.setattr(script, "DEFAULT_JSON", str(path))
    # The mock can't count bodies by itself; pretend each Base Feature has one.
    root = adsk.fusion.Design.cast.return_value.rootComponent
    comp = root.occurrences.addNewComponent.return_value.component
    comp.features.baseFeatures.add.return_value.bodies.count = 1
    script.run(None)
    ui = adsk.core.Application.get.return_value.userInterface
    msg = ui.messageBox.call_args[0][0]
    assert msg.startswith("Built:"), msg
    assert "rear upright" in msg and "rear rocker" in msg


def test_cli_parts_writes_files(tmp_path, capsys):
    main(["parts", str(FRONT), str(REAR), "--cases", str(CASES), "--parts", str(PARTS),
          "--out", str(tmp_path)])
    out = capsys.readouterr().out
    assert "Hand calculations" in out and "LOW" not in out
    d = json.loads((tmp_path / "optimized_front_parts.json").read_text())
    assert {"upright", "rocker", "pushrod_bracket"} <= set(d["parts"])
    assert (tmp_path / "parts_hand_calcs.csv").exists()


# ---------------------------------------------------------------------------
# FEA (only when gmsh / scikit-fem / pyamg are installed)
# ---------------------------------------------------------------------------

def _bar_part():
    """A 120 x 20 x 20 mm bar with a hole near each end, 100 mm apart."""
    x, y = np.array([1.0, 0, 0]), np.array([0, 1.0, 0])
    solids = [Box("bar", np.array([50.0, 0, 0]), x, y, 120, 20, 20)]
    cuts = [Cylinder(f"h{i}", np.array([c, -11.0, 0]), np.array([c, 11.0, 0]), 4) for i, c in ((1, 0), (2, 100))]
    holes = {k: Hole(k, np.array([c, -10.0, 0]), np.array([c, 10.0, 0]), 4) for k, c in (("A", 0), ("B", 100))}
    return Part("test bar", "7075-T6", solids, cuts, holes, "A")


@needs_fea
def test_fea_matches_beam_theory():
    """Validation: a cantilever bar. 1000 N at 100 mm gives M / Z = 75 MPa at the support
    hole; 10 mm from it (outside the 'near the support' zone) beam theory says
    M / Z = 1000 x 90 / 1333 = 67.5 MPa. The FEA peak away from the support should be close."""
    from suspkin.fea import solve_part
    r = solve_part(_bar_part(), {"down": {"B": np.array([0, 0, -1000.0])}},
                   {"e_gpa": 71.7, "poisson": 0.33}, 4.0, away_mm=8.0)[0]
    assert r.max_vm_away_mpa == pytest.approx(67.5, rel=0.1)


@needs_fea
def test_fea_load_is_applied_in_full():
    """The bearing-load spreading puts exactly the requested force on the hole."""
    from suspkin import fea
    part = _bar_part()
    r1 = fea.solve_part(part, {"a": {"B": np.array([0, 0, -1000.0])}}, {"e_gpa": 71.7, "poisson": 0.33}, 5.0)[0]
    r2 = fea.solve_part(part, {"a": {"B": np.array([0, 0, -2000.0])}}, {"e_gpa": 71.7, "poisson": 0.33}, 5.0)[0]
    assert r2.max_disp_mm == pytest.approx(2 * r1.max_disp_mm, rel=1e-6)  # linear


@needs_fea
def test_step_export(built, tmp_path):
    from suspkin.fea import write_step
    hp, parts, _ = built["front"]
    vol = write_step(parts["pushrod_bracket"], tmp_path / "b.step")
    text = (tmp_path / "b.step").read_text(errors="ignore")
    assert text.startswith("ISO-10303-21") and vol > 1000


@needs_fea
def test_fea_fillets_round_the_rocker_corners(built, tmp_path):
    """The arm-to-pivot-tube corners get rounded: the filleted rocker is a bit heavier."""
    from suspkin.fea import write_step
    hp, parts, _ = built["front"]
    rocker = parts["rocker"]
    sharp = write_step(rocker, tmp_path / "r0.step")
    rounded = write_step(rocker, tmp_path / "r3.step", fillet_mm=3.0)
    assert rounded > sharp
    assert not any("couldn't" in n for n in rocker.notes), rocker.notes
