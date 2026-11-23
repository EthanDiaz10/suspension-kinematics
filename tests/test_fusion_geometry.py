"""Tests for the pure-Python half of the Fusion script.

Fusion can't run in CI, so everything the script calculates lives in
``cad/fusion/SuspensionModel/suspension_geometry.py`` and is checked here,
including against the solver's own numbers.
"""

import ast
import json
import math
import py_compile
import sys
from pathlib import Path

import pytest

import suspkin as sk
from suspkin import metrics as m

ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "cad" / "fusion" / "SuspensionModel"
sys.path.insert(0, str(SCRIPT_DIR))
import suspension_geometry as geo  # noqa: E402

OPTIMIZED = ROOT / "data" / "optimized_front.json"
BASELINE = ROOT / "data" / "baseline_fsae_front.json"


@pytest.fixture
def model():
    return geo.load_model(OPTIMIZED)


def test_hardpoints_file_matches_solver_loader(model):
    hp = sk.load_hardpoints(OPTIMIZED)
    assert model["kind"] == "hardpoints"
    for name, p in {**hp.chassis, **hp.upright}.items():
        assert model["points"][name] == pytest.approx(tuple(p))
    assert model["wheel_axis"] == pytest.approx(tuple(hp.wheel_axis))


def test_poses_file_picks_nearest_travel(tmp_path):
    hp = sk.load_hardpoints(BASELINE)
    poses = sk.bump_sweep(hp, travel_mm=50, step_mm=10)
    path = tmp_path / "base_poses.json"
    sk.write_poses_json(poses, path, name=hp.name)

    raw = geo.read_json(path)
    assert geo.is_poses_file(raw)
    assert geo.available_travels(raw) == pytest.approx([-50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50])

    model = geo.load_model(path, travel_mm=22.0)    # nearest stored pose is +20
    assert model["travel_mm"] == pytest.approx(20.0)
    target = next(p for p in poses if abs(p.travel - 20.0) < 1e-9)
    assert model["points"]["UBJ"] == pytest.approx(tuple(target.points["UBJ"]), abs=1e-4)


def test_missing_point_gives_clear_error():
    raw = json.loads(OPTIMIZED.read_text())
    del raw["upright"]["RTO"]
    with pytest.raises(ValueError, match="RTO"):
        geo.parse_model(raw)


def test_wrong_file_type_and_units_rejected():
    with pytest.raises(ValueError, match="not a hardpoints file"):
        geo.parse_model({"foo": 1})
    raw = json.loads(OPTIMIZED.read_text())
    raw["units"] = "in"
    with pytest.raises(ValueError, match="units"):
        geo.parse_model(raw)


def test_mm_to_cm_and_y_up_mapping_is_right_handed():
    assert geo.to_fusion_cm((10.0, 20.0, 30.0)) == pytest.approx((1.0, 2.0, 3.0))
    ex, ey, ez = (geo.to_fusion_cm(v, "Y") for v in ((10, 0, 0), (0, 10, 0), (0, 0, 10)))
    assert ez == pytest.approx((0.0, 1.0, 0.0))            # SAE up -> Fusion +Y
    assert geo.cross(ex, ey) == pytest.approx(ez)          # no mirroring
    with pytest.raises(ValueError):
        geo.to_fusion_cm((0, 0, 0), "X")


def test_kingpin_angles_match_solver(model):
    pose = sk.DoubleWishbone(sk.load_hardpoints(OPTIMIZED)).solve(0.0)
    caster, kpi = geo.kingpin_angles_deg(model["points"])
    assert caster == pytest.approx(m.caster_deg(pose), abs=1e-6)
    assert kpi == pytest.approx(m.kpi_deg(pose), abs=1e-6)


def test_kingpin_line_hits_ground_at_solver_point(model):
    pose = sk.DoubleWishbone(sk.load_hardpoints(OPTIMIZED)).solve(0.0)
    ground, top = geo.kingpin_line(model["points"])
    assert ground == pytest.approx(tuple(m.steer_axis_ground_point(pose)), abs=1e-6)
    assert top[2] > model["points"]["UBJ"][2]
    # ground, LBJ and top all lie on one line
    d1 = geo.unit(geo.sub(top, ground))
    d2 = geo.unit(geo.sub(model["points"]["LBJ"], ground))
    assert geo.norm(geo.cross(d1, d2)) < 1e-9


def test_trim_segment():
    a, b = (0.0, 0.0, 0.0), (100.0, 0.0, 0.0)
    s, e = geo.trim_segment(a, b, 10.0, 5.0)
    assert s == pytest.approx((10.0, 0.0, 0.0))
    assert e == pytest.approx((95.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        geo.trim_segment(a, b, 60.0, 50.0)


def test_tubes_are_trimmed_and_on_link_lines(model):
    pts = model["points"]
    segs = geo.tube_segments(pts, ball_radius_mm=11.0, rod_end_radius_mm=11.0, overlap_mm=1.0)
    assert [s["name"] for s in segs] == [lk[0] for lk in geo.skeleton_links(pts)]
    for seg, (_n, a, b, _g) in zip(segs, geo.skeleton_links(pts)):
        assert geo.distance(seg["start"], pts[a]) == pytest.approx(10.0)
        assert geo.distance(seg["end"], pts[b]) == pytest.approx(10.0)
        full = geo.distance(pts[a], pts[b])
        assert geo.distance(seg["start"], seg["end"]) == pytest.approx(full - 20.0)


def test_rod_end_axes_are_perpendicular_to_links(model):
    pts = model["points"]
    for link in geo.skeleton_links(pts):
        _n, a, b, group = link
        axis = geo.rod_end_axis(pts, link)
        assert geo.norm(axis) == pytest.approx(1.0)
        assert geo.dot(axis, geo.unit(geo.sub(pts[b], pts[a]))) == pytest.approx(0.0, abs=1e-12)
        if group in geo.PIVOT_AXES:
            # A-arm rod-end bolts point roughly along the front-rear pivot axis
            f, r = geo.PIVOT_AXES[group]
            assert abs(geo.dot(axis, geo.unit(geo.sub(pts[r], pts[f])))) > 0.9


def test_link_lengths_match_solver_links(model):
    lengths = geo.link_lengths(model["points"])
    pts = model["points"]
    assert lengths["Tie rod"] == pytest.approx(geo.distance(pts["RTI"], pts["RTO"]))
    assert all(100 < v < 500 for v in lengths.values())


def test_tyre_is_centred_on_wheel(model):
    inner, outer, radius = geo.wheel_cylinder(model["points"], model["wheel_axis"], 180.0)
    mid = geo.lerp(inner, outer, 0.5)
    assert mid == pytest.approx(model["points"]["WC"])
    assert geo.distance(inner, outer) == pytest.approx(180.0)
    assert radius == pytest.approx(228.0, abs=1.0)


def test_helper_uses_only_standard_library():
    """Fusion's Python has no numpy/scipy, so the helper must not import them."""
    tree = ast.parse((SCRIPT_DIR / "suspension_geometry.py").read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    assert imported <= {"__future__", "json", "math"}


def test_fusion_entry_script_compiles():
    py_compile.compile(str(SCRIPT_DIR / "SuspensionModel.py"), doraise=True)


def test_manifest_is_valid_json():
    manifest = json.loads((SCRIPT_DIR / "SuspensionModel.manifest").read_text())
    assert manifest["type"] == "script"


def test_fusion_script_runs_against_a_fake_api(monkeypatch, tmp_path):
    """Smoke test: run the whole script with Fusion's API replaced by mocks.

    This can't check the CAD result, but it catches typos, wrong variable
    names and logic errors in the script before Ethan runs it in Fusion.
    """
    import importlib
    from unittest import mock

    adsk = mock.MagicMock()
    for name, mod in {"adsk": adsk, "adsk.core": adsk.core, "adsk.fusion": adsk.fusion}.items():
        monkeypatch.setitem(sys.modules, name, mod)
    import SuspensionModel as script
    script = importlib.reload(script)

    ui = adsk.core.Application.get.return_value.userInterface
    step = str(tmp_path / "out.step")
    dlg = ui.createFileDialog.return_value
    dlg.showSave.return_value = adsk.core.DialogResults.DialogOK
    dlg.filename = step
    # The mock can't count bodies by itself; pretend each Base Feature has one.
    root = adsk.fusion.Design.cast.return_value.rootComponent
    comp = root.occurrences.addNewComponent.return_value.component
    comp.features.baseFeatures.add.return_value.bodies.count = 1
    monkeypatch.setattr(script, "DEFAULT_JSON", str(OPTIMIZED))

    script.run(None)

    message = ui.messageBox.call_args[0][0]
    assert message.startswith("Built: "), message
    assert "STEP: " + step in message
    caster, kpi = geo.kingpin_angles_deg(geo.load_model(OPTIMIZED)["points"])
    assert "KPI {:.2f}".format(kpi) in message


def test_fusion_script_falls_back_when_components_are_refused(monkeypatch):
    """Part Design documents allow one component; the script must still finish."""
    import importlib
    from unittest import mock

    adsk = mock.MagicMock()
    for name, mod in {"adsk": adsk, "adsk.core": adsk.core, "adsk.fusion": adsk.fusion}.items():
        monkeypatch.setitem(sys.modules, name, mod)
    import SuspensionModel as script
    script = importlib.reload(script)

    ui = adsk.core.Application.get.return_value.userInterface
    root = adsk.fusion.Design.cast.return_value.rootComponent
    root.occurrences.addNewComponent.side_effect = RuntimeError("not allowed")
    # Fusion also refuses to rename the root component (seen in Fusion 2026)
    type(root).name = mock.PropertyMock(side_effect=RuntimeError("root component name cannot be changed"))
    root.features.baseFeatures.add.return_value.bodies.count = 1
    monkeypatch.setattr(script, "DEFAULT_JSON", str(OPTIMIZED))
    monkeypatch.setattr(script, "EXPORT_STEP", False)

    script.run(None)

    message = ui.messageBox.call_args[0][0]
    assert message.startswith("Built: "), message
    assert "only one component" in message


# --- Phase 4: pushrod, rocker and damper ------------------------------------
def legacy_direct_acting_raw():
    """A Phase 3 style hardpoints file: SHU on the upright, no rocker block."""
    raw = json.loads(OPTIMIZED.read_text())
    del raw["rocker"]
    raw["upright"]["SHU"] = [0.0, 545.0, 145.0]
    return raw


def test_rocker_points_and_axis_are_read(model):
    hp = sk.load_hardpoints(OPTIMIZED)
    assert geo.has_rocker(model["points"])
    assert model["rocker_axis"] == pytest.approx(tuple(hp.rocker.axis))
    assert model["pushrod_mount"] == "lower_arm"
    for name, p in hp.rocker.points.items():
        assert model["points"][name] == pytest.approx(tuple(p))
    assert [lk[1:3] for lk in geo.skeleton_links(model["points"])][-1] == ("PRI", "PRO")


def test_poses_file_carries_rocker_points_at_any_travel(tmp_path):
    hp = sk.load_hardpoints(OPTIMIZED)
    poses = sk.bump_sweep(hp, travel_mm=30, step_mm=10)
    path = tmp_path / "opt_poses.json"
    sk.write_poses_json(poses, path, name=hp.name, rocker=hp.rocker)
    model = geo.load_model(path, travel_mm=30.0)
    assert model["rocker_axis"] == pytest.approx(tuple(hp.rocker.axis))
    bump = poses[-1]
    for name in ("PRO", "PRI", "SHR"):
        assert model["points"][name] == pytest.approx(tuple(bump.points[name]), abs=1e-4)
    # The pushrod is still its design length in the drawn pose.
    lengths = geo.link_lengths(model["points"])
    assert lengths["Pushrod"] == pytest.approx(hp.rocker.pushrod_length, abs=1e-3)


def test_old_direct_acting_file_still_draws():
    model = geo.parse_model(legacy_direct_acting_raw())
    assert model["rocker_axis"] is None
    assert geo.skeleton_links(model["points"])[-1][1:3] == ("SHC", "SHU")
    assert "Damper (eye to eye)" not in geo.link_lengths(model["points"])


def test_rocker_block_without_axis_is_rejected():
    raw = json.loads(OPTIMIZED.read_text())
    del raw["rocker"]["axis"]
    with pytest.raises(ValueError, match="axis"):
        geo.parse_model(raw)


def test_pushrod_has_rod_ends_bolted_parallel_to_the_rocker(model):
    pts = model["points"]
    link = geo.PUSHROD_LINK
    assert geo.end_styles(pts, link) == ("rod_end", "rod_end")
    pushrod = geo.unit(geo.sub(pts["PRO"], pts["PRI"]))
    at_rocker = geo.rod_end_axis(pts, link, at="PRI", rocker_axis=model["rocker_axis"])
    at_arm = geo.rod_end_axis(pts, link, at="PRO", rocker_axis=model["rocker_axis"])
    for axis in (at_rocker, at_arm):
        assert geo.dot(axis, pushrod) == pytest.approx(0.0, abs=1e-12)
    assert abs(geo.dot(at_rocker, model["rocker_axis"])) > 0.99
    lower_pivot = geo.unit(geo.sub(pts["LAA"], pts["LAF"]))
    assert abs(geo.dot(at_arm, lower_pivot)) > 0.9


def test_rocker_is_flat_and_damper_parts_line_up(model):
    pts = model["points"]
    assert geo.rocker_plane_error_deg(pts, model["rocker_axis"]) < 1.0
    a0, a1 = geo.rocker_axis_line(pts, model["rocker_axis"], half_length_mm=30.0)
    assert geo.lerp(a0, a1, 0.5) == pytest.approx(pts["RKP"])
    body_start, body_end, shaft_end = geo.damper_segments(pts, 110.0, 11.0)
    shc, shr = pts["SHC"], pts["SHR"]
    assert geo.distance(shc, body_start) == pytest.approx(11.0)
    assert geo.distance(body_start, body_end) == pytest.approx(110.0)
    assert geo.distance(shaft_end, shr) == pytest.approx(11.0)
    lengths = geo.link_lengths(pts)
    assert lengths["Damper (eye to eye)"] == pytest.approx(geo.distance(shc, shr))


def test_fusion_script_builds_rocker_and_damper_components(monkeypatch):
    """With a rocker in the JSON, the fake Fusion run adds two more components."""
    import importlib
    from unittest import mock

    def run_with(path):
        adsk = mock.MagicMock()
        for name, mod in {"adsk": adsk, "adsk.core": adsk.core, "adsk.fusion": adsk.fusion}.items():
            monkeypatch.setitem(sys.modules, name, mod)
        import SuspensionModel as script
        script = importlib.reload(script)
        root = adsk.fusion.Design.cast.return_value.rootComponent
        root.occurrences.addNewComponent.return_value.component.features.baseFeatures.add \
            .return_value.bodies.count = 1
        monkeypatch.setattr(script, "DEFAULT_JSON", str(path))
        monkeypatch.setattr(script, "EXPORT_STEP", False)
        script.run(None)
        ui = adsk.core.Application.get.return_value.userInterface
        return ui.messageBox.call_args[0][0], root.occurrences.addNewComponent.call_count

    message, n_rocker = run_with(OPTIMIZED)
    assert message.startswith("Built: "), message
    assert "Damper (eye to eye)" in message
    # 4 tube parts + rocker + damper + tyre
    assert n_rocker == 7


def test_fusion_script_still_runs_on_a_direct_acting_file(monkeypatch, tmp_path):
    import importlib
    from unittest import mock

    path = tmp_path / "old.json"
    path.write_text(json.dumps(legacy_direct_acting_raw()))
    adsk = mock.MagicMock()
    for name, mod in {"adsk": adsk, "adsk.core": adsk.core, "adsk.fusion": adsk.fusion}.items():
        monkeypatch.setitem(sys.modules, name, mod)
    import SuspensionModel as script
    script = importlib.reload(script)
    root = adsk.fusion.Design.cast.return_value.rootComponent
    root.occurrences.addNewComponent.return_value.component.features.baseFeatures.add \
        .return_value.bodies.count = 1
    monkeypatch.setattr(script, "DEFAULT_JSON", str(path))
    monkeypatch.setattr(script, "EXPORT_STEP", False)
    script.run(None)
    message = adsk.core.Application.get.return_value.userInterface.messageBox.call_args[0][0]
    assert message.startswith("Built: "), message
    assert "direct-acting" in message
    assert root.occurrences.addNewComponent.call_count == 5  # 4 parts + tyre
