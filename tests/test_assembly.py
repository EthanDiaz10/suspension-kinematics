"""Phase 10: whole-car assembly (``suspkin assembly`` and the Fusion CarAssembly script)."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

import suspkin as sk
from suspkin.assembly import (MIRROR, build_assembly, group_bounds, load_config, mirror_solid,
                              solid_points)
from suspkin.parts import PartsConfig

ROOT = Path(__file__).resolve().parents[1]
FRONT = ROOT / "data" / "optimized_front.json"
REAR = ROOT / "data" / "optimized_rear.json"
CONFIG = ROOT / "data" / "assembly.json"
PARTS = ROOT / "data" / "parts.json"
FUSION = ROOT / "cad" / "fusion" / "CarAssembly"


@pytest.fixture(scope="module")
def hps():
    return sk.load_hardpoints(FRONT), sk.load_hardpoints(REAR)


@pytest.fixture(scope="module")
def asm(hps):
    return build_assembly(*hps, load_config(CONFIG), PartsConfig.load(PARTS))


def groups(asm):
    return {g["name"]: g for g in asm["groups"]}


def component(group, name):
    return next(c for c in group["components"] if c["name"] == name)


def bodies(comp):
    return {b["name"]: b for b in comp["bodies"]}


def test_every_group_is_there(asm):
    assert set(groups(asm)) == {
        "Front left corner", "Front right corner", "Front anti-roll bar", "Steering rack",
        "Rear left corner", "Rear right corner", "Rear anti-roll bar", "Chassis frame"}
    names = {c["name"] for c in groups(asm)["Rear left corner"]["components"]}
    assert {"Upper A-arm", "Lower A-arm", "Toe link", "Upright", "Pushrod", "Pushrod bracket",
            "Rocker", "Damper", "ARB drop link", "Wheel"} == names
    assert "Tie rod" in {c["name"] for c in groups(asm)["Front left corner"]["components"]}
    assert asm["body_count"] == sum(len(c["bodies"]) for g in asm["groups"] for c in g["components"])


def test_right_side_is_the_left_mirrored(asm):
    g = groups(asm)
    for axle in ("Front", "Rear"):
        left, right = g[f"{axle} left corner"], g[f"{axle} right corner"]
        for cl, cr in zip(left["components"], right["components"]):
            for bl, br in zip(cl["bodies"], cr["bodies"]):
                assert [mirror_solid(s) for s in bl["solids"]] == br["solids"]
        lo, hi = group_bounds(right)
        assert hi[1] < 0 < group_bounds(left)[0][1]


def test_rear_is_one_wheelbase_back(asm, hps):
    front, rear = hps
    wb = rear.extra["vehicle"]["wheelbase_mm"]
    assert asm["wheelbase_mm"] == wb
    lbj_front = np.array(bodies(component(groups(asm)["Front left corner"], "Lower A-arm"))
                         ["Clevis LBJ (front)"]["solids"][0]["p1"])
    lbj_rear = np.array(bodies(component(groups(asm)["Rear left corner"], "Lower A-arm"))
                        ["Clevis LBJ (front)"]["solids"][0]["p1"])
    # Same clevis on each corner: it moves by the difference in LBJ plus one wheelbase.
    d = lbj_rear - lbj_front
    assert d == pytest.approx(rear.upright["LBJ"] - front.upright["LBJ"] - [wb, 0, 0], abs=1e-3)


def test_arm_tubes_lie_on_the_link_lines_and_rod_ends_are_square(asm, hps):
    hp = hps[0]
    arm = bodies(component(groups(asm)["Front left corner"], "Upper A-arm"))
    for leg, inner in (("front leg", "UAF"), ("rear leg", "UAA")):
        s = arm[f"Upper A-arm {leg}"]["solids"][0]
        link = hp.upright["UBJ"] - hp.chassis[inner]
        for p in (s["p1"], s["p2"]):
            off = np.cross(np.array(p) - hp.chassis[inner], link / np.linalg.norm(link))
            assert np.linalg.norm(off) < 1e-3
        eye = arm[f"Rod end {inner}"]["solids"][0]
        axis = np.array(eye["p2"]) - np.array(eye["p1"])
        assert abs(axis @ link) / np.linalg.norm(axis) / np.linalg.norm(link) < 1e-4   # 4-decimal rounding
        assert (np.array(eye["p1"]) + eye["p2"]) / 2 == pytest.approx(hp.chassis[inner], abs=1e-3)


def test_tyres_sit_on_the_ground_at_the_contact_patch(asm):
    for name in ("Front left corner", "Rear right corner"):
        tyre = bodies(component(groups(asm)[name], "Wheel"))["Tyre"]["solids"][0]
        p1, p2 = np.array(tyre["p1"]), np.array(tyre["p2"])
        a = (p2 - p1) / np.linalg.norm(p2 - p1)
        # Lowest point of the tyre's outer edge: centre minus radius along "down in the wheel plane".
        down = np.array([0, 0, -1.0]) - a[2] * a
        down /= np.linalg.norm(down)
        lowest = min(p1[2], p2[2]) + tyre["radius"] * down[2]
        assert lowest == pytest.approx(0.0, abs=4.0)   # camber tips the edge a few mm


def test_upright_parts_come_from_phase9(asm, hps):
    up = component(groups(asm)["Front left corner"], "Upright")
    assert up["material"] == "7075-T6"
    assert any(c["type"] == "cylinder" for c in up["bodies"][0]["cuts"])   # the bearing bore


def test_arb_bar_spans_both_drop_links(asm, hps):
    bar = bodies(component(groups(asm)["Front anti-roll bar"], "Front anti-roll bar"))["Torsion bar"]
    s = bar["solids"][0]
    arp = hps[0].arb.points["ARP"]
    assert s["p1"][1] < -arp[1] + 1 and s["p2"][1] > arp[1] - 1
    assert s["p1"][0] == pytest.approx(arp[0]) and s["p1"][2] == pytest.approx(arp[2])


def test_mirror_flips_only_y():
    s = {"type": "box", "center": [1, 2, 3], "length_dir": [0, 1, 0], "width_dir": [1, 0, 0],
         "length": 1, "width": 1, "height": 1}
    m = mirror_solid(s)
    assert m["center"] == [1, -2, 3] and m["length_dir"] == [0, -1, 0] and m["width_dir"] == [1, 0, 0]
    assert np.allclose(MIRROR, [1, -1, 1])


def test_all_solids_are_valid(asm):
    for g in asm["groups"]:
        for c in g["components"]:
            for b in c["bodies"]:
                assert b["solids"], b["name"]
                for s in b["solids"] + b["cuts"]:
                    assert np.isfinite(solid_points(s)).all()
                    if s["type"] == "cylinder":
                        assert s["radius"] > 0
                        assert np.linalg.norm(np.subtract(s["p2"], s["p1"])) > 0.5, b["name"]


def test_cli_writes_the_assembly(tmp_path, capsys):
    from suspkin.cli import main
    main(["assembly", str(FRONT), str(REAR), "--config", str(CONFIG), "--parts", str(PARTS),
          "--out", str(tmp_path)])
    raw = json.loads((tmp_path / "car_assembly.json").read_text())
    assert len(raw["groups"]) == 8
    out = capsys.readouterr().out
    assert "Rear anti-roll bar" in out and "Space frame:" in out and "No frame tube comes within" in out


# --- the Fusion script, with Fusion's API replaced by mocks -----------------

def _load_script(monkeypatch):
    import importlib
    from unittest import mock
    adsk = mock.MagicMock()
    for name, mod in {"adsk": adsk, "adsk.core": adsk.core, "adsk.fusion": adsk.fusion}.items():
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.syspath_prepend(str(FUSION))
    import CarAssembly as script
    return adsk, importlib.reload(script)


def test_fusion_script_compiles_and_manifest_is_valid():
    import py_compile
    py_compile.compile(str(FUSION / "CarAssembly.py"), doraise=True)
    assert json.loads((FUSION / "CarAssembly.manifest").read_text())["type"] == "script"


def test_fusion_script_finds_results_folder():
    sys.path.insert(0, str(FUSION))
    try:
        from unittest import mock
        with mock.patch.dict(sys.modules, {"adsk": mock.MagicMock(), "adsk.core": mock.MagicMock(),
                                           "adsk.fusion": mock.MagicMock()}):
            import importlib
            import CarAssembly as script
            script = importlib.reload(script)
            p = Path(script.default_json_path(str(FUSION)))
            assert p == ROOT / "results" / "car_assembly.json"
    finally:
        sys.path.remove(str(FUSION))


def test_fusion_script_runs_against_a_fake_api(monkeypatch, asm, tmp_path):
    adsk, script = _load_script(monkeypatch)
    path = tmp_path / "car_assembly.json"
    path.write_text(json.dumps(asm))
    monkeypatch.setattr(script, "DEFAULT_JSON", str(path))
    ui = adsk.core.Application.get.return_value.userInterface
    script.run(None)
    message = ui.messageBox.call_args[0][0]
    assert message.startswith("Built: "), message
    assert "Front left corner: 10 parts" in message
    assert "STEP: " + str(tmp_path / "car_assembly.step") in message
    # Every box handed to Fusion has exactly perpendicular directions.
    for g in asm["groups"]:
        for c in g["components"]:
            for b in c["bodies"]:
                for s in b["solids"]:
                    if s["type"] == "box":
                        lv, wv = (np.array(v) for v in script.box_axes(s["length_dir"], s["width_dir"]))
                        assert abs(lv @ wv) < 1e-12


def test_fusion_script_falls_back_to_parametric(monkeypatch, asm, tmp_path):
    adsk, script = _load_script(monkeypatch)
    path = tmp_path / "car_assembly.json"
    path.write_text(json.dumps(asm))
    monkeypatch.setattr(script, "DEFAULT_JSON", str(path))
    monkeypatch.setattr(script, "EXPORT_STEP", False)
    design = adsk.fusion.Design.cast.return_value
    from unittest import mock
    type(design).designType = mock.PropertyMock(side_effect=RuntimeError("no direct"))
    script.run(None)
    message = adsk.core.Application.get.return_value.userInterface.messageBox.call_args[0][0]
    assert message.startswith("Built: "), message
    assert "parametric" in message


def test_fusion_script_rejects_a_wrong_file(monkeypatch, tmp_path):
    adsk, script = _load_script(monkeypatch)
    path = tmp_path / "x.json"
    path.write_text(json.dumps({"parts": {}}))
    monkeypatch.setattr(script, "DEFAULT_JSON", str(path))
    script.run(None)
    message = adsk.core.Application.get.return_value.userInterface.messageBox.call_args[0][0]
    assert "suspkin assembly" in message


# --- Phase 10 step 2: the space frame -----------------------------------------

def test_frame_clears_every_suspension_part(asm):
    from suspkin.frame import frame_clearances
    assert frame_clearances(asm, near_mm=3.0) == []


def test_frame_is_symmetric_and_sensible(asm):
    frame = groups(asm)["Chassis frame"]
    tubes = bodies(frame["components"][0])
    for name, b in tubes.items():
        if name.endswith("(left)"):
            right = tubes[name.replace("(left)", "(right)")]
            assert [mirror_solid(s) for s in b["solids"]] == right["solids"]
    f = asm["frame"]
    assert 20 < f["mass_kg"] < 45          # FSAE steel frames are roughly 25-40 kg
    lo, hi = group_bounds(frame)
    assert lo[2] > 50                      # ground clearance under the lowest tube centre
    assert hi[2] > 1000                    # the main hoop is above a seated driver's helmet


def test_every_wishbone_pickup_has_a_tab_to_a_frame_node(asm, hps):
    from suspkin.frame import frame_nodes
    front, rear = hps
    cfg = load_config(CONFIG)
    nodes = frame_nodes(front, rear, cfg["frame"], asm["wheelbase_mm"])
    tabs = bodies(groups(asm)["Chassis frame"]["components"][1])
    for prefix, hp, off in (("F", front, 0.0), ("R", rear, -asm["wheelbase_mm"])):
        axle = "Front" if prefix == "F" else "Rear"
        for k in ("UAF", "UAA", "LAF", "LAA"):
            tab = tabs[f"{axle} {k} pickup tab (left)"]["solids"][0]
            pickup = hp.chassis[k] + [off, 0, 0]
            # the tab's long axis runs from the node to the pickup
            d = pickup - nodes[f"{prefix}_{k}"]
            assert np.cross(tab["length_dir"], d / np.linalg.norm(d)) == pytest.approx(0, abs=1e-4)
            assert nodes[f"{prefix}_{k}"][1] < pickup[1]       # node is inboard of the pickup


def test_segment_distance():
    from suspkin.frame import _segment_distance
    a = np.array
    assert _segment_distance(a([0, 0, 0.]), a([1, 0, 0.]), a([0, 1, 1.]), a([1, 1, 1.])) == pytest.approx(2 ** 0.5)
    assert _segment_distance(a([0, 0, 0.]), a([2, 0, 0.]), a([1, -1, 0.]), a([1, 1, 0.])) == pytest.approx(0)
    assert _segment_distance(a([0, 0, 0.]), a([1, 0, 0.]), a([3, 0, 0.]), a([4, 0, 0.])) == pytest.approx(2)
