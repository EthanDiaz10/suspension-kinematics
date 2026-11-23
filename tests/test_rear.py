"""Phase 7: the rear corner (toe link, anti-squat) and the whole car.

The rear corner uses the same solver as the front: its toe link is the "tie
rod" (RTI on the chassis, RTO on the upright), just never moved by a rack.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest
import suspkin as sk
from suspkin.cli import main
from suspkin.fullcar import camber_vs_g, full_car, placeholder_check
from suspkin.kinematics import LINKS, DoubleWishbone
from suspkin.optimize import Goal, OptimizeConfig, Variable, measure, optimize, score
from suspkin.roll import G, load_transfer, roll_metrics, roll_summary, roll_sweep
from suspkin.sideview import VehicleParams

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
REAR_BASE = DATA / "baseline_fsae_rear.json"
REAR_OPT = DATA / "optimized_rear.json"
FRONT_OPT = DATA / "optimized_front.json"

sys.path.insert(0, str(ROOT / "cad" / "fusion" / "SuspensionModel"))
import suspension_geometry as geo


@pytest.fixture(scope="module")
def rear_sweep():
    hp = sk.load_hardpoints(REAR_OPT)
    poses = sk.bump_sweep(hp, 50, 5)
    return hp, poses, sk.compute_metrics(poses, VehicleParams.from_hardpoints(hp))


@pytest.fixture(scope="module")
def car():
    return full_car(sk.load_hardpoints(FRONT_OPT), sk.load_hardpoints(REAR_OPT))


# --- the rear corner on its own ----------------------------------------------
@pytest.mark.parametrize("path", [REAR_BASE, REAR_OPT])
def test_rear_files_load_as_a_driven_rear_axle_and_fit_the_rim(path):
    hp = sk.load_hardpoints(path)
    v = VehicleParams.from_hardpoints(hp)
    assert v.axle == "rear" and v.driven
    assert all(c.ok for c in sk.check_packaging(hp))


def test_toe_link_keeps_its_length(rear_sweep):
    hp, poses, _m = rear_sweep
    model = DoubleWishbone(hp)
    for p in poses:
        for c, u in LINKS:
            assert np.linalg.norm(p.points[u] - p.points[c]) == pytest.approx(
                model.link_lengths[(c, u)], abs=1e-6)
        assert p.rack == 0.0  # nothing steers the rear


def test_optimised_rear_kinematics(rear_sweep):
    hp, _poses, m = rear_sweep
    s = sk.summary(m, VehicleParams.from_hardpoints(hp))
    assert s["toe_range_deg"] < 0.02                       # almost no bump steer
    assert abs(s["rc_migration_vs_chassis_mm_per_mm"]) < 0.06
    assert 20 < s["anti_squat_pct"] < 30                   # target 25%
    assert -0.5 < s["camber_gain_deg_per_10mm"] < -0.35
    assert s["static_camber_deg"] == pytest.approx(-1.75)
    assert 0.7 < s["motion_ratio"] < 0.85


def test_anti_squat_and_anti_lift_by_hand(rear_sweep):
    """Rear anti-squat (inboard drive) = -(dx/dz of the wheel centre) x L / h,
    and anti-lift (outboard brakes) uses the contact patch and the rear brake share.

    Positive anti-squat needs the wheel to move REARWARD as it goes into bump,
    which also checks the sign convention for a rear axle.
    """
    hp, poses, m = rear_sweep
    v = VehicleParams.from_hardpoints(hp)
    i0 = int(np.argmin(np.abs(m["travel_mm"])))
    t = m["travel_mm"]
    wc_x = np.array([p.points["WC"][0] for p in poses])
    cp_x = np.array([p.points["CP"][0] for p in poses])
    wc_z = np.array([p.points["WC"][2] for p in poses])
    cp_z = np.array([p.points["CP"][2] for p in poses])
    dxdz_wc = np.gradient(wc_x, t)[i0] / np.gradient(wc_z, t)[i0]
    dxdz_cp = np.gradient(cp_x, t)[i0] / np.gradient(cp_z, t)[i0]
    assert dxdz_wc < 0  # wheel centre moves rearward in bump
    k = v.wheelbase_mm / v.cg_height_mm * 100
    assert m["anti_drive_pct"][i0] == pytest.approx(-dxdz_wc * k, rel=0.02)
    assert m["anti_brake_pct"][i0] == pytest.approx(-dxdz_cp * k * (1 - v.front_brake_bias), rel=0.02)
    assert 5 < m["anti_brake_pct"][i0] < 12  # anti-lift target 10%


def test_rear_roll_summary_has_no_placeholder_car_numbers():
    hp = sk.load_hardpoints(REAR_OPT)
    v = VehicleParams.from_hardpoints(hp)
    s = roll_summary(roll_metrics(roll_sweep(hp, 2, 0.5), v), v)
    assert "front_load_transfer_share_pct" not in s
    assert s["roll_stiffness_bar_nm_per_deg"] > 0  # the rear bar is in


def test_steer_command_refuses_a_rear_corner():
    with pytest.raises(SystemExit, match="toe link is fixed"):
        main(["steer", str(REAR_OPT)])


# --- optimiser: the anti-squat goal --------------------------------------------
def test_anti_drive_goal_needs_a_driven_axle():
    front = sk.load_hardpoints(DATA / "baseline_fsae_front.json")
    achieved = measure(front, 50, 25)
    assert np.isnan(achieved["anti_drive"])
    with pytest.raises(ValueError, match="driven"):
        score(achieved, {"anti_drive": Goal(25.0, 5.0, 1.0)})


def test_optimiser_moves_anti_squat_toward_target():
    """Tilting the upper arm in side view (one pivot only) buys anti-squat."""
    hp = sk.load_hardpoints(REAR_BASE)
    cfg = OptimizeConfig(
        variables=[Variable("upper front pivot z", ["UAF"], "z", (-20, 20))],
        goals={"anti_drive": Goal(15.0, 5.0, 1.0)},
        travel_mm=20, step_mm=10, max_iter=15,
    )
    res = optimize(hp, cfg)
    assert abs(res.after["anti_drive"] - 15.0) < 1.0
    assert abs(res.before["anti_drive"]) < 2.0


# --- whole car -------------------------------------------------------------------
def test_total_load_transfer_is_weight_times_cg_height_over_track():
    """With equal tracks, the two axles' load transfer adds up to W h / t,
    whatever the stiffness split or roll centre heights (statics)."""
    v = VehicleParams()
    W = v.mass_kg * G
    for kf, kr, rcf, rcr in ((400, 400, 20, 40), (700, 200, 0, 60), (300, 500, 35, 10)):
        lt = load_transfer(kf, kr, rcf, rcr, 1200.0, 1200.0, v)
        total = lt["front_load_transfer_n_per_g"] + lt["rear_load_transfer_n_per_g"]
        assert total == pytest.approx(W * v.cg_height_mm / 1200.0)


def test_full_car_balance(car):
    n = car.numbers
    assert n["front_weight_pct"] == pytest.approx(47.0)
    # A few percent more load transfer at the front than the weight split.
    assert 50 < n["front_load_transfer_share_pct"] < 54
    assert 0.7 < n["roll_gradient_deg_per_g"] < 0.95
    assert n["front_rc_height_mm"] < n["rear_rc_height_mm"]  # roll axis rises to the rear
    for axle in ("front", "rear"):
        assert n[f"{axle}_inside_wheel_load_n"] > 0          # no wheel lift at 1.5 g
        assert n[f"{axle}_outer_camber_at_design_g_deg"] == pytest.approx(-1.0, abs=0.1)
        assert 2.0 < n[f"{axle}_ride_frequency_hz"] < 4.0


def test_full_car_uses_the_real_rear_and_matches_updated_placeholders(car):
    """The front file's rear_* numbers were copied from the real rear, so a
    front-only `suspkin roll` must agree with `suspkin car`."""
    for name, (placeholder, real) in placeholder_check(car).items():
        assert placeholder == pytest.approx(real, abs=0.1), name
    front = sk.load_hardpoints(FRONT_OPT)
    v = VehicleParams.from_hardpoints(front)
    alone = roll_summary(roll_metrics(roll_sweep(front, 3, 0.25), v), v)
    assert alone["front_load_transfer_share_pct"] == pytest.approx(
        car.numbers["front_load_transfer_share_pct"], abs=0.05)


def test_stiffer_rear_bar_moves_load_transfer_rearward():
    front = sk.load_hardpoints(FRONT_OPT)
    rear = sk.load_hardpoints(REAR_OPT)
    base = full_car(front, rear, 2, 0.5).numbers
    rear.arb.rate_nm_per_deg *= 2
    stiffer = full_car(front, rear, 2, 0.5).numbers
    assert stiffer["front_load_transfer_share_pct"] < base["front_load_transfer_share_pct"]
    assert stiffer["roll_gradient_deg_per_g"] < base["roll_gradient_deg_per_g"]


def test_files_in_the_wrong_order_are_rejected():
    front = sk.load_hardpoints(FRONT_OPT)
    rear = sk.load_hardpoints(REAR_OPT)
    with pytest.raises(ValueError, match="FRONT"):
        full_car(rear, front, 1, 0.5)


def test_camber_vs_g_starts_at_static_camber(car):
    c = camber_vs_g(car)
    assert c["lateral_g"][0] == 0.0
    assert c["front_outer_camber_deg"][0] == pytest.approx(car.front.hp.static_camber_deg, abs=1e-6)
    assert c["rear_outer_camber_deg"][0] == pytest.approx(car.rear.hp.static_camber_deg, abs=1e-6)


def test_car_command_writes_its_plot(tmp_path, capsys):
    main(["car", str(FRONT_OPT), str(REAR_OPT), "--roll", "2", "--step", "0.5",
          "--out", str(tmp_path)])
    out = capsys.readouterr().out
    assert "front_load_transfer_share_pct" in out
    assert (tmp_path / "optimized_front+optimized_rear_car.png").exists()


# --- Fusion: building the rear corner ----------------------------------------------
def test_fusion_reads_a_rear_corner_and_names_the_toe_link():
    model = geo.load_model(REAR_OPT)
    assert model["axle"] == "rear" and model["wheelbase_mm"] == 1550.0
    names = [lk[0] for lk in geo.skeleton_links(model["points"], model["axle"])]
    assert "Toe link" in names and "Tie rod" not in names
    assert "Toe link" in geo.link_lengths(model["points"], "rear")
    assert geo.part_name("Tie rod", "rear") == "Toe link"
    assert geo.part_name("Tie rod", "front") == "Tie rod"
    # Front files (with or without a vehicle block) stay front corners.
    assert geo.load_model(FRONT_OPT)["axle"] == "front"


def test_place_on_car_moves_the_rear_back_one_wheelbase_only():
    model = geo.load_model(REAR_OPT)
    placed = geo.place_on_car(model)
    for k, p in model["points"].items():
        q = placed["points"][k]
        assert q[0] == pytest.approx(p[0] - 1550.0) and q[1:] == pytest.approx(p[1:])
    assert geo.link_lengths(placed["points"], "rear") == pytest.approx(
        geo.link_lengths(model["points"], "rear"))
    front = geo.load_model(FRONT_OPT)
    assert geo.place_on_car(front)["points"] == front["points"]


def test_rear_poses_file_carries_the_axle(tmp_path):
    main(["sweep", str(REAR_OPT), "--step", "25", "--out", str(tmp_path)])
    raw = json.loads((tmp_path / "optimized_rear_poses.json").read_text())
    assert raw["vehicle"] == {"axle": "rear", "wheelbase_mm": 1550.0}
    assert geo.parse_model(raw, travel_mm=25)["axle"] == "rear"


def test_fusion_script_builds_a_rear_corner(monkeypatch):
    import importlib
    from unittest import mock

    adsk = mock.MagicMock()
    for name, mod in {"adsk": adsk, "adsk.core": adsk.core, "adsk.fusion": adsk.fusion}.items():
        monkeypatch.setitem(sys.modules, name, mod)
    import SuspensionModel as script
    script = importlib.reload(script)
    root = adsk.fusion.Design.cast.return_value.rootComponent
    root.occurrences.addNewComponent.return_value.component.features.baseFeatures.add \
        .return_value.bodies.count = 1
    monkeypatch.setattr(script, "DEFAULT_JSON", str(REAR_OPT))
    monkeypatch.setattr(script, "EXPORT_STEP", False)
    script.run(None)
    message = adsk.core.Application.get.return_value.userInterface.messageBox.call_args[0][0]
    assert message.startswith("Built: "), message
    assert "Toe link" in message
    assert "moved -1550 mm in X" in message
