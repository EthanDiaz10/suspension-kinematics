"""Whole car: put a real front corner and a real rear corner together.

Up to Phase 6 the whole-car roll numbers (roll per g, the front/rear load
transfer split) came from ONE front file, with placeholder numbers for the
rear axle in its ``vehicle`` block. Now that the rear corner exists, this
module runs a roll sweep on each axle and feeds both sets of real numbers
into the same Milliken formulas (``roll.load_transfer``).

What it does
------------
1. Roll sweep of the front axle (``front.json``) and of the rear axle
   (``rear.json``), each with its own springs, anti-roll bar and geometry.
2. From each sweep: roll stiffness (springs + bar), roll centre height, track,
   and how the outside tyre's camber changes with body roll.
3. Whole car (``load_transfer``): the roll axis through both roll centres,
   roll per g, and how the lateral load transfer splits front/rear.
4. For each axle: the outside tyre's camber to the road at the design
   cornering load, and the static camber that would put it on target.
5. Tyre loads at the design load: does the inside wheel stay on the ground?

Which numbers come from which file
----------------------------------
Car-level numbers (mass, weight split, CG height, wheelbase, design g, target
camber) come from the FRONT file's ``vehicle`` block. Each axle's spring rate
comes from its own file. The rear_* placeholders in the front file are NOT
used here; ``placeholder_check`` reports how far off they were, so they can
be updated to keep ``suspkin roll`` on the front file honest.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .hardpoints import Hardpoints
from .roll import (
    G,
    camber_at_roll,
    load_transfer,
    roll_metrics,
    roll_summary,
    roll_sweep,
)
from .sideview import VehicleParams


@dataclass
class AxleResult:
    """One axle's roll sweep and its headline numbers."""

    label: str                       # "front" or "rear"
    hp: Hardpoints
    vehicle: VehicleParams           # this file's own vehicle block (spring rate)
    roll: dict[str, np.ndarray]      # roll_metrics columns
    summary: dict[str, float]        # roll_summary (single-axle numbers only)

    @property
    def stiffness(self) -> float:
        return self.summary["roll_stiffness_nm_per_deg"]

    @property
    def rc_height(self) -> float:
        return self.summary["rc_height_mm"]

    @property
    def track(self) -> float:
        r = self.roll["roll_deg"]
        return float(self.roll["track_mm"][int(np.argmin(np.abs(r)))])


def analyse_axle(hp: Hardpoints, label: str, roll_deg: float = 3.0,
                 step_deg: float = 0.25) -> AxleResult:
    """Roll sweep of one axle and its single-axle summary."""
    vehicle = VehicleParams.from_hardpoints(hp)
    roll = roll_metrics(roll_sweep(hp, roll_deg, step_deg), vehicle)
    # Axle-only numbers: the whole-car part is done in full_car() with real
    # numbers for both axles, not the placeholders.
    summary = roll_summary(roll, vehicle, whole_car=False)
    return AxleResult(label, hp, vehicle, roll, summary)


@dataclass
class CarResult:
    front: AxleResult
    rear: AxleResult
    car: VehicleParams               # car-level numbers (from the front file)
    numbers: dict[str, float]        # everything worth printing, see full_car()


def full_car(front_hp: Hardpoints, rear_hp: Hardpoints, roll_deg: float = 3.0,
             step_deg: float = 0.25) -> CarResult:
    """Whole-car roll and load transfer from a front and a rear hardpoints file.

    ``numbers`` holds:

    * ``front_*`` / ``rear_*``: roll stiffness (springs, bar, total), roll
      centre height and track for each axle;
    * the ``load_transfer`` outputs: roll axis height under the CG, roll
      moment arm, roll per g, roll at the design load, front share of roll
      stiffness and of lateral load transfer, and each axle's load transfer;
    * ``*_outer_camber_at_design_g_deg``: each axle's outside tyre camber to
      the road at the design load, and ``*_static_camber_for_target_deg``,
      the static camber that would put it at the target;
    * ``*_inside_wheel_load_n``: inside-tyre load at the design load (below 0
      would mean the wheel lifts off);
    * ``*_ride_frequency_hz``: each axle's natural frequency in heave.
    """
    front = analyse_axle(front_hp, "front", roll_deg, step_deg)
    rear = analyse_axle(rear_hp, "rear", roll_deg, step_deg)
    car = front.vehicle  # car-level numbers live in the front file
    if front.vehicle.axle != "front" or rear.vehicle.axle != "rear":
        raise ValueError("pass the FRONT hardpoints file first and the REAR one second "
                         "(check 'axle' in each file's vehicle block)")
    for key in ("wheelbase_mm", "cg_height_mm"):
        f, r = getattr(front.vehicle, key), getattr(rear.vehicle, key)
        if abs(f - r) > 1e-6:
            raise ValueError(f"the front and rear files disagree on {key} ({f:g} vs {r:g})")

    n: dict[str, float] = {}
    for ax in (front, rear):
        s = ax.summary
        n[f"{ax.label}_roll_stiffness_springs_nm_per_deg"] = s["roll_stiffness_springs_nm_per_deg"]
        n[f"{ax.label}_roll_stiffness_bar_nm_per_deg"] = s["roll_stiffness_bar_nm_per_deg"]
        n[f"{ax.label}_roll_stiffness_nm_per_deg"] = s["roll_stiffness_nm_per_deg"]
        n[f"{ax.label}_rc_height_mm"] = ax.rc_height
        n[f"{ax.label}_track_mm"] = ax.track
        n[f"{ax.label}_wheel_rate_n_per_mm"] = s["wheel_rate_n_per_mm"]

    lt = load_transfer(front.stiffness, rear.stiffness, front.rc_height, rear.rc_height,
                       front.track, rear.track, car)
    n.update(lt)

    phi = lt["roll_at_design_g_deg"]
    for ax in (front, rear):
        c = camber_at_roll(ax.roll, phi, car.target_outside_camber_deg)
        n[f"{ax.label}_outer_camber_at_design_g_deg"] = c["outer_camber_at_design_g_deg"]
        n[f"{ax.label}_static_camber_for_target_deg"] = c["static_camber_for_target_deg"]

    # Tyre loads at the design cornering load. Static load per wheel, minus
    # (inside) or plus (outside) that axle's load transfer.
    W = car.mass_kg * G
    A = car.design_lateral_g
    for label, share, dw in (("front", car.front_weight_fraction, lt["front_load_transfer_n_per_g"]),
                             ("rear", 1.0 - car.front_weight_fraction, lt["rear_load_transfer_n_per_g"])):
        static = W * share / 2.0
        n[f"{label}_static_wheel_load_n"] = static
        n[f"{label}_inside_wheel_load_n"] = static - dw * A
        n[f"{label}_outside_wheel_load_n"] = static + dw * A
        # Ride frequency: one corner's mass bouncing on its wheel rate,
        # f = sqrt(k / m) / (2 pi). Whole car treated as sprung, rigid tyres,
        # so the real number is a bit lower. FSAE cars are often 2.5-4 Hz.
        k_wheel = n[f"{label}_wheel_rate_n_per_mm"] * 1000.0      # N/m
        n[f"{label}_ride_frequency_hz"] = float(np.sqrt(k_wheel / (static / G)) / (2 * np.pi))
    return CarResult(front, rear, car, n)


def placeholder_check(result: CarResult) -> dict[str, tuple[float, float]]:
    """The front file's rear_* placeholders next to the real rear numbers.

    Returns name -> (placeholder, real). Copy the real values into the front
    file's vehicle block so ``suspkin roll`` on the front file alone agrees
    with ``suspkin car``.
    """
    v = result.car
    return {
        "rear_roll_stiffness_nm_per_deg": (v.rear_roll_stiffness_nm_per_deg, result.rear.stiffness),
        "rear_rc_height_mm": (v.rear_rc_height_mm, result.rear.rc_height),
        "rear_track_mm": (v.rear_track_mm, result.rear.track),
    }


def camber_vs_g(result: CarResult, g_max: float | None = None, n: int = 41) -> dict[str, np.ndarray]:
    """Outside-tyre camber (to the road) of each axle against lateral g.

    Body roll grows in a straight line with g (``roll per g``), and each axle's
    roll sweep says what camber goes with each roll angle, so this just maps
    one onto the other. Points past the end of a roll sweep come out NaN.
    """
    g_max = g_max if g_max is not None else 1.25 * result.car.design_lateral_g
    g = np.linspace(0.0, g_max, n)
    roll = g * result.numbers["roll_gradient_deg_per_g"]
    out = {"lateral_g": g, "roll_deg": roll}
    for ax in (result.front, result.rear):
        r = ax.roll["roll_deg"]
        c = np.interp(roll, r, ax.roll["camber_right_deg"], left=np.nan, right=np.nan)
        out[f"{ax.label}_outer_camber_deg"] = c
    return out
