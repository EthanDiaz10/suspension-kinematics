"""suspkin: parametric double-wishbone suspension kinematics.

Typical use::

    import suspkin as sk
    hp = sk.load_hardpoints("data/baseline_fsae_front.json")
    poses = sk.bump_sweep(hp, travel_mm=50, step_mm=1)
    metrics = sk.compute_metrics(poses)
    sk.plot_sweep(metrics, "sweep.png")

    steer = sk.steering_metrics(sk.steer_sweep(hp, rack_mm=25))   # Ackermann
    roll = sk.roll_metrics(sk.roll_sweep(hp, roll_deg=3))          # body roll

    rear = sk.load_hardpoints("data/optimized_rear.json")           # whole car
    car = sk.full_car(sk.load_hardpoints("data/optimized_front.json"), rear)
    print(car.numbers["front_load_transfer_share_pct"])

    loads = sk.analyse_loads(car, sk.LoadsConfig.load("data/load_cases.json"))  # link forces
"""

from .arb import ARBGeometry
from .fullcar import CarResult, full_car
from .export import write_metrics_csv, write_metrics_table, write_poses_json
from .hardpoints import Hardpoints, load_hardpoints, save_hardpoints
from .kinematics import DoubleWishbone, Pose, bump_sweep, steer_sweep
from .metrics import compute_metrics, summary
from .optimize import OptimizeConfig, optimize
from .packaging import check_packaging
from .loads import LoadCase, LoadsConfig, analyse_loads, solve_link_forces, tyre_loads
from .plotting import (plot_comparison, plot_front_view, plot_full_car, plot_loads, plot_roll,
                       plot_roll_front_view, plot_steering, plot_sweep)
from .roll import (RollModel, RollPose, load_transfer, roll_metrics, roll_summary, roll_sweep,
                   whole_car_roll)
from .rocker import RockerGeometry, rocker_checks
from .sideview import VehicleParams
from .steering import steering_metrics, steering_summary

__all__ = [
    "ARBGeometry", "LoadCase", "LoadsConfig", "analyse_loads", "solve_link_forces",
    "tyre_loads", "plot_loads", "whole_car_roll", "load_transfer", "CarResult", "full_car", "plot_full_car",
    "DoubleWishbone", "Hardpoints", "OptimizeConfig", "Pose", "RockerGeometry",
    "VehicleParams", "bump_sweep", "check_packaging", "compute_metrics",
    "load_hardpoints", "optimize", "plot_comparison", "plot_front_view",
    "plot_roll", "plot_roll_front_view", "plot_steering", "plot_sweep",
    "RollModel", "RollPose", "roll_metrics", "roll_summary", "roll_sweep", "rocker_checks", "save_hardpoints",
    "steer_sweep", "steering_metrics", "steering_summary", "summary",
    "write_metrics_csv", "write_metrics_table", "write_poses_json",
]
__version__ = "0.8.0"
