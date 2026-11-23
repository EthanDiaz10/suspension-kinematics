"""suspkin: parametric double-wishbone suspension kinematics.

Typical use::

    import suspkin as sk
    hp = sk.load_hardpoints("data/baseline_fsae_front.json")
    poses = sk.bump_sweep(hp, travel_mm=50, step_mm=1)
    metrics = sk.compute_metrics(poses)
    sk.plot_sweep(metrics, "sweep.png")
"""

from .export import write_metrics_csv, write_poses_json
from .hardpoints import Hardpoints, load_hardpoints, save_hardpoints
from .kinematics import DoubleWishbone, Pose, bump_sweep
from .metrics import compute_metrics, summary
from .plotting import plot_front_view, plot_sweep

__all__ = [
    "DoubleWishbone", "Hardpoints", "Pose", "bump_sweep", "compute_metrics",
    "load_hardpoints", "plot_front_view", "plot_sweep", "save_hardpoints",
    "summary", "write_metrics_csv", "write_poses_json",
]
__version__ = "0.1.0"
