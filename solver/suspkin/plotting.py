"""Standard plots of the bump, steering and roll sweeps (matplotlib)."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # draw to files; works without a display
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# A small, colour-blind-safe palette. Series 1 is used when a panel has one line.
SERIES = ("#2a78d6", "#eb6834")
INK, INK_MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"

# (column, panel title, y-axis label)
PANELS = (
    ("camber_deg", "Camber", "deg  (− = top in)"),
    ("toe_deg", "Toe / bump steer", "deg  (+ = toe-in)"),
    ("roll_center_height_mm", "Roll centre height above ground", "mm"),
    ("roll_center_shift_mm", "Roll centre movement vs chassis", "mm  (+ = up)"),
    ("motion_ratio", "Motion ratio", "damper / wheel"),
    ("caster_deg", "Caster", "deg"),
    ("anti_brake_pct", "Anti-dive (front) / anti-lift (rear)", "%"),
    ("spring_length_mm", "Damper length", "mm"),
    ("mech_trail_mm", "Mechanical (caster) trail", "mm"),
)


def _style(ax, title, ylabel, zero_line=True):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=11, color=INK, fontweight="bold")
    ax.set_xlabel("Wheel travel (mm, + = bump)", fontsize=9, color=INK_MUTED)
    ax.set_ylabel(ylabel, fontsize=9, color=INK_MUTED)
    ax.grid(True, color=GRID, linewidth=0.8)
    if zero_line:  # marks static ride height on the travel axis
        ax.axvline(0, color=INK_MUTED, linewidth=0.8, linestyle=":")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_MUTED, labelsize=8)


def plot_sweep(metrics: dict[str, np.ndarray], path: str | Path | None = None,
               title: str = "Bump sweep") -> plt.Figure:
    """Nine small panels, one metric each, all against wheel travel."""
    t = metrics["travel_mm"]
    fig, axes = plt.subplots(3, 3, figsize=(13, 10), facecolor=SURFACE)
    for ax, (col, name, ylabel) in zip(axes.flat, PANELS):
        y = metrics[col]
        ax.plot(t, y, color=SERIES[0], linewidth=2)
        _style(ax, name, ylabel)
        # Avoid a misleading zoomed-in axis when a curve is almost flat.
        span = np.nanmax(y) - np.nanmin(y)
        if span < 0.05:
            mid = np.nanmean(y)
            ax.set_ylim(mid - 0.05, mid + 0.05)
    fig.suptitle(title, x=0.01, ha="left", fontsize=13, color=INK, fontweight="bold")
    fig.tight_layout()
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=SURFACE)
    return fig


def plot_comparison(runs: dict[str, dict[str, np.ndarray]], path: str | Path | None = None,
                    title: str = "Before vs after") -> plt.Figure:
    """Same nine panels as ``plot_sweep``, with one line per design.

    ``runs`` maps a label (e.g. "baseline") to that design's metrics.
    Up to two designs keep the colours distinct and colour-blind safe.
    """
    if len(runs) > len(SERIES):
        raise ValueError(f"plot_comparison supports up to {len(SERIES)} designs")
    fig, axes = plt.subplots(3, 3, figsize=(13, 10), facecolor=SURFACE)
    for ax, (col, name, ylabel) in zip(axes.flat, PANELS):
        for (label, m), color in zip(runs.items(), SERIES):
            ax.plot(m["travel_mm"], m[col], color=color, linewidth=2, label=label)
        _style(ax, name, ylabel)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right", ncol=len(runs), frameon=False,
               fontsize=10, labelcolor=INK_MUTED)
    fig.suptitle(title, x=0.01, ha="left", fontsize=13, color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=SURFACE)
    return fig


def _tyre_outline_yz(pose, radius: float, width: float) -> np.ndarray:
    """Front-view outline of the tyre: a rectangle in the wheel plane.

    ``axis`` is the spin axis; ``up`` is the in-plane direction closest to
    vertical. The four corners are WC +/- radius*up +/- (width/2)*axis.
    """
    wc, axis = pose.points["WC"], pose.wheel_axis
    up = np.array([0.0, 0.0, 1.0]) - axis[2] * axis
    up /= np.linalg.norm(up)
    corners = [wc + sr * radius * up + sw * width / 2 * axis
               for sr, sw in ((1, -1), (1, 1), (-1, 1), (-1, -1), (1, -1))]
    return np.array(corners)[:, 1:]


def plot_front_view(poses, path: str | Path | None = None,
                    title: str = "Front view (looking rearward)",
                    tyre_width: float = 180.0) -> plt.Figure:
    """Stick-figure front view at droop, static and bump, with the tyre outline."""
    picks = [poses[0], min(poses, key=lambda p: abs(p.travel)), poses[-1]]
    styles = [(":", 1.2), ("-", 2.0), ("--", 1.2)]
    fig, ax = plt.subplots(figsize=(7, 5), facecolor=SURFACE)
    for pose, (ls, lw) in zip(picks, styles):
        P = pose.points
        yz = lambda k: (P[k][1], P[k][2])  # noqa: E731
        segs = [("UAF", "UBJ"), ("UAA", "UBJ"), ("LAF", "LBJ"), ("LAA", "LBJ"),
                ("RTI", "RTO"), ("UBJ", "LBJ")]
        if "SHR" in P:   # pushrod, rocker triangle and damper
            segs += [("PRO", "PRI"), ("RKP", "PRI"), ("RKP", "SHR"), ("PRI", "SHR"),
                     ("SHR", "SHC")]
        else:            # old direct-acting spring/damper
            segs += [("SHC", "SHU")]
        for i, (a, b) in enumerate(segs):
            (y1, z1), (y2, z2) = yz(a), yz(b)
            ax.plot([y1, y2], [z1, z2], color=SERIES[0], linestyle=ls, linewidth=lw,
                    label=f"{pose.travel:+.0f} mm" if i == 0 else None)
        radius = np.linalg.norm(P["WC"] - P["CP"])
        outline = _tyre_outline_yz(pose, radius, tyre_width)
        ax.plot(outline[:, 0], outline[:, 1], color=INK_MUTED, linestyle=ls, linewidth=1)
    _style(ax, title, "Z (mm)", zero_line=False)
    ax.set_xlabel("Y (mm, + = outboard / left)", fontsize=9, color=INK_MUTED)
    ax.set_aspect("equal")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)
    fig.tight_layout()
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=SURFACE)
    return fig


def plot_steering(steer: dict[str, np.ndarray], path: str | Path | None = None,
                  title: str = "Steering sweep") -> plt.Figure:
    """Three panels from ``steering_metrics``: steer angles, inner vs outer
    against ideal Ackermann, and Ackermann percentage."""
    r = steer["rack_mm"]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.4), facecolor=SURFACE)

    ax = axes[0]
    ax.plot(r, steer["steer_left_deg"], color=SERIES[0], linewidth=2, label="left wheel")
    ax.plot(r, steer["steer_right_deg"], color=SERIES[1], linewidth=2, label="right wheel")
    _style(ax, "Steer angle", "deg  (+ = pointing left)")
    ax.set_xlabel("Rack travel (mm, + = steer left)", fontsize=9, color=INK_MUTED)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)

    left_turn = r > 0  # by symmetry the right turn is the same
    ax = axes[1]
    ax.plot(steer["outer_deg"][left_turn], steer["inner_deg"][left_turn], color=SERIES[0],
            linewidth=2, label="this design")
    ax.plot(steer["outer_deg"][left_turn], steer["ideal_inner_deg"][left_turn], color=INK_MUTED,
            linewidth=1.2, linestyle="--", label="100% Ackermann")
    ax.plot(steer["outer_deg"][left_turn], steer["outer_deg"][left_turn], color=INK_MUTED,
            linewidth=1.0, linestyle=":", label="parallel steer")
    _style(ax, "Inner vs outer wheel", "inner wheel angle (deg)", zero_line=False)
    ax.set_xlabel("Outer wheel angle (deg)", fontsize=9, color=INK_MUTED)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)

    ax = axes[2]
    ax.plot(steer["outer_deg"][left_turn], steer["ackermann_pct"][left_turn], color=SERIES[0], linewidth=2)
    ax.axhline(0, color=INK_MUTED, linewidth=0.8, linestyle=":")
    _style(ax, "Ackermann", "%  (100 = ideal, 0 = parallel)", zero_line=False)
    ax.set_xlabel("Outer wheel angle (deg)", fontsize=9, color=INK_MUTED)

    fig.suptitle(title, x=0.01, ha="left", fontsize=13, color=INK, fontweight="bold")
    fig.tight_layout()
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=SURFACE)
    return fig


ROLL_XLABEL = "Body roll (deg, + = leans right, as in a left turn)"


def plot_roll(roll: dict[str, np.ndarray], path: str | Path | None = None,
              title: str = "Roll sweep") -> plt.Figure:
    """Six panels from ``roll_metrics``, all against body roll angle.

    At positive roll the right wheel is the outside wheel (in bump).
    """
    r = roll["roll_deg"]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8.2), facecolor=SURFACE)

    def two(ax, left_key, right_key, name, ylabel):
        """One panel with the left (inside at + roll) and right (outside) wheel."""
        ax.plot(r, roll[left_key], color=SERIES[0], linewidth=2, label="left wheel")
        ax.plot(r, roll[right_key], color=SERIES[1], linewidth=2, label="right wheel")
        _style(ax, name, ylabel)
        ax.set_xlabel(ROLL_XLABEL, fontsize=9, color=INK_MUTED)

    two(axes[0, 0], "camber_left_deg", "camber_right_deg",
        "Camber relative to the road", "deg  (− = top leans in)")
    axes[0, 0].legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)
    two(axes[0, 1], "steer_left_deg", "steer_right_deg", "Roll steer", "deg  (+ = pointing left)")
    # Roll steer is usually tiny: don't let matplotlib zoom in on noise.
    lo, hi = axes[0, 1].get_ylim()
    if hi - lo < 0.1:
        mid = (hi + lo) / 2
        axes[0, 1].set_ylim(mid - 0.05, mid + 0.05)
    two(axes[1, 0], "travel_left_mm", "travel_right_mm", "Wheel travel vs chassis", "mm  (+ = bump)")
    two(axes[1, 1], "damper_left_mm", "damper_right_mm", "Damper compression", "mm  (+ = shorter)")

    ax = axes[0, 2]
    ax.plot(r, roll["rc_height_mm"], color=SERIES[0], linewidth=2, label="height above road")
    ax.plot(r, roll["rc_lateral_mm"], color=SERIES[1], linewidth=2, label="sideways (+ = left)")
    _style(ax, "Roll centre", "mm")
    ax.set_xlabel(ROLL_XLABEL, fontsize=9, color=INK_MUTED)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)

    ax = axes[1, 2]
    ax.plot(r, roll["spring_roll_moment_nm"], color=SERIES[0], linewidth=2, label="springs")
    if np.any(roll["bar_roll_moment_nm"] != 0):  # only show the bar when there is one
        ax.plot(r, roll["bar_roll_moment_nm"], color=SERIES[1], linewidth=2, label="anti-roll bar")
        ax.plot(r, roll["total_roll_moment_nm"], color=INK_MUTED, linewidth=1.5,
                linestyle="--", label="total")
        ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)
    _style(ax, "Roll moment pushing back", "N·m")
    ax.set_xlabel(ROLL_XLABEL, fontsize=9, color=INK_MUTED)

    fig.suptitle(title, x=0.01, ha="left", fontsize=13, color=INK, fontweight="bold")
    fig.tight_layout()
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=SURFACE)
    return fig


def plot_roll_front_view(rp, path: str | Path | None = None, title: str | None = None,
                         tyre_width: float = 180.0, arb_mount: str = "rocker") -> plt.Figure:
    """Front view of the whole axle at one roll angle, as seen from the road.

    ``rp`` is a ``RollPose`` (see ``roll.py``). Draws both corners, the tyres,
    the body centreline, and the roll-centre construction: a line from each
    contact patch through its instant centre, crossing at the roll centre.
    ``arb_mount`` is what the bar's drop link hangs off (only used to draw it).
    """
    from .roll import mirror_point, roll_center_chassis  # avoid an import loop

    title = title or f"Front view at {rp.roll_deg:+.1f}° roll (looking rearward)"
    fig, ax = plt.subplots(figsize=(10, 5.2), facecolor=SURFACE)
    segs = [("UAF", "UBJ"), ("UAA", "UBJ"), ("LAF", "LBJ"), ("LAA", "LBJ"),
            ("RTI", "RTO"), ("UBJ", "LBJ")]
    if "SHR" in rp.left.points:
        segs += [("PRO", "PRI"), ("RKP", "PRI"), ("RKP", "SHR"), ("PRI", "SHR"), ("SHR", "SHC")]
    if "ARA" in rp.left.points:  # anti-roll bar drop link and arm
        segs += [("ARL", "ARA"), ("ARA", "ARP")]
        if arb_mount == "rocker" and "RKP" in rp.left.points:
            segs += [("RKP", "ARL")]  # the tab on the rocker the drop link bolts to
    sides = (("left", rp.left.points, rp.left, SERIES[0]),
             ("right", rp.right_points(), rp.right_src, SERIES[1]))
    for name, pts, src_pose, color in sides:
        g = {k: rp.to_ground(v) for k, v in pts.items()}
        for i, (a, b) in enumerate(segs):
            ax.plot([g[a][1], g[b][1]], [g[a][2], g[b][2]], color=color, linewidth=1.8,
                    label=f"{name} corner" if i == 0 else None)
        # Tyre outline: a rectangle in the wheel plane, as in plot_front_view.
        axis = src_pose.wheel_axis if name == "left" else mirror_point(src_pose.wheel_axis)
        axis_g = rp.to_ground(axis) - rp.to_ground(np.zeros(3))
        up = np.array([0.0, 0.0, 1.0]) - axis_g[2] * axis_g
        up /= np.linalg.norm(up)
        radius = np.linalg.norm(g["WC"] - g["CP"])
        corners = np.array([g["WC"] + sr * radius * up + sw * tyre_width / 2 * axis_g
                            for sr, sw in ((1, -1), (1, 1), (-1, 1), (-1, -1), (1, -1))])
        ax.plot(corners[:, 1], corners[:, 2], color=INK_MUTED, linewidth=1)

    if "ARP" in rp.left.points:  # the bar itself, straight across the car
        a, b = rp.to_ground(rp.left.points["ARP"]), rp.to_ground(rp.right_points()["ARP"])
        ax.plot([a[1], b[1]], [a[2], b[2]], color=INK_MUTED, linewidth=2.5, label="anti-roll bar")

    # Body centreline (the chassis's own vertical axis), to show the lean.
    top, bottom = rp.to_ground(np.array([0.0, 0.0, 450.0])), rp.to_ground(np.array([0.0, 0.0, 0.0]))
    ax.plot([bottom[1], top[1]], [bottom[2], top[2]], color=INK_MUTED, linestyle="--",
            linewidth=1, label="body centreline")
    ax.axhline(0.0, color=INK, linewidth=1.2)  # the road

    rc = roll_center_chassis(rp)
    if rc is not None:
        rc_g = rp.to_ground(np.array([0.0, rc[0], rc[1]]))
        for pts in (rp.left.points, rp.right_points()):
            cp = rp.to_ground(pts["CP"])
            ax.plot([cp[1], rc_g[1]], [cp[2], rc_g[2]], color=INK_MUTED, linestyle=":", linewidth=1)
        ax.plot(rc_g[1], rc_g[2], marker="o", color=INK, markersize=6, linestyle="none",
                label=f"roll centre ({rc_g[1]:+.0f}, {rc_g[2]:.0f}) mm")

    _style(ax, title, "Z above road (mm)", zero_line=False)
    ax.set_xlabel("Y (mm, + = left)", fontsize=9, color=INK_MUTED)
    ax.set_aspect("equal")
    # Legend under the plot so it never covers the rocker or the tyres.
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_MUTED, loc="upper center",
              bbox_to_anchor=(0.5, -0.16), ncol=5)
    fig.tight_layout()
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=SURFACE)
    return fig


def plot_full_car(result, path: str | Path | None = None, title: str | None = None) -> plt.Figure:
    """Three panels from ``fullcar.full_car``: camber vs g, roll stiffness, inside tyre loads.

    ``result`` is a ``fullcar.CarResult``.
    """
    from .fullcar import camber_vs_g  # here, so plotting doesn't import the solver at load time

    n = result.numbers
    car = result.car
    curves = camber_vs_g(result)
    g = curves["lateral_g"]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), facecolor=SURFACE)

    # 1. Outside-tyre camber to the road as the cornering load builds up.
    ax = axes[0]
    for colour, axle in zip(SERIES, ("front", "rear")):
        ax.plot(g, curves[f"{axle}_outer_camber_deg"], color=colour, linewidth=2,
                label=f"{axle} outside tyre")
    ax.axhline(car.target_outside_camber_deg, color=INK_MUTED, linewidth=1, linestyle="--",
               label=f"target {car.target_outside_camber_deg:+g}°")
    _style(ax, "Outside-tyre camber vs cornering load", "deg to the road  (− = top leans in)",
           zero_line=False)
    ax.axvline(car.design_lateral_g, color=INK_MUTED, linewidth=0.8, linestyle=":")
    ax.set_xlabel(f"Lateral acceleration (g)   dotted line = design load {car.design_lateral_g:g} g",
                  fontsize=9, color=INK_MUTED)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)

    # 2. Roll stiffness of each axle, split into springs and anti-roll bar.
    ax = axes[1]
    labels = ("front", "rear")
    springs = [n[f"{a}_roll_stiffness_springs_nm_per_deg"] for a in labels]
    bars = [n[f"{a}_roll_stiffness_bar_nm_per_deg"] for a in labels]
    y = np.arange(2)
    ax.barh(y, springs, height=0.5, color=SERIES[0], edgecolor=SURFACE, linewidth=2, label="springs")
    ax.barh(y, bars, left=springs, height=0.5, color=SERIES[1], edgecolor=SURFACE, linewidth=2,
            label="anti-roll bar")
    for yi, s, b in zip(y, springs, bars):
        ax.text(s + b + 8, yi, f"{s + b:.0f} N·m/°", va="center", fontsize=9, color=INK)
    ax.set_yticks(y, labels)
    ax.set_ylim(2.0, -0.5)  # front on top, with room for the legend below the bars
    ax.set_axisbelow(True)  # grid behind the bars
    _style(ax, "Roll stiffness", "", zero_line=False)
    ax.set_xlabel("N·m per degree of body roll", fontsize=9, color=INK_MUTED)
    ax.set_xlim(0, max(s + b for s, b in zip(springs, bars)) * 1.3)
    ax.grid(True, axis="x", color=GRID, linewidth=0.8)
    ax.grid(False, axis="y")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED, loc="lower center", ncol=2)

    # 3. Inside tyre load: it falls as load moves to the outside. Zero = lift-off.
    ax = axes[2]
    for colour, axle in zip(SERIES, labels):
        load = n[f"{axle}_static_wheel_load_n"] - n[f"{axle}_load_transfer_n_per_g"] * g
        ax.plot(g, load, color=colour, linewidth=2, label=f"{axle} inside tyre")
    ax.axhline(0, color=INK_MUTED, linewidth=0.8)
    _style(ax, "Inside-tyre load (0 = wheel lifts)", "N", zero_line=False)
    ax.axvline(car.design_lateral_g, color=INK_MUTED, linewidth=0.8, linestyle=":")
    ax.set_xlabel("Lateral acceleration (g)", fontsize=9, color=INK_MUTED)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)

    headline = (f"front share of load transfer {n['front_load_transfer_share_pct']:.1f}% "
                f"(front weight {n['front_weight_pct']:.0f}%)   ·   "
                f"roll {n['roll_gradient_deg_per_g']:.2f}°/g   ·   "
                f"roll centres F {n['front_rc_height_mm']:.0f} mm / R {n['rear_rc_height_mm']:.0f} mm")
    fig.suptitle(title or "Whole car in steady cornering", x=0.01, ha="left", fontsize=13,
                 color=INK, fontweight="bold")
    fig.text(0.01, 0.905, headline, fontsize=10, color=INK_MUTED)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=SURFACE)
    return fig


def plot_loads(result, path: str | Path | None = None, title: str | None = None) -> plt.Figure:
    """Link forces in every load case, and each member's safety factor.

    ``result`` is a ``loads.LoadsResult``. Left: one grid per corner, a row per
    link and a column per load case, coloured orange for tension (pull) and
    blue for compression (push), with the force in kN written in each cell.
    Right: the lowest safety factor of each member group on each axle, against
    the required one.
    """
    from matplotlib.colors import LinearSegmentedColormap

    from .loads import GROUP_LABELS, MEMBER_GROUPS

    # Diverging: blue = compression, neutral grey at zero, orange = tension.
    cmap = LinearSegmentedColormap.from_list("push_pull", [SERIES[0], "#efeeea", SERIES[1]])
    corners = list(result.corners.values())
    vmax = max(abs(x) for c in corners for f in c.forces for x in f.members.values()) / 1000.0

    fig = plt.figure(figsize=(16, 7.2), facecolor=SURFACE)
    grid = fig.add_gridspec(2, 2, width_ratios=(2.3, 1.0), hspace=0.55, wspace=0.28)
    names = {"upper_front": "upper A-arm, front leg", "upper_rear": "upper A-arm, rear leg",
             "lower_front": "lower A-arm, front leg", "lower_rear": "lower A-arm, rear leg",
             "tie_rod": "tie rod", "pushrod": "pushrod"}
    for row, c in enumerate(corners):
        ax = fig.add_subplot(grid[row, 0])
        table = c.member_table()
        data = np.array([table[m] for m in MEMBER_GROUPS]) / 1000.0  # kN
        ax.imshow(data, cmap=cmap, vmin=-vmax, vmax=vmax, aspect="auto")
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                ax.text(j, i, f"{data[i, j]:+.1f}", ha="center", va="center", fontsize=8, color=INK)
        labels = [names[m] if not (m == "tie_rod" and c.axle == "rear") else "toe link"
                  for m in MEMBER_GROUPS]
        ax.set_yticks(range(len(labels)), labels, fontsize=8, color=INK_MUTED)
        ax.set_xticks(range(len(c.labels)), [lb.replace(" (", "\n(") for lb in c.labels],
                      fontsize=8, color=INK_MUTED)
        ax.tick_params(length=0)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.set_title(f"{c.axle.capitalize()}-left corner: link force, kN  "
                     "(+ orange = tension, − blue = compression)",
                     loc="left", fontsize=11, color=INK, fontweight="bold")

    # Safety factors: lowest of tension, buckling and rod end per group and axle.
    ax = fig.add_subplot(grid[:, 1])
    groups = list(GROUP_LABELS)
    y = np.arange(len(groups))
    height = 0.36
    top = 0.0
    for k, (axle, colour) in enumerate(zip(("front", "rear"), SERIES)):
        sfs = [min(m.sf for m in result.checks if m.axle == axle and m.group == g) for g in groups]
        top = max(top, max(sfs))
        ax.barh(y + (k - 0.5) * height, sfs, height=height, color=colour, edgecolor=SURFACE,
                linewidth=2, label=axle)
        for yi, s in zip(y, sfs):
            ax.text(s + 0.1, yi + (k - 0.5) * height, f"{s:.1f}", va="center", fontsize=8, color=INK)
    req = result.config.required_sf
    ax.axvline(req, color=INK_MUTED, linewidth=1, linestyle="--")
    ax.text(req + 0.1, len(groups) - 0.45, f"required {req:g}", fontsize=8, color=INK_MUTED)
    ax.set_yticks(y, [GROUP_LABELS[g] for g in groups])
    ax.set_ylim(len(groups) - 0.3, -0.7)
    ax.set_xlim(0, top * 1.18)
    ax.set_axisbelow(True)
    _style(ax, "Lowest safety factor", "", zero_line=False)
    ax.set_xlabel("capacity / worst load (tension, buckling or rod end)", fontsize=9, color=INK_MUTED)
    ax.grid(True, axis="x", color=GRID, linewidth=0.8)
    ax.grid(False, axis="y")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED, loc="lower right")

    fig.suptitle(title or "Suspension link forces and safety factors", x=0.01, ha="left",
                 fontsize=13, color=INK, fontweight="bold")
    fig.subplots_adjust(left=0.13, right=0.98, top=0.9, bottom=0.08)
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=SURFACE)
    return fig
