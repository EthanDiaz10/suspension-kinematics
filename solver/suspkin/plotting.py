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
    ax.plot(r, roll["spring_roll_moment_nm"], color=SERIES[0], linewidth=2)
    _style(ax, "Roll moment the springs push back with", "N·m  (springs only, no anti-roll bar)")
    ax.set_xlabel(ROLL_XLABEL, fontsize=9, color=INK_MUTED)

    fig.suptitle(title, x=0.01, ha="left", fontsize=13, color=INK, fontweight="bold")
    fig.tight_layout()
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=SURFACE)
    return fig


def plot_roll_front_view(rp, path: str | Path | None = None, title: str | None = None,
                         tyre_width: float = 180.0) -> plt.Figure:
    """Front view of the whole axle at one roll angle, as seen from the road.

    ``rp`` is a ``RollPose`` (see ``roll.py``). Draws both corners, the tyres,
    the body centreline, and the roll-centre construction: a line from each
    contact patch through its instant centre, crossing at the roll centre.
    """
    from .roll import mirror_point, roll_center_chassis  # avoid an import loop

    title = title or f"Front view at {rp.roll_deg:+.1f}° roll (looking rearward)"
    fig, ax = plt.subplots(figsize=(10, 5.2), facecolor=SURFACE)
    segs = [("UAF", "UBJ"), ("UAA", "UBJ"), ("LAF", "LBJ"), ("LAA", "LBJ"),
            ("RTI", "RTO"), ("UBJ", "LBJ")]
    if "SHR" in rp.left.points:
        segs += [("PRO", "PRI"), ("RKP", "PRI"), ("RKP", "SHR"), ("PRI", "SHR"), ("SHR", "SHC")]
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
              bbox_to_anchor=(0.5, -0.16), ncol=4)
    fig.tight_layout()
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=SURFACE)
    return fig
