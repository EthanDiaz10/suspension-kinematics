"""Standard plots of a bump sweep (matplotlib)."""

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
    ("motion_ratio", "Motion ratio", "spring / wheel"),
    ("caster_deg", "Caster", "deg"),
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
    """Six small panels, one metric each, all against wheel travel."""
    t = metrics["travel_mm"]
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), facecolor=SURFACE)
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
    """Same six panels as ``plot_sweep``, with one line per design.

    ``runs`` maps a label (e.g. "baseline") to that design's metrics.
    Up to two designs keep the colours distinct and colour-blind safe.
    """
    if len(runs) > len(SERIES):
        raise ValueError(f"plot_comparison supports up to {len(SERIES)} designs")
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), facecolor=SURFACE)
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
                ("RTI", "RTO"), ("UBJ", "LBJ"), ("SHC", "SHU")]
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
