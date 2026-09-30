#!/usr/bin/env python3
# Copyright (c) 2026 Team 6. Licensed under the Apache License, Version 2.0.
"""Render the report figures from a recorded trial session.

    python3 tools/plot_metrics.py trials/20260928-163202 --outdir figures

Reads ``metrics_trials.csv`` (written by ``analyze_trials.py``) and emits vector
PDFs sized for inclusion in LaTeX. Figures carry no titles: the caption names
them, which is the convention in a paper and avoids saying it twice.

Only matplotlib and numpy are needed -- no ROS, no bag -- so this runs anywhere
the CSVs have been copied to.

Figures:

    fig_error_distribution.pdf  every trial's final error, with both tolerances
                                drawn. The argument that the 5 mm threshold sat
                                inside the error distribution, made visible.
    fig_residual_rates.pdf      the forward-kinematics residual against the two
                                publish rates, on a log axis.
    fig_error_vs_distance.pdf   final error against travel distance.
    fig_settle_vs_distance.pdf  settle time against travel distance.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# --- palette ---------------------------------------------------------------
# Light surface only: these are printed. Data marks use categorical slot 1;
# thresholds use status colors, which never carry meaning alone here -- every
# rule is directly labelled. Validated against the light surface
# (CVD dE 23.8, normal-vision dE 31.6, both clear of the floors).
SURFACE = "#fcfcfb"
SERIES_1 = "#2a78d6"  # blue
CRITICAL = "#d03b3b"  # red -- the tolerance that proved meaningless
GOOD = "#0ca30c"  # green -- the tolerance that works
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRIDLINE = "#e1e0d9"
BASELINE = "#c3c2b7"

# The three-run residual comparison spans separate sessions, so it cannot be
# read out of a single metrics CSV. These are the measured maxima; update them
# together with the sessions they came from.
#   20 Hz RSP / 50 Hz sim   session 20260928-162207
#   20 Hz RSP / 200 Hz sim  session 20260928-162706
#   200 Hz RSP / 200 Hz sim session 20260928-163202
RESIDUAL_BY_RATE = [
    ("20 Hz publisher\n50 Hz simulator", 609.67),
    ("20 Hz publisher\n200 Hz simulator", 604.77),
    ("200 Hz publisher\n200 Hz simulator", 0.21),
]


def style() -> None:
    """Apply the chart chrome: thin marks, hairline recessive axes."""
    plt.rcParams.update({
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "font.family": "sans-serif",
        "font.size": 9,
        "axes.labelsize": 9,
        "axes.labelcolor": INK_SECONDARY,
        "axes.edgecolor": BASELINE,
        "axes.linewidth": 0.6,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "xtick.color": INK_MUTED,
        "ytick.color": INK_MUTED,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "grid.color": GRIDLINE,
        "grid.linewidth": 0.6,
        # Solid hairlines only. Dashed grid reads as "threshold" when it is
        # just a grid -- dashing is reserved here for the actual thresholds.
        "grid.linestyle": "-",
        "legend.frameon": False,
        "pdf.fonttype": 42,  # embed as TrueType, not Type 3
    })


def read_trials(path: Path) -> dict[str, np.ndarray]:
    """Load metrics_trials.csv into column arrays."""
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"{path} has no rows")

    def column(name: str) -> np.ndarray:
        return np.array([float(r[name]) if r[name] else math.nan for r in rows])

    return {name: column(name) for name in rows[0]}


def finish(fig, ax, path: Path, xlabel: str, ylabel: str, grid_axis: str) -> None:
    """Label, grid and write one figure."""
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(axis=grid_axis, zorder=0)
    ax.set_axisbelow(True)
    fig.tight_layout(pad=0.4)
    fig.savefig(path)
    plt.close(fig)
    print(f"  wrote {path}")


def plot_error_distribution(
    data: dict[str, np.ndarray], outdir: Path, width: float
) -> None:
    """Every trial's final error, sorted, with both tolerances drawn.

    A dot plot rather than a histogram: with 20 values, binning hides the very
    thing the figure exists to show -- that no gap separates the trials that
    passed at 5 mm from those that failed.
    """
    error_mm = np.sort(data["position_error_m"] * 1e3)
    rank = np.arange(1, len(error_mm) + 1)

    fig, ax = plt.subplots(figsize=(width, width * 0.62))

    for tolerance, color, label in ((5.0, CRITICAL, "5 mm"), (10.0, GOOD, "10 mm")):
        passing = int((error_mm <= tolerance).sum())
        ax.axvline(tolerance, color=color, linewidth=1.0, linestyle=(0, (4, 3)), zorder=2)
        ax.text(
            tolerance,
            len(error_mm) + 0.9,
            f" {label} tolerance\n {passing}/{len(error_mm)} pass",
            color=color,
            fontsize=7.5,
            va="bottom",
            ha="left" if tolerance > 6 else "right",
            linespacing=1.4,
        )

    ax.plot(
        error_mm,
        rank,
        marker="o",
        markersize=4,
        linestyle="none",
        color=SERIES_1,
        markeredgecolor=SURFACE,  # 2px surface ring, for overlapping marks
        markeredgewidth=0.8,
        zorder=3,
    )

    # Room for the right-hand tolerance label to sit outside its rule without
    # running into the axis edge.
    ax.set_xlim(0, 13.5)
    ax.set_ylim(0, len(error_mm) + 4.5)
    finish(
        fig,
        ax,
        outdir / "fig_error_distribution.pdf",
        "Final position error (mm)",
        "Trial, sorted by error",
        "x",
    )


def plot_residual_rates(outdir: Path, width: float) -> None:
    """Residual against publish rate, on a log axis.

    Lollipops rather than bars: the values span four orders of magnitude, and a
    bar on a log axis no longer encodes magnitude by its length. A dot encodes
    position, which a log axis does not break.
    """
    labels = [label for label, _ in RESIDUAL_BY_RATE]
    values = np.array([value for _, value in RESIDUAL_BY_RATE])
    y = np.arange(len(values))[::-1]

    fig, ax = plt.subplots(figsize=(width, width * 0.44))
    ax.hlines(y, 0.05, values, color=SERIES_1, linewidth=1.0, alpha=0.45, zorder=2)
    ax.plot(
        values, y, marker="o", markersize=6, linestyle="none",
        color=SERIES_1, markeredgecolor=SURFACE, markeredgewidth=0.8, zorder=3,
    )

    for value, row in zip(values, y):
        ax.text(
            value * 1.45, row,
            f"{value:g} µm" if value >= 1 else f"{value:.2f} µm",
            va="center", ha="left", fontsize=8, color=INK_PRIMARY,
        )

    ax.set_xscale("log")
    ax.set_xlim(0.05, 4000)
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=8, color=INK_SECONDARY, linespacing=1.3)
    ax.set_ylim(-0.6, len(values) - 0.4)
    finish(
        fig, ax,
        outdir / "fig_residual_rates.pdf",
        "Max forward-kinematics residual (µm, log scale)",
        "",
        "x",
    )


def plot_against_distance(
    data: dict[str, np.ndarray], outdir: Path, width: float
) -> None:
    """Final error and settle time against travel distance."""
    distance_cm = data["distance_m"] * 1e2

    for key, scale, ylabel, filename in (
        ("position_error_m", 1e3, "Final position error (mm)", "fig_error_vs_distance.pdf"),
        ("settle_time_s", 1.0, "Settle time (s)", "fig_settle_vs_distance.pdf"),
    ):
        values = data[key] * scale
        valid = ~np.isnan(values)
        x, y = distance_cm[valid], values[valid]

        fig, ax = plt.subplots(figsize=(width, width * 0.62))
        ax.plot(
            x, y, marker="o", markersize=4.5, linestyle="none",
            color=SERIES_1, markeredgecolor=SURFACE, markeredgewidth=0.8, zorder=3,
        )

        if len(x) > 2:
            # A least-squares line, drawn faintly and annotated with r so the
            # strength of the relationship is stated rather than implied by the
            # slope. r = 0.37 is a tendency, not a law.
            slope, intercept = np.polyfit(x, y, 1)
            span = np.array([x.min(), x.max()])
            ax.plot(span, slope * span + intercept,
                    color=INK_MUTED, linewidth=0.9, zorder=2)
            r = float(np.corrcoef(x, y)[0, 1])
            ax.text(
                0.97, 0.06, f"r = {r:+.2f}", transform=ax.transAxes,
                ha="right", va="bottom", fontsize=8, color=INK_SECONDARY,
            )

        ax.set_ylim(0, max(y.max() * 1.18, 1e-9))
        finish(fig, ax, outdir / filename, "Travel distance (cm)", ylabel, "y")


def main() -> int:
    """Entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("session", type=Path, help="Directory holding metrics_trials.csv")
    parser.add_argument("--outdir", type=Path, default=None)
    parser.add_argument(
        "--width", type=float, default=5.0,
        help="Figure width in inches; 3.4 suits a two-column paper",
    )
    args = parser.parse_args()

    trials_csv = args.session / "metrics_trials.csv"
    if not trials_csv.is_file():
        parser.error(f"{trials_csv} not found -- run analyze_trials.py first")

    outdir = args.outdir or (args.session / "figures")
    outdir.mkdir(parents=True, exist_ok=True)

    style()
    data = read_trials(trials_csv)
    print(f"Read {len(data['index'])} trials from {trials_csv}")
    plot_error_distribution(data, outdir, args.width)
    plot_residual_rates(outdir, args.width)
    plot_against_distance(data, outdir, args.width)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())