#!/usr/bin/env python3
"""Plot the evaluator's runs on the saved map.

Figures:
    scenarios  the map with the spawn and the S1-S5 goals (and, with --config, the planned routes)
    compare    2 x 3 grid, one panel per scenario: config A (before) vs config B (after);
               planned path as a pale band, driven path as a line, each panel zoomed to its route
    speed      as compare, with config B's driven path coloured by speed

Usage (from ~/ee3305):
    python3 src/ee3305_bringup/scripts/plot_runs.py scenarios --config base
    python3 src/ee3305_bringup/scripts/plot_runs.py compare base astar
    python3 src/ee3305_bringup/scripts/plot_runs.py speed base rpp
Options: --labels A B, --run N, --all-runs, --logs DIR, -o FILE
"""

import argparse
import os
import sys

sys.dont_write_bytecode = True  # no __pycache__ in src/ when importing analyze_runs

import matplotlib
matplotlib.use("Agg")  # render to files without a display
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.collections import LineCollection
from matplotlib.colors import BoundaryNorm, LinearSegmentedColormap, ListedColormap, Normalize
from matplotlib.lines import Line2D

# Shared loaders, so both scripts read runs and the map the same way
from analyze_runs import DEFAULT_MAP, SCRIPT_DIR, load_csv, load_map, load_result, split_paths

DEFAULT_SCENARIOS = os.path.join(SCRIPT_DIR, "..", "params", "scenarios.yaml")
SCENARIO_NAMES = ["S1", "S2", "S3", "S4", "S5"]

# Colours (fixed per role in every figure)
BEFORE = "#eb6834"   # config A
AFTER = "#2a78d6"    # config B
INK = "#0b0b0b"      # text, markers
INK_2 = "#52514e"    # secondary text, axes
# Map shading: walls, unknown, free
MAP_COLOURS = ListedColormap(["#4a4946", "#e3e2de", "#fcfcfb"])
MAP_NORM = BoundaryNorm([0, 100, 230, 256], MAP_COLOURS.N)  # pixel 0 = wall, 205 = unknown, 254 = free
# Planned path band
PLAN_WIDTH, PLAN_ALPHA = 5.0, 0.3
# Speed ramp: light (slow) to dark (fast)
SPEED_CMAP = LinearSegmentedColormap.from_list("speed", ["#86b6ef", "#3987e5", "#1c5cab", "#0d366b"])

plt.rcParams.update({"font.size": 8, "axes.edgecolor": INK_2, "axes.labelcolor": INK_2,
                     "xtick.color": INK_2, "ytick.color": INK_2, "axes.titlesize": 9})


# --------------------------------------------------------------------- loading

def load_run(logs_dir, config, scenario, run):
    """Load one finished run folder, or None."""
    folder = os.path.join(logs_dir, f"{config}_{scenario}_{run}")
    if not os.path.exists(os.path.join(folder, "result.csv")):
        return None
    data = {"result": load_result(os.path.join(folder, "result.csv"))}
    for topic in ["odom", "path", "goal"]:
        data[topic] = load_csv(os.path.join(folder, f"{topic}.csv"))
    return data


def runs_to_draw(logs_dir, config, scenario, run, all_runs):
    """Run N of a config and scenario, or every repeat with --all-runs."""
    if not all_runs:
        found = load_run(logs_dir, config, scenario, run)
        return [found] if found else []
    prefix = f"{config}_{scenario}_"
    numbers = sorted(int(n[len(prefix):]) for n in os.listdir(logs_dir)
                     if n.startswith(prefix) and n[len(prefix):].isdigit())
    return [r for r in (load_run(logs_dir, config, scenario, k) for k in numbers) if r]


def driven(run):
    """Driven trajectory from the moment the goal was sent: (x, y, speed)."""
    odom, goal = run["odom"], run["goal"]
    t_goal = goal["t"][0] if len(goal["t"]) else odom["t"][0]
    keep = odom["t"] >= t_goal
    return odom["x"][keep], odom["y"][keep], np.abs(odom["v"][keep])


def first_plan(run):
    """First published path as (x, y), or None."""
    paths = split_paths(run["path"])
    return (paths[0][1], paths[0][2]) if paths else None


# --------------------------------------------------------------------- drawing

def draw_map(ax, image, meta):
    """Draw the map image in world metres (extent = its edges from origin and resolution)."""
    res, (ox, oy) = meta["resolution"], meta["origin"][:2]
    rows, cols = image.shape
    ax.imshow(image, cmap=MAP_COLOURS, norm=MAP_NORM, extent=[ox, ox + cols * res, oy, oy + rows * res],
              interpolation="nearest")


def zoom_to(ax, xs, ys, margin=0.4, min_span=1.6):
    """Zoom onto the given points plus a margin, equal scale on both axes."""
    xs, ys = np.concatenate(xs), np.concatenate(ys)
    cx, cy = (xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2
    half_x = max(xs.max() - xs.min(), min_span) / 2 + margin
    half_y = max(ys.max() - ys.min(), min_span) / 2 + margin
    ax.set_xlim(cx - half_x, cx + half_x)
    ax.set_ylim(cy - half_y, cy + half_y)
    ax.set_aspect("equal", adjustable="datalim")  # widen limits rather than shrink the panel


def mark_start_goal(ax, spawn, goal):
    """Spawn as a circle, goal as a star."""
    ax.plot(spawn[0], spawn[1], "o", ms=6, mfc="white", mec=INK, mew=1.2, zorder=5)
    ax.plot(goal[0], goal[1], "*", ms=10, mfc=INK, mec="white", mew=0.6, zorder=5)


def speed_line(ax, x, y, v, vmax):
    """Trajectory as segments coloured by the speed at each segment's start."""
    points = np.column_stack([x, y]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lines = LineCollection(segments, cmap=SPEED_CMAP, norm=Normalize(0, vmax), linewidth=2.0,
                           capstyle="round", zorder=4)
    lines.set_array(v[:-1])
    ax.add_collection(lines)
    return lines


def grid_figure():
    """2 x 3 grid at report width: five scenario panels and a legend panel."""
    fig, axes = plt.subplots(2, 3, figsize=(7.0, 5.2), constrained_layout=True)
    return fig, axes.ravel()


def plot_compare(args, image, meta, scenarios, speed_mode):
    """compare / speed figure: config A vs config B on each scenario."""
    config_a, config_b = args.configs
    label_a, label_b = args.labels or (config_a, config_b)
    fig, axes = grid_figure()
    spawn = (scenarios["spawn"]["x"], scenarios["spawn"]["y"])
    vmax = 0.22  # just above max_lin_vel
    lines = None
    # Fainter, thinner lines when drawing every repeat
    alpha, width = (0.55, 1.0) if args.all_runs else (1.0, 1.5)

    # One panel per scenario
    for ax, name in zip(axes, SCENARIO_NAMES):
        goal = scenarios["scenarios"][name]["goal"]
        draw_map(ax, image, meta)
        xs, ys = [np.array([spawn[0], goal[0]])], [np.array([spawn[1], goal[1]])]
        missing = []
        for config, colour in [(config_a, BEFORE), (config_b, AFTER)]:
            runs = runs_to_draw(args.logs, config, name, args.run, args.all_runs)
            if not runs:
                missing.append(config)
            for run in runs:
                x, y, v = driven(run)
                plan = first_plan(run)
                # Planned path: pale band underneath
                if plan is not None and not speed_mode:
                    ax.plot(plan[0], plan[1], "-", color=colour, lw=PLAN_WIDTH, alpha=PLAN_ALPHA,
                            zorder=3, solid_capstyle="round")
                    xs.append(plan[0]); ys.append(plan[1])
                # Driven path: line on top (speed-coloured for config B in speed mode)
                if speed_mode and config == config_b:
                    lines = speed_line(ax, x, y, v, vmax)
                else:
                    ax.plot(x, y, "-", color=colour, lw=1.0 if speed_mode else width, alpha=alpha,
                            zorder=6 if speed_mode else 4, solid_capstyle="round")
                xs.append(x); ys.append(y)
        mark_start_goal(ax, spawn, goal)
        zoom_to(ax, xs, ys)
        ax.set_title(f"({'abcde'[SCENARIO_NAMES.index(name)]}) {name}", color=INK, loc="left")
        if missing:
            ax.text(0.03, 0.04, "no run: " + ", ".join(missing), transform=ax.transAxes,
                    color=INK_2, fontsize=7, bbox=dict(fc="white", ec="none", alpha=0.8))

    # Sixth panel: legend (and speed scale)
    legend_ax = axes[5]
    legend_ax.axis("off")
    if speed_mode:
        handles = [Line2D([], [], color=BEFORE, lw=1.0, label=f"{label_a}: driven"),
                   Line2D([], [], color=SPEED_CMAP(0.7), lw=2.0, label=f"{label_b}: driven (colour = speed)")]
    else:
        handles = [Line2D([], [], color=BEFORE, lw=1.5, label=f"{label_a}: driven"),
                   Line2D([], [], color=BEFORE, lw=PLAN_WIDTH, alpha=PLAN_ALPHA, label=f"{label_a}: planned"),
                   Line2D([], [], color=AFTER, lw=1.5, label=f"{label_b}: driven"),
                   Line2D([], [], color=AFTER, lw=PLAN_WIDTH, alpha=PLAN_ALPHA, label=f"{label_b}: planned")]
    handles += [Line2D([], [], marker="o", ls="", ms=6, mfc="white", mec=INK, label="Spawn (start)"),
                Line2D([], [], marker="*", ls="", ms=10, mfc=INK, mec="white", label="Goal")]
    legend_ax.legend(handles=handles, loc="upper left", frameon=False, labelcolor=INK)
    if speed_mode and lines is not None:
        bar = fig.colorbar(lines, ax=legend_ax, orientation="horizontal", fraction=0.12, pad=0.02, aspect=18)
        bar.set_label("Speed (m/s)", color=INK_2)
    for ax in axes[:5]:
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
    return fig


def plot_scenarios(args, image, meta, scenarios):
    """Overview figure: whole map, spawn, labelled goals, optional planned routes."""
    fig, ax = plt.subplots(figsize=(7.0, 5.2), constrained_layout=True)
    draw_map(ax, image, meta)
    spawn = (scenarios["spawn"]["x"], scenarios["spawn"]["y"])
    for name in SCENARIO_NAMES:
        goal = scenarios["scenarios"][name]["goal"]
        if args.config:
            run = load_run(args.logs, args.config, name, args.run)
            plan = first_plan(run) if run else None
            if plan is not None:
                ax.plot(plan[0], plan[1], "-", color=AFTER, lw=1.5, zorder=3, solid_capstyle="round")
        ax.plot(goal[0], goal[1], "*", ms=11, mfc=INK, mec="white", mew=0.6, zorder=5)
        ax.annotate(name, goal, xytext=(6, 4), textcoords="offset points", color=INK, fontsize=9,
                    fontweight="bold", bbox=dict(fc="white", ec="none", alpha=0.8, pad=1))
    ax.plot(spawn[0], spawn[1], "o", ms=7, mfc="white", mec=INK, mew=1.3, zorder=5)
    ax.annotate("Spawn", spawn, xytext=(-8, 6), textcoords="offset points", ha="right", color=INK,
                fontsize=9, bbox=dict(fc="white", ec="none", alpha=0.8, pad=1))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    if args.config:
        ax.legend(handles=[Line2D([], [], color=AFTER, lw=1.5, label=f"Planned route ({args.config})")],
                  loc="lower right", frameon=True, labelcolor=INK)
    return fig


# ------------------------------------------------------------------------ main

def main():
    parser = argparse.ArgumentParser(description="Plot evaluator runs on the saved map.")
    parser.add_argument("figure", choices=["scenarios", "compare", "speed"], help="which figure to draw")
    parser.add_argument("configs", nargs="*", help="compare/speed: the two configs, before then after")
    parser.add_argument("--config", help="scenarios: also draw this config's planned routes")
    parser.add_argument("--labels", nargs=2, help="legend names for the two configs")
    parser.add_argument("--run", type=int, default=1, help="which repeat to draw (default 1)")
    parser.add_argument("--all-runs", action="store_true", help="draw every repeat")
    parser.add_argument("--logs", default="~/ee3305/logs", help="folder holding the run folders")
    parser.add_argument("--map", default=DEFAULT_MAP, help="map yaml")
    parser.add_argument("--scenarios", default=DEFAULT_SCENARIOS, help="scenarios yaml")
    parser.add_argument("-o", "--output", help="output image (.png, or .pdf/.svg for vector)")
    args = parser.parse_args()
    args.logs = os.path.expanduser(args.logs)
    if args.figure != "scenarios" and len(args.configs) != 2:
        parser.error(f"{args.figure} needs two configs, e.g. {args.figure} base rpp")

    image, meta = load_map(args.map)
    with open(args.scenarios) as f:
        scenarios = yaml.safe_load(f)

    # Draw the chosen figure
    if args.figure == "scenarios":
        fig = plot_scenarios(args, image, meta, scenarios)
        default_name = "scenarios.png"
    else:
        fig = plot_compare(args, image, meta, scenarios, speed_mode=(args.figure == "speed"))
        default_name = f"{args.figure}_{args.configs[0]}_vs_{args.configs[1]}.png"

    # Save (default: <logs>/figures/)
    output = args.output or os.path.join(args.logs, "figures", default_name)
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    fig.savefig(output, dpi=200)
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
