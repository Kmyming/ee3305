#!/usr/bin/env python3
"""Compute the evaluation metrics from the evaluator's run logs.

Reads every finished run folder logs/<config>_<scenario>_<run>/ and writes, into the logs folder:
    runs.csv     one row of metrics per run
    summary.csv  one row per (config, scenario): mean and sample std of each metric

Metrics (sim seconds, metres), measured from goal sent to arrival:
    success          1 if the goal was reached, else 0
    time_to_goal     goal sent -> first odom point within goal_tol of the goal
    driven_length    length of the driven trajectory
    planned_length   length of the first path planned to each goal (summed over a mission's legs)
    cte_rms, cte_max distance from the robot to the nearest point of the path it was following
                     (the latest /path, resampled to points CTE_SPACING apart)
    min_clearance    closest distance from the robot's centre to a wall cell of the saved map
    plans            number of paths published

Usage (from ~/ee3305):
    python3 src/ee3305_bringup/scripts/analyze_runs.py [logs_dir] [--map map.yaml]
"""

import argparse
import csv
import math
import os

import numpy as np
import yaml
from PIL import Image

# Paths relative to this file: scripts/ -> ee3305_bringup/
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MAP = os.path.join(SCRIPT_DIR, "..", "maps", "ee3305.yaml")

# Clearance ignores points within this distance (m) of the start (the spawn is near a wall)
START_EXCLUSION = 0.3

# Paths are resampled to points this far apart (m) for the cross-track error
CTE_SPACING = 0.01

# Metrics, in output column order
METRICS = ["success", "time_to_goal", "driven_length", "planned_length",
           "cte_rms", "cte_max", "min_clearance", "plans"]


# ---------------------------------------------------------------- loading files

def load_csv(path):
    """Load a numeric CSV as {column name: numpy array}."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return {col: np.array([float(row[col]) for row in rows]) for col in reader.fieldnames}


def load_result(path):
    """Load result.csv's single row as a dict of strings."""
    with open(path, newline="") as f:
        return next(csv.DictReader(f))


def find_runs(logs_dir):
    """Load every run folder that has a result.csv; skip (and report) unfinished runs."""
    runs = []
    for name in sorted(os.listdir(logs_dir)):
        folder = os.path.join(logs_dir, name)
        # Run folders hold odom.csv; anything else (e.g. figures/) is not a run
        if not os.path.exists(os.path.join(folder, "odom.csv")):
            continue
        if not os.path.exists(os.path.join(folder, "result.csv")):
            print(f"skipping {name}: no result.csv (run did not finish)")
            continue
        run = {"name": name, "result": load_result(os.path.join(folder, "result.csv"))}
        for topic in ["odom", "cmd_vel", "goal", "path_request", "path"]:
            run[topic] = load_csv(os.path.join(folder, f"{topic}.csv"))
        runs.append(run)
    # Order by config, scenario, then run number
    runs.sort(key=lambda r: (r["result"]["config"], r["result"]["scenario"], int(r["result"]["run"])))
    return runs


def load_map(yaml_path):
    """Load the map image (numpy array) and its yaml settings."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), meta["image"])))
    return image, meta


def split_paths(path):
    """Split path.csv into a list of (t, x, y) per published path, poses in order."""
    paths = []
    for pid in np.unique(path["path_id"]):
        rows = path["path_id"] == pid
        order = np.argsort(path["i"][rows])
        paths.append((path["t"][rows][0], path["x"][rows][order], path["y"][rows][order]))
    return paths


def first_plan_per_leg(paths, tol=0.15):
    """The first path planned to each goal; a new leg starts when the path's end point moves."""
    firsts, last_end = [], None
    for t, x, y in paths:
        end = (x[-1], y[-1])
        if last_end is None or math.hypot(end[0] - last_end[0], end[1] - last_end[1]) > tol:
            firsts.append((t, x, y))
        last_end = end
    return firsts


# ------------------------------------------------------------------- geometry

def wall_points(yaml_path):
    """World (x, y) of the centre of every occupied cell of the saved map."""
    image, meta = load_map(yaml_path)
    # Pixel -> occupancy probability (map_server convention), then threshold to walls
    if meta.get("negate", 0) == 0:
        occupancy = (255 - image) / 255.0
    else:
        occupancy = image / 255.0
    rows, cols = np.nonzero(occupancy > meta["occupied_thresh"])
    # Cell centre in metres; image row 0 is the top of the map, so flip the row index
    res, origin_x, origin_y = meta["resolution"], meta["origin"][0], meta["origin"][1]
    x = origin_x + (cols + 0.5) * res
    y = origin_y + (image.shape[0] - 1 - rows + 0.5) * res
    return x, y


def polyline_length(x, y):
    """Total length of the line through the given points."""
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


def resample_path(path_x, path_y, spacing=CTE_SPACING):
    """Points every `spacing` metres along the path, joining its poses with straight lines."""
    # Distance along the path at each pose, dropping repeated poses
    along = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(path_x), np.diff(path_y)))])
    keep = np.concatenate([[True], np.diff(along) > 0])
    path_x, path_y, along = path_x[keep], path_y[keep], along[keep]
    if along[-1] == 0:
        return path_x[:1], path_y[:1]
    # Evenly spaced distances (plus the end), then the matching x and y
    targets = np.append(np.arange(0.0, along[-1], spacing), along[-1])
    return np.interp(targets, along, path_x), np.interp(targets, along, path_y)


def nearest_distance(px, py, qx, qy, chunk=2000):
    """For each point p, the distance to the nearest point q."""
    out = [np.empty(0)]
    # (p points x q points) table of distances, smallest in each row, in chunks of p to limit memory
    for i in range(0, len(px), chunk):
        out.append(np.hypot(px[i:i + chunk, None] - qx, py[i:i + chunk, None] - qy).min(axis=1))
    return np.concatenate(out)


# -------------------------------------------------------------------- metrics

def compute_metrics(run, walls):
    """Compute every metric for one run; NaN where undefined."""
    res, odom, goal = run["result"], run["odom"], run["goal"]
    goal_x, goal_y = float(res["goal_x"]), float(res["goal_y"])
    goal_tol = float(res["goal_tol"])

    # Keep odom from the moment the goal was sent
    t_goal = goal["t"][0] if len(goal["t"]) else odom["t"][0]
    after_goal = odom["t"] >= t_goal
    t, x, y = odom["t"][after_goal], odom["x"][after_goal], odom["y"][after_goal]

    # Cut at arrival: the first point within goal_tol
    dist_to_goal = np.hypot(x - goal_x, y - goal_y)
    inside = np.nonzero(dist_to_goal < goal_tol)[0]
    end = inside[0] + 1 if len(inside) else len(t)
    t, x, y = t[:end], x[:end], y[:end]

    m = {}
    # Outcome and lengths
    m["success"] = 1.0 if res["result"] == "reached" else 0.0
    m["time_to_goal"] = t[-1] - t_goal if len(inside) else math.nan
    m["driven_length"] = polyline_length(x, y)

    paths = split_paths(run["path"])
    m["plans"] = float(len(paths))
    legs = first_plan_per_leg(paths, goal_tol)
    m["planned_length"] = sum(polyline_length(px, py) for _, px, py in legs) if paths else math.nan

    # Cross-track error: each odom point against the latest path published before it
    errors = []
    for k, (t_path, path_x, path_y) in enumerate(paths):
        t_next = paths[k + 1][0] if k + 1 < len(paths) else math.inf
        following = (t >= t_path) & (t < t_next)
        if np.any(following):
            qx, qy = resample_path(path_x, path_y)
            errors.append(nearest_distance(x[following], y[following], qx, qy))
    if errors:
        errors = np.concatenate(errors)
        m["cte_rms"] = float(np.sqrt(np.mean(errors ** 2)))
        m["cte_max"] = float(np.max(errors))
    else:
        m["cte_rms"] = m["cte_max"] = math.nan

    # Minimum clearance: distance to the nearest wall point, once the robot has left the start
    left_start = np.hypot(x - x[0], y - y[0]) > START_EXCLUSION
    clearance = nearest_distance(x[left_start], y[left_start], walls[0], walls[1])
    m["min_clearance"] = float(np.min(clearance)) if len(clearance) else math.nan
    return m


def mean_std(values):
    """Mean and sample std (ddof=1) of the non-NaN values; std is NaN for fewer than 2."""
    v = np.asarray(values, dtype=float)
    v = v[~np.isnan(v)]
    if len(v) == 0:
        return math.nan, math.nan
    return float(np.mean(v)), (float(np.std(v, ddof=1)) if len(v) > 1 else math.nan)


def fmt(value):
    """Format a number for the CSVs: 4 decimals, NaN as an empty cell."""
    return "" if value is None or (isinstance(value, float) and math.isnan(value)) else f"{value:.4f}"


# ------------------------------------------------------------------------ main

def main():
    parser = argparse.ArgumentParser(description="Summarise the evaluator's run logs into runs.csv and summary.csv.")
    parser.add_argument("logs_dir", nargs="?", default="~/ee3305/logs", help="folder holding the run folders")
    parser.add_argument("--map", default=DEFAULT_MAP, help="map yaml used for the clearance metric")
    args = parser.parse_args()
    logs_dir = os.path.expanduser(args.logs_dir)

    runs = find_runs(logs_dir)
    if not runs:
        print(f"no finished runs in {logs_dir}")
        return
    walls = wall_points(args.map)

    # runs.csv: one row per run
    runs_path = os.path.join(logs_dir, "runs.csv")
    with open(runs_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["config", "scenario", "run", "result"] + METRICS)
        for run in runs:
            run["metrics"] = compute_metrics(run, walls)
            res = run["result"]
            writer.writerow([res["config"], res["scenario"], res["run"], res["result"]]
                            + [fmt(run["metrics"][m]) for m in METRICS])

    # Group runs by (config, scenario)
    groups = {}
    for run in runs:
        key = (run["result"]["config"], run["result"]["scenario"])
        groups.setdefault(key, []).append(run["metrics"])

    # summary.csv: mean and std per group
    summary_path = os.path.join(logs_dir, "summary.csv")
    with open(summary_path, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["config", "scenario", "n_runs"]
        for m in METRICS:
            header += [f"{m}_mean", f"{m}_std"]
        writer.writerow(header)
        for (config, scenario), group in groups.items():
            row = [config, scenario, len(group)]
            for m in METRICS:
                mean, std = mean_std([g[m] for g in group])
                row += [fmt(mean), fmt(std)]
            writer.writerow(row)
            ttg_mean, ttg_std = mean_std([g["time_to_goal"] for g in group])
            success = mean_std([g["success"] for g in group])[0]
            print(f"{config:>10} {scenario}: {len(group)} runs, success {success:.0%}, "
                  f"time to goal {ttg_mean:.1f} ± {ttg_std:.1f} s")

    print(f"wrote {runs_path} ({len(runs)} runs) and {summary_path} ({len(groups)} groups)")


if __name__ == "__main__":
    main()
