# EE3305 / ME3243 Team 09: running the system

Run every command from the workspace root (`~/ee3305`), in a terminal with only ROS 2 Jazzy sourced.

```bash
colcon build --symlink-install
source install/setup.bash
```

The settings under test are in `src/ee3305_bringup/params/run.yaml`, and the scenario goals are in `params/scenarios.yaml` (S1 to S5, and the waypoint mission M1).

| `run.yaml` setting | Baseline | Improved |
|---|---|---|
| `planner: algorithm` | `dijkstra` | `thetastar` |
| `planner: smoothing_window` | `0.0` | `0.6` |
| `controller: controller_mode` | `baseline_pp` | `rpp` |
| `behavior: use_fsm` | `false` | `true` |

## 1. Launch only

```bash
ros2 launch ee3305_bringup run.launch.py                  # Gazebo and RViz
ros2 launch ee3305_bringup run.launch.py headless:=True   # no GUI
```

The robot spawns at the map origin and waits for a goal. In RViz, set one with **2D Goal Pose**.

## 2. Send a goal or waypoints by hand (no evaluation)

With the launch running, in a second terminal:

```bash
# one goal (the S2 goal)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 5.3, y: 1.6}}}"

# a list of waypoints, driven in order (needs use_fsm: true)
ros2 topic pub --once /waypoints geometry_msgs/msg/PoseArray \
  "{header: {frame_id: map}, poses: [{position: {x: -2.0, y: -0.3}}, {position: {x: 5.3, y: 1.6}}]}"
```

Use `controller_mode: rpp` for waypoint lists: baseline Pure Pursuit cannot turn on the spot when the next waypoint is behind the robot.

## 3. One evaluated run, then analysis

```bash
# terminal 1
ros2 launch ee3305_bringup run.launch.py headless:=True

# terminal 2: sends the scenario goal, logs every topic to logs/test_S2_1/, exits on arrival or time-out
ros2 run ee3305_py evaluator --ros-args -p scenario:=S2 -p config:=test -p run:=1 \
  -p log_dir:=$PWD/logs -p use_sim_time:=true
```

Stop the launch with Ctrl-C, then analyse every run folder in `logs/`:

```bash
python3 src/ee3305_bringup/scripts/analyze_runs.py logs           # logs/runs.csv and logs/summary.csv
python3 src/ee3305_bringup/scripts/plot_runs.py scenarios         # map with the S1 to S5 goals
python3 src/ee3305_bringup/scripts/plot_runs.py compare base test # before/after, saved in logs/figures/
```

## 4. Batch runs with the script

```bash
./src/ee3305_bringup/scripts/run_scenario.sh <config> <scenario> <n>

./src/ee3305_bringup/scripts/run_scenario.sh base S2 3       # logs/base_S2_1 to _3
for s in S1 S2 S3 S4 S5; do ./src/ee3305_bringup/scripts/run_scenario.sh full $s 3; done
./src/ee3305_bringup/scripts/run_scenario.sh full M1 3       # waypoint mission
```

`<config>` is only a label: set `run.yaml` before each batch. Each run starts a fresh headless simulation, runs the evaluator (capped at 600 s), stops everything and saves a copy of `run.yaml` in its folder. Then run the analysis commands from step 3.
