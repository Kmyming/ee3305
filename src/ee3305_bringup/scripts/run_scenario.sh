#!/bin/bash
# Runs one test scenario n times in the headless sim, logging each run with the evaluator.
# Usage:   ./run_scenario.sh <config> <scenario> <n>
# Example: ./run_scenario.sh base S1 3   ->  logs/base_S1_1, logs/base_S1_2, logs/base_S1_3

#1. Check the arguments
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <config> <scenario> <n>"
    exit 1
fi
CONFIG="$1"
SCENARIO="$2"
N="$3"

#2. Go to the workspace (this file is in src/ee3305_bringup/scripts/) and load it
cd "$(dirname "$0")/../../.." || exit 1
source install/setup.bash

#3. Job control, so background processes respond to kill -INT like Ctrl-C
set -m

#4. Stop whatever is running: the evaluator, the launch, then any Gazebo leftovers
EVAL_PID=""
LAUNCH_PID=""
stop_all(){
    if [ -n "$EVAL_PID" ]; then
        kill -INT "$EVAL_PID" 2>/dev/null
        wait "$EVAL_PID" 2>/dev/null
        EVAL_PID=""
    fi
    if [ -n "$LAUNCH_PID" ]; then
        kill -INT "$LAUNCH_PID" 2>/dev/null
        wait "$LAUNCH_PID" 2>/dev/null
        LAUNCH_PID=""
    fi
    pkill -9 ruby
}
trap 'echo "Stopping..."; stop_all; exit 130' INT

#5. One pass per run
for i in $(seq 1 "$N"); do
    RUN_DIR="logs/${CONFIG}_${SCENARIO}_${i}"
    mkdir -p "$RUN_DIR"
    cp src/ee3305_bringup/params/run.yaml "$RUN_DIR/"
    echo "=== $CONFIG $SCENARIO: run $i of $N"

    # the sim and the team's nodes, in the background, output saved in the run folder
    ros2 launch ee3305_bringup run.launch.py headless:=True > "$RUN_DIR/launch.log" 2>&1 & LAUNCH_PID=$!

    # the evaluator, capped at 600 s in case the run never starts
    timeout -s INT 600 ros2 run ee3305_py evaluator --ros-args \
        -p config:="$CONFIG" -p scenario:="$SCENARIO" -p run:="$i" \
        -p log_dir:="$PWD/logs" -p use_sim_time:=true &
    EVAL_PID=$!
    wait "$EVAL_PID"
    code=$?
    EVAL_PID=""

    stop_all

    #report how the run ended
    if [ "$code" -eq 0 ] && [ -f "$RUN_DIR/result.csv" ]; then
        echo "    result: $(tail -n 1 "$RUN_DIR/result.csv" | cut -d, -f4)"
    elif [ "$code" -eq 124 ]; then
        echo "    no result: hit the 600 s cap"
    else
        echo "    no result: evaluator exit code $code (see $RUN_DIR/launch.log)"
    fi
    sleep 5
done
