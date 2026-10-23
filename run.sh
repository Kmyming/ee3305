#!/bin/bash
source install/setup.bash
ros2 launch ee3305_bringup run.launch.py cpp:=False headless:=False libgl:=False
./kill.sh # run in case gz cannot be interrupted properly.
