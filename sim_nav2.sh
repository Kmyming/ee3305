#!/bin/bash
source install/setup.bash
ros2 launch ee3305_bringup sim_nav2.launch.py headless:=False libgl:=False
./kill.sh
