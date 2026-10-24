#!/bin/bash
# make sure the SLAM (cartographer) node is running.
ros2 run nav2_map_server map_saver_cli -f src/ee3305_bringup/maps/ee3305
./bd.sh
