#!/bin/bash
conda activate ros2_env
source install/setup.bash
sudo chmod 666 /dev/ttyTHS1
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libstdc++.so.6 python
ros2 launch supre_robot_control  follower_trajectory.launch.py