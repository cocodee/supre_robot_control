#!/bin/bash
conda activate ros2_env
source install/setup.bash
ros2 launch supre_robot_control  leader_follower.launch.py