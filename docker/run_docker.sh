#!/bin/bash
IMAGE_NAME="misumi_base:ros2"

docker run -it -v $(pwd):/supre_robot_control $IMAGE_NAME /bin/bash