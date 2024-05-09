#!/bin/bash

export proj_name="ros_cpp_pubsub"

source /opt/ros/rolling/setup.bash
cd /ros2_ws/src/$proj_name
source /ros2_ws/src/$proj_name/install/setup.bash

echo "Running listener node"
ros2 run $proj_name listener
