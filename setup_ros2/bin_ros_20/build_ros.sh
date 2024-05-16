#!/bin/bash
source common.sh
source $ros_source

cd $ros_workspace
rosdep install -i --from-path $ros_workspace --rosdistro $ros_distro -y
colcon build --packages-select ${ros_package}

echo "Build complete"
