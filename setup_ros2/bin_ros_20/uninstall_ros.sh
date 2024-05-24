#!/bin/bash
source ../demo_package/common.sh

echo "> Removing $ros_workspace"
rm -rf $ros_workspace

echo "> Removing $ros_distro_ws"
rm -rf $ros_distro_ws

echo "> Removing ROS apt sources"
sudo rm /etc/apt/sources.list.d/ros2.list
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list

echo "> Removing Python virtual environment"
rm -rf $HOME/venv
