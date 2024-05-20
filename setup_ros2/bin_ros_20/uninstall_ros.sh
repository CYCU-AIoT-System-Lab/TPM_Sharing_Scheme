#!/bin/bash
source common.sh

rm -rf $ros_workspace
rm -rf $ros_distro_ws
sudo rm /etc/apt/sources.list.d/ros2.list
rm -rf $HOME/venv
