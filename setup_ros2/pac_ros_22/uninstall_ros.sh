#!/bin/bash

echo "> Removing sources ..."
sudo rm -rf /etc/apt/sources.list.d/ros2*.list
sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
sudo rm -rf $HOME/.ros

echo "> Removing installed packages ..."
sudo apt remove -y ros-humble-desktop
sudo apt remove -y ros-humble-ros-base
sudo apt remove -y ros-dev-tools
sudo apt autoremove -y

#echo "> Removing installed directories ..."
#sudo rm -rf /opt/ros
