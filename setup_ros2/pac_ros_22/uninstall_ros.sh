#!/bin/bash

echo "> Removing sources ..."
sudo rm -rf /etc/apt/sources.list.d/ros2*.list

echo "> Removing installed directories ..."
sudo rm -rf /opt/ros
