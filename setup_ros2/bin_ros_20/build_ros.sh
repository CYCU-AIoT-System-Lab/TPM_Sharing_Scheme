#!/bin/bash
source common.sh
source $ros_source
script=$(realpath "$0")
script_path=$(dirname "$script")

echo "> Copying source files"
cp $script_path/*.cpp ${package_dir}/src

echo "> Installing dependencies ..."
cd $ros_workspace
rosdep install -i --from-path $ros_workspace --rosdistro $ros_distro -y

echo "> Building package ..."
colcon build --packages-select ${ros_package}

echo "Build complete"
