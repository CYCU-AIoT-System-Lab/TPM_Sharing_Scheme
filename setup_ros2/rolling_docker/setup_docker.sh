#!/bin/bash

ros2_image="ros:rolling-ros-core-noble"
ros2_container="rolling_container"
proj_name="ros_cpp_pubsub"

echo "Pulling from source"
sudo docker pull $ros2_image

echo "Build new project image from Dockerfile"
sudo docker build --build-arg PACKAGE_NAME=$proj_name -t ${proj_name}_pkg_image .

echo "Create new container from image"
sudo docker run -it --name ${proj_name}_container ${proj_name}_pkg_image
#sudo docker run --name ${proj_name}_container ${proj_name}_pkg_image
