#!/bin/bash
source $HOME/venv/bin/activate || echo "> Python 3.8 virtual environment not found, defaulting to system python3"

# Try to source ROS2 workspace
echo "> Trying to source ROS2 workspace setup.bash, if it exists..."
ros_source="${ros_distro_ws}/ros2-linux/setup.bash" && source $ros_source || \
    ros_source="${ros_distro_ws}/install/*setup.bash" && source $ros_source || \
    ros_source="/opt/ros/$ros_distro/setup.bash"
    ros_source="${ros_distro_ws}/local_setup.bash" && source $ros_source || \
    { echo "ROS is not installed in default location \"~/ros2_${ros_distro} or not installed!"; exit 1; }

echo "> Sourcing complete!"
