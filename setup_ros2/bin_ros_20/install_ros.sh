#!/bin/bash
source common.sh || { echo "Error: common.sh not found"; exit 1; }

# Note: This script is for Ubuntu 20.04 (Focal Fossa) not older or newer versions.
# Ref: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# Ref: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html
# Ref: --> https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

echo "> Setting locale ..."
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

echo "> Creating python 3.8 virtual environment ..."
sudo apt install -y python3.8-venv
python3.8 -m venv $HOME/venv
source $HOME/venv/bin/activate

echo "> Add the ROS 2 apt repository ..."
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "> Install development tools and ROS tools ..."
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

python3 -m pip install -U \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   "pytest>=5.3" \
   pytest-repeat \
   pytest-rerunfailures

echo "> Get library dependencies ..."
sudo apt install -y --fix-missing \
    build-essential \
    libacl1-dev \
    libtinyxml2-dev \
    libasio-dev \
    libxaw7-dev \
    pyqt5-dev \
    python3-pyqt5 \
    qtcreator \
    qt5-default \
    libfreetype6-dev

# empy version must be below 4.0
# https://robotics.stackexchange.com/questions/109773/colcon-build-problem
# https://stackoverflow.com/questions/77642155/attributeerror-module-object-has-no-attribute-raw-opt/77656642#77656642
pip3 install \
    empy==3.3.4 \
    lark \
    catkin_pkg

echo "> Get ROS2 code ..."
mkdir -p $ros_distro_ws/src
cd $ros_distro_ws
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

echo "> Install dependencies using rosdep ..."
sudo apt upgrade
sudo rosdep init
rosdep update
cd $ros_distro_ws
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# If python related issues, try to re-perform \"rosdep install\" command 21 lines above
# rviz* can't be compiled due to rviz_rendering dependency issue
# *qt* can't be compiled due to PyQt5 dependency issue
echo "> Build the code in the workspace ..."
cd $ros_distro_ws
#colcon build --symlink-install \
#    --packages-ignore rviz_rendering \
#                      rviz_rendering_tests \
#                      rviz_common \
#                      rviz_visual_testing_framework \
#                      rviz_default_plugins \
#                      rviz2 \
#                      qt_gui_cpp \
#                      rqt_gui_cpp
colcon build --symlink-install \
    --packages-skip-regex rviz* \
    --packages-ignore qt_gui_cpp \
                      rqt_gui_cpp

echo "> Finished installing ROS 2 Humble"
