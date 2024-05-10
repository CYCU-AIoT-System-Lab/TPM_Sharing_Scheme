#!/bin/bash
set +x

sudo apt install -y curl gnupg2 lsb-release
wget -nv --show-progress https://github.com/ros2/ros2/releases/download/release-humble-20240222/ros2-humble-20240222-linux-jammy-arm64.tar.bz2 --output-document ros2-humble-20240222-linux-jammy-arm64.tar.bz2
tar -xf ros2-humble-20240222-linux-jammy-arm64.tar.bz2 -C .
cd ros2-linux
sudo apt install -y python-rosdep
sudo rosdep init
rosdep update
sudo apt install -y python-pip
pip install -U rosdep rosinstall_generator wstool rosinstall
curl -s https://packagecloud.io/install/repositories/dirk-thomas/vcstool/script.deb.sh | sudo bash
#sudo apt install python3-vcstool
rosdep install --from-paths share --ignore-src --rosdistro humble -y --skip-keys "fastcdr fastrtps rmw_connextdds iceoryx_binding_c urdfdom_headers ignition-math6 ignition-cmake2 python3-pykdl cyclonedds python3-lark-parser python3-rosdistro-modules python3-catkin-pkg-modules"
sudo apt install -y libpython3-dev python3-pip
pip3 install -U argcomplete
