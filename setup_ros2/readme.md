# Setup ROS2

For raspberry pi platform, docker is required; for jetson nano, use normal binary installation.

ROS2 Humble is prefered then Iron for longer support (LTS 2027).

## Structure

- [./bin_ros_18](./bin_ros_18): ROS2 humble binary installation for approach 1
- [./bin_ros_20](./bin_ros_20): ROS2 humble binary installation for approach 3
- [./rolling_docker](./rolling_docker): ROS2 rolling arm linux docker environment for approach 2
- [./upgrade_jetson](./upgrade_jetson): Upgrade Jetpack Ubuntu 18.04 to 20.04 for approach 3
- [./readme.md](./readme.md): This file

## Development Approach

1. Jetson Nano: ROS2 Humble, binary installation, rclpy can't be imported and installed
2. Jetson Nano: ROS2 Rolling, docker environment, local machine pub sub success, cross machine pub sub failed
3. Jetson Nano: Upgrade Jetpack Ubuntu 18.04 to 20.04, ROS2 Humble, binary installation, proven successful
4. Jetson Nano: Upgrade Jetpack Ubuntu 18.04 to 22.04, ROS2 Humble, debian package installation, not worked on
5. Raspberry Pi: Ubuntu 24.04, ROS2 Humble, binary installation, success by labmate

## References

1. [https://docs.ros.org/en/foxy/How-To-Guides/Installing-on-Raspberry-Pi.html](https://docs.ros.org/en/foxy/How-To-Guides/Installing-on-Raspberry-Pi.html) 
2. [https://robotics.stackexchange.com/questions/105772/which-ros2-distro-iron-or-humble](https://robotics.stackexchange.com/questions/105772/which-ros2-distro-iron-or-humble)
3. [https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html)
4. [https://github.com/ros2/ros2/releases](https://github.com/ros2/ros2/releases)
