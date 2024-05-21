# Binary ROS2 Installation on Ubuntu 20.04 and after

## Demo

Local execution video available at YouTube: [https://youtu.be/Ygegn1MnVTE](https://youtu.be/Ygegn1MnVTE).

Cross machine execution video available at YouTube: [https://youtu.be/SdiJkYvwXNQ](https://youtu.be/SdiJkYvwXNQ).

## Structure

### Utilities

- [./install_ros.sh](./install_ros.sh): Install ROS2 binary and build from source on Jetson Nano with Ubuntu 20.04.
- [./uninstall_ros.sh](./uninstall_ros.sh): Uninstall ROS2 from Jetson Nano.
- [./setup_ros_package.sh](./setup_ros_package.sh): Create a new package with CPP source code in this directory.
    1. Check executing path.
    2. Create ROS working space and package.
    3. Create initial file structure in package.
    4. Modify `package.xml` and `CMakeLists.txt` for the package.
    5. Call `build_ros.sh`.
- [./build_ros_package.sh](./build_ros_package.sh): Perform `colcon build` for this package.
    1. Copy source file to package directory.
    2. Use `rosdep` to install erquired dependencies.
    3. Use `colcon build` to build the package.
- [./remove_ros_package.sh](./remove_ros_package.sh): Remove this package from workspace.
    1. Remove the entire package.
- [./execute_ros_package.sh](./execute_ros_package.sh): Execute user selected package with number input.
    1. List out all nodes in the package.
    2. Execute the selected node.
- [./common.sh](./common.sh): General settings for utilities.
    1. Set bash execution mode.
    2. Set common variables.
    3. Set nodes in package.
- [./load_ros_env.sh](./load_ros_env.sh): Load ROS2 environment variables.

### Source Code

- [./publisher_member_function.cpp](./publisher_member_function.cpp): A publisher example script from ROS2 tutorial.
- [./subscriber_member_function.cpp](./subscriber_member_function.cpp): A subscriber example script from ROS2 tutorial.
- [./controller.cpp](./controller.cpp): A controller for demo.
- [./car.cpp](./car.cpp): A control protocol development helper dummy.

## Debugging

1. If can't exit spawned ROS2 process, use `Ctrl + z` to pause the process and `kill -9 <PID>` to kill the process.
2. If executing `build_ros_package.sh` shows error like `remote_controller: No definition of [std_msgs] for OS version [focal]`:
    1. Execute `./remove_ros_package.sh`
    2. Execute `./setup_ros_package.sh`
