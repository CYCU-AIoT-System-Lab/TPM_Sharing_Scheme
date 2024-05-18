#!/bin/bash
source common.sh
source load_ros_env.sh
script=$(realpath "$0")
script_path=$(dirname "$script")

# Check if script is run from its directory
if [[ ! $script_path == *"/setup_ros2/bin_ros_20"* ]]; then
    echo "Please run this script from its directory"
    exit 1
fi

echo "> Entering ROS2 workspace"
mkdir -p $ros_workspace
cd $ros_workspace

echo "> Creating ROS2 package"
ros2 pkg create --build-type ament_cmake ${ros_package}

echo "> Modifying build dependencies"
sed -i '11 i <depend>rclcpp</depend>\n<depend>std_msgs</depend>'                    ${package_dir}/package.xml
sed -i '13 i find_package(rclcpp REQUIRED)\nfind_package(std_msgs REQUIRED)'        ${package_dir}/CMakeLists.txt
sed -i '16 i add_executable('${package_node1}' src/publisher_member_function.cpp)'  ${package_dir}/CMakeLists.txt
sed -i '17 i add_executable('${package_node2}' src/subscriber_member_function.cpp)' ${package_dir}/CMakeLists.txt
sed -i '18 i add_executable('${package_node3}' src/'${package_node3}'.cpp)'         ${package_dir}/CMakeLists.txt
sed -i '19 i add_executable('${package_node4}' src/'${package_node4}'.cpp)'         ${package_dir}/CMakeLists.txt
sed -i '20 i ament_target_dependencies('${package_node1}' rclcpp std_msgs)'         ${package_dir}/CMakeLists.txt
sed -i '21 i ament_target_dependencies('${package_node2}' rclcpp std_msgs)'         ${package_dir}/CMakeLists.txt
sed -i '22 i ament_target_dependencies('${package_node3}' rclcpp std_msgs)'         ${package_dir}/CMakeLists.txt
sed -i '23 i ament_target_dependencies('${package_node4}' rclcpp std_msgs)'         ${package_dir}/CMakeLists.txt
sed -i '24 i install(TARGETS\n\t'${package_node1}'\n\t'${package_node2}'\n\t'${package_node3}'\n\t'${package_node4}'\n\tDESTINATION\n\tlib/${PROJECT_NAME})\n' ${package_dir}/CMakeLists.txt

cd $script_path
source build_package.sh
