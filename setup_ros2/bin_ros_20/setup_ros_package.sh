#!/bin/bash
source common.sh

script=$(realpath "$0")
script_path=$(dirname "$script")
echo $script_path

if [[ ! $script_path == *"/setup_ros2/bin_ros_20"* ]]; then
    echo "Please run this script from its directory"
    exit 1
fi

mkdir -p $ros_workspace
cd $ros_workspace
source $ros_source

ros2 pkg create --build-type ament_cmake ${ros_package}

cp $script_path/*.cpp ${package_dir}/src
sed -i '11 i <depend>rclcpp</depend>\n<depend>std_msgs</depend>' ${package_dir}/package.xml
sed -i '13 i find_package(rclcpp REQUIRED)\nfind_package(std_msgs REQUIRED)' ${package_dir}/CMakeLists.txt
sed -i '16 i add_executable('${package_node1}' src/publisher_member_function.cpp)\n' ${package_dir}/CMakeLists.txt
sed -i '17 i add_executable('${package_node2}' src/subscriber_member_function.cpp)\n' ${package_dir}/CMakeLists.txt
sed -i '18 i add_executable('${package_node3}' src/sender.cpp)\n' ${package_dir}/CMakeLists.txt
sed -i '19 i add_executable('${package_node4}' src/receiver.cpp)\n' ${package_dir}/CMakeLists.txt
sed -i '20 i ament_target_dependencies('${package_node1}' rclcpp std_msgs)\n' ${package_dir}/CMakeLists.txt
sed -i '21 i ament_target_dependencies('${package_node2}' rclcpp std_msgs)' ${package_dir}/CMakeLists.txt
sed -i '22 i ament_target_dependencies('${package_node3}' rclcpp std_msgs)\n' ${package_dir}/CMakeLists.txt
sed -i '23 i ament_target_dependencies('${package_node4}' rclcpp std_msgs)\n' ${package_dir}/CMakeLists.txt
sed -i '24 i install(TARGETS\n\t'${package_node1}'\n\t'${package_node2}'\n\t'${package_node3}'\n\t'${package_node4}'\n\tDESTINATION\n\tlib/${PROJECT_NAME})\n' ${package_dir}/CMakeLists.txt

cd $script_path
source build_ros.sh
