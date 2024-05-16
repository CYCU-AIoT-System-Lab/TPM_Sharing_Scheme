#!/bin/bash
source common.sh
source $ros_source
source $ros_workspace/install/setup.bash

echo -e "\nAvailable nodes in package:"
echo -e "1. ${ros_package} | ${package_node1}"
echo -e "2. ${ros_package} | ${package_node2}"
echo -e "3. ${ros_package} | ${package_node3}"
echo -e "4. ${ros_package} | ${package_node4}"
echo -e "Else, exit.\n"

read -rsn1 -p "Press number key to execute ..." key
echo -e "\n"

if [ "$key" == "1" ]; then
    ros2 run ${ros_package} ${package_node1}
elif [ "$key" == "2" ]; then
    ros2 run ${ros_package} ${package_node2}
elif [ "$key" == "3" ]; then
    ros2 run ${ros_package} ${package_node3}
elif [ "$key" == "4" ]; then
    ros2 run ${ros_package} ${package_node4}
else
    echo "Invalid key"
fi
