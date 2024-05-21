#!/bin/bash
source common.sh
source load_ros_env.sh
echo "> Trying to source ROS2 workspace setup.bash ..."
source $ros_workspace/install/setup.bash

# ==========================================================
# PARAMS
# ----------------------------------------------------------
# Note: To communicate cross machine, they must be in the same LAN
# cross machine in LAN as client: "http://<ipv4>", like "http://192.168.1.113"
# cross machine in LAN as master: "master"
# local: ""
ros_master_uri="master"
ros_master_uri="http://192.168.1.113"
ros_master_local=""
ros_port=11311
ros_domain_id=5
# ==========================================================

echo -e "\nRemember to configure the parameters!"
if [ $ros_master_uri == "master" ]; then
    echo "> Running as LAN master ..."
    ip_sentence="$(hostname -I)"
    ip_array=($ip_sentence)
    export ROS_MASTER_URI="http://${ip_array[0]}:$ros_port"
    echo "> Set ROS_MASTER_URI=$(echo $ROS_MASTER_URI)"
    export ROS_DOMAIN_ID=$ros_domain_id
    echo "> Set ROS_DOMAIN_ID=$(echo $ROS_DOMAIN_ID)"
elif [[ $ros_master_uri == *"http://"* ]]; then
    echo "> Running as LAN client ..."
    export ROS_MASTER_URI="$ros_master_uri:$ros_port"
    echo "> Set ROS_MASTER_URI=$(echo $ROS_MASTER_URI)"
    export ROS_DOMAIN_ID=$ros_domain_id
    echo "> Set ROS_DOMAIN_ID=$(echo $ROS_DOMAIN_ID)"
else
    echo "> Running locally ..."
fi

export ROS_MASTER_URI="default"

echo -e "\nAvailable nodes in package:"
echo -e "1. ${ros_package} | ${package_node1}"
echo -e "2. ${ros_package} | ${package_node2}"
echo -e "3. ${ros_package} | ${package_node3}"
echo -e "4. ${ros_package} | ${package_node4}"
echo -e "Else, exit.\n"

read -rsn1 -p "Press number key to execute ..." key
echo -e "\n"

if [ "$key" == "1" ]; then
    exec_cmd_arr=(ros2 run ${ros_package} ${package_node1})
elif [ "$key" == "2" ]; then
    exec_cmd_arr=(ros2 run ${ros_package} ${package_node2})
elif [ "$key" == "3" ]; then
    exec_cmd_arr=(ros2 run ${ros_package} ${package_node3} -pi 10 -si 1 -cpn 1 -rpn 1 -t control_signal)
elif [ "$key" == "4" ]; then
    exec_cmd_arr=(ros2 run ${ros_package} ${package_node4})
else
    echo "Invalid key"
    exit 1
fi

echo "> Executing -> ${exec_cmd_arr[@]}"
"${exec_cmd_arr[@]}"
