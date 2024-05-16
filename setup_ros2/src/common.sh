#!/bin/bash
#set -x

ros_distro="humble"
ros_package="remote_controller"
ros_source="/home/$USER/ros2_${ros_distro}/ros2-linux/setup.bash"
ros_workspace="/home/$USER/ros2_ws"
package_dir="${ros_workspace}/${ros_package}"

package_node1="talker"      # default demo
package_node2="listener"    # default demo
package_node3="sender"
package_node4="receiver"
