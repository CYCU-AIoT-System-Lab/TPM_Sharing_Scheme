#!/bin/bash
#set -x
set -e

ros_distro="humble"
ros_package="remote_controller"
ros_workspace="/home/$USER/ros2_ws"
ros_distro_ws="/home/$USER/ros2_${ros_distro}"
ros_source="${ros_distro_ws}/ros2-linux/setup.bash"
package_dir="${ros_workspace}/${ros_package}"

package_node1="talker"      # default demo
package_node2="listener"    # default demo
package_node3="controller"
package_node4="car"
