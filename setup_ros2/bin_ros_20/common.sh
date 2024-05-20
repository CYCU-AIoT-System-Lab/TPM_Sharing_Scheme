#!/bin/bash
#set -x
set -e

ros_distro="humble"
ros_package="remote_controller"
ros_workspace="$HOME/ros2_ws"
ros_distro_ws="$HOME/ros2_${ros_distro}"
package_dir="${ros_workspace}/${ros_package}"

package_node1="talker"          # default demo
package_node2="listener"        # default demo
package_node3="controller"
package_node4="dummy"
