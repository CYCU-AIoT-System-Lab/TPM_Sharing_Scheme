#!/bin/bash
source ~/.bash_aliases
set -x

sudo sed -i '/#*WaylandEnable*=*false/c\WaylandEnable=false' /etc/gdm3/custom.conf
sudo sed -i '/#*Driver*"nvidia"/c\Driver="nvidia"' /etc/X11/xorg.conf
sudo sed -i '/Prompt=normal/c\Prompt=never' /etc/update-manager/release-upgrades

# Perform required reboot
echo ""
read -rsn1 -p "Press any key to perform required reboot ..."
reboot
