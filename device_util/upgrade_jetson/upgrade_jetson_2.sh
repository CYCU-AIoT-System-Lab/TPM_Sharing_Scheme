#!/bin/bash

# Don't reboot when asked
echo -e "This process can't be scripted! Answer with default options beside rebooting."
echo -e "Execute \"upgrade_jetson_3.sh\" after this script finishes.\n"

sudo do-release-upgrade || { echo -e "\ndo-release-upgrade failed, likely due to time out"; exit 1; }

# Should automatically reboot
