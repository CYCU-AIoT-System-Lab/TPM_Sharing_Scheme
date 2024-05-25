#!/bin/bash
script=$(realpath "$0")
script_path=$(dirname "$script")
source ~/.bash_aliases

# Don't reboot when asked
echo -e "This process can't be scripted! Answer with default options beside rebooting."
echo -e "Execute \"upgrade_jetson_3.sh\" after this script finishes.\n"

echo "> Creating progress flag file ..."
rm      $script_path/upgrade_jetson_1_done.flag
touch   $script_path/upgrade_jetson_2_done.flag

sudo do-release-upgrade || { echo -e "\ndo-release-upgrade failed, likely due to time out"; exit 1; }

# Should automatically reboot
