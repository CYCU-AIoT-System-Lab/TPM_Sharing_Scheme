#!/bin/bash

script=$(realpath "$0")
script_path=$(dirname "$script")
dirname=$(basename "$script_path")
filename=$(basename "$0")
source "../common/functions.sh"
source "./function_boot.sh"
load_preset "./config.ini"

echo_notice "$dirname" "$filename" "Disabling $SERVICE_FILE_3 at startup"
sudo systemctl disable $SERVICE_FILE_3

echo_notice "$dirname" "$filename" "Shutdown $SERVICE_FILE_3"
sudo systemctl stop $SERVICE_FILE_3

echo_notice "$dirname" "$filename" "Remove $SERVICE_FILE_3"
sudo rm "$SYSTEMD_DIR/$SERVICE_FILE_3"

echo_notice "$dirname" "$filename" "Reload daemon"
sudo systemctl daemon-reload

echo_notice "$dirname" "$filename" "Remove auxiliary files"
rm $HASH_DEMO_DATA || :
rm $SWTPM_NVM || :

echo_notice "$dirname" "$filename" "Calling SWTPM remove script"
bash remove_swtpm_daemon.sh

echo_notice "$dirname" "$filename" "$script finished"
