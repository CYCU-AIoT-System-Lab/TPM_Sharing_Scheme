#!/bin/bash

script=$(realpath "$0")
script_path=$(dirname "$script")
dirname=$(basename "$script_path")
filename=$(basename "$0")
source "../common/functions.sh"
source "./function_acsroutine.sh"
load_preset "./config.ini"

remove_libtrace () {
    echo_notice "$dirname" "$filename" "libtrace removing"
    cd "$SETUPLIBTRACE_DIRNAME/hw4_libtrace_setup"
    sudo bash ./remove.sh
    sudo rm -rf $SETUPLIBTRACE_DIRNAME
    sudo rm -rf $LIBTRACEDEP_DIRNAME
    sudo rm -rf $SETUPLIBTRACE_ZIPNAME
}

remove_server () {
    echo_notice "$dirname" "$filename" "server-side removing"
}

remove_client () {
    echo_notice "$dirname" "$filename" "client-side removing"
}

if [ $job_setup_libtrace    -eq 1 ]; then remove_libtrace; fi
if [ $is_server             -eq 1 ]; then remove_server; fi
if [ $is_server             -ne 1 ]; then remove_client; fi

echo_notice "$dirname" "$filename" "$script finished"
