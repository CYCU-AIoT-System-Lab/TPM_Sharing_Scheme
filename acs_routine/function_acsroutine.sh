#!/bin/bash
#set -x
set -e

verbose=0 # 1 for verbose, 0 for silent
script=$(realpath "$0")
script_path=$(dirname "$script")
dirname=$(basename "$script_path")
filename=$(basename "$0")

load_preset () {
    if [ $verbose -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Loading config file..."
    fi
    parse "$1" ""

    if [ $verbose -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Checking var..."
    fi
    check_var install_platform 1

    if [ $verbose -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Loading preset..."
    fi
    working_dir="/opt/update_swtpm"
    install_dir="$working_dir/deps"
}
if [ $verbose -eq 1 ]; then
    echo_notice "$dirname" "$filename" "Loaded function: load_preset"
fi

# Note: `clear_preset` function is not yet implemented
