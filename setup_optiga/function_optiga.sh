#!/bin/bash

# common/function.sh is required to be sourced before using functions in this file

verbose=0 # 0: no verbose, 1: verbose

# Usage: source "config.ini"
# Input variable: $1: config.ini
load_preset () {
    #echo_notice "setup_optiga" "function_optiga" "Env var to config file..."
    #sed -i 's@${HOME}@'"$HOME"'@' "$1" # replace ${HOME} with $HOME
    
    echo_notice "setup_optiga" "function_optiga" "Loading config file..."
    parse "$1" ""

    echo_notice "setup_optiga" "function_optiga" "Checking var..."
    check_var install_platform 1
    check_var optiga_url 1
}
if [ $verbose == 1 ]; then
    echo_notice "setup_optiga" "function_optiga" "Loaded function: load_preset"
fi

clear_preset () {
    echo_notice "setup_optiga" "function_optiga" "Clearing preset..."
    unset install_platform
    unset optiga_url
}
if [ $verbose == 1 ]; then
    echo_notice "setup_optiga" "function_optiga" "Loaded function: clear_preset"
fi
