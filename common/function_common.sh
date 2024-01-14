#!/bin/bash

# Function: load common function, include env to config file, load config, var check
# Usage: source "./function.sh" "config.ini"
# Input variable: $1: common/function.sh
#                 $2: config.ini
load_preset () {
    echo_notice "common" "setup" "Loading preset..."
    source "$1"
    echo_notice "common" "setup" "Env var to config file..."
    sed -i 's@${HOME}@'"$HOME"'@' "$2" # replace ${HOME} with $HOME
    echo_notice "common" "setup" "Loading config..."
    parse "$2" "display"
    echo_notice "common" "setup" "Checking var..."
    check_var install_platform 1
}
echo_notice "common" "function" "Loaded function: load_preset"

clear_preset () {
    echo_notice "common" "remove" "Clearing preset..."
}
echo_notice "common" "function" "Loaded function: clear_preset"
