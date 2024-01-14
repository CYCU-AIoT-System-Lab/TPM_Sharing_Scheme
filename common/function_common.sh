#!/bin/bash

# Function: include env to config file, load config, var check
# Usage: source "config.ini"
# Input variable: $1: config.ini
load_preset () {
    echo_notice "common" "setup" "Env var to config file..."
    sed -i 's@${HOME}@'"$HOME"'@' "$2" # replace ${HOME} with $HOME

    echo_notice "common" "setup" "Loading config file..."
    parse "$2" "display"

    echo_notice "common" "setup" "Checking var..."
    check_var install_platform 1
}

clear_preset () {
    echo "hi"
}
