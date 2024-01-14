#!/bin/bash

# Function: load common function, include env to config file, load config, var check
# Usage: source "./function.sh" "config.ini"
# Input variable: $1: common/function.sh
#                 $2: config.ini
load_preset () {
    # Load common function
    source "$1"
    echo_notice "common" "setup" "Env var to config file..."
    # Replace all ${HOME} with $HOME (in config file)
    sed -i 's@${HOME}@'"$HOME"'@' "$2" # replace ${HOME} with $HOME
    # Load config
    parse "$2" "display"
    # Check var
    check_var install_platform 1
}

clear_preset () {
    echo "hi"
}
