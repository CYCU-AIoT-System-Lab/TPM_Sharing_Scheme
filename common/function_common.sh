#!/bin/bash

# Function: load common function, include env to config file, load config, var check
# Usage: source "./function.sh" "config.ini"
# Input variable: $1: common/function.sh
#                 $2: config.ini
load_preset () {
    # load common function
    source "$1"
    # env to config file
    sed -i 's@${HOME}@'"$HOME"'@' "$2" # replace ${HOME} with $HOME
    # load config
    parse "$2" "display"
    # var check
    check_var install_platform 1
}

clear_preset () {
    
}
