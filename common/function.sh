#!/bin/bash

dir_name="common"
file_name="function"

# Function: load terminal special characters
# Usage: load_special_chars
# No input variable
load_special_chars () {
    readonly BOLD="\033[1m"
    readonly BLUE="\033[34m"
    readonly RED="\033[31m"
    readonly GREEN="\033[32m"
    readonly YELLOW="\033[33m"
    readonly END="\033[0m"
    readonly CLEAR_LINE="\033[2K"
}
load_special_chars
echo -e "${BOLD}${BLUE}[NOTICE-${dir_name}/${file_name}]${END} Loaded and Activated function: load_special_chars"

# Function: notice message
# Usage: echo_notice "filename" "unit" "message"
# Input variable: $1: filename
#                 $2: unit
#                 $3: message
echo_notice () {
    echo -e "${BOLD}${BLUE}[NOTICE-$1/$2]${END} $3"
}
echo_notice "common" "function" "Loaded function: echo_notice"

# Function: warn message
# Usage: echo_warn "filename" "unit" "message"
# Input variable: $1: filename
#                 $2: unit
#                 $3: message
echo_warn () {
    echo -e "${BOLD}${YELLOW}[WARN-$1/$2]${END} $3"
}
echo_notice "common" "function" "Loaded function: echo_warn"

# Function: error message
# Usage: echo_error "filename" "unit" "message" "exit code"
# Input variable: $1: filename
#                 $2: unit
#                 $3: message
#                 $4: exit code
echo_error () {
    echo -e "${BOLD}${RED}[ERROR-$1/$2]${END} $3"
    exit $4
}
echo_notice "common" "function" "Loaded function: echo_error"

# Function: parse config file
# Usage: parse "config.ini"
# Input variable: $1: config filepath
#                 $2: "display" display config item
parse () {
    var_cnt=0
    while read -r k e v; do
        if [[ $k == \#* ]]; then
            continue
        fi
        if [[ $k == "" ]]; then
            continue
        fi
        if [[ $k == "["* ]]; then
            continue
        fi
        if [[ $e != "=" ]]; then
            echo "Error: $k is not a valid config item."
            echo "Valid config item should be like this: key = value"
            continue
        fi
        if [[ $v == "" ]]; then
            echo "Error: $k is empty."
            echo "Valid config item should be like this: key = value"
            continue
        fi
        #declare "$k"="$v" # This is not working in function
        readonly "$k"="$v"
        var_cnt=$((var_cnt+1))
        if [[ $2 == "display" ]]; then
            echo "Loaded config item: $k = $v"
        fi
    done < "$1"
    echo_notice "common" "function" "Loaded ${BOLD}${GREEN}$var_cnt${END} config items from ${BOLD}${GREEN}$1${END}"
}
echo_notice "common" "function" "Loaded function: parse"
