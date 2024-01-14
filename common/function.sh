#!/bin/bash

dir_name="common"
file_name="function"

# Function: load terminal special characters
# Usage: load_special_chars
# No input variable
load_special_chars () {
    eval BOLD="\033[1m"
    eval BLUE="\033[34m"
    eval RED="\033[31m"
    eval GREEN="\033[32m"
    eval YELLOW="\033[33m"
    eval END="\033[0m"
    eval CLEAR_LINE="\033[2K"
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
            echo_error "common" "function" "Invalid config item, valid config item should be like this: <key> = <value>" 1
            continue
        fi
        if [[ $v == "" ]]; then
            echo_error "common" "function" "Error: $k is empty, Valid config item should be like this: <key> = <value>" 1
            continue
        fi
        #declare "$k"="$v" # This is not working in function
        #readonly "$k"="$v" # Can't be easily unset
        eval "$k"="$v"
        var_cnt=$((var_cnt+1))
        if [[ $2 == "display" ]]; then
            echo "Loaded config item: $k = $v"
        fi
    done < "$1"
    echo_notice "common" "function" "Loaded ${BOLD}${GREEN}$var_cnt${END} config items from ${BOLD}${GREEN}$1${END}"
}
echo_notice "common" "function" "Loaded function: parse"

# Function: logging date and time from executable output to file
# Usage: log_date_time "command to execute" "format" "log file" "method"
# $1: command to execute
# $2: date & time format (EX: %Y/%m/%d-%H:%M:%S)
# $3: log file
# $4: "default" or "tee"
log_date_time () {
    if [ $4 == "tee" ]; then
        $1 2>&1 | ts "[$2]" | tee -a $3 > /dev/null
    else
        $1 2>&1 | ts "[$2]" 2>&1 &>> $3
    fi
    #$1 >| $3
}
echo_notice "common" "function" "Loaded function: log_date_time"

# Function: check variable is empty and exit if empty
# Usage check_var "variable" "exit code"
# $1: variable
# $2: exit code
check_var () {
    if [ -z "${!1}" ]; then
        echo_error "common" "function" "Error: variable $1 is empty! Check your config.ini file." $2
    fi
}
echo_notice "common" "function" "Loaded function: check_var"

# Function: clear directory
# Usage: clear_dir "directory"
# $1: directory to clear content
# $2: clear this directory too (== "rmdir")
# check_var is advised to use before this function
clear_dir () {
    echo "Removing content in $1"
    sudo rm -rf "$1/*"
    if [ "$2" == "rmdir" ]; then
        sudo rmdir $1
    fi
}
echo_notice "common" "function" "Loaded function: clear_dir"
