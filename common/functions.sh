#!/bin/bash

dir_name="common"
file_name="functions"
verbose=0 # 0: no verbose, 1: verbose

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
if [ $verbose == 1 ]; then
    echo -e "${BOLD}${BLUE}[NOTICE-${dir_name}/${file_name}]${END} Loaded and Activated function: load_special_chars"
fi

# Function: notice message
# Usage: echo_notice "filename" "unit" "message"
# Input variable: $1: filename
#                 $2: unit
#                 $3: message
echo_notice () {
    echo -e "${BOLD}${BLUE}[NOTICE-$1/$2]${END} $3"
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: echo_notice"
fi

# Function: warn message
# Usage: echo_warn "filename" "unit" "message"
# Input variable: $1: filename
#                 $2: unit
#                 $3: message
echo_warn () {
    echo -e "${BOLD}${YELLOW}[WARN-$1/$2]${END} $3"
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: echo_warn"
fi

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
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: echo_error"
fi

# Function: error handling
# Usage: Add the following lines to the beginning of the script
# 1. set -eE -o functrace
# 2. trap 'failure "$LINENO" "$BASH_COMMAND" "$FUNCNAME" "$BASH_SOURCE"' ERR
# Source: https://www.gnu.org/software/bash/manual/html_node/The-Set-Builtin.html
set -eE -o functrace # need to be just before trap (this location)
failure () {
    local line_no=$1
    local bash_cmd=$2
    local bash_fun=$3
    local bash_src=$4
    echo_error "common" "function" "Error: source: $BOLD$bash_src$END, $BOLD$bash_fun$END has failed at line $BOLD$line_no$END, command: $BOLD$bash_cmd$END" 1
}
trap 'failure "$LINENO" "$BASH_COMMAND" "$FUNCNAME" "$BASH_SOURCE"' ERR
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded and Activated function: failure"
fi

# Function: continue execution when error occurs
# Usage: err_conti_exec "command string" "message 1" "message 2"
# Input variable: $1: command string (ex: "sudo apt update && echo "update done")
#                 $2: message 1
#                 $3: message 2
err_conti_exec () {
    $1 || echo_warn "$2" "$3" "Warning: \"$1\" failed, but continue execution..."
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded and Activated function: err_conti_exec"
fi

# Function: exit execution when error occurs
# Usage: err_exit_exec "command string" "message 1" "message 2" exit code
# Input variable: $1: command string (ex: "sudo apt update && echo "update done")
#                 $2: message 1
#                 $3: message 2
#                 $4: exit code
err_exit_exec () {
    $1 || echo_warn "$2" "$3" "Error: \"$1\" failed, but continue execution..." $4; exit $2
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded and Activated function: err_exit_exec"
fi

# Function: retry execution when error occurs
# Usage: err_retry_exec "command string" "interval in second" "retry times" "message 1" "message 2" "exit code
# Input variable: $1: command string (ex: "sudo apt update && echo "update done")
#                 $2: interval in second (ex: 5, true to skip interval)
#                 $3: retry times (ex: 3)
#                 $4: message 1
#                 $5: message 2
#                 $6: exit code
err_retry_exec () {
    local retry_cnt=0
    until
        $1
    do
        retry_cnt=$((retry_cnt+1))
        if [ $retry_cnt -eq $3 ]; then
            echo_error "$4" "$5" "Error: \"$1\" failed after $3 retries" $6
        fi
        echo_warn "$4" "$5" "Warning: \"$1\" failed, retrying in $2 seconds..."
        sleep $2
    done
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded and Activated function: err_retry_exec"
fi

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
        if [ -z "${!k}" ]; then
            :
        else
            echo_warn "common" "function" "Warning: $k is already set, will not overwrite it"
            continue
        fi
        #declare "$k"="$v" # This is not working in function
        #readonly "$k"="$v" # Can't be easily unset
        eval "$k"='$v'
        var_cnt=$((var_cnt+1))
        if [[ $2 == "display" ]]; then
            echo "Loaded config item: $k = $v"
        fi
    done < "$1"
    echo_notice "common" "function" "Loaded ${BOLD}${GREEN}$var_cnt${END} config items from ${BOLD}${GREEN}$1${END}"
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: parse"
fi

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
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: log_date_time"
fi

# Function: check variable is empty and exit if empty
# Usage check_var "variable" "exit code"
# $1: variable
# $2: exit code
check_var () {
    if [ -z "${!1}" ]; then
        echo_error "common" "function" "Error: variable $1 is empty! Check your config.ini file." $2
    fi
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: check_var"
fi

# Function: clear directory
# Usage: clear_dir "directory"
# $1: directory to clear content
# check_var is advised to use before this function
clear_dir () {
    echo "Removing content in $1"
    err_conti_exec "sudo rm -rf $1/" "common" "functions_clear_dir"
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: clear_dir"
fi

# Function: install package with apt
# Usage: aptins "package name"
# $1: package name
aptins () {
    echo_notice "common" "setup" "Installing ${BOLD}${GREEN}$1${END}..."
    err_retry_exec "sudo apt $apt_gflag -o DPkg::Lock::Timeout=300 install $1 -y" 1 5 "common" "functions_aptins" 1
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: aptins"
fi

# Function: open new gnome-terminal and execute command
# Usage: newGterm "tab_name" "bash flag" "command" "finish action"
# $1: tab name  (ex: "Brower Tab")
# $2: bash flag (ex: "-x", can be empty "")
# $3: command   (ex: "firefox")
# $4: finish action (ex: 1: sleep infinitly, 2: exec bash)
newGterm () {
    if [ $4 == 1 ]; then
        gnome-terminal -t "$1" --active -- bash $2 -c "$3; echo -e \"\nprogram terminated\nctrl+c to exit\n\"; sleep infinity"
    elif [ $4 == 2 ]; then
        gnome-terminal -t "$1" --active -- bash $2 -c "$3; exec bash"
    else
        echo_error "common" "function" "Error: Invalid finish action, should be 1 or 2" 1
    fi
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: newGterm"
fi

# Function: open new lxterminal and execute command
# Usage: newLXterm "tab_name" "command" "finish action"
# $1: tab name  (ex: "Brower Tab")
# $2: command   (ex: "firefox")
# $3: finish action (ex: 1: sleep infinitly, 2: exec bash)
newLXterm () {
    if [ $3 == 1 ]; then
        lxterminal -t "$1" -e "$2; echo -e \"\nprogram terminated\nctrl+c to exit\n\"; sleep infinity" &
    elif [ $3 == 2 ]; then
        lxterminal -t "$1" -e "$2; exec bash" &
    else
        echo_error "common" "function" "Error: Invalid finish action, should be 1 or 2" 1
    fi
    sleep 1 # wait for lxterminal to open
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: newLXterm"
fi

# Check file exist and remove it
# Usage: cfer "file" "message1" "message2" "message3"
# Input variable: $1: file name
#                 $2: message1
#                 $3: message2
#                 $4: message3
cfer () {
    if [ -f "$1" ]; then
        echo_notice "$2" "$3" "$4"
        err_conti_exec "sudo rm $1" "common" "functions_cfer"
    elif [ -L "$1" ]; then
        echo_notice "$2" "$3" "$4"
        sudo unlink "$1"
        err_conti_exec "sudo rm $1" "common" "functions_cfer"
    elif [ -d "$1" ]; then
        echo_notice "$2" "$3" "$4"
        err_conti_exec "sudo rmdir $1" "common" "functions_cfer"
    fi
}
if [ $verbose == 1 ]; then
    echo_notice "common" "function" "Loaded function: cfer"
fi
