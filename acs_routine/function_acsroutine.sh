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
    check_var is_server 1

    check_var job_setup_libtrace 1
    check_var job_setup 1
    check_var job_exec 1

    check_var acs_demo_server_ip 1
    check_var acs_port 1

    check_var acs_demo_client_ip 1
    check_var tpm_command_port 1
    check_var tpm_socket_port 1

    if [ $verbose -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Loading preset..."
    fi
    # mysql
    mysql_user="tpm2ACS"
    mysql_password="123456"
    mysql_database="tpm2"
    # format
    log4j_time_format="%Y/%m/%d-%H:%M:%S"
    log4j_line_number=100
    # path
    base_dir="/opt"

    # server-side must be raspberry pi with TPM9670 installed
    if [ $is_server -eq 1 ]; then
        if [ $install_platform -ne 3 ]; then
            echo_error "$dirname" "$filename" "No supported platform, please check your configuration" 1
        fi
    fi
}
if [ $verbose -eq 1 ]; then
    echo_notice "$dirname" "$filename" "Loaded function: load_preset"
fi

# Note: `clear_preset` function is not yet implemented
