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

    #check_var acs_demo_server_ip 1
    #check_var acs_port 1

    #check_var acs_demo_client_ip 1
    #check_var tpm_command_port 1
    #check_var tpm_socket_port 1

    check_var interval 1

    if [ $verbose -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Loading preset..."
    fi
    source "$script_path/../setup_ibmtpm/function_ibmtpm.sh"
    setup_ibmtpm_config="$script_path/../setup_ibmtpm/config.ini"
    if ! [ -f "$setup_ibmtpm_config" ]; then
        echo_error "$dirname" "$filename" "Submodule \`setup_ibmtpm\` has not been installed!" 1
    fi
    load_preset "$setup_ibmtpm_config"

    # mysql
    #mysql_user="tpm2ACS"
    MYSQL_USER=$mysql_user
    #mysql_password="123456"
    MYSQL_PASSWORD=$mysql_password
    #mysql_database="tpm2"
    MYSQL_DATABASE=$mysql_database
    MYSQL_HOST=$acs_demo_server_ip
    MYSQL_PORT=0 # default

    # format
    #log4j_time_format="%Y/%m/%d-%H:%M:%S"
    LOG4J_TIME_FORMAT=$log4j_time_format

    # path
    #base_dir="/opt"
    BASE_DIR=$base_dir

    # network
    ACS_DEMO_SERVER_IP=$acs_demo_server_ip
    ACS_PORT=$acs_port
    ACS_DEMO_CLIENT_IP=$acs_demo_client_ip

    # server-side must be raspberry pi with TPM9670 installed
    if [ $is_server -eq 1 ]; then
        if [ $install_platform -ne 3 ]; then
            echo_error "$dirname" "$filename" "Platform $install_platform is not supported, only 3. Please check your configuration" 1
        fi
    fi

    clear_preset # clear preset from setup_ibmtpm
}
if [ $verbose -eq 1 ]; then
    echo_notice "$dirname" "$filename" "Loaded function: load_preset"
fi

# Note: `clear_preset` function is not yet implemented
