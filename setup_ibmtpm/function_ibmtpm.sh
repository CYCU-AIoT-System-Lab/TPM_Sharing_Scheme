#!/bin/bash

# common/function.sh is required to be sourced before using functions in this file

verbose=0 # 0: no verbose, 1: verbose
exit_code=1

dirname="setup_ibmtpm"
filename="function_ibmtpm"

load_preset () {
    #if [ $verbose == 1 ]; then
    #    echo_notice "${dirname}" "${filename}" "Env var to config file..."
    #fi

    if [ $verbose == 1 ]; then
        echo_notice "${dirname}" "${filename}" "Loading config file..."
    fi
    parse "$1" ""

    if [ $verbose == 1 ]; then
        echo_notice "${dirname}" "${filename}" "Checking var..."
    fi
    check_var install_platform $exit_code
    check_var acs_demo_server_ip $exit_code
    check_var acs_demo_server_port $exit_code
    check_var acs_demo_client_ip $exit_code
    check_var tpm_command_port $exit_code
    check_var tpm_socket_port $exit_code
    check_var acs_port $exit_code

    check_var new_swtpm $exit_code
    if [ $new_swtpm -ne 1 ]; then
        echo_warn "${dirname}" "${filename}" "Using legacy swtpm..."
        echo_notice "${dirname}" "${filename}" "If you want to use updated swtpm, please make sure submodule \`update_swtpm\` is installed, and set \`new_swtpm=1\`"
    else
        echo_notice "${dirname}" "${filename}" "Using updated swtpm..."
    fi


    check_var verMode $exit_code
    check_var TPMMode $exit_code
    check_var acsMode $exit_code
    check_var SCmachineMode $exit_code
    check_var force_acs_sql_setting $exit_code
    check_var install_req $exit_code
    check_var setup_ibmtpmtss_env $exit_code
    check_var compile_ibmtpmtss $exit_code
    check_var setup_ibmswtpm_env $exit_code
    check_var compile_ibmswtpm $exit_code
    check_var setup_ibmacs_env $exit_code
    check_var compile_ibmacs $exit_code
    check_var open_demo_webpage $exit_code
    check_var generate_CA $exit_code
    check_var activate_TPM_server $exit_code
    check_var activate_TPM_client $exit_code
    check_var generate_EK $exit_code
    check_var retrieve_EK $exit_code
    check_var set_acs_sql_setting $exit_code
    check_var active_ACS_Demo_Server $exit_code
    check_var active_ACS_Demo_Client $exit_code
    check_var active_ACS_Demo_verify $exit_code
    check_var print_log_path $exit_code
    check_var open_all_logs $exit_code

    if [ $verbose == 1 ]; then
        echo_notice "$message1" "$message2" "Loading preset..."
    fi
    # ==== user ====
    user="user"
    # ==== version ====
    ibmtss_ver="1.6.0" # Platform 1: 2.1.1, Platform 3: 1.6.0
    ibmtpm_ver="1682"
    ibmacs_ver="1658"
    # ==== mysql ====
    mysql_user="tpm2ACS"
    mysql_password="123456"
    mysql_database="tpm2"
    # ==== network ====
    repo_url="https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/main/setup_ibmtpm"
    # ==== format ====
    log4j_time_format="%Y/%m/%d-%H:%M:%S" # default: %Y/%m/%d-%H:%M:%S
    log4j_line_number="100"
    # ==== path ====
    base_dir="/opt"
    html_dir="/var/www/html/acs"
    c_json_lib_dir="/usr/include/json-c"
    c_json_lib_link_dir="/usr/include/json"
    tpm_data_dir="/home/$user/tpm2"
    swtpm_socket_device="/dev/swtpm"
    # ==== flag ====
    wget_gflag="-q --show-progress --no-check-certificate --no-cache --no-cookies"
    make_gflag=""
    sudo_gflag="-E"
    bash_gflag="-x"
    tar_gflag="-zxf"
}
if [ $verbose == 1 ]; then
    echo_notice "setup_ibmtpm" "function_ibmtpm" "Loaded function: load_preset"
fi

clear_preset () {
    if [ $verbose == 1 ]; then
        echo_notice "setup_ibmtpm" "function_ibmtpm" "Clearing preset..."
    fi
    unset install_platform
    unset user
    unset base_dir
    unset html_dir
    unset c_json_lib_dir
    unset c_json_lib_link_dir
    unset tpm_data_dir
    unset repo_url
    unset acs_demo_server_ip
    unset acs_demo_server_port
    unset acs_demo_client_ip
    unset tpm_command_port
    unset tpm_socket_port
    unset acs_port
    unset ibmtss_ver
    unset ibmtpm_ver
    unset ibmacs_ver
    unset verMode
    unset TPMMode
    unset acsMode
    unset SCmachineMode
    unset force_acs_sql_setting
    unset mysql_user
    unset mysql_password
    unset mysql_database
    unset log4j_time_format
    unset log4j_line_number
    unset wget_gflag
    unset make_gflag
    unset sudo_gflag
    unset bash_gflag
    unset tar_gflag
    unset install_req
    unset setup_ibmtpmtss_env
    unset compile_ibmtpmtss
    unset setup_ibmswtpm_env
    unset compile_ibmswtpm
    unset setup_ibmacs_env
    unset compile_ibmacs
    unset open_demo_webpage
    unset generate_CA
    unset activate_TPM_server
    unset activate_TPM_client
    unset generate_EK
    unset retrieve_EK
    unset set_acs_sql_setting
    unset active_ACS_Demo_Server
    unset active_ACS_Demo_Client
    unset active_ACS_Demo_verify
    unset print_log_path
    unset open_all_logs
}
if [ $verbose == 1 ]; then
    echo_notice "setup_ibmtpm" "function_ibmtpm" "Loaded function: clear_preset"
fi
