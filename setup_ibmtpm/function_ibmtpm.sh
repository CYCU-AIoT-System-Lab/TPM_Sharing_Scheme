#!/bin/bash

# common/function.sh is required to be sourced before using functions in this file

verbose=0 # 0: no verbose, 1: verbose
exit_code=1

load_preset () {
    #if [ $verbose == 1 ]; then
    #    echo_notice "setup_ibmtpm" "function_common" "Env var to config file..."
    #fi

    if [ $verbose == 1 ]; then
        echo_notice "setup_ibmtpm" "function_common" "Loading config file..."
    fi
    parse "$1" ""

    if [ $verbose == 1 ]; then
        echo_notice "setup_ibmtpm" "function_common" "Checking var..."
    fi
    check_var install_platform $exit_code
    check_var user $exit_code
    check_var base_dir $exit_code
    check_var html_dir $exit_code
    check_var c_json_lib_dir $exit_code
    check_var c_json_lib_link_dir $exit_code
    check_var tpm_data_dir $exit_code
    check_var RSAEK_cert $exit_code
    check_var ECCEK_cert $exit_code
    check_var repo_url $exit_code
    check_var acs_demo_server_ip $exit_code
    check_var acs_demo_server_port $exit_code
    check_var acs_demo_client_ip $exit_code
    check_var tpm_command_port $exit_code
    check_var acs_port $exit_code
    check_var ibmtss_ver $exit_code
    check_var ibmtpm_ver $exit_code
    check_var ibmacs_ver $exit_code
    check_var verMode $exit_code
    check_var TPMMode $exit_code
    check_var acsMode $exit_code
    check_var SCmachineMode $exit_code
    check_var force_acs_sql_setting $exit_code
    check_var acsClientMode $exit_code
    check_var mysql_user $exit_code
    check_var mysql_password $exit_code
    check_var mysql_database $exit_code
    check_var log4j_time_format $exit_code
    check_var log4j_line_number $exit_code
    check_var wget_gflag $exit_code
    check_var make_gflag $exit_code
    check_var sudo_gflag $exit_code
    check_var bash_gflag $exit_code
    check_var tar_gflag $exit_code
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
    check_var retrieve_hardware_NV $exit_code
    check_var set_acs_sql_setting $exit_code
    check_var active_ACS_Demo_Server $exit_code
    check_var active_ACS_Demo_Client $exit_code
    check_var active_ACS_Demo_verify $exit_code
    check_var print_log_path $exit_code
    check_var open_all_logs $exit_code
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
    unset RSAEK_cert
    unset ECCEK_cert
    unset repo_url
    unset acs_demo_server_ip
    unset acs_demo_server_port
    unset acs_demo_client_ip
    unset tpm_command_port
    unset acs_port
    unset ibmtss_ver
    unset ibmtpm_ver
    unset ibmacs_ver
    unset verMode
    unset TPMMode
    unset acsMode
    unset SCmachineMode
    unset force_acs_sql_setting
    unset acsClientMode
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
    unset retrieve_hardware_NV
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
