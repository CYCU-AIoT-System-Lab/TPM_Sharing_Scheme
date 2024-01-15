#!/bin/bash

# common/function.sh is required to be sourced before using functions in this file

# Function: include env to config file, load config, var check
# Usage: source "config.ini"
# Input variable: $1: config.ini
load_preset () {
    echo_notice "common" "function_common" "Env var to config file..."
    sed -i 's@${HOME}@'"$HOME"'@' "$1" # replace ${HOME} with $HOME

    echo_notice "common" "function_common" "Loading config file..."
    parse "$1" "display"

    echo_notice "common" "function_common" "Checking var..."
    check_var install_platform 1
    check_var cmake_ver 1
    check_var cmake_build 1
    check_var valgrind_ver 1
    check_var nvim_config_url 1
    check_var nvim_dir 1
    check_var apport_dir 1
    check_var cmake_dir 1
    check_var valgrind_dir 1
    check_var wget_gflag 1
    check_var make_gflag 1
    check_var sudo_gflag 1
    check_var apt_gflag 1
    check_var job_install_req 1
    check_var job_config_nvim 1
    check_var job_update_src 1
    check_var job_change_all_sh_mod 1
    check_var job_config_apport 1
    check_var job_reload_term 1
    check_var job_setup_environment 1
    check_var job_setup_ibmtpm 1
    check_var job_socket_com 1
    check_var job_setup_optiga 1
    check_var job_enable_ssh 1
    check_var job_enable_pi_spi 1
}
echo_notice "common" "function_common" "Loaded function: load_preset"

clear_preset () {
    echo_notice "common" "function_common" "Clearing preset..."
    unset install_platform
    unset cmake_ver
    unset cmake_build
    unset valgrind_ver
    unset nvim_config_url
    unset nvim_dir
    unset apport_dir
    unset cmake_dir
    unset valgrind_dir
    unset wget_gflag
    unset make_gflag
    unset sudo_gflag
    unset apt_gflag
    unset job_install_req
    unset job_config_nvim
    unset job_update_src
    unset job_change_all_sh_mod
    unset job_config_apport
    unset job_reload_term
    unset job_setup_environment
    unset job_setup_ibmtpm
    unset job_socket_com
    unset job_setup_optiga
    unset job_enable_ssh
    unset job_enable_pi_spi
}
echo_notice "common" "function_common" "Loaded function: clear_preset"
