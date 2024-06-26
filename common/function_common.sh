#!/bin/bash

# common/function.sh is required to be sourced before using functions in this file

verbose=0 # 0: no verbose, 1: verbose

dirname="common"
filename="function_common"

# Function: include env to config file, load config, var check
# Usage: source "config.ini"
# Input variable: $1: config.ini
load_preset () {
    #if [ $verbose == 1 ]; then
    #    echo_notice "${dirname}" "${filename}" "Env var to config file..."
    #fi
    #sed -i 's@${HOME}@'"$HOME"'@' "$1" # replace ${HOME} with $HOME

    if [ $verbose == 1 ]; then
        echo_notice "${dirname}" "${filename}" "Loading config file..."
    fi
    parse "$1" ""

    if [ $verbose == 1 ]; then
        echo_notice "${dirname}" "${filename}" "Checking var..."
    fi
    check_var install_platform 1

    check_var job_update_src 1
    check_var job_change_all_sh_mod 1
    check_var job_enable_ssh 1
    check_var job_enable_pi_spi 1
    check_var job_config_nvim 1
    check_var job_config_apport 1
    check_var job_install_req 1
    check_var job_compile_req 1
    check_var job_reload_term 1

    check_var job_common 1
    check_var job_setup_environment 1
    check_var job_setup_ibmtpm 1
    check_var job_socket_com 1
    check_var job_setup_optiga 1
    check_var job_deploy_repo 1
    check_var job_setup_mbc_last 1
    check_var job_update_swtpm 1

    if [ $verbose == 1 ]; then
        echo_notice "$dirname" "$filename" "Loading preset..."
    fi
    # ==== version ====
    cmake_ver="3.29"        # cmake version
    cmake_build="6"         # cmake build
    valgrind_ver="3.22.0"   # valgrind version
    libssl_ver="1.1.1w"     # libssl version
    # ==== url ====
    nvim_config_url="https://raw.githubusercontent.com/belongtothenight/config-files/main/ubuntu_init.vim"  # nvim config url
    # ==== directory ====
    current_dir=$(pwd)
    nvim_dir="$HOME/.config/nvim"       # nvim installation directory
    apport_dir="$HOME/.config/apport"   # apport installation directory
    cmake_dir="/opt/cmake"              # cmake installation directory
    valgrind_dir="/opt/valgrind"        # valgrind installation directory
    libssl_dir="/opt/libssl"            # libssl installation directory
    # ==== flag ====
    wget_gflag="-q --show-progress --no-check-certificate --no-cache --no-cookies"
    make_gflag="-s"
    sudo_gflag="-E"
    apt_gflag="-q"

    # Disable jobs in this directory if not activated
    if [ ! $job_common -eq 1 ]; then
        echo_warn "$dirname" "$filename" "Skipping common..."
        job_update_src=0
        job_change_all_sh_mod=0
        job_enable_ssh=0
        job_enable_pi_spi=0
        job_config_nvim=0
        job_config_apport=0
        job_install_req=0
        job_compile_req=0
        job_reload_term=0
    fi
}
if [ $verbose == 1 ]; then
    echo_notice "${dirname}" "${filename}" "Loaded function: load_preset"
fi

clear_preset () {
    if [ $verbose == 1 ]; then
        echo_notice "${dirname}" "${filename}" "Clearing preset..."
    fi
    unset install_platform
    unset cmake_ver
    unset cmake_build
    unset valgrind_ver
    unset libssl_ver
    unset nvim_config_url
    unset nvim_dir
    unset apport_dir
    unset cmake_dir
    unset valgrind_dir
    unset libssl_dir
    unset wget_gflag
    unset make_gflag
    unset sudo_gflag
    unset apt_gflag
    unset job_install_req
    unset job_compile_req
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
    unset job_deploy_repo
    unset job_setup_mbc_last
    unset job_update_swtpm
}
if [ $verbose == 1 ]; then
    echo_notice "${dirname}" "${filename}" "Loaded function: clear_preset"
fi
