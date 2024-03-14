#!/bin/bash

# common/function.sh is required to be sourced before using functions in this file

verbose=0 # 0: no verbose, 1: verbose
exit_code=1

dirname="deploy_repo"
filename="function_deploy_repo"

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
    check_var repo_ssh_link $exit_code
    check_var apt_rm_dep $exit_code
    check_var apt_dep $exit_code
    check_var pip_dep $exit_code
    check_var pip_dep_flag $exit_code
    check_var main_path $exit_code
    check_var deploy_repo $exit_code
    check_var exec_main $exit_code
}
if [ $verbose == 1 ]; then
    echo_notice "${dirname}" "${filename}" "Loaded function: load_preset"
fi

clear_preset () {
    if [ $verbose == 1 ]; then
        echo_notice "${dirname}" "${filename}" "Clearing preset..."
    fi
    unset repo_ssh_link
    unset apt_rm_dep
    unset apt_dep
    unset pip_dep
    unset pip_dep_flag
    unset main_path
    unset deploy_repo
    unset exec_main
}
if [ $verbose == 1 ]; then
    echo_notice "${dirname}" "${filename}" "Loaded function: clear_preset"
fi
