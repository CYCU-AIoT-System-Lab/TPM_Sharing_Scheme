#!/bin/bash

source "../common/functions.sh"
source "./function_deploy_repo.sh"
load_preset "./config.ini"

dirname="deploy_repo"
filename="setup"

deploy_repo () {
    echo_notice "${dirname}" "${filename}-deploy_repo" "Verifying SSH link ..."
    if [[ $repo_ssh_link == git@github.com:* ]]; then
        :
    else
        echo_error "${dirname}" "${filename}-deploy_repo" "SSH link is not valid"
        exit 1
    fi

    echo_notice "${dirname}" "${filename}-deploy_repo" "Cloning repository ..."
    err_retry_exec "git clone $repo_ssh_link" 1 5 "${dirname}" "${filename}-deploy_repo"

    echo_notice "${dirname}" "${filename}-deploy_repo" "Solving dependencies ..."
    err_retry_exec "sudo apt remove ${apt_rm_dep}" 1 5 "${dirname}" "${filename}-deploy_repo"
    err_retry_exec "python3 -m pip install --upgrade pip" 1 5 "${dirname}" "${filename}-deploy_repo"
    err_retry_exec "aptins ${apt_dep}" 1 5 "${dirname}" "${filename}-deploy_repo"
    err_retry_exec "python3 -m pip install ${pip_dep_flag} ${pip_dep}" 1 5 "${dirname}" "${filename}-deploy_repo"
}

exec_main () {
    echo_notice "${dirname}" "${filename}" "Running main script ..."
    python3 ${main_path}
}

echo_notice "${dirname}" "${filename}" "Running setup script ..."

if [ $deploy_repo == 1 ]; then deploy_repo; fi
if [ $exec_main   == 1 ]; then exec_main;   fi

clear_preset

echo_notice "${dirname}" "${filename}" "Setup complete"
