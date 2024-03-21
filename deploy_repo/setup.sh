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

    echo_notice "${dirname}" "${filename}-deploy_repo" "Executing setup script ..."
    current_path="$(pwd)"
    cd "$(pwd)$(dirname "${setup_path}")"
    bash "$(basename "${setup_path}")"
    cd "${current_path}"
}

exec_main () {
    echo_notice "${dirname}" "${filename}" "Running main script ..."
    current_path="$(pwd)"
    cd "$(dirname "${main_path}")"
    bash "$(basename "${main_path}")"
    cd "${current_path}"
}

echo_notice "${dirname}" "${filename}" "Running setup script ..."

if [ $deploy_repo == 1 ]; then deploy_repo; fi
if [ $exec_main   == 1 ]; then exec_main;   fi

clear_preset

echo_notice "${dirname}" "${filename}" "Setup complete"
