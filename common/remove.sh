#!/bin/bash +x

source "./function.sh"
source "./function_common.sh"
load_preset "./config.ini"

clear_dir $apport_dir "rmdir"
clear_dir $cmake_dir "rmdir"
clear_dir $valgrind_dir "rmdir"
echo_notice "common" "remove" "Clear done"

if [ ${job_setup_environment} -eq 1 ]; then
    echo_notice "common" "remove" "Running environment setup Not Implemneted Yet!"
    cd ../setup_environment
    install_platform=$install_platform bash ./remove.sh
else
    echo_warn "common" "remove" "Invalid Argument: $job_setup_environment ! Skipping setup_environment..."
fi

if [ ${job_setup_ibmtpm} -eq 1 ]; then
    echo_notice "common" "remove" "Running ibmtpm setup..."
    cd ../setup_ibmtpm
    install_platform=$install_platform bash ./remove.sh
else
    echo_warn "common" "remove" "Invalid Argument: $job_setup_ibmtpm ! Skipping setup_ibmtpm..."
fi

if [ ${job_socket_com} -eq 1 ]; then
    echo_notice "common" "remove" "Running socket_com setup..."
    cd ../socket_com
    install_platform=$install_platform bash ./remove.sh
else
    echo_warn "common" "remove" "Invalid Argument: $job_socket_com ! Skipping setup_socket_com..."
fi

if [ ${job_setup_optiga} -eq 1 ]; then
    echo_notice "common" "remove" "Running optiga setup..."
    cd ../setup_optiga
    install_platform=$install_platform bash ./remove.sh
else
    echo_warn "common" "remove" "Invalid Argument: $job_setup_optiga ! Skipping setup_optiga..."
fi

clear_preset
