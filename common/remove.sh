#!/bin/bash +x

source "./functions.sh"
source "./function_common.sh"
load_preset "./config.ini"

dirname="common"
filename="remove"

if [ ${job_config_apport} -eq 1 ]; then
    clear_dir $apport_dir
fi

if [ ${job_compile_req} -eq 1 ]; then
    if [ ${install_platform} -eq 1 ] || [ ${install_platform} -eq 4 ] || [ ${install_platform} -eq 5 ]; then
        clear_dir $cmake_dir
        clear_dir $valgrind_dir
        clear_dir /opt/openssl
    else
        :
    fi
fi

if [ ${job_setup_environment} -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running environment setup Not Implemneted Yet!"
    cd ../setup_environment
    install_platform=$install_platform bash ./remove.sh
else
    echo_warn "${dirname}" "${filename}" "Invalid Argument: $job_setup_environment ! Skipping remove of setup_environment..."
fi

if [ ${job_setup_ibmtpm} -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running ibmtpm remove..."
    cd ../setup_ibmtpm
    install_platform=$install_platform bash ./remove.sh
else
    echo_warn "${dirname}" "${filename}" "Invalid Argument: $job_setup_ibmtpm ! Skipping remove of setup_ibmtpm..."
fi

if [ ${job_socket_com} -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running socket_com remove..."
    cd ../socket_com
    install_platform=$install_platform bash ./remove.sh
else
    echo_warn "${dirname}" "${filename}" "Invalid Argument: $job_socket_com ! Skipping remove of setup_socket_com..."
fi

if [ ${job_setup_optiga} -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running optiga remove..."
    cd ../setup_optiga
    install_platform=$install_platform bash ./remove.sh
else
    echo_warn "${dirname}" "${filename}" "Invalid Argument: $job_setup_optiga ! Skipping remove of setup_optiga..."
fi

if [ ${job_deploy_repo} -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running deploy_repo remove..."
    cd ../deploy_repo
    install_platform=$install_platform bash ./remove.sh
else
    echo_warn "${dirname}" "${filename}" "Invalid Argument: $job_deploy_repo ! Skipping remove of setup_deploy_repo..."
fi

if [ ${job_setup_mbc_last} -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running setup_mbc_last remove..."
    cd ../boot
    install_platform=$install_platform bash ./remove.sh
else
    echo_warn "${dirname}" "${filename}" "Invalid Argument: $job_setup_mbc_last ! Skipping remove of setup_mbc_last..."
fi

clear_preset
