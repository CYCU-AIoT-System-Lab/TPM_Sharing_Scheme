#!/bin/bash
set -e

verbose=0 # 1 for verbose, 0 for silent
script=$(realpath "$0")
script_path=$(dirname "$script")
dirname=$(basename "$script_path")
filename=$(basename "$0")

load_preset () {
    source "../setup_ibmtpm/function_ibmtpm.sh"
    load_preset "../setup_ibmtpm/config.ini" # If this file is not found, generate it through running setup_ibmtpm

    if [ $verbose -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Loading config file..."
    fi
    parse "$1" ""

    if [ $verbose -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Checking var..."
    fi
    check_var install_platform 1

    check_var use_swtpm 1

    if [ $verbose -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Loading preset..."
    fi
    #SWTPM_SERVER_PORT=2321
    SWTPM_SERVER_PORT=$tpm_command_port
    #SWTPM_CTRL_PORT=2322
    SWTPM_CTRL_PORT=$tpm_socket_port
    SWTPM_SOCKET_DEVICE=$swtpm_socket_device
    clear_preset # clear preset of setup_ibmtpm

    USE_SWTPM=$use_swtpm
    SWTPM_NVM="$PWD/swtpm_nvm.data"

    VERBOSE=true

    TPM2_AUTH_OWNER="owner123"
    TPM2_AUTH_ENDORSEMENT="endorsement123"
    TPM2_AUTH_LOCKOUT="lockout123"
    TPM2_AUTH_ATTRIBUTE="authwrite|authread|read_stclear"
    TPM2_AUTH_NV="nv123"
    TPM2_NVM_INDEX="0x1500016"

    HASH_SCRIPT="$PWD/hash.sh"
    HASH_TARGET="$PWD/dir_list.txt"
    HASH_DEMO_DATA="$PWD/hash_target.txt"
    HASH="sha256" # can be sha1, sha256, sha384, sha512
    PCR_IDX_INIT_ZERO=0
    PCR_IDX_MIN=0
    PCR_IDX_MAX=23
    INITIAL_ZERO_PCR="0000000000000000000000000000000000000000000000000000000000000000"
    
    SYSTEMD_DIR="/etc/systemd/system"

    SERVICE_FILE_1="swtpm.service"
    SERVICE_FILE_2="activate_swtpm.service"
    SERVICE_FILE_3="mbc_last.service"

    LAUNCH_SCRIPT_1="$PWD/launch_swtpm.sh"
    LAUNCH_SCRIPT_2="$PWD/activate_swtpm.sh"
    LAUNCH_SCRIPT_3="$PWD/mbc_last.sh"

    if [ $USE_SWTPM -eq 1 ]; then
        # SWTPM socket interface env var
        export TPM2TOOLS_TCTI="swtpm:port=$SWTPM_SERVER_PORT"
    fi
}
if [ $verbose -eq 1 ]; then
    echo_notice "$dirname" "$filename" "Loaded function: load_preset"
fi

# Note: `clear_preset` function is not yet implemented
