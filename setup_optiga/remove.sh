#!/bin/bash

source "../common/functions.sh"
source "./function_optiga.sh"
load_preset "./config.ini"

dirname="setup_optiga"
filename="remove"
script=$(realpath "$0")
script_path=$(dirname "$script")

echo_notice "${dirname}" "${filename}" "Uninstalling dependencies..."
cd "$script_path/optiga-tpm-explorer/tpm2-tss/" && sudo make uninstall
cd "$script_path/optiga-tpm-explorer/tpm2-tools/" && sudo make uninstall
cd "$script_path/optiga-tpm-explorer/tpm2-abrmd/" && sudo make uninstall
cd "$script_path/optiga-tpm-explorer/tpm2-tss-engine/" && sudo make uninstall

echo_notice "${dirname}" "${filename}" "Removing directories..."
clear_dir "$script_path/optiga-tpm-explorer"

clear_preset
