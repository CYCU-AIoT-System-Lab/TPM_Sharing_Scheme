#!/bin/bash

source "../common/functions.sh"
source "./function_optiga.sh"
load_preset "./config.ini"

dirname="setup_optiga"
filename="remove"

echo_notice "${dirname}" "${filename}" "Removing directories..."
clear_dir "./optiga-tpm-explorer"

clear_preset
