#!/bin/bash

source "../common/functions.sh"
source "./function_optiga.sh"
load_preset "./config.ini"

echo_notice "common" "remove" "Removing directories..."
clear_dir "./optiga-tpm-explorer" "rmdir"

clear_preset
