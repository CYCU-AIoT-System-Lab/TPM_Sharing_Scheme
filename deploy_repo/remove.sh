#!/bin/bash

source "../common/functions.sh"
source "./function_deploy_repo.sh"
load_preset "./config.ini"

dirname="deploy_repo"
filename="remove"

cd "$(dirname "${remove_path}")"
bash "$(basename "${remove_path}")"

clear_dir "mmWAVE_Radar"

clear_preset
