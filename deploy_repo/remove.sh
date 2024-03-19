#!/bin/bash

source "../common/functions.sh"
source "./function_deploy_repo.sh"
load_preset "./config.ini"

dirname="deploy_repo"
filename="remove"

clear_dir "mmWAVE_Radar"

cd "$(dirname "${remove_path}")"
bash "$(basename "${remove_path}")"

clear_preset
