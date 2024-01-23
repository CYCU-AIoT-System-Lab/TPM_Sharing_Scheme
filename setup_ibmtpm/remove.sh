#!/bin/bash +x

source "../common/functions.sh"
source "./function_ibmtpm.sh"
load_preset "./config.ini"

clear_dir "$html_dir/../"
clear_dir $tpm_data_dir "rmdir"
clear_dir "${base_dir}/ibmacs" "rmdir"
clear_dir "${base_dir}/ibmtpm" "rmdir"
clear_dir "${base_dir}/ibmtss" "rmdir"
cfer "${base_dir}/ibmtss" "setup_ibmtpm" "remove" "Removing old ibmtss symbolic link ..."
cfer "${base_dir}/ibmtpm" "setup_ibmtpm" "remove" "Removing old ibmtpm symbolic link ..."
cfer "${base_dir}/ibmacs" "setup_ibmtpm" "remove" "Removing old ibmacs symbolic link ..."
sudo mysql -Bse "DROP DATABASE IF EXISTS $mysql_database;"
echo_notice "setup_ibmtpm" "remove" "Clear done"

clear_preset
echo_notice "setup_ibmtpm" "remove" "Unset var done"
