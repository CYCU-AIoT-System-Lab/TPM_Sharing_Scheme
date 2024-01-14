#!/bin/bash +x

source "../common/function.sh"
source "./function_ibmtpm.sh"
load_preset "./config.ini"

clear_dir $base_dir
clear_dir "$html_dir/../"
clear_dir $tpm_data_dir "rmdir"
sudo mysql -Bse "DROP DATABASE IF EXISTS $mysql_database;"
echo_notice "setup_ibmtpm" "remove" "Clear done"

clear_preset
echo_notice "setup_ibmtpm" "remove" "Unset var done"
