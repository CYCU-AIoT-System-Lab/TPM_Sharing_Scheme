#!/bin/bash +x

source ../common/function.sh
parse "./config.ini" "display"

check_var base_dir 1
check_var html_dir 1
check_var tpm_data_dir 1
check_var mysql_database 1
clear_dir $base_dir
clear_dir "$html_dir/../"
clear_dir $tpm_data_dir "rmdir"
sudo mysql -Bse "DROP DATABASE IF EXISTS $mysql_database;"

echo_notice "setup_ibmtpm" "remove" "Done"
