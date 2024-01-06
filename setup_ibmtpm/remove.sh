#!/bin/bash +x

source ../common/function.sh
parse "./config.ini" "display"

clear_dir $base_dir
clear_dir "$html_dir/../"
clear_dir $tpm_data_dir "rmdir"
sudo mysql -Bse "DROP DATABASE IF EXISTS $mysql_database;"

echo_notice "setup_ibmtpm" "remove" "Done"
