#!/bin/bash +x

source "../common/functions.sh"
source "./function_ibmtpm.sh"
load_preset "./config.ini"

clear_dir "$(basename -- $html_dir)"
clear_dir "$tpm_data_dir"
clear_dir "${base_dir}/ibmacs${ibmacs_ver}"
clear_dir "${base_dir}/ibmtpm${ibmtpm_ver}"
clear_dir "${base_dir}/ibmtss${ibmtss_ver}"
cfer "${base_dir}/ibmtss" "setup_ibmtpm" "remove" "Removing old ibmtss symbolic link ..."
cfer "${base_dir}/ibmtpm" "setup_ibmtpm" "remove" "Removing old ibmtpm symbolic link ..."
cfer "${base_dir}/ibmacs" "setup_ibmtpm" "remove" "Removing old ibmacs symbolic link ..."
cfer "${c_json_lib_link_dir}" "setup_ibmtpm" "remove" "Removing old c_json_lib symbolic link ..."
sudo mysql -Bse "DROP DATABASE IF EXISTS $mysql_database;"
sudo mysql -Bse "DROP DATABASE IF EXISTS machines;"

clear_preset
