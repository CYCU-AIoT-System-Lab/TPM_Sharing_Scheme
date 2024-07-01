#!/bin/bash +x

source "../common/functions.sh"
source "./function_ibmtpm.sh"
load_preset "./config.ini"
script=$(realpath "$0")
script_path=$(dirname "$script")

#echo_notice "${dirname}" "${filename}" "Uninstalling dependencies..."
#cd "${base_dir}/ibmacs${ibmacs_ver}/acs" && sudo make uninstall    # no uninstall target provided
#cd "${base_dir}/ibmtpm${ibmtpm_ver}/src" && sudo make uninstall    # no uninstall target provided
#cd "${base_dir}/ibmtss${ibmtss_ver}" && sudo make uninstall        # no uninstall target provided

if ! [ $install_platform -eq 3 ]; then
    echo_notice "${dirname}" "${filename}" "Removing swtpm device ..."
    sudo rm -rf $swtpm_socket_device
fi

echo_notice "${dirname}" "${filename}" "Removing directories..."
clear_dir "$(basename -- $html_dir)"
clear_dir "$tpm_data_dir"
clear_dir "${base_dir}/ibmacs${ibmacs_ver}"
clear_dir "${base_dir}/ibmtpm${ibmtpm_ver}"
clear_dir "${base_dir}/ibmtss${ibmtss_ver}"

echo_notice "${dirname}" "${filename}" "Removing symlink..."
cfer "${base_dir}/ibmtss" "setup_ibmtpm" "remove" "Removing old ibmtss symbolic link ..."
cfer "${base_dir}/ibmtpm" "setup_ibmtpm" "remove" "Removing old ibmtpm symbolic link ..."
cfer "${base_dir}/ibmacs" "setup_ibmtpm" "remove" "Removing old ibmacs symbolic link ..."
cfer "${c_json_lib_lin_linkk_dir}" "setup_ibmtpm" "remove" "Removing old c_json_lib symbolic link ..."

echo_notice "${dirname}" "${filename}" "Removing DB..."
sudo mysql -Bse "DROP DATABASE IF EXISTS $mysql_database;"
sudo mysql -Bse "DROP DATABASE IF EXISTS machines;"

clear_preset
sudo rm $script_path/config.ini
