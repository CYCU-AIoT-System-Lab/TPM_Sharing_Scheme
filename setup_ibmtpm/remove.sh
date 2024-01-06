#!/bin/bash +x

source ../common/function.sh
parse "./config.ini" "display"

if [ -z ${base_dir+x} ]; then
    echo "common/function.sh or setup_ibmtpm/config.ini is not found"
    exit 1
fi

# $1: directory to clear content
# $2: clear this directory too (== "rmdir")
clear_dir () {
    echo "Removing content in $1"
    cd $1
    sudo rm -rf *
    if [ "$2" == "rmdir" ]; then
        sudo rmdir $1
    fi
}

clear_dir $base_dir
clear_dir "$html_dir/../"
clear_dir $c_json_lib_dir "rmdir"
sudo mysql -Bse "DROP DATABASE IF EXISTS $mysql_database;"

echo_notice "setup_ibmtpm" "remove" "Done"
