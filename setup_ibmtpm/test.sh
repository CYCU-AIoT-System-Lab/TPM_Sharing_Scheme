#!/bin/bash

conf_file="./config.ini"
source "../common/function.sh"

load_special_chars
parse "$conf_file"

echo_notice "test" "test" "test"

echo "download_dir->$download_dir"
echo "base_dir----->$base_dir"
echo "html_dir----->$html_dir"
echo "random------->$random"


