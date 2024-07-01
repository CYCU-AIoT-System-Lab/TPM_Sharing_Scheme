#!/bin/bash

script=$(realpath "$0")
script_path=$(dirname "$script")
dirname=$(basename "$script_path")
filename=$(basename "$0")
source "../common/functions.sh"
source "./function_acsroutine.sh"
load_preset "./config.ini"

# clone libtrace install program
#   - https://github.com/belongtothenight/ACN_Code/releases
setup_libtrace () {
    echo_notice "$dirname" "$filename" "libtrace setting up"
}

# install & compile SERVER program:
#   - [ ] ACS DB parsing
#   - [ ] detect anomaly client traffic
#   - [ ] block anomaly client traffic
server_setup () {
    echo_notice "$dirname" "$filename" "server-side setting up"
}

# execute SERVER program
#   - [O] open ACS demo server
#   - [O] open ACS demo server webpage
#   - [ ] routine ACS DB parsing for state change
#   - [ ] constant traffic monitoring
#   - [ ] block client if top two condition fails
server_exec () {
    #1
    echo_notice "$dirname" "$filename" "ACS server starting"
    lc1="source $script_path/../common/functions.sh"
    lc2="echo_notice \"$dirname\" \"$filename\" 'Launching ACS server'"
    lc3="cd $base_dir/ibmacs"
    lc4="export LD_LIBRARY_PATH=/opt/ibmtss/utils:$LD_LIBRARY_PATH"
    lc5="export ACS_PORT=$acs_port ACS_SQL_USERID=\"$mysql_user\" ACS_SQL_PASSWORD=\"$mysql_password\""
    lc6="./server -vv -root $base_dir/ibmtss/utils/certificates/rootcerts.txt -imacert imakey.der"
    #lc6="log_date_time \"./server -vv -root $base_dir/ibmtss/utils/certificates/rootcerts.txt -imacert imakey.der\" \"$log4j_time_format\" \"$base_dir/ibmacs/serverenroll.log4j\" \"default\""
    newLXterm "ACS SERVER" "sudo bash -c \"$lc1; $lc2; $lc3; $lc4; $lc5; service apache2 restart; $lc6 || :\"" 1
    #sudo bash -c "$lc2; $lc3; $lc4; $lc5; $lc6 || { pkill apache2 && service apache2 restart && $lc6; }" # debug use

    echo_notice "$dirname" "$filename" "ACS website starting"
    lc1="source $PWD/../common/functions.sh"
    lc2="chromium-browser --new-window localhost/acs"
    newLXterm "ACS Webpage" "$lc1; $lc2" 1

    #2
    #3
    #4
}

# install & compile CLIENT program:
#   - send attestation info to server
#   - detect anomaly server traffic
#   - block anomaly server traffic
client_setup () {
    echo_notice "$dirname" "$filename" "client-side setting up"
}

# execute CLIENT program
#   - routine send attestation info
#   - constant traffic monitoring
#   - block client if top one condition fails
client_exec () {
    echo_notice "$dirname" "$filename" "client-side executing"
}

if [ $job_setup_libtrace    -eq 1 ]; then setup_libtrace; fi
if [ $is_server -eq 1 ]; then
    if [ $job_setup -eq 1 ]; then server_setup; fi
    if [ $job_exec  -eq 1 ]; then server_exec; fi
else
    if [ $job_setup -eq 1 ]; then client_setup; fi
    if [ $job_exec  -eq 1 ]; then client_exec; fi
fi

echo_notice "$dirname" "$filename" "$script finished"
