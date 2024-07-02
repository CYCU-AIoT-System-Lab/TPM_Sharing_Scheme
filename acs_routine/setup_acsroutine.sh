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
    lc3="cd $BASE_DIR/ibmacs"
    lc4="export LD_LIBRARY_PATH=$BASE_DIR/ibmtss/utils:$LD_LIBRARY_PATH ACS_PORT=$ACS_PORT ACS_SQL_USERID=\"$MYSQL_USER\" ACS_SQL_PASSWORD=\"$MYSQL_PASSWORD\" TPM_INTERFACE_TYPE=dev"
    lc5="./server -vv -root $BASE_DIR/ibmtss/utils/certificates/rootcerts.txt -imacert imakey.der"
    #lc5="log_date_time \"./server -vv -root $BASE_DIR/ibmtss/utils/certificates/rootcerts.txt -imacert imakey.der\" \"$LOG4J_TIME_FORMAT\" \"$BASE_DIR/ibmacs/serverenroll.log4j\" \"default\""
    newLXterm "ACS SERVER" "sudo bash -c \"$lc1; $lc2; $lc3; $lc4; service apache2 restart; $lc5 || :\"" 1
    #sudo bash -c "$lc2; $lc3; $lc4; $lc5 || { pkill apache2 && service apache2 restart && $lc5; }" # debug use

    echo_notice "$dirname" "$filename" "ACS website starting"
    lc1="source $PWD/../common/functions.sh"
    lc2="chromium-browser --new-window localhost/acs"
    newLXterm "ACS Webpage" "$lc1; $lc2" 1

    #2
    #3
    #4
}

# install & compile CLIENT program:
#   - [ ] send attestation info to server
#   - [ ] detect anomaly server traffic
#   - [ ] block anomaly server traffic
enroll_client=0 # This is not neccesary since it is performed by setup_ibmtpm
client_setup () {
    if [ $enroll_client -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Enrolling client"
        sudo bash -c "\
            cd $BASE_DIR/ibmacs; \
            export LD_LIBRARY_PATH=\"$BASE_DIR/ibmtss/utils:$LD_LIBRARY_PATH\"; \
            $BASE_DIR/ibmacs/clientenroll -alg ec -vv -ho $ACS_DEMO_SERVER_IP -ma $ACS_DEMO_CLIENT_IP -co akeccert.pem" || { echo "please inspect the error message! if it is \`already exists machine error\`, this can be ignored."; }
    fi
}

# execute CLIENT program
#   - [O] routine send attestation info
#   - [ ] constant traffic monitoring
#   - [ ] block client if top one condition fails
approach=2
client_exec () {
    echo_notice "$dirname" "$filename" "client-side executing"
    echo "approach: $approach"
    if [ $approach -eq 1 ]; then
        # this approach tries to execute client only with systemd managed swtpm
        # but it seems to need swtpm not handeled by systemd
        p1="$BASE_DIR/ibmacs/tpm2bios.log"
        p2="$BASE_DIR/ibmacs/imasig.log"
        # cmd 1&2 can add "-v" flag for verbose
            #$BASE_DIR/ibmtss/utils/createekcert -rsa 2048 -cakey cakey.pem -capwd rrrr -vv
            #$BASE_DIR/ibmtss/utils/createekcert -ecc nistp256 -cakey cakeyecc.pem -capwd rrrr -caalg ec -vv
            #$BASE_DIR/ibmtss/utils/nvread -ha 01c00002 | awk 'NR==1 {next} {print}' | xxd -r -ps | base64 | sed -e '1i -----BEGIN CERTIFICATE-----' -e '$a -----END CERTIFICATE-----' > VMW_EK_CACERT.pem
            #$BASE_DIR/ibmtss/utils/nvread -ha 01c0000a | awk 'NR==1 {next} {print}' | xxd -r -ps | base64 | sed -e '1i -----BEGIN CERTIFICATE-----' -e '$a -----END CERTIFICATE-----' > VMW_EKECC_CACERT.pem
            #$BASE_DIR/ibmtss/utils/eventextend -if $p1 -tpm; \
            #$BASE_DIR/ibmtss/utils/imaextend -if $p2 -le; \
        sudo bash -c "\
            cd $BASE_DIR/ibmacs; \
            export LD_LIBRARY_PATH=\"$BASE_DIR/ibmtss/utils:$LD_LIBRARY_PATH\"; \
            $BASE_DIR/ibmacs/client -alg ec -vv -ifb $p1 -ifi $p2 -ho $ACS_DEMO_SERVER_IP -ma $ACS_DEMO_CLIENT_IP"
    elif [ $approach -eq 2 ]; then
        # this approach directly calls setup_ibmtpm for consistantly remote attestation
        #1> disable swtpm daemon
        sudo systemctl stop swtpm.service

        #2> display help info
        echo_notice "$dirname" "$filename" "remember to restore swtpm daemon with following commands"
        echo "sudo systemctl start swtpm.service"
        echo "sleep 1"
        echo "tpm2_startup -c"
        cd ../setup_ibmtpm
        #3> call setup_ibmtpm with interval var
        echo_notice "$dirname" "$filename" "launching Remote Attestation script in setup_ibmtpm"
        echo_notice "$dirname" "$filename" "choose ${RED}${BOLD}config_RA_client_launch.ini${END}"
		lc1="echo \"launching Remote Attestation script in setup_ibmtpm\" && \
			 echo -e \"choose \033[31m\033[1mconfig_RA_client_launch.ini\033[0m\""
        lc2="sudo install_platform=5 user=$USER interval=$interval bash ./setup_sudo.sh || :"
        newGterm "TPM SERVER" "$bash_gflag +e" "$lc1; $lc2" 1
    fi
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
