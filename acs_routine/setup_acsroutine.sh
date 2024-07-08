#!/bin/bash

script=$(realpath "$0")
script_path=$(dirname "$script")
dirname=$(basename "$script_path")
filename=$(basename "$0")
source "$script_path/../common/functions.sh"
source "$script_path/function_acsroutine.sh"
load_preset "$script_path/config.ini"
script=$(realpath "$0")
script_path=$(dirname "$script")
dirname=$(basename "$script_path")
filename=$(basename "$0")

# clone libtrace install program
#   - https://github.com/belongtothenight/ACN_Code/releases
setup_libtrace () {
    echo_notice "$dirname" "$filename" "Disabling wget certification check"
    WGET_CONFIG_STR="check_certificate = off"
    echo "$WGET_CONFIG_STR" >> $WGET_RC_PATH

    echo_notice "$dirname" "$filename" "Downloading libtrace setup script"
    sudo wget $WGET_FLAG "https://github.com/belongtothenight/ACN_Code/archive/refs/tags/1.0.0.tar.gz" -O $SETUP_LIBTRACE_ZIPNAME || { rm -rf $SETUP_LIBTRACE_ZIPNAME; echo -e "${RED}${BOLD}download failed, removing partial file!${END}"; exit 1; }
    if ! [ -d $SETUPLIBTRACE_DIRNAME ]; then sudo mkdir $SETUPLIBTRACE_DIRNAME; fi
    sudo tar $TAR_FLAG $SETUP_LIBTRACE_ZIPNAME -C $SETUPLIBTRACE_DIRNAME $TAR_ADD_FLAG

    echo_notice "$dirname" "$filename" "Enabling wget certification check"
    sed -i "/$WGET_CONFIG_STR/d" $WGET_RC_PATH

    echo_notice "$dirname" "$filename" "Changing libtrace dependency installing directory"
    sudo sed -i "/program_install_dir=\"*\"/c program_install_dir=\"$LIBTRACEDEP_DIRNAME\"" "$SETUPLIBTRACE_DIRNAME/hw4_libtrace_setup/common_functions.sh"

    echo_notice "$dirname" "$filename" "Changing file mode"
    cd "$SETUPLIBTRACE_DIRNAME"
    sudo find $SETUPLIBTRACE_DIRNAME -type f -iname "*.sh" -exec sudo chmod +x {} \;

    echo_notice "$dirname" "$filename" "Launching libtrace installation script"
    cd "$SETUPLIBTRACE_DIRNAME/hw4_libtrace_setup"
    sudo mkdir $LIBTRACEDEP_DIRNAME || :
    sudo bash ./setup.sh || :
}

# compile attestlog_AD and traffic_AD
setup_acsroutine () {
    echo_notice "$dirname" "$filename" "Compiling binaries for ACS Remote Attestation routine"
    ./bootstrap.sh
    ./configure
    make
}

# execute SERVER program
#   - [O] open ACS demo server
#   - [O] open ACS demo server webpage
#   - [O] constant traffic monitoring
#   - [O] routine ACS DB parsing for state change
#   - [O] block client if top two condition fails
server_exec_task1=1
server_exec_task2=1
server_exec_task3=1
server_exec_task4=1
server_exec () {
    if [ $server_exec_task1 -eq 1 ]; then
        echo_notice "$dirname" "$filename" "ACS server starting"
        lc1="source $script_path/../common/functions.sh"
        lc2="echo_notice \"$dirname\" \"$filename\" 'Launching ACS server'"
        lc3="cd $BASE_DIR/ibmacs"
        lc4="export LD_LIBRARY_PATH=$BASE_DIR/ibmtss/utils:$LD_LIBRARY_PATH ACS_PORT=$ACS_PORT ACS_SQL_USERID=\"$MYSQL_USER\" ACS_SQL_PASSWORD=\"$MYSQL_PASSWORD\" TPM_INTERFACE_TYPE=dev"
        lc5="./server -vv -root $BASE_DIR/ibmtss/utils/certificates/rootcerts.txt -imacert imakey.der"
        #lc5="log_date_time \"./server -vv -root $BASE_DIR/ibmtss/utils/certificates/rootcerts.txt -imacert imakey.der\" \"$LOG4J_TIME_FORMAT\" \"$BASE_DIR/ibmacs/serverenroll.log4j\" \"default\""
        newLXterm "ACS SERVER" "sudo bash -c \"$lc1; $lc2; $lc3; $lc4; service apache2 restart; $lc5 || :\"" 1
        #sudo bash -c "$lc2; $lc3; $lc4; $lc5 || { pkill apache2 && service apache2 restart && $lc5; }" # debug use
    fi
    if [ $server_exec_task2 -eq 1 ]; then
        echo_notice "$dirname" "$filename" "ACS website starting"
        lc1="source $PWD/../common/functions.sh"
        lc2="chromium-browser --new-window localhost/acs"
        newLXterm "ACS Webpage" "$lc1; $lc2" 1
    fi
    if [ $server_exec_task3 -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Launching traffic monitoring binary"
        lc1="source $script_path/../common/functions.sh"
        lc2="echo_notice \"$dirname\" \"$filename\" 'Traffic monitoring'"
        lc3="sudo $script_path/traffic_AD/traffic_AD -H $TRAFFIC_MONITOR_HOST -i $TRAFFIC_MONITOR_INTERFACE -s 30 -c 3 -t 1000 -v || :"
        #bash -c "$lc1; $lc2; $lc3"
        newLXterm "Traffic Monitoring" "bash -c \"$lc1; $lc2; $lc3\"" 1
    fi
    if [ $server_exec_task4 -eq 1 ]; then
        echo_notice "$dirname" "$filename" "Launching ACS DB monitoring binary"
        #bash -c "MYSQL_HOST=${MYSQL_HOST} MYSQL_USER=${MYSQL_USER} MYSQL_PASSWORD=${MYSQL_PASSWORD} MYSQL_DATABASE=${MYSQL_DATABASE} MYSQL_PORT=${MYSQL_PORT} interval=${interval} ${script_path}/launch_attestlog_AD.sh"
        newLXterm "ACS DB Parsing" "MYSQL_HOST=${MYSQL_HOST} MYSQL_USER=${MYSQL_USER} MYSQL_PASSWORD=${MYSQL_PASSWORD} MYSQL_DATABASE=${MYSQL_DATABASE} MYSQL_PORT=${MYSQL_PORT} interval=${interval} ${script_path}/launch_attestlog_AD.sh" 1
    fi
}

# execute CLIENT program
#   - [O] routine send attestation info
#   - [O] constant traffic monitoring
#   - [O] block client if top one condition fails
client_exec_approach=2
client_exec_task1=1
client_exec_task2=1
client_exec () {
    if [ $client_exec_task1 -eq 1 ]; then
        #echo "client_exec_approach: $approach"
        if [ $client_exec_approach -eq 1 ]; then
            # this client_exec_approach tries to execute client only with systemd managed swtpm
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
        elif [ $client_exec_approach -eq 2 ]; then
            # this client_exec_approach directly calls setup_ibmtpm for consistantly remote attestation
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
    fi
    if [ $client_exec_task1 -eq 2 ]; then
        echo_notice "$dirname" "$filename" "Launching traffic monitoring binary"
        lc1="source $script_path/../common/functions.sh"
        lc2="echo_notice \"$dirname\" \"$filename\" 'Traffic monitoring'"
        lc3="sudo $script_path/traffic_AD/traffic_AD -H $TRAFFIC_MONITOR_HOST -i $TRAFFIC_MONITOR_INTERFACE -s 30 -c 3 -t 1000 -v || :"
        #bash -c "$lc1; $lc2; $lc3"
        newLXterm "Traffic Monitoring" "bash -c \"$lc1; $lc2; $lc3\"" 1
    fi
}

if [ $job_setup_libtrace    -eq 1 ]; then setup_libtrace; fi
if [ $job_setup             -eq 1 ]; then setup_acsroutine; fi
if [ $is_server -eq 1 ]; then
    if [ $job_exec  -eq 1 ]; then server_exec; fi
else
    if [ $job_exec  -eq 1 ]; then client_exec; fi
fi

echo_notice "$dirname" "$filename" "$script finished"
