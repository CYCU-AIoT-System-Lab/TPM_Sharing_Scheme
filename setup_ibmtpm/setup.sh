#!/bin/bash

# ==================================================================================================
# Parameters
# Param - path
download_dir="/home/user/Downloads"      # default: /home/user/Downloads
base_dir="/opt"                          # default: /opt
html_dir="/var/www/html/acs"             # default: /var/www/html/acs
c_json_lib_dir="/usr/include/json-c"     # default: /usr/include/json-c
c_json_lib_link_dir="/usr/include/json"  # default: /usr/include/json
nvim_dir="/home/user/.config/nvim"       # default: /home/user/.config/nvim
bashrc_dir="/home/user/.bashrc"          # default: /home/user/.bashrc
tpm_data_dir="/home/user/tpm2"           # default: /home/user/tpm2
# Param - filename
RSAEK_cert="cakey.pem"                   # default: cakey.pem
ECCEK_cert="cakeyecc.pem"                # default: cakeyecc.pem
# Param - url
repo_url="https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/setup_ibmtpm/setup_ibmtpm"
nvim_config_url="https://raw.githubusercontent.com/belongtothenight/config-files/main/ubuntu_init.vim"
acs_demo_server_ip="localhost"           # default: localhost
acs_demo_server_port="80"                # default: 80
acs_demo_client_ip="localhost"           # default: localhost
# Param - user account
user_name="user"                         # default: user
# Param - version
ibmtss_ver="2.1.1"                       # default: 2.1.1
ibmtpm_ver="1682"                        # default: 1682
ibmacs_ver="1658"                        # default: 1658
# Param - port
tpm_command_port="2321"                  # default: 2321
acs_port="2323"                          # default: 2323
# Param - mode
verMode=2                                # 1: TPM 2.0,      2: TPM 1.2 & 2.0      # default: 2
TPMMode=2                                # 1: Physical TPM, 2: Software TPM       # default: 2
acsMode=1                                # 1: Server,       2: Client             # default: 1
SCmachineMode=1                          # 1: Same machine, 2: Different machine  # default: 1 (server and client)
force_acs_sql_setting=0                  # 0: No,           1: Yes                # default: 0 (not tested)
acsClientMode=1                          # 1: Local,        2: Remote             # default: 1
# Param - mysql
mysql_user="tpm2ACS"                     # default: tpm2ACS
mysql_password="123456"                  # default: 123456
mysql_database="tpm2"                    # default: tpm2
# Param - job
default_job_1=0                          # 0: No, 1: Yes  # default: 1
default_job_0=0                          # 0: No, 1: Yes  # default: 0
install_req=$default_job_1               # 0: No, 1: Yes  # default: 1
setup_ibmtpmtss_env=$default_job_1       # 0: No, 1: Yes  # default: 1
#compile_ibmtpmtss=$default_job_1         # 0: No, 1: Yes  # default: 1
compile_ibmtpmtss=1       # 0: No, 1: Yes  # default: 1
setup_ibmswtpm_env=$default_job_1        # 0: No, 1: Yes  # default: 1
compile_ibmswtpm=$default_job_1          # 0: No, 1: Yes  # default: 1
setup_ibmacs_env=$default_job_1          # 0: No, 1: Yes  # default: 1
compile_ibmacs=$default_job_1            # 0: No, 1: Yes  # default: 1
open_demo_webpage=$default_job_1         # 0: No, 1: Yes  # default: 1
generate_CA=$default_job_0               # 0: No, 1: Yes  # default: 0 (not implemented)
activate_TPM_server=$default_job_0       # 0: No, 1: Yes  # default: 0
activate_TPM_client=$default_job_0       # 0: No, 1: Yes  # default: 0
generate_EK=$default_job_1               # 0: No, 1: Yes  # default: 1
retrieve_hardware_NV=$default_job_0      # 0: No, 1: Yes  # default: 0 (not implemented)
set_acs_sql_setting=$default_job_0       # 0: No, 1: Yes  # default: 0
active_ACS_Demo_Server=$default_job_1    # 0: No, 1: Yes  # default: 1
active_ACS_Demo_Client=$default_job_1    # 0: No, 1: Yes  # default: 1 (can't enroll)
active_ACS_Demo_verify=$default_job_1    # 0: No, 1: Yes  # default: 1 (can't verify)
# ==================================================================================================

dn_ibmtss="ibmtss"
dn_ibmtpm="ibmtpm"
dn_ibmacs="ibmacs"
fn_ibmtss="${dn_ibmtss}${ibmtss_ver}.tar.gz"
fn_ibmtpm="${dn_ibmtpm}${ibmtpm_ver}.tar.gz"
fn_ibmacs="${dn_ibmacs}${ibmacs_ver}.tar.gz"
file_ibmtss="${download_dir}/${fn_ibmtss}"
file_ibmtpm="${download_dir}/${fn_ibmtpm}"
file_ibmacs="${download_dir}/${fn_ibmacs}"
sym_link_ibmtss="${base_dir}/${dn_ibmtss}"
sym_link_ibmtpm="${base_dir}/${dn_ibmtpm}"
sym_link_ibmacs="${base_dir}/${dn_ibmacs}"
path_ibmtss="${sym_link_ibmtss}${ibmtss_ver}"
path_ibmtpm="${sym_link_ibmtpm}${ibmtpm_ver}"
path_ibmacs="${sym_link_ibmacs}${ibmacs_ver}"
path_NV="${sym_link_ibmtpm}/src/NVChip"
tss_cert_rootcert_dir="${sym_link_ibmtss}/utils/certificates"
acs_demo_url="${acs_demo_server_ip}:${acs_demo_server_port}/acs"
acs_demo_server_log_dir="${sym_link_ibmacs}/serverenroll.log4j"
acs_demo_client_log_dir="${sym_link_ibmacs}/clientenroll.log4j"
swtpm_bios_log_dir="${sym_link_ibmtss}/utils/tpm2bios.log"
acs_demo_verify_tpm2bios_log_dir="${sym_link_ibmtss}/utils/b.log4j"
ima_sig_log_dir="${sym_link_ibmtss}/utils/imasig.log4j"
acs_demo_verify_imasig_log_dir="${sym_link_ibmtss}/utils/i.log4j"
acs_demo_verify_client_log_dir="${sym_link_ibmtss}/utils/client.log4j"

BOLD='\033[1m'
BLUE='\033[34m'
RED='\033[31m'
GREEN='\033[32m'
ORANGE='\033[33m'
NC='\033[0m'

# $1: file/unit
# $2: message
function echo_notice () {
	echo -e "${BOLD}${BLUE}[NOTICE-setup_ibmtpm/$1]${NC} $2"
}

# $1: file/unit
# $2: message
function echo_warn () {
	echo -e "${BOLD}${ORANGE}[WARN-setup_ibmtpm/$1]${NC} $2"
}

# Install requirements for development, building, and testing
# Download ibmtss, ibmtpm, and ibmacs from sourceforge
# Extract ibmtss, ibmtpm, and ibmacs
# Only need to setup once (can re-run)
install_req () {
	echo_notice "setup-install_req" "Starting: install_req"

	echo_notice "setup-install_req" "Creating directories ..."
    sudo mkdir "${path_ibmtss}"
    sudo mkdir "${path_ibmtpm}"
    sudo mkdir "${path_ibmacs}"

	echo_notice "setup-install_req" "Downloading IBMTPMTSS ..."
    sudo wget "https://sourceforge.net/projects/ibmtpm20tss/files/${fn_ibmtss}/download" -O "${path_ibmtss}/${fn_ibmtss}"

	echo_notice "setup-install_req" "Downloading IBMSWTPM ..."
    sudo wget "https://sourceforge.net/projects/ibmswtpm2/files/${fn_ibmtpm}/download" -O "${path_ibmtpm}/${fn_ibmtpm}"

	echo_notice "setup-install_req" "Downloading IBMACS ..."
    sudo wget "https://sourceforge.net/projects/ibmtpm20acs/files/${fn_ibmacs}/download" -O "${path_ibmacs}/${fn_ibmacs}"

	echo_notice "setup-install_req" "Extracting IBMTPMTSS ..."
    sudo tar -zxf "${path_ibmtss}/${fn_ibmtss}" -C ${path_ibmtss}

	echo_notice "setup-install_req" "Extracting IBMSWTPM ..."
    sudo tar -zxf "${path_ibmtpm}/${fn_ibmtpm}" -C ${path_ibmtpm}

	echo_notice "setup-install_req" "Extracting IBMACS ..."
    sudo tar -zxf "${path_ibmacs}/${fn_ibmacs}" -C ${path_ibmacs}

	echo_notice "setup-install_req" "Complete: install_req"
}

# Set environment variables for ibmtss, and create symbolic link to ibmtss
# Can re-run to update environment variables to switch between physical TPM and software TPM
# Only need to setup once (can re-run)
setup_ibmtpmtss_env () {
	echo_notice "setup-setup_ibmtpmtss_env" "Starting: setup_ibmtpmtss_env"

	echo_notice "setup-setup_ibmtpmtss_env" "Setting path env variable ..."
    if [ $TPMMode == 1 ]; then
        # for Pysical TPM
        export TPM_INTERFACE_TYPE=dev
    elif [ $TPMMode == 2 ]; then
        # for Software TPM
        export TPM_INTERFACE_TYPE=socsim
    else 
        echo -e "${BOLD}${RED}Invalid TPMMode${NC}"
		echo_warn "setup-setup_ibmtpmtss_env" "Invalid TPMMode"
        exit 1
    fi
    export TPM_COMMAND_PORT="${tpm_command_port}"

	echo_notice "setup-setup_ibmtpmtss_env" "Creating symbolic link to ${path_ibmtss} ..."
    sudo ln -s "${path_ibmtss}" "${base_dir}/ibmtss"

	echo_notice "setup-setup_ibmtpmtss_env" "Complete: setup_ibmtpmtss_env"
}

# Compile ibmtss
# Can be run multiple times for code adjustment
compile_ibmtpmtss () {
	echo_notice "setup-compile_ibmtpmtss" "Starting: compile_ibmtpmtss"

	echo_notice "setup-compile_ibmtpmtss" "Cleaning up with make ..."
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmtss}/utils/"
        make -f makefiletpm20 clean
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmtss}/utils/"
        make -f makefiletpmc clean
        cd "${path_ibmtss}/utils12/"
        make -f makefiletpmc clean
    else 
		echo_warn "setup-compile_ibmtpmtss" "Invalid verMode"
        exit 1
    fi

	echo_notice "setup-compile_ibmtpmtss" "Compiling IBMTSS ..."
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmtss}/utils/"
        make -f makefiletpm20
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmtss}/utils/"
        make -f makefiletpmc
        cd "${path_ibmtss}/utils12/"
        make -f makefiletpmc
    else 
		echo_warn "setup-compile_ibmtpmtss" "Invalid verMode"
        exit 1
    fi

	echo_notice "setup-compile_ibmtpmtss" "Complete: compile_ibmtpmtss"
}

# Create symbolic link to ibmswtpm
# Only need to setup once (can re-run)
setup_ibmswtpm_env () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Setting up IBMSWTPM Environment${NC}${end_spacer}"

    echo -e "${BOLD}${BLUE}Creating symbolic link to ${path_ibmtpm} ......${NC}"
    ln -s "${path_ibmtpm}" "${base_dir}/ibmtpm"

    echo -e "${start_spacer}>>${BOLD}${GREEN}Setting up IBMSWTPM Environment Complete${NC}${end_spacer}"
}

# Compile ibmswtpm
# Can be run multiple times for code adjustment
compile_ibmswtpm () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Compiling IBMTPM${NC}${end_spacer}"

    echo -e "${BOLD}${BLUE}Cleaning up ......${NC}"
    cd "${path_ibmtpm}/src/"
    make clean

    echo -e "${BOLD}${BLUE}Compiling IBMTPM ......${NC}"
    cd "${path_ibmtpm}/src/"
    make

    echo -e "${start_spacer}>>${BOLD}${GREEN}Compiling IBMTPM Complete${NC}${end_spacer}"
}

# Install requirements for ibmacs, create mysql database, set environment variables, link directories, and generate directory for webpage
# Only need to setup once (can re-run)
setup_ibmacs_env () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Setting Up IBMACS Environment${NC}${end_spacer}"

    echo -e "${BOLD}${BLUE}Installing IBMACS dependencies ......${NC}"
    if [ $acsMode == 1 ]; then
        # for Server
        apt-get install -y libjson-c-dev apache2 php php-dev php-mysql mysql-server libmysqlclient-dev libssl-dev
    elif [ $acsMode == 2 ]; then
        # for Client
        apt-get install -y libjson-c-dev libssl-dev
    else 
        echo -e "${BOLD}${RED}Invalid acsMode${NC}"
        exit 1
    fi

    if [ $acsMode == 1 ]; then
        echo -e "${BOLD}${BLUE}Setting database ......${NC}"
        cd "${path_ibmacs}/acs/"
        mysql -Bse "CREATE DATABASE IF NOT EXISTS ${mysql_database};"
        mysql -Bse "CREATE USER IF NOT EXISTS '${mysql_user}'@'${acs_demo_server_ip}' IDENTIFIED BY '${mysql_password}';"
        mysql -Bse "GRANT ALL PRIVILEGES ON ${mysql_database}.* TO '${mysql_user}'@'${acs_demo_server_ip}';"
        mysql -D ${mysql_database} < "${path_ibmacs}/acs/dbinit.sql"
    fi

    echo -e "${BOLD}${BLUE}Setting include path ......${NC}"
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${path_ibmtss}/utils:${path_ibmtss}/utils12"
    export PATH="${PATH}:${path_ibmtss}/utils:${path_ibmtss}/utils12"

    echo -e "${BOLD}${BLUE}Creating symbolic link to ${path_ibmacs} ......${NC}"
    ln -s "${path_ibmacs}/acs" "${base_dir}/ibmacs"

    echo -e "${BOLD}${BLUE}Setting html directory ......${NC}"
    mkdir ${html_dir}
    chown root ${html_dir}
    chgrp root ${html_dir}
    chmod 777 ${html_dir}

    echo -e "${BOLD}${BLUE}Creating symbolic link to ${c_json_lib_dir} ......${NC}"
    ln -s "${c_json_lib_dir}" "${c_json_lib_link_dir}"

    echo -e "${BOLD}${BLUE}Setting include path ......${NC}"
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils"
        export LIBRARY_PATH="${path_ibmtss}/utils"
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
        export LIBRARY_PATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
    else 
        echo -e "${BOLD}${RED}Invalid verMode${NC}"
        exit 1
    fi

    echo -e "${start_spacer}>>${BOLD}${GREEN}Setting Up IBMACS Environment Complete${NC}${end_spacer}"
}

# Compile ibmacs
# Can be run multiple times for code adjustment
compile_ibmacs () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Compiling IBMACS${NC}${end_spacer}"

    echo -e "${BOLD}${BLUE}Compiling IBMACS and setting include path ......${NC}"
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils"
        export LIBRARY_PATH="${path_ibmtss}/utils"
        make clean
        make
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
        export LIBRARY_PATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
        if [ $acsMode == 1 ]; then
            # for Server
            make -f makefiletpmc clean
            make -f makefiletpmc
        elif [ $acsMode == 2 ]; then
            # for Client
            make -f makefiletpm12 clean
            make -f makefiletpm12
            make -f makefiletpmc clean
            make -f makefiletpmc
        else 
            echo -e "${BOLD}${RED}Invalid acsMode${NC}"
            exit 1
        fi
    else 
        echo -e "${BOLD}${RED}Invalid verMode${NC}"
        exit 1
    fi

    echo -e "${start_spacer}>>${BOLD}${GREEN}Compiling IBMACS Complete${NC}${end_spacer}"
}

# Open demo webpage with firefox
# Can be run multiple times
open_demo_webpage () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Opening Demo Webpage${NC}${end_spacer}"

    echo -e "${BOLD}${BLUE}Opening demo webpage ......${NC}"
    # start firefox without root on new terminal
    command1="sudo -u ${user_name} bash -c \"firefox --new-tab -url ${acs_demo_url} --new-tab -url ${repo_url} &\""
    gnome-terminal -t "Demo Firefox Website" -- bash -c "${command1}; exec bash"

    echo -e "${start_spacer}>>${BOLD}${GREEN}Opening Demo Webpage Complete${NC}${end_spacer}"
}

# Generate CA certificate and key
# Only need to setup once (can re-run)
generate_CA () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Generating CA${NC}${end_spacer}"

    echo -e "${BOLD}${ORANGE}Function not implemented${NC}"
    echo -e "${BOLD}${ORANGE}Refer to ${sym_link_ibmacs}/README.txt line 171 for steps.${NC}"
    echo -e "${BOLD}${GREEN}Generated CAs in ${sym_link_ibmtss}/utils ...... ${NC}"
    ls "${sym_link_ibmtss}/utils/"*.pem
    echo -e "${BOLD}${GREEN}Generated CAs in ${sym_link_ibmacs} ...... ${NC}"
    ls "${sym_link_ibmacs}/"*.pem

    echo -e "${start_spacer}>>${BOLD}${GREEN}Generating CA Complete${NC}${end_spacer}"
}

# Activate TPM Server in new terminal
# Only need to setup once (can re-run)
activate_TPM_server () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Activating TPM${NC}${end_spacer}"

    # apply TPMMode for ibmtss
    setup_ibmtpmtss_env

    echo -e "${BOLD}${ORANGE}Starting TPM simulator (server) on new temrinal ......${NC}"
    cd "${sym_link_ibmtpm}/src/"
    command="echo \"starting TPM simulator (server)\"; ./tpm_server"
    gnome-terminal -t "TPM SERVER" --active -- bash -c "${command}; exec bash"

    echo -e "${start_spacer}>>${BOLD}${GREEN}Activating TPM Complete${NC}${end_spacer}"
}

# Activate TPM Client in current terminal
# Only need to setup once (can re-run)
activate_TPM_client () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Activating TPM${NC}${end_spacer}"

    echo -e "${BOLD}${ORANGE}Starting TPM simulator (client) on new temrinal ......${NC}"
    cd "${sym_link_ibmtss}/utils/"
    ./powerup
    ./startup

    echo -e "${start_spacer}>>${BOLD}${GREEN}Activating TPM Complete${NC}${end_spacer}"
}

# Create EK certificate and key, activated TPM on new terminal
# Only need to setup once (can re-run)
generate_EK () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Generating EK${NC}${end_spacer}"

    activate_TPM_server

    activate_TPM_client

    echo -e "${BOLD}${ORANGE}Backing up NVChip ......${NC}"
    cp "${path_NV}" "${path_NV}.bak"

    echo -e "${BOLD}${ORANGE}Generating RSAEK and load into NV ......${NC}"
    ./createekcert -rsa 2048 -cakey $RSAEK_cert -capwd rrrr -v

    echo -e "${BOLD}${ORANGE}Generating ECCEK and load into NV ......${NC}"
    ./createekcert -ecc nistp256 -cakey $ECCEK_cert -capwd rrrr -caalg ec -v

    echo -e "${start_spacer}>>${BOLD}${GREEN}Generating EK Complete${NC}${end_spacer}"
}

# Retrieve hardware NVChip
# Only need to setup once (can re-run)
retrieve_hardware_NV () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Retrieving Hardware NVChip${NC}${end_spacer}"

    echo -e "${BOLD}${ORANGE}Not implemented${NC}"

    echo -e "${start_spacer}>>${BOLD}${GREEN}Retrieving Hardware NVChip Complete${NC}${end_spacer}"
}

# Set ACS MYSQL setting
# Only need to setup once (can re-run)
set_acs_sql_setting () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Setting ACS MYSQL Setting${NC}${end_spacer}"

    echo -e "${BOLD}${ORANGE}Setting ACS MYSQL Setting ......${NC}"
    export ACS_SQL_USERID="${mysql_user}"
    export ACS_SQL_PASSWORD="${mysql_password}"

    if [ $force_acs_sql_setting == 1 ]; then
        echo -e "${BOLD}${ORANGE}Forcing ACS MYSQL Setting ......${NC}"
        cp "${html_dir}/dbconnect.php" "${html_dir}/dbconnect.php.bak"
        sed -i "s/\$connect = new mysqli(\$acs_sql_host, \$acs_sql_userid, \$acs_sql_password, \$acs_sql_database);/\$connect = new mysqli(${acs_demo_server_ip}, ${mysql_user}, ${mysql_password}, ${mysql_database});/g" "${html_dir}/dbconnect.php"
    else
        echo -e "${BOLD}${ORANGE}Not Forcing ACS MYSQL Setting ......${NC}"
    fi

    echo -e "${start_spacer}>>${BOLD}${GREEN}Setting ACS MYSQL Setting Complete${NC}${end_spacer}"
}

# Active ACS Demo Server
# Can be run multiple times
active_ACS_Demo_Server () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Activating ACS Demo Server${NC}${end_spacer}"

    if [ $SCmachineMode == 1 ]; then
        echo -e "${BOLD}${BLUE}Activating ACS Demo on same machine ......${NC}"
        mkdir "${tpm_data_dir}"
        export TPM_DATA_DIR="${tpm_data_dir}"
    elif [ $SCmachineMode == 2 ]; then
        echo -e "${BOLD}${BLUE}Activating ACS Demo on different machine ......${NC}"
    else 
        echo -e "${BOLD}${RED}Invalid SCmachineMode${NC}"
        exit 1
    fi
    activate_TPM_server
    activate_TPM_client

    echo -e "${BOLD}${BLUE}Replacing path in ${tss_cert_rootcert_dir}/rootcerts.txt ......${NC}"
    cp "${tss_cert_rootcert_dir}/rootcerts.txt" "${tss_cert_rootcert_dir}/rootcerts.txt.bak"
    sed -i "s/\/home\/kgold\/tss2/\\${base_dir}\/${dn_ibmtss}/g" "${tss_cert_rootcert_dir}/rootcerts.txt"
    export ACS_PORT="${acs_port}"

    set_acs_sql_setting

    echo -e "${BOLD}${BLUE}Activating ACS Server Demo on new terminal ......${NC}"
    command="cd ${path_ibmacs}/acs; ./server -v -root ${tss_cert_rootcert_dir}/rootcerts.txt -imacert imakey.der >| ${acs_demo_server_log_dir}"
    gnome-terminal -t "ACS SERVER" --active -- bash -c "${command}; exec bash"

    echo -e "${start_spacer}>>${BOLD}${GREEN}Activating ACS Demo Server Complete${NC}${end_spacer}"
}

# Active ACS Demo Client
# Can be run multiple times
active_ACS_Demo_Client () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Activating ACS Demo Client${NC}${end_spacer}"

    cd "${path_ibmacs}/acs"
    if [ $acsClientMode == 1 ]; then
        echo -e "${BOLD}${BLUE}Activating ACS Client Demo on local machine ......${NC}"
        ./clientenroll -alg rsa -v -ho ${acs_demo_server_ip} -co akcert.pem >| ${acs_demo_client_log_dir}
    elif [ $acsClientMode == 2 ]; then
        echo -e "${BOLD}${BLUE}Activating ACS Client Demo on remote machine ......${NC}"
        ./clientenroll -alg ec -v -ho ${acs_demo_server_ip} -ma ${acs_demo_client_ip} -co akeccert.pem >| ${acs_demo_client_log_dir}
    else 
        echo -e "${BOLD}${RED}Invalid acsClientMode${NC}"
        exit 1
    fi

    echo -e "${start_spacer}>>${BOLD}${GREEN}Activating ACS Demo Client Complete${NC}${end_spacer}"
}

# Active ACS Demo verify
# Can be run multiple times
active_ACS_Demo_verify () {
    echo -e "${start_spacer}>>${BOLD}${GREEN}Verifying ACS Demo${NC}${end_spacer}"

    if [ $TPMMode == 1 ]; then
        # for Pysical TPM
        echo -e "${BOLD}${BLUE}Ignore when working with Physical TPM${NC}"
    elif [ $TPMMode == 2 ]; then
        # for Software TPM
        cd "${sym_link_ibmtss}/utils/"
        echo -e "${BOLD}${BLUE}Checking TPM2BIOS.LOG ......${NC}"
        ${sym_link_ibmtss}/utils/eventextend -if ${swtpm_bios_log_dir} -tpm -v >| ${acs_demo_verify_tpm2bios_log_dir}

        echo -e "${BOLD}${BLUE}Checking IMASIG.LOG ......${NC}"
        ${sym_link_ibmtss}/utils/imaextend -if ${ima_sig_log_dir} -le -v >| ${acs_demo_verify_imasig_log_dir}

        if [ $acsClientMode == 1 ]; then
            # for Local
            ${sym_link_ibmacs}/client -alg rsa -ifb ${swtpm_bios_log_dir} -ifi ${ima_sig_log_dir} -ho ${acs_demo_server_ip} -v >| ${acs_demo_verify_client_log_dir}
        elif [ $acsClientMode == 2 ]; then
            # for Remote
            ${sym_link_ibmacs}/client -alg ec -ifb ${swtpm_bios_log_dir} -ifi ${ima_sig_log_dir} -ho ${acs_demo_server_ip} -v -ma ${acs_demo_client_ip} >| ${acs_demo_verify_client_log_dir}
        else 
            echo -e "${BOLD}${RED}Invalid acsClientMode${NC}"
            exit 1
        fi
    else 
        echo -e "${BOLD}${RED}Invalid TPMMode${NC}"
        exit 1
    fi

    echo -e "${start_spacer}>>${BOLD}${GREEN}Verifying ACS Demo Complete${NC}${end_spacer}"
}

echo_notice "setup" "Running setup script ..."

if [ $install_req            == 1 ]; then install_req;                 fi
if [ $setup_ibmtpmtss_env    == 1 ]; then setup_ibmtpmtss_env;         fi
if [ $compile_ibmtpmtss      == 1 ]; then compile_ibmtpmtss;           fi
if [ $setup_ibmswtpm_env     == 1 ]; then setup_ibmswtpm_env;          fi
if [ $compile_ibmswtpm       == 1 ]; then compile_ibmswtpm;            fi
if [ $setup_ibmacs_env       == 1 ]; then setup_ibmacs_env;            fi
if [ $compile_ibmacs         == 1 ]; then compile_ibmacs;              fi
if [ $open_demo_webpage      == 1 ]; then open_demo_webpage;           fi
if [ $generate_CA            == 1 ]; then generate_CA;                 fi
if [ $activate_TPM_server    == 1 ]; then activate_TPM_server;         fi
if [ $activate_TPM_client    == 1 ]; then activate_TPM_client;         fi
if [ $generate_EK            == 1 ]; then generate_EK;                 fi
if [ $retrieve_hardware_NV   == 1 ]; then retrieve_hardware_NV;        fi
if [ $set_acs_sql_setting    == 1 ]; then set_acs_sql_setting;         fi
if [ $active_ACS_Demo_Server == 1 ]; then active_ACS_Demo_Server;      fi
if [ $active_ACS_Demo_Client == 1 ]; then active_ACS_Demo_Client;      fi
if [ $active_ACS_Demo_verify == 1 ]; then active_ACS_Demo_verify;      fi

echo_notice "setup" "Setup complete"
