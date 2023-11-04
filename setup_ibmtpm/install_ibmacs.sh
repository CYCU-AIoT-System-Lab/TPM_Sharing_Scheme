#!/bin/bash

# ==================================================================================================
# Parameters
base_dir="/opt"
ibmacs_ver="1658"
html_dir="/var/www/html/acs"
c_src_dir="/usr/include/json-c"
c_link_dir="/usr/include/json"
acsMode=1 # 1: Server, 2: Client
verMode=1 # 1: TPM 2.0, 2: TPM 1.2 & 2.0
# ==================================================================================================

BOLD='\033[1m'
BLUE='\033[34m'
RED='\033[31m'
GREEN='\033[32m'
NC='\033[0m'

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMACS${NC}\n====================================================\n"

path_ibmtss="${base_dir}/ibmtss${ibmtss_ver}/"
path_ibmacs="${base_dir}/ibmacs${ibmacs_ver}/"

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
    mysql -Bse "CREATE DATABASE IF NOT EXISTS tpm2;"
    mysql -Bse "CREATE USER IF NOT EXISTS 'tpm2ACS'@'localhost' IDENTIFIED BY '123456';"
    mysql -Bse "GRANT ALL PRIVILEGES ON tpm2.* TO 'tpm2ACS'@'localhost';"
    mysql -D tpm2 < "${path_ibmacs}/acs/dbinit.sql"
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

echo -e "${BOLD}${BLUE}Creating symbolic link to ${c_src_dir} ......${NC}"
ln -s "${c_src_dir}" "${c_link_dir}"

echo -e "${BOLD}${BLUE}Compiling IBMACS and setting include path ......${NC}"
if [ $verMode == 1 ]; then
    # for TPM 2.0
    cd "${path_ibmacs}/acs/"
    export CPATH="${path_ibmtss}/utils"
    export LIBRARY_PATH="${path_ibmtss}/utils"
    make clean
    make
elif [ $verMode == 2]; then
    # for TPM 1.2 & 2.0
    cd "${path_ibmacs}/acs/"
    export CPATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
    export LIBRARY_PATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
    if [ $acsMode == 1 ]; then
        # for Server
        make -f makefiletpm12 clean
        make -f makefiletpm12
    elif [ $acsMode == 2 ]; then
        # for Client
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

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMACS Complete${NC}\n====================================================\n"
